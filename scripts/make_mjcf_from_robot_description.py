#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import argparse
import bpy
import mujoco
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import PyKDL
import sys

from urdf_parser_py.urdf import URDF

from ament_index_python.packages import get_package_share_directory
from xml.dom import minidom

# These tags will be parsed from inputs and added to the converted MJCF
XML_INPUT_TAGS = [
    "sensor",
    "actuator",
    "default",
    "option",
    "tendon",
    "equality",
    "contact",
]

DECOMPOSED_PATH_NAME = "decomposed"
COMPOSED_PATH_NAME = "full"
CONVERTER_INPUTS_NAME = "converter_inputs"


def add_mujoco_info(raw_xml):
    dom = minidom.parseString(raw_xml)

    mujoco_element = dom.createElement("mujoco")
    compiler_element = dom.createElement("compiler")
    compiler_element.setAttribute("meshdir", "assets")
    compiler_element.setAttribute("balanceinertia", "true")
    compiler_element.setAttribute("discardvisual", "false")
    compiler_element.setAttribute("strippath", "false")

    mujoco_element.appendChild(compiler_element)

    robot_element = dom.getElementsByTagName("robot")

    robot_element[0].appendChild(mujoco_element)

    # Use minidom to format the string with line breaks and indentation
    formatted_xml = dom.toprettyxml(indent="    ")

    # Remove extra newlines that minidom adds after each tag
    formatted_xml = "\n".join([line for line in formatted_xml.splitlines() if line.strip()])

    return formatted_xml


def remove_tag(xml_string, tag_to_remove):
    xmldoc = minidom.parseString(xml_string)
    nodes = xmldoc.getElementsByTagName(tag_to_remove)

    for node in nodes:
        parent = node.parentNode
        parent.removeChild(node)

    return xmldoc.toprettyxml()


def extract_mesh_info(raw_xml):
    robot = URDF.from_xml_string(raw_xml)
    mesh_info_dict = {}

    robot_materials = dict()
    for material in robot.materials:
        robot_materials[material.name] = material

    def resolve_color(visual):
        mat = visual.material
        if mat is None:
            return (1.0, 1.0, 1.0, 1.0)
        if mat.color:
            return tuple(mat.color.rgba)
        if mat.name:
            if mat.name in robot_materials:
                ref = robot_materials[mat.name]
                if ref and ref.color:
                    return tuple(ref.color.rgba)
        return (1.0, 1.0, 1.0, 1.0)

    for link in robot.links:
        for vis in link.visuals:
            geom = vis.geometry
            if not (geom and hasattr(geom, "filename")):
                continue

            uri = geom.filename  # full URI
            stem = pathlib.Path(uri).stem  # filename without extension
            scale = " ".join(f"{v}" for v in geom.scale) if geom.scale else "1.0 1.0 1.0"
            rgba = resolve_color(vis)

            # If the same stem appears more than once, keep the first, or change as you prefer
            mesh_info_dict.setdefault(
                stem,
                {
                    "filename": uri,
                    "scale": scale,
                    "color": rgba,
                },
            )

    return mesh_info_dict


def replace_package_names(xml_data):
    # Regular expression to find all package names in "package://{package_name}/"
    pattern = r"package://([^/]+)/"

    # Find all matches for the package name (but no duplicates)
    package_names = set(re.findall(pattern, xml_data))

    # Replace all of the package looks up with absolute paths
    for package_name in package_names:
        old_string = f"package://{package_name}/"
        replace_string = f"{get_package_share_directory(package_name)}/"
        print(f"replacing {old_string} with {replace_string}")
        xml_data = xml_data.replace(old_string, replace_string)

    # get rid of absolute filepaths
    xml_data = xml_data.replace("file://", "")

    return xml_data


def convert_to_objs(mesh_info_dict, directory, xml_data, convert_stl_to_obj, decompose_dict):
    # keep track of files we have already processed so we don't do it again
    converted_filenames = []

    # clean assets directory and remake required paths
    if os.path.exists(f"{directory}assets/"):
        shutil.rmtree(f"{directory}assets/")
    os.makedirs(f"{directory}assets/{COMPOSED_PATH_NAME}", exist_ok=True)
    os.makedirs(f"{directory}assets/{DECOMPOSED_PATH_NAME}", exist_ok=True)

    for mesh_name in mesh_info_dict:
        mesh_item = mesh_info_dict[mesh_name]
        filename = os.path.basename(mesh_item["filename"])
        filename_no_ext = os.path.splitext(filename)[0]
        filename_ext = os.path.splitext(filename)[1]
        full_filepath = mesh_item["filename"]
        if full_filepath in converted_filenames:
            pass

        print(f"processing {full_filepath}")

        # Clear any existing objects in the scene
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete(use_global=False)

        # if we want to decompose the mesh, put it in decomposed filepath, otherwise put it in full
        if filename_no_ext in decompose_dict:
            assets_relative_filepath = f"{DECOMPOSED_PATH_NAME}/{filename_no_ext}/"
            os.makedirs(f"{directory}assets/{assets_relative_filepath}", exist_ok=True)
        else:
            assets_relative_filepath = f"{COMPOSED_PATH_NAME}/"
        assets_relative_filepath += filename_no_ext

        # Import the .stl or .dae file
        if filename_ext.lower() == ".stl":
            if convert_stl_to_obj:
                bpy.ops.wm.stl_import(filepath=full_filepath)

                # bring in file color from urdf
                new_mat = bpy.data.materials.new(name="new_mat_color")
                new_mat.diffuse_color = mesh_item["color"]
                o = bpy.context.selected_objects[0]
                o.active_material = new_mat

                bpy.ops.wm.obj_export(
                    filepath=f"{directory}assets/{assets_relative_filepath}.obj", forward_axis="Y", up_axis="Z"
                )
                xml_data = xml_data.replace(full_filepath, f"{assets_relative_filepath}.obj")

            else:
                shutil.copy2(full_filepath, f"{directory}assets/{assets_relative_filepath}.stl")
                xml_data = xml_data.replace(full_filepath, f"{assets_relative_filepath}.stl")
                pass
        elif filename_ext.lower() == ".obj":
            shutil.copy2(full_filepath, f"{directory}assets/{assets_relative_filepath}.obj")
            xml_data = xml_data.replace(full_filepath, f"{assets_relative_filepath}.obj")
            pass
            # objs are ok as is
        elif filename_ext.lower() == ".dae":
            # set z axis to up in the dae file because that is how mujoco expects it
            z_up_dae_txt = set_up_axis_to_z_up(full_filepath)

            # make a temporary file rather than overwriting the old one
            temp_file = tempfile.NamedTemporaryFile(suffix=".dae", delete=False)
            temp_filepath = temp_file.name
            try:
                with open(temp_filepath, "w") as f:
                    f.write(z_up_dae_txt)
                    # import into blender
                    bpy.ops.wm.collada_import(filepath=temp_filepath)
            finally:
                temp_file.close()
                os.remove(temp_filepath)

            bpy.ops.wm.obj_export(
                filepath=f"{directory}assets/{assets_relative_filepath}.obj", forward_axis="Y", up_axis="Z"
            )
            xml_data = xml_data.replace(full_filepath, f"{assets_relative_filepath}.obj")
        else:
            print(f"Can't convert {full_filepath} \n\tOnly stl and dae file extensions are supported at the moment")
            print(f"extension: {filename_ext}")
            pass

        # keep track of what we have been working on
        converted_filenames.append(full_filepath)

    return xml_data


def set_up_axis_to_z_up(dae_file_path):
    # Parse the DAE file from the in-memory file-like object using minidom
    dom = minidom.parse(dae_file_path)

    # Find the <asset> element
    asset = dom.getElementsByTagName("asset")

    if not asset:
        # Create the <asset> element if it doesn't exist
        asset_element = dom.createElement("asset")
        dom.documentElement.appendChild(asset_element)
    else:
        asset_element = asset[0]

    # Find the 'up_axis' tag in the asset element
    up_axis = asset_element.getElementsByTagName("up_axis")

    # If the 'up_axis' tag is found, update or add it
    if up_axis:
        up_axis_element = up_axis[0]
        # If it's not already set to Z_UP, update it
        if up_axis_element.firstChild.nodeValue != "Z_UP":
            up_axis_element.firstChild.nodeValue = "Z_UP"
            print(f"Updated 'up_axis' to 'Z_UP' for {dae_file_path}")
        else:
            print(f"'up_axis' is already 'Z_UP' for {dae_file_path}")
    else:
        # If the 'up_axis' tag doesn't exist, create it and set it to Z_UP
        new_up_axis = dom.createElement("up_axis")
        new_up_axis.appendChild(dom.createTextNode("Z_UP"))
        asset_element.appendChild(new_up_axis)
        print(f"Added 'up_axis' with value 'Z_UP' for {dae_file_path}")

    # Convert the DOM back to a string
    modified_data = dom.toprettyxml(indent="  ")

    # You can return the modified data if you need to further process it
    return modified_data


def run_obj2mjcf(output_filepath, decompose_dict):
    # remove the folders in the asset directory so that we are clean to run obj2mjcf
    with os.scandir(f"{output_filepath}assets/{COMPOSED_PATH_NAME}") as entries:
        for entry in entries:
            if entry.is_dir():
                shutil.rmtree(entry.path)

    # remove the folders in the asset directory so that we are clean to run obj2mjcf
    top_level_path = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}"
    for item in os.listdir(top_level_path):
        first_level_path = os.path.join(top_level_path, item)
        if os.path.isdir(first_level_path):
            # Now check inside this first-level directory
            for sub_item in os.listdir(first_level_path):
                second_level_path = os.path.join(first_level_path, sub_item)
                if os.path.isdir(second_level_path):
                    shutil.rmtree(second_level_path)

    # run obj2mjcf to generate folders of processed objs
    cmd = ["obj2mjcf", "--obj-dir", f"{output_filepath}assets/{COMPOSED_PATH_NAME}", "--save-mjcf"]
    subprocess.run(cmd)

    # run obj2mjcf to generate folders of processed objs with decompose option for decomposed components
    for mesh_name, threshold in decompose_dict.items():
        cmd = [
            "obj2mjcf",
            "--obj-dir",
            f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}/{mesh_name}",
            "--save-mjcf",
            "--decompose",
            "--coacd-args.threshold",
            threshold,
        ]
        subprocess.run(cmd)


def update_obj_assets(dom, output_filepath, mesh_info_dict):
    # Find the <asset> element
    asset = dom.getElementsByTagName("asset")

    # If there are no assets then we don't need to worry about obj conversions, but we still
    # support mesh-less URDFs
    if len(asset) == 0:
        print("No assets in URDF, skipping conversions...")
        return dom

    # Find the <worldbody> element
    worldbody = dom.getElementsByTagName("worldbody")
    worldbody_element = worldbody[0]
    worldbody_geoms = worldbody_element.getElementsByTagName("geom")

    # get all of the mesh tags in the asset element
    asset_element = asset[0]
    meshes = asset_element.getElementsByTagName("mesh")

    # obj
    full_decomposed_path = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}"
    full_composed_path = f"{output_filepath}assets/{COMPOSED_PATH_NAME}"
    decomposed_dirs = [
        name for name in os.listdir(full_decomposed_path) if os.path.isdir(os.path.join(full_decomposed_path, name))
    ]
    composed_dirs = [
        name for name in os.listdir(full_composed_path) if os.path.isdir(os.path.join(full_composed_path, name))
    ]

    for mesh in meshes:
        mesh_name = mesh.getAttribute("name")

        # This should definitely be there, otherwise something is horribly wrong
        scale = mesh_info_dict[mesh_name]["scale"]

        mesh_path = ""
        if mesh_name in decomposed_dirs:
            composed_type = DECOMPOSED_PATH_NAME
            mesh_path = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}/{mesh_name}/{mesh_name}/{mesh_name}.xml"
        elif mesh_name in composed_dirs:
            composed_type = COMPOSED_PATH_NAME
            mesh_path = f"{output_filepath}assets/{COMPOSED_PATH_NAME}/{mesh_name}/{mesh_name}.xml"

        if mesh_path:
            sub_dom = minidom.parse(mesh_path)
            # Find the <asset> element
            sub_asset = sub_dom.getElementsByTagName("asset")
            sub_asset_element = sub_asset[0]

            # remove the old mesh element that is not separated
            asset_element.removeChild(mesh)

            # bring in the new elements
            sub_meshes = sub_asset_element.getElementsByTagName("mesh")
            for sub_mesh in sub_meshes:
                sub_mesh_file = sub_mesh.getAttribute("file")
                if composed_type == DECOMPOSED_PATH_NAME:
                    sub_mesh.setAttribute("file", f"{composed_type}/{mesh_name}/{mesh_name}/{sub_mesh_file}")
                else:
                    sub_mesh.setAttribute("file", f"{composed_type}/{mesh_name}/{sub_mesh_file}")
                if scale:
                    sub_mesh.setAttribute("scale", scale)
                asset_element.appendChild(sub_mesh)

            # bring in the materials
            sub_materials = sub_asset_element.getElementsByTagName("material")
            for sub_material in sub_materials:
                asset_element.appendChild(sub_material)

            sub_body = sub_dom.getElementsByTagName("body")
            sub_body = sub_body[0]

            # change the geoms
            body = sub_dom.getElementsByTagName("body")
            body_element = body[0]
            sub_geoms = body_element.getElementsByTagName("geom")
            for geom_element in worldbody_geoms:
                if geom_element.getAttribute("mesh") == mesh_name:
                    pos = geom_element.getAttribute("pos")
                    quat = geom_element.getAttribute("quat")

                    parent = geom_element.parentNode
                    parent.removeChild(geom_element)
                    for sub_geom in sub_geoms:
                        sub_geom_local = sub_geom.cloneNode(False)
                        sub_geom_local.setAttribute("pos", pos)
                        sub_geom_local.setAttribute("quat", quat)
                        parent.appendChild(sub_geom_local)

    return dom


def update_non_obj_assets(dom, output_filepath):
    # We want to take the group 1 objects that get created, and turn them into the equivalent
    # but both in group 2 and in group 3. That means taking something like this
    #     <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="finger_v6"/>
    # and turning it into this
    #     <geom mesh="finger_v6" class="visual" pos="0 0 0" quat="0.707107 0.707107 0 0"/>
    #     <geom mesh="finger_v6" class="collision" pos="0 0 0" quat="0.707107 0.707107 0 0"/>
    #
    # To do this, we need to add in class visual, and class collision to them, keep the rgba on the visual one, and
    # get rid of the other components (type, contype, conaffinity, group, density)
    #
    # We can tell that we need to modify it because it will have a contype attribute attached to it (not the best way
    # but I guess it works for now)

    # Find the <worldbody> element
    worldbody = dom.getElementsByTagName("worldbody")
    worldbody_element = worldbody[0]

    # get all of the geom elements in the worldbody element
    worldbody_geoms = worldbody_element.getElementsByTagName("geom")

    # elements to remove
    remove_attributes = ["contype", "conaffinity", "group", "density"]

    for geom in worldbody_geoms:
        if not geom.hasAttribute("contype"):
            pass
        else:
            collision_geom = geom.cloneNode(False)

            # if there is no type associated, make the type sphere explicitly
            if not collision_geom.hasAttribute("type"):
                collision_geom.setAttribute("type", "sphere")

            # set to collision class
            collision_geom.setAttribute("class", "collision")
            for attribute in remove_attributes:
                if collision_geom.hasAttribute(attribute):
                    collision_geom.removeAttribute(attribute)

            # most of the components are the same between collision and visual, so just copy it
            visual_geom = collision_geom.cloneNode(False)
            visual_geom.setAttribute("class", "visual")

            # remove rgba from collision geom bc it isn't necessary
            if geom.hasAttribute("rgba"):
                collision_geom.removeAttribute("rgba")

            # get the parent of the geom node, and remove the old element
            parent = geom.parentNode
            parent.removeChild(geom)
            # add the new collision and visual specific elements
            parent.appendChild(collision_geom)
            parent.appendChild(visual_geom)

    return dom


def add_mujoco_inputs(dom, mujoco_inputs):

    root = dom.documentElement

    # Insert input elements
    # TODO: Insert at the top of the DOM?
    # TODO: Figure out how to add sensor and camera sites to the listed frames automatically
    # TODO: Do we need these to be XML_INPUT_TAGS? could this just be everything?
    for key in mujoco_inputs:
        # converter inputs is special and we don't add it
        if key != CONVERTER_INPUTS_NAME:
            for node in mujoco_inputs[key]:
                root.appendChild(node)

    return dom


def add_free_joint(dom, urdf, joint_name="floating_base_joint"):
    robot = URDF.from_xml_string(urdf)
    root_link = robot.get_root()
    if root_link == "world":
        print("Not adding a free joint because world is the URDF root")
        return

    # get the world body
    world_body = dom.getElementsByTagName("worldbody")[0]

    # make a new body with our base
    root_body = dom.createElement("body")
    root_body.setAttribute("name", root_link)

    # make a free joint under that body
    free_joint = dom.createElement("freejoint")
    free_joint.setAttribute("name", "floating_base_joint")
    root_body.appendChild(free_joint)

    # move the previous body underneath the new body and joint
    while world_body.hasChildNodes():
        child = world_body.firstChild
        world_body.removeChild(child)
        root_body.appendChild(child)

    world_body.appendChild(root_body)

    return dom


def add_links_as_sites(urdf, dom, add_free_joint):

    def remove_epsilons(vector, epsilon=1e-8):
        return [value if abs(value) > epsilon else 0 for value in vector]

    # get the static tfs from the urdf
    tfs = get_urdf_transforms(urdf)

    # check to see if we would have added the free joint
    robot = URDF.from_xml_string(urdf)
    root_link = robot.get_root()
    # if the root link is world, we wouldn't have added the free link
    if root_link == "world":
        add_free_joint = False

    # make a map so that we have a reference of parents to list of sites
    site_map = {}
    site_map["root_item"] = []
    for link, (parent, tf, is_root) in tfs.items():
        pos = remove_epsilons(tf.p)
        quat = remove_epsilons(tf.M.GetQuaternion())
        # change x y z w to w x y z for ros to mujoco convention
        quat = [quat[3], quat[0], quat[1], quat[2]]
        # root items are special because they don't attach to a body
        if is_root and not add_free_joint:
            site_map["root_item"].append((link, pos, quat))
        else:
            if parent not in site_map:
                site_map[parent] = [(link, pos, quat)]
            else:
                site_map[parent].append((link, pos, quat))

    # add the site elements for each of the bodies
    for parent, _ in site_map.items():
        for node in dom.getElementsByTagName("body"):
            if node.getAttribute("name") == parent:
                for site in site_map[parent]:
                    new_site = dom.createElement("site")
                    new_site.setAttribute("name", site[0])
                    new_site.setAttribute("pos", " ".join(map(str, site[1])))
                    new_site.setAttribute("quat", " ".join(map(str, site[2])))
                    node.appendChild(new_site)

    # add the site elements for the root element into worldbody
    node = dom.getElementsByTagName("worldbody")
    for site in site_map["root_item"]:
        print(link, site)
        new_site = dom.createElement("site")
        new_site.setAttribute("name", site[0])
        new_site.setAttribute("pos", " ".join(map(str, site[1])))
        new_site.setAttribute("quat", " ".join(map(str, site[2])))
        node[0].appendChild(new_site)

    # Convert the DOM back to a string
    modified_data = dom.toprettyxml(indent="  ")
    modified_data = "\n".join([line for line in modified_data.splitlines() if line.strip()])

    # remove the first line bc mujoco doesn't like the xml tag at the beginning
    modified_lines = modified_data.splitlines(True)
    modified_lines.pop(0)
    modified_data = "".join(modified_lines)

    return dom


def rotate_quaternion(q1, q2):
    """
    Returns q1*q2
    """
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2

    return [
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
    ]


def add_cameras_from_sites(dom):
    """
    Adds cameras for any sites that end with "_color_optical_frame". We make the assumption that any
    link/site with that name is the color frame for a physical camera matching the REP 103 standard for
    optical frames. As noted in the ROS image message,
    https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg#L7

        > +x should point to the right in the image
        > +y should point down in the image
        > +z should point into to plane of the image

    Mujoco Cameras are oriented slightly differently, https://mujoco.readthedocs.io/en/latest/modeling.html#cameras

        > Cameras look towards the negative Z axis of the camera frame, while positive X and Y correspond
        > to right and up in the image plane, respectively.

    Therefore, given the site name, the camera is added at the same position but it is rotated pi radians about the
    x axis to ensure the simulated camera's images match those from the URDF.

    TODO: Decide how to handle name and frame conventions. For the realsenses, the frame names include
          the camera name, image type, and suffix, for instance:
          "wrist_mounted_camera_color_optical_frame" and "wrist_mounted_camera_depth_optical_frame".
          We just want a single camera streaming both images, but there isn't a
          "wrist_mounted_camera_optical_frame" in the URDF. So for now we're just attaching both to the
          color frame and naming the camera "wrist_mounted_camera_color". We could alternatively have
          the frame name passed as an argument to the mujoco system, but that is additional work in the
          drivers.
    """

    # We only attach to the color frame, this is flimsy as noted in the comment above. It also
    # causes name changes in the camera.
    camera_suffix = "_color_optical_frame"
    x_rotation = [0.0, 1.0, 0.0, 0.0]  # Rotation by pi around x axis

    # Construct all cameras for relevant sites in xml and add them as children to the same parent
    for node in dom.getElementsByTagName("site"):
        name = node.getAttribute("name")
        if name.endswith(camera_suffix):
            camera_name = name[: -len(camera_suffix)] + "_color"
            camera_pos = node.getAttribute("pos")
            quat = [float(x) for x in node.getAttribute("quat").split()]
            camera_quat = rotate_quaternion(x_rotation, quat)

            # We could adjust these at some point down the road, maybe pass them as inputs that we parse?
            camera_node = dom.createElement("camera")
            camera_node.setAttribute("name", camera_name)
            camera_node.setAttribute("fovy", "58")
            camera_node.setAttribute("mode", "fixed")
            camera_node.setAttribute("resolution", "640 480")
            camera_node.setAttribute("pos", camera_pos)
            camera_node.setAttribute("quat", " ".join(map(str, camera_quat)))

            print(f"Adding camera: {camera_name}")
            print(f"  {camera_pos}")
            print(f"  {camera_quat}")
            node.parentNode.appendChild(camera_node)

    return dom


def fix_mujoco_description(
    output_filepath, mesh_info_dict, mujoco_inputs, urdf, decompose_dict, request_add_free_joint
):
    full_filepath = f"{output_filepath}mujoco_description.xml"
    filename, extension = os.path.splitext(f"{output_filepath}mujoco_description.xml")
    destination_file = filename + "_formatted" + extension
    # shutil.copy2(full_filepath, destination_file)

    # Run conversions for mjcf
    run_obj2mjcf(output_filepath, decompose_dict)

    # Parse the DAE file
    dom = minidom.parse(full_filepath)

    if request_add_free_joint:
        dom = add_free_joint(dom, urdf)

    # Update and add the new fixed assets
    dom = update_obj_assets(dom, output_filepath, mesh_info_dict)
    dom = update_non_obj_assets(dom, output_filepath)

    # Add the mujoco input elements
    dom = add_mujoco_inputs(dom, mujoco_inputs)

    # Add links as sites
    dom = add_links_as_sites(urdf, dom, request_add_free_joint)

    # Add cameras based on site names
    dom = add_cameras_from_sites(dom)

    # Write the updated file
    with open(destination_file, "w") as file:
        # Convert the DOM back to a string
        modified_data = dom.toprettyxml(indent="  ")
        modified_data = "\n".join([line for line in modified_data.splitlines() if line.strip()])

        # remove the first line bc mujoco doesn't like the xml tag at the beginning
        modified_lines = modified_data.splitlines(True)
        modified_lines.pop(0)
        modified_data = "".join(modified_lines)
        file.write(modified_data)


def parse_inputs_xml(filename=None):
    """
    Grabs a few additional xml settings from specified file. Supports the tags 'default',
    'option', 'actuator', and 'sensor'. These will be included in the final xml.

    E.g.,

    <mujoco_inputs>
        <option .../>

        <default>
            ...
        </default>

        <actuator>
            ...
        </actuator>

        <sensor>
            ...
        </sensor>
    </mujoco_inputs>
    """
    mujoco_inputs = dict()
    # {x : [] for x in XML_INPUT_TAGS}
    if not filename:
        return mujoco_inputs

    print(f"Parsing mujoco elements from {filename}")
    dom = minidom.parse(filename)
    root = dom.documentElement

    # We only parse the direct children of the root node, which should be called
    # "mujoco_defaults".
    if root.tagName != "mujoco_inputs":
        raise ValueError(f"Root tag in defaults xml must be 'mujoco_inputs', not {root.tagName}")

    for child in root.childNodes:
        if not child.nodeType == child.ELEMENT_NODE:
            continue

        if child.tagName not in mujoco_inputs:
            mujoco_inputs[child.tagName] = []
        mujoco_inputs[child.tagName].append(child)

    return mujoco_inputs


def get_urdf_from_rsp(args=None):
    # Isolate ROS in a function

    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters

    class ParameterClient(Node):
        def __init__(self, node_name="/robot_state_publisher/get_parameters"):
            super().__init__("parameter_client")
            self.node_name = node_name
            self.client = self.create_client(GetParameters, node_name)

        def get_params(self, params):
            print()
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")

            self.req = GetParameters.Request()
            self.req.names = params
            self.future = self.client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    rclpy.init(args=args)

    param_client = ParameterClient()
    response = param_client.get_params(["robot_description"])
    if response is not None:
        urdf = response.values[0].string_value
    else:
        param_client.get_logger().error("Failed to call service")
    param_client.destroy_node()
    rclpy.try_shutdown()

    return urdf


def get_xml_from_file(urdf_file=None):
    """
    Optionally parse a URDF from file. Can create a URDF from /robot_description with:

    ros2 topic echo  --full-length --once /robot_description  | \
        sed -e 's/^data: "//' -e 's/"$//' -e 's/\\n/\n/g' -e 's/\\\"/\"/g' -e 's/---//g' > \
        /tmp/robot_description.urdf
    """
    with open(urdf_file) as file:
        urdf = file.read()
    return urdf


def get_urdf_transforms(urdf_string):
    robot = URDF.from_xml_string(urdf_string)

    results = {}

    def make_transform_from_origin(origin):
        rpy = origin.rpy if origin else [0, 0, 0]
        xyz = origin.xyz if origin else [0, 0, 0]

        rot = PyKDL.Rotation.RPY(*rpy)
        trans = PyKDL.Vector(*xyz)
        return PyKDL.Frame(rot, trans)

    def get_parent_chain(link, transform=PyKDL.Frame.Identity()):
        # if the link is the root of the urdf, there is no parent!
        # we return true for third item to show it is root
        if link == robot.get_root():
            return (link, transform, True)
        parent_joint, _ = robot.parent_map[link]

        # get the transform from the next parent in line to the target link
        joint_obj = robot.joint_map[parent_joint]
        link_obj = robot.link_map[link]

        link_tf = make_transform_from_origin(link_obj.origin)

        # if it is fixed, we need to keep going.
        # get the transform for the link and the next joint in line
        if joint_obj.type == "fixed":
            joint_tf = make_transform_from_origin(joint_obj.origin)
            transform = joint_tf * link_tf * transform
            return get_parent_chain(joint_obj.parent, transform)
        else:
            # if it isn't fixed, we are done here!

            # actually, maybe we need this, not sure. I'm not sure if the
            # mujoco goes to the end of the joint
            # transform = link_tf*transform
            return (link, transform, False)

    for link in robot.links:
        results[link.name] = get_parent_chain(link.name)

    return results


def get_decompose_items(mujoco_inputs):
    decompose_dict = dict()
    if CONVERTER_INPUTS_NAME in mujoco_inputs:
        for mujoco_input in mujoco_inputs[CONVERTER_INPUTS_NAME]:
            for child in mujoco_input.childNodes:
                if child.nodeType == child.ELEMENT_NODE and child.tagName == "decompose_mesh":
                    name = child.getAttribute("mesh_name")
                    threshold = "0.05"
                    if not child.hasAttribute("threshold"):
                        print(f"defaulting threshold to 0.05 for decompose of {name}")
                    else:
                        threshold = child.getAttribute("threshold")
                    decompose_dict[name] = threshold
    return decompose_dict


def main(args=None):

    parser = argparse.ArgumentParser(description="Convert a full URDF to MJCF for use in Mujoco")
    parser.add_argument("-u", "--urdf", required=False, help="Optionally pass an existing URDF file")
    parser.add_argument(
        "-r", "--robot_description", required=False, help="Optionally pass the robot description string"
    )
    parser.add_argument(
        "-m",
        "--mujoco_inputs",
        required=False,
        help="Optionally specify a defaults xml for default settings, actuators, options, and additional sensors",
    )
    parser.add_argument("-o", "--output", default="mjcf_data", help="Generated output path")
    parser.add_argument("-c", "--convert_stl_to_obj", action="store_true", help="If we should convert .stls to .objs")
    parser.add_argument(
        "-f", "--add_free_joint", action="store_true", help="Adds a free joint as the base link for mobile robots"
    )

    # remove ros args to make argparser heppy
    args_without_filename = sys.argv[1:]
    while "--ros-args" in args_without_filename:
        args_without_filename.remove("--ros-args")

    parsed_args = parser.parse_args(args_without_filename)

    if parsed_args.urdf:
        urdf = get_xml_from_file(parsed_args.urdf)
    elif parsed_args.robot_description:
        urdf = parsed_args.robot_description
    else:
        urdf = get_urdf_from_rsp(args)

    request_add_free_joint = parsed_args.add_free_joint

    convert_stl_to_obj = parsed_args.convert_stl_to_obj

    mujoco_inputs = parse_inputs_xml(parsed_args.mujoco_inputs)

    # Grab the output directory and ensure it ends with '/'
    output_filepath = os.path.join(parsed_args.output, "")

    # Add required mujoco tags to the starting URDF
    xml_data = add_mujoco_info(urdf)

    # get rid of collision data, assuming the visual data is much better resolution.
    # not sure if this is the best move...
    xml_data = remove_tag(xml_data, "collision")

    xml_data = replace_package_names(xml_data)
    mesh_info_dict = extract_mesh_info(xml_data)
    decompose_dict = get_decompose_items(mujoco_inputs)
    xml_data = convert_to_objs(mesh_info_dict, output_filepath, xml_data, convert_stl_to_obj, decompose_dict)

    print("writing data to robot_description_formatted.urdf")
    robot_description_filename = "robot_description_formatted.urdf"
    with open(output_filepath + "robot_description_formatted.urdf", "w") as file:
        # Remove extra newlines that minidom adds after each tag
        xml_data = "\n".join([line for line in xml_data.splitlines() if line.strip()])
        file.write(xml_data)

    model = mujoco.MjModel.from_xml_path(f"{output_filepath}{robot_description_filename}")
    mujoco.mj_saveLastXML(f"{output_filepath}mujoco_description.xml", model)

    # Converts objs for use in mujoco, adds tags, inputs, sites, and sensors to the final xml
    fix_mujoco_description(output_filepath, mesh_info_dict, mujoco_inputs, urdf, decompose_dict, request_add_free_joint)

    shutil.copy2(f'{get_package_share_directory("mujoco_ros2_simulation")}/resources/scene.xml', output_filepath)


if __name__ == "__main__":
    main()
