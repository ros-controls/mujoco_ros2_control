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
import mujoco
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import PyKDL
import sys
import json
import math

import numpy as np
import trimesh  # Added trimesh

from urdf_parser_py.urdf import URDF

from ament_index_python.packages import get_package_share_directory
from xml.dom import minidom


# Hardcoded relative paths for mujoco asset outputs
DECOMPOSED_PATH_NAME = "decomposed"
COMPOSED_PATH_NAME = "full"


def add_mujoco_info(raw_xml, output_filepath, publish_topic):
    dom = minidom.parseString(raw_xml)

    mujoco_element = dom.createElement("mujoco")
    compiler_element = dom.createElement("compiler")

    # Use relative path for fixed directory otherwise use absolute path
    if not publish_topic:
        asset_dir = "assets"
    else:
        asset_dir = os.path.join(output_filepath, "assets")

    compiler_element.setAttribute("assetdir", asset_dir)
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


def extract_mesh_info(raw_xml, asset_dir, decompose_dict):
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

            # Select the mesh file: use a pre-generated OBJ if available and valid; otherwise use the original
            is_pre_generated = False
            new_uri = uri  # default fallback

            if asset_dir:
                if stem in decompose_dict:
                    # Decomposed mesh: check if a pre-generated OBJ exists and threshold matches
                    mesh_file = f"{asset_dir}/{DECOMPOSED_PATH_NAME}/{stem}/{stem}/{stem}.obj"
                    settings_file = f"{asset_dir}/{DECOMPOSED_PATH_NAME}/metadata.json"

                    if os.path.exists(mesh_file) and os.path.exists(settings_file):
                        try:
                            with open(settings_file) as f:
                                data = json.load(f)
                                used_threshold = float(data.get(f"{stem}"))
                        except (FileNotFoundError, PermissionError, json.JSONDecodeError) as e:
                            print(f"Warning: could not read thresholds for {stem}: {e}")
                            used_threshold = None
                        # Use existing decomposed object only if it has the same threshold, otherwise regenerate it.
                        if used_threshold is not None and math.isclose(
                            used_threshold, float(decompose_dict[stem]), rel_tol=1e-9
                        ):
                            new_uri = mesh_file
                            is_pre_generated = True
                        else:
                            print(
                                f"Existing decomposed obj for {stem} has different threshold {used_threshold} "
                                f"than required {decompose_dict[stem]}. Regenerating..."
                            )
                else:
                    # Composed mesh: check if a pre-generated OBJ exists
                    mesh_file = f"{asset_dir}/{COMPOSED_PATH_NAME}/{stem}/{stem}.obj"

                    if os.path.exists(mesh_file):
                        new_uri = mesh_file
                        is_pre_generated = True

            scale = " ".join(f"{v}" for v in geom.scale) if geom.scale else "1.0 1.0 1.0"
            rgba = resolve_color(vis)

            # If the same stem appears more than once, keep the first, or change as you prefer
            mesh_info_dict.setdefault(
                stem,
                {
                    "is_pre_generated": is_pre_generated,
                    "filename": new_uri,
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


# get required images from dae so that we can copy them to the temporary filepath
def get_images_from_dae(dae_path):

    dae_dir = os.path.dirname(dae_path)

    doc = minidom.parse(dae_path)

    image_paths = []
    seen = set()

    # access data from dae files with this structure to access image_filepath
    # <library_images>
    #     <image id="id" name="name">
    #         <init_from>image_filepath</init_from>
    #     </image>
    # </library_images>
    for image in doc.getElementsByTagName("image"):
        init_from_elems = image.getElementsByTagName("init_from")
        if not init_from_elems:
            continue

        path = init_from_elems[0].firstChild
        if path is None:
            continue

        image_path = path.nodeValue.strip()

        # Resolve relative paths
        if not os.path.isabs(image_path):
            image_path = os.path.normpath(os.path.join(dae_dir, image_path))

        # make sure it is an image
        if image_path.lower().endswith((".png", ".jpg", ".jpeg")):
            # ignore duplucates
            if image_path not in seen:
                seen.add(image_path)
                image_paths.append(image_path)

    return image_paths


# Change all files that match "material_{some_int}.{png, jpg, jpeg}"
# in a specified directory to be "material_{modifier}_{some_int}.{png, jpg, jpeg}"
# This is important because trimesh puts out materials that look like
# material_{some_int}.{png, jpg, jpeg}, and they need to be indexed per item
def rename_material_textures(dir_path, modifier):
    dir_path = pathlib.Path(dir_path)

    # regex to match files we want to modify
    pattern = re.compile(r"^material_(\d+)\.(png|jpg|jpeg)$", re.IGNORECASE)

    for path in dir_path.iterdir():
        if not path.is_file():
            continue

        m = pattern.match(path.name)
        if not m:
            continue

        # extract important components and reorder
        index, ext = m.groups()
        new_name = f"material_{modifier}_{index}.{ext}"
        new_path = path.with_name(new_name)

        print(f"{path.name} -> {new_name}")
        path.rename(new_path)


def extract_rgba(visual):
    """
    Extracts the rgba values for a trimesh object. This apparently is pretty tricky because the materials can
    come in a number of forms. This is basically chat gpt's best guess of how to do this, but seems to work
    for my cases.

    :param visual: visual component of a trimesh scene
    """
    # 1) Material base color
    mat = getattr(visual, "material", None)
    if mat is not None:
        # PBR
        if isinstance(mat, trimesh.visual.material.PBRMaterial) and mat.baseColorFactor is not None:
            return np.array(mat.baseColorFactor)

        # Simple / Phong
        if hasattr(mat, "diffuse") and mat.diffuse is not None:
            rgb = np.array(mat.diffuse)
            alpha = getattr(mat, "alpha", 1.0)
            return np.concatenate([rgb, [alpha]])

    # 2) Per-vertex colors
    if visual.kind == "vertex" and visual.vertex_colors is not None:
        vc = visual.vertex_colors
        if vc.shape[1] == 4:
            return vc[0] / 255.0
        else:
            return np.append(vc[0] / 255.0, 1.0)

    # 3) I don't think this should ever happen, but just in case
    return np.array([0.7, 0.7, 0.7, 1.0])


def convert_to_objs(mesh_info_dict, directory, xml_data, convert_stl_to_obj, decompose_dict):
    # keep track of files we have already processed so we don't do it again
    converted_filenames = []
    # keep track of what material number we are on to get unique materials
    mtl_num = 0

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

        # if we want to decompose the mesh, put it in decomposed filepath, otherwise put it in full
        if filename_no_ext in decompose_dict:
            assets_relative_filepath = f"{DECOMPOSED_PATH_NAME}/{filename_no_ext}/"
            os.makedirs(f"{directory}assets/{assets_relative_filepath}", exist_ok=True)
        else:
            assets_relative_filepath = f"{COMPOSED_PATH_NAME}/"
        assets_relative_filepath += filename_no_ext

        output_path = f"{directory}assets/{assets_relative_filepath}.obj"

        # Import the .stl or .dae file
        if filename_ext.lower() == ".stl":
            if filename_no_ext in decompose_dict and not convert_stl_to_obj:
                raise ValueError("The --convert_stl_to_obj argument must be specified to decompose .stl mesh")
            if convert_stl_to_obj:
                # Load mesh using trimesh
                mesh = trimesh.load(full_filepath)

                # bring in file color from urdf
                if "color" in mesh_item:
                    # trimesh expects 0-255 uint8 for colors
                    rgba = mesh_item["color"]
                    # make a material for the rgba values and export it
                    mtl_modifier = f"m{mtl_num}"
                    mtl_name = "mtl_" + mtl_modifier
                    material = trimesh.visual.material.SimpleMaterial(
                        name=mtl_name, diffuse=rgba, glossiness=1000, specular=[0.2, 0.2, 0.2]
                    )  # RGBA
                    mesh.visual = trimesh.visual.TextureVisuals(material=material)

                    # increment material number
                    mtl_num = mtl_num + 1

                # Export to OBJ
                mesh.export(output_path, include_color=True, mtl_name=mtl_name)
                xml_data = xml_data.replace(full_filepath, f"{assets_relative_filepath}.obj")

            else:
                shutil.copy2(full_filepath, f"{directory}assets/{assets_relative_filepath}.stl")
                xml_data = xml_data.replace(full_filepath, f"{assets_relative_filepath}.stl")
                pass
        elif filename_ext.lower() == ".obj":
            # If import .obj files from URDF
            if not mesh_item["is_pre_generated"]:
                shutil.copy2(full_filepath, f"{directory}assets/{assets_relative_filepath}.obj")
                # If the .obj depends on a mtl should copy also this
                old_directory = os.path.dirname(mesh_item["filename"])
                if os.path.exists(old_directory + "/material.mtl"):
                    final_path = os.path.dirname(assets_relative_filepath)
                    shutil.copy2(old_directory + "/material.mtl", f"{directory}assets/{final_path}/material.mtl")
            pass
            # objs are ok as is
        elif filename_ext.lower() == ".dae":
            # keep track of the image files that we need to copy from the dae
            image_files = get_images_from_dae(full_filepath)
            # keep track of the files that were copied to tmp directory to delete
            copied_image_files = []

            # set z axis to up in the dae file because that is how mujoco expects it
            z_up_dae_txt = set_up_axis_to_z_up(full_filepath)

            # make a temporary file rather than overwriting the old one
            temp_file = tempfile.NamedTemporaryFile(suffix=".dae", mode="w+", delete=False)
            temp_filepath = temp_file.name
            temp_folder = os.path.dirname(temp_filepath)
            try:
                temp_file.write(z_up_dae_txt)
                temp_file.close()

                # copy relevant images into the temporary directory
                for image_file in image_files:
                    shutil.copy2(image_file, temp_folder)
                    copied_image_files.append(f"{temp_folder}/{os.path.basename(image_file)}")

                # Load scene using trimesh
                scene = trimesh.load(temp_filepath, force=trimesh.scene)

                # The default glossiness was way too high, so here we iterate over each visual
                # element, assign default glossiness and specular but keep the rgba values
                for geom in scene.geometry.values():
                    visual = geom.visual
                    rgba = extract_rgba(visual)

                    new_mat = trimesh.visual.material.SimpleMaterial(
                        diffuse=rgba[:3],
                        alpha=rgba[3],
                        glossiness=1000,
                        specular=[0.2, 0.2, 0.2],
                    )

                    visual.material = new_mat

                # give the material a unique name so that it can be properly referenced
                mtl_modifier = f"m{mtl_num}"
                mtl_name = "mtl_" + mtl_modifier
                mtl_filepath = os.path.dirname(output_path) + f"/{mtl_name}"

                scene.export(output_path, include_color=True, mtl_name=mtl_name)
                # rename textures
                rename_material_textures(dir_path=os.path.dirname(output_path), modifier=mtl_modifier)

                # we need to modify the material names to not all be material_X so they don't conflict
                # all of the objs will have a line that looks like
                #   usemtl material_0
                # and all of the mtl files will have a line that looks like
                #   newmtl material_0
                # We will modify both of them to be numberd by mtl_num like so
                #   usemtl material_{mtl_num}_0
                #   newmtl material_{mtl_num}_0

                if os.path.exists(mtl_filepath):
                    for filepath in [mtl_filepath, output_path]:
                        with open(filepath) as f:
                            data = f.read()
                        data = data.replace("material_", f"material_{mtl_modifier}_")
                        with open(filepath, "w") as f:
                            f.write(data)
                # increment
                mtl_num = mtl_num + 1
            finally:
                if os.path.exists(temp_filepath):
                    os.remove(temp_filepath)

            # get rid of copied image files in the temporary file directory
            for copied_image_file in copied_image_files:
                if os.path.exists(copied_image_file):
                    os.remove(copied_image_file)

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

    thresholds_file = os.path.join(f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}", "metadata.json")

    if os.path.exists(thresholds_file):
        with open(thresholds_file) as f:
            thresholds_data = json.load(f)
    else:
        thresholds_data = {}

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

        thresholds_data[mesh_name] = float(threshold)

    with open(thresholds_file, "w") as f:
        json.dump(thresholds_data, f, indent=4)


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

            # bring in the textures, and modify filepath to properly reference filepaths
            sub_textures = sub_asset_element.getElementsByTagName("texture")
            for sub_texture in sub_textures:
                if sub_texture.hasAttribute("file"):
                    sub_texture_file = sub_texture.getAttribute("file")
                    if composed_type == DECOMPOSED_PATH_NAME:
                        sub_texture.setAttribute("file", f"{composed_type}/{mesh_name}/{mesh_name}/{sub_texture_file}")
                    else:
                        sub_texture.setAttribute("file", f"{composed_type}/{mesh_name}/{sub_texture_file}")
                    asset_element.appendChild(sub_texture)

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
    """
    We want to take the group 1 objects that get created, and turn them into the equivalent
    but both in group 2 and in group 3. That means taking something like this
        <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="finger_v6"/>
    and turning it into this
        <geom mesh="finger_v6" class="visual" pos="0 0 0" quat="0.707107 0.707107 0 0"/>
        <geom mesh="finger_v6" class="collision" pos="0 0 0" quat="0.707107 0.707107 0 0"/>

    To do this, we need to add in class visual, and class collision to them, keep the rgba on the visual one, and
    get rid of the other components (type, contype, conaffinity, group, density)

    We can tell that we need to modify it because it will have a contype attribute attached to it (not the best way
    but I guess it works for now)
    """

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


def add_mujoco_inputs(dom, raw_inputs, scene_inputs):
    """
    Copies all elements under the "raw_inputs" and "scene_inputs" XML tags directly in the provided dom.
    This is useful for adding things like actuators, options, or defaults or scene-specific elements. But any tag that
    is supported in the MJCF can be added here.
    """
    root = dom.documentElement

    if scene_inputs:
        for child in scene_inputs.childNodes:
            if child.nodeType == child.ELEMENT_NODE:
                imported_node = dom.importNode(child, True)
                root.appendChild(imported_node)

    if raw_inputs:
        for child in raw_inputs.childNodes:
            if child.nodeType == child.ELEMENT_NODE:
                imported_node = dom.importNode(child, True)
                root.appendChild(imported_node)

    return dom


def get_processed_mujoco_inputs(processed_inputs_element):
    """
    Returns the processed inputs as dictionaries from the specified processed_inputs_element.

    Right now this supports tags for decomposing meshes and attaching cameras or lidar sensors to sites.
    """

    decompose_dict = dict()
    cameras_dict = dict()
    modify_element_dict = dict()
    lidar_dict = dict()

    if not processed_inputs_element:
        return decompose_dict, cameras_dict, modify_element_dict, lidar_dict

    for child in processed_inputs_element.childNodes:
        if child.nodeType != child.ELEMENT_NODE:
            continue

        # Grab meshes to decompose
        if child.tagName == "decompose_mesh":
            name = child.getAttribute("mesh_name")
            threshold = "0.05"
            if not child.hasAttribute("threshold"):
                print(f"defaulting threshold to 0.05 for decompose of {name}")
            else:
                threshold = child.getAttribute("threshold")
            print(f"Will decompose mesh with name: {name}")
            decompose_dict[name] = threshold

        # Grab cameras
        if child.nodeType == child.ELEMENT_NODE and child.tagName == "camera":
            camera_element = child
            site_name = camera_element.getAttribute("site")
            camera_name = camera_element.getAttribute("name")

            # We don't need this in the MJCF
            camera_element.removeAttribute("site")
            cameras_dict[site_name] = camera_element

            print(f"Will add camera ({camera_name}) for site ({site_name})")

        # Grab replicates
        if child.nodeType == child.ELEMENT_NODE and child.tagName == "lidar":
            lidar_element = child
            site_name = lidar_element.getAttribute("ref_site")
            sensor_name = lidar_element.getAttribute("sensor_name")
            min_angle = float(lidar_element.getAttribute("min_angle"))
            max_angle = float(lidar_element.getAttribute("max_angle"))
            angle_increment = float(lidar_element.getAttribute("angle_increment"))

            num_sensors = int((max_angle - min_angle) / angle_increment) + 1

            doc = minidom.Document()
            site = doc.createElement("site")
            site.setAttribute("name", sensor_name)
            site.setAttribute("pos", "0.0 0.0 0.0")
            site.setAttribute("quat", "0.0 0.0 0.0 1.0")

            replicate = doc.createElement("replicate")
            replicate.setAttribute("site", site_name)
            replicate.setAttribute("count", str(num_sensors))
            replicate.setAttribute("sep", "-")
            replicate.setAttribute("offset", "0 0 0")
            replicate.setAttribute("euler", f"0 {angle_increment} 0")
            replicate.setAttribute("min_angle", lidar_element.getAttribute("min_angle"))

            replicate.appendChild(site)

            # We don't need this in the MJCF
            lidar_dict[site_name] = replicate

            print(f"Will add replicate tag at site ({site_name})")

        # Grab modify element information
        if child.nodeType == child.ELEMENT_NODE and child.tagName == "modify_element":
            modify_element_element = child

            # get the attributes from the modify_element_tag
            attr_dict = {attr.name: attr.value for attr in modify_element_element.attributes.values()}
            # we must have a name element
            if "name" not in attr_dict or "type" not in attr_dict:
                raise ValueError("'name' and 'type' must be in the attributes of a 'modify_element' tag!")

            # remove the name and type entries because those will be used as the key in the returned dict
            element_name = attr_dict["name"]
            element_type = attr_dict["type"]
            del attr_dict["name"]
            del attr_dict["type"]
            modify_element_dict[(element_type, element_name)] = attr_dict

            print(f"Will add the following attributes to {element_type} '{element_name}':")
            for key, value in attr_dict.items():
                print(f"  {key}: {value}")

    return decompose_dict, cameras_dict, modify_element_dict, lidar_dict


def parse_inputs_xml(filename=None):
    """
    This script can accept inputs in the form of an xml file. This allows users to inject data
    into the MJCF that is not necessarily included in the URDF.

    E.g.,

    <mujoco_inputs>

        <!-- The contents of `raw_inputs` will be copied and pasted directly into the MJCF -->
        <raw_inputs>
            <option integrator="implicitfast"/>

            <default>
                ...
            </default>

            <actuator>
                ...
            </actuator>
        </raw_inputs>

        <!-- Specific inputs that require processing from the conversion script-->
        <processed_inputs>
            <!-- Specifying decompose_mesh will set the `decompose` flag when using obj2mjcf -->
            <!-- <decompose_mesh mesh_name="shoulder_link" threshold="0.05"/> -->

            <!-- The camera with the specified values will be added at the specified site name. -->
            <!-- The position and quaterinion will be filled in by the converter -->
            <camera site="camera_color_optical_frame" name="camera" fovy="58" mode="fixed" resolution="640 480"/>

            <!-- Adds a lidar tag below a site tag to support a lidar sensor. -->
            <!-- In the URDF, we assume that that the sensor frame has the Z-axis pointed directly up in the sensor -->
            <!-- frame. This tag will create rangefinders in the MJCF between min_angle and max_angle at each -->
            <!-- angle_increment, and the drivers combine them into a single LaserScan message. -->
            <lidar ref_site="lidar_sensor_frame"
                   sensor_name="rf"
                   min_angle="0"
                   max_angle="1.57"
                   angle_increment="0.025"
            />

        </processed_inputs>
    </mujoco_inputs>
    """

    if not filename:
        return None, None

    print(f"Parsing mujoco elements from: {filename}")

    dom = minidom.parse(filename)
    root = dom.documentElement

    raw_inputs = None
    processed_inputs = None

    if root.tagName == "mujoco_inputs":
        # The file itself is a standalone xml
        for child in root.childNodes:
            if child.nodeType != child.ELEMENT_NODE:
                continue
            if child.tagName == "raw_inputs":
                raw_inputs = child
            elif child.tagName == "processed_inputs":
                processed_inputs = child

    elif root.tagName == "robot":
        # The file is a URDF
        mujoco_inputs_node = None

        # find <mujoco_inputs>
        for child in root.childNodes:
            if child.nodeType == child.ELEMENT_NODE and child.tagName == "mujoco_inputs":
                mujoco_inputs_node = child
                break

        if mujoco_inputs_node is None:
            # URDF without mujoco_inputs is allowed
            return None, None

        # parse children of <mujoco_inputs>
        for child in mujoco_inputs_node.childNodes:
            if child.nodeType != child.ELEMENT_NODE:
                continue
            if child.tagName == "raw_inputs":
                raw_inputs = child
            elif child.tagName == "processed_inputs":
                processed_inputs = child
    else:
        raise ValueError(
            f"Root tag in file must be either 'mujoco_inputs' (standalone XML) or 'robot' (URDF), not '{root.tagName}'"
        )

    return raw_inputs, processed_inputs


def parse_scene_xml(filename=None):
    """
    This script can accept the scene in the form of an xml file. This allows users to inject this data
    into the MJCF that is not necessarily included in the URDF.
    """

    if not filename:
        return None

    print(f"Parsing mujoco scene from: {filename}")

    dom = minidom.parse(filename)
    root = dom.documentElement

    scene_inputs = None

    if root.tagName == "mujoco":
        # The file itself is a standalone xml
        scene_inputs = root
        return scene_inputs

    elif root.tagName == "robot":
        # The file is a URDF
        mujoco_inputs_node = None

        # find <mujoco_inputs>
        for child in root.childNodes:
            if child.nodeType == child.ELEMENT_NODE and child.tagName == "mujoco_inputs":
                mujoco_inputs_node = child
                break

        if mujoco_inputs_node is None:
            # URDF without mujoco_inputs is allowed
            return None

        # parse children of <mujoco_inputs>
        for child in mujoco_inputs_node.childNodes:
            if child.nodeType != child.ELEMENT_NODE:
                continue
            if child.tagName == "scene":
                scene_inputs = child
    else:
        raise ValueError(
            f"Root tag in file must be either 'mujoco_inputs' (standalone XML) or 'robot' (URDF), not '{root.tagName}'"
        )

    return scene_inputs


def add_free_joint(dom, urdf, joint_name="floating_base_joint"):
    """
    This change is on the mjcf side, and replaces the "free" joint type with a freejoint tag.
    This is a special item which explicitly sets all stiffness/damping to 0.
    https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-freejoint
    """
    robot = URDF.from_xml_string(urdf)
    root_link = robot.get_root()
    if root_link == "world":
        print("Not adding a free joint because world is the URDF root")
        return

    # Find all joint elements
    joints = dom.getElementsByTagName("joint")

    succesfully_fixed = False

    # Locate the one with name="virtual_base_joint" of type="free"
    for joint in joints:
        if joint.getAttribute("name") == "virtual_base_joint" and joint.getAttribute("type") == "free":
            # Create the new freejoint element
            new_joint = dom.createElement("freejoint")
            new_joint.setAttribute("name", joint_name)

            # Replace the old joint with the new one
            joint.parentNode.replaceChild(new_joint, joint)
            succesfully_fixed = True
            break

    if not succesfully_fixed:
        raise ValueError("Did not find a joint of name virtual_base_joint and type free. What did you just do????")

    return dom


def add_links_as_sites(urdf, dom, add_free_joint):
    """
    Add all links from the URDF as sites in the MJCF. This is handy as all rigid bodies are
    mashed into a single named object in the MJCF, so we lose site names (like camera links, for instance).
    To make it easier to connect sensors, etc, we simply add all the removed links as named
    sites in the MJFC.
    """

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


def multiply_quaternion(q1, q2):
    """
    Returns q1 * q2.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return [
        -x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2,
        x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2,
        -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2,
        x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2,
    ]


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert an Euler RPY angles to a quaternion, [w, x, y, z]
    """
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)

    return [qw, qx, qy, qz]


def add_cameras_from_sites(dom, cameras_dict):
    """
    Adds cameras to the sites listed in the cameras_dict. We make the assumption that any
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
    """

    x_rotation = [0.0, 1.0, 0.0, 0.0]  # Rotation by pi around x axis

    # Construct all cameras for relevant sites in xml and add them as children to the same parent
    for node in dom.getElementsByTagName("site"):
        site_name = node.getAttribute("name")
        if site_name in cameras_dict:
            camera_pos = node.getAttribute("pos")
            quat = [float(x) for x in node.getAttribute("quat").split()]
            camera_quat = multiply_quaternion(quat, x_rotation)

            camera = cameras_dict[site_name]
            camera.setAttribute("pos", camera_pos)
            camera.setAttribute("quat", " ".join(map(str, camera_quat)))

            print(f"Adding camera to {site_name}, attributes:")
            for i in range(camera.attributes.length):
                attr = camera.attributes.item(i)
                print(f"  {attr.name}: {attr.value}")

            node.parentNode.appendChild(camera)

    return dom


def add_lidar_from_sites(dom, lidar_dict):
    """
    Creates a replicates tag from mujoco inputs below the specified site name and lidar tag.

    Replicates must be under to a body, so we add a massless body with an identical transform to support
    attaching the sensor's replicates.

    We assume that the site in the URDF has the Z-axis pointed up, whereas the rangefinder's sensor has the
    Z-axis pointed along the sensor and rotate about the Y-axis. For the sake of this conversion, we assume
    that the first replicate's Z-axis is offset from the URDF's X-axis by `min_angle`. So we rotate the
    position from the matched site in the URDF accordingly.

    If you draw this out, the XYZ euler transform from one to the other should be:
    [pi/2, pi/2, 0] * [0, min_angle, 0].
    """

    x_form = [0.5, 0.5, 0.5, 0.5]  # pi/2 around x, pi/2 about y

    # Construct all lidar sensor bodies for relevant sites in xml and add them as children to the same parent
    for node in dom.getElementsByTagName("site"):
        site_name = node.getAttribute("name")
        if site_name in lidar_dict:
            replicate = lidar_dict[site_name]

            # Handle conversion of the frames by applying the site transform, rangefinder transform, then
            # min_angle transform (rotation about Y)
            site_quat = [float(x) for x in node.getAttribute("quat").split()]
            min_angle = float(replicate.getAttribute("min_angle"))
            min_angle_quat = euler_to_quaternion(0, min_angle, 0)
            tmp_quat = multiply_quaternion(site_quat, x_form)
            lidar_quat = multiply_quaternion(tmp_quat, min_angle_quat)

            # Create the new body element and add the replicate as a child
            new_body = dom.createElement("body")
            new_body.setAttribute("name", site_name + "_lidar_body")
            new_body.setAttribute("pos", node.getAttribute("pos"))
            new_body.setAttribute("quat", " ".join(map(str, lidar_quat)))

            # No longer need the tag
            replicate.removeAttribute("min_angle")
            new_body.appendChild(replicate)

            print(f"Adding replicates to {site_name}, attributes:")
            print("    pos: ", new_body.getAttribute("pos"))
            print("    quat: ", new_body.getAttribute("quat"))
            for i in range(replicate.attributes.length):
                attr = replicate.attributes.item(i)
                print(f"  {attr.name}: {attr.value}")

            node.parentNode.appendChild(new_body)

    return dom


def add_modifiers(dom, modify_element_dict):
    """
    Modify elements that are a part of the worldbody tag by adding attributes.
    These attributes are defined as a part of the modify_element_dict and are stored with the key as the name of the
    element to be modified, and the value being another dictionary mapping attribute names to attribute values.

    This method will leave the attributes that were already a part of the element in the mjcf file, and append the
    new values, but will overwrite the old values with newly provided ones if there are conflicts.

    The modify_element_dict looks like
    key (tuple): (element_type (str), element_name (str))
    value (dict):
        key[0]: attribute_name (str)
        value[0]: attribute_value (str)
        key[1]: attribute_name (str)
        value[1]: attribute_value (str)
        ...
    """

    # Get the joint elements underneath the <worldbody> element
    worldbody = dom.getElementsByTagName("worldbody")
    worldbody_element = worldbody[0]

    # Figure out what element types we need to look at
    types = []
    for key in modify_element_dict:
        if key[0] not in types:
            types.append(key[0])

    # work on each set of element types at a time
    for element_type in types:
        element_set = worldbody_element.getElementsByTagName(element_type)
        # check if the each element needs modification
        for element in element_set:
            potential_key = (element.tagName, element.getAttribute("name"))
            if potential_key in modify_element_dict:
                # apply attributes to the elements
                attr_dict = modify_element_dict[potential_key]
                for attr_name, attr_value in attr_dict.items():
                    element.setAttribute(attr_name, attr_value)

    return dom


def copy_pre_generated_meshes(output_filepath, mesh_info_dict, decompose_dict):
    """
    Copies pre-generated mesh folders into the final MJCF assets structure.
    """

    for mesh_name in mesh_info_dict:
        mesh_item = mesh_info_dict[mesh_name]
        filename = os.path.basename(mesh_item["filename"])
        filename_no_ext = os.path.splitext(filename)[0]
        full_path = mesh_item["filename"]
        mesh_dir = os.path.dirname(os.path.splitext(full_path)[0])

        if mesh_item["is_pre_generated"]:
            if filename_no_ext in decompose_dict:
                dst_base = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}/{filename_no_ext}/{filename_no_ext}/"
            else:
                dst_base = f"{output_filepath}assets/{COMPOSED_PATH_NAME}/{filename_no_ext}"

            shutil.copytree(mesh_dir, dst_base, dirs_exist_ok=True)


def fix_mujoco_description(
    output_filepath,
    mesh_info_dict,
    raw_inputs,
    scene_inputs,
    urdf,
    decompose_dict,
    cameras_dict,
    modify_element_dict,
    lidar_dict,
    request_add_free_joint,
):
    """
    Handles all necessary post processing from the originally converted MJCF.
    """
    full_filepath = f"{output_filepath}mujoco_description.xml"
    filename, extension = os.path.splitext(f"{output_filepath}mujoco_description.xml")
    destination_file = filename + "_formatted" + extension
    # shutil.copy2(full_filepath, destination_file)

    # Run conversions for mjcf
    run_obj2mjcf(output_filepath, decompose_dict)

    # Copy pre-geerated mesh folders to the final directory
    copy_pre_generated_meshes(output_filepath, mesh_info_dict, decompose_dict)

    # Parse the DAE file
    dom = minidom.parse(full_filepath)

    if request_add_free_joint:
        dom = add_free_joint(dom, urdf)

    # Update and add the new fixed assets
    dom = update_obj_assets(dom, output_filepath, mesh_info_dict)
    dom = update_non_obj_assets(dom, output_filepath)

    # Add the mujoco input elements
    dom = add_mujoco_inputs(dom, raw_inputs, scene_inputs)

    # Add links as sites
    dom = add_links_as_sites(urdf, dom, request_add_free_joint)

    # Add cameras based on site names
    dom = add_cameras_from_sites(dom, cameras_dict)

    # Add replicates based on site names
    dom = add_lidar_from_sites(dom, lidar_dict)

    # modify elements based on modify_element tags
    dom = add_modifiers(dom, modify_element_dict)

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


def get_urdf_from_rsp(args=None):
    """
    Pulls the robot description from the /robot_description topic, if available.
    """

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


def publish_model_on_topic(publish_topic, output_filepath, args=None):

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

    # Remove leading slash if present
    publish_topic = publish_topic.lstrip("/")

    # --- Node ROS2 for model publishing MJCF ---
    class MjcfPublisher(Node):
        def __init__(self, mjcf_path):
            super().__init__("mjcf_publisher")

            qos_profile = QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )

            self.publisher_ = self.create_publisher(String, publish_topic, qos_profile)
            self.mjcf_path = mjcf_path
            self.publish_mjcf()

        def publish_mjcf(self):
            with open(self.mjcf_path) as f:
                xml_content = f.read()
            msg = String()
            msg.data = xml_content
            self.publisher_.publish(msg)

    rclpy.init(args=args)
    mjcf_path = os.path.join(output_filepath, "mujoco_description_formatted.xml")
    mjcf_node = MjcfPublisher(mjcf_path)

    try:
        rclpy.spin(mjcf_node)
    except KeyboardInterrupt:
        pass
    finally:
        mjcf_node.destroy_node()
        rclpy.try_shutdown()


def add_urdf_free_joint(urdf):
    """
    Adds a free joint to the top of the urdf. This makes Mujoco create a
    floating joint so that a base is free to move, like on an AMR.
    """

    # get the old root link
    robot = URDF.from_xml_string(urdf)
    old_root = robot.get_root()

    if old_root == "world":
        print("Not adding a free joint because world is the URDF root")
        return

    urdf_dom = minidom.parseString(urdf)

    # Get the <robot> root element
    robot_elem = urdf_dom.getElementsByTagName("robot")[0]

    ###################################
    # virtual base link
    virtual_link = urdf_dom.createElement("link")
    virtual_link.setAttribute("name", "virtual_base")

    ###################################
    # joint of virtual base link to dummy link
    virtual_joint = urdf_dom.createElement("joint")
    virtual_joint.setAttribute("name", "virtual_base_joint")
    virtual_joint.setAttribute("type", "floating")

    # <parent link="virtual_base"/>
    parent_elem = urdf_dom.createElement("parent")
    parent_elem.setAttribute("link", "virtual_base")
    virtual_joint.appendChild(parent_elem)

    # <child link="old_root"/>
    child_elem = urdf_dom.createElement("child")
    # replace with your real root link name
    child_elem.setAttribute("link", old_root)
    virtual_joint.appendChild(child_elem)

    # <origin xyz="0 0 0" rpy="0 0 0"/>
    origin_elem = urdf_dom.createElement("origin")
    origin_elem.setAttribute("xyz", "0 0 0")
    origin_elem.setAttribute("rpy", "0 0 0")
    virtual_joint.appendChild(origin_elem)

    # Insert the elements at the top of the robot definition
    robot_elem.insertBefore(virtual_joint, robot_elem.firstChild)
    robot_elem.insertBefore(virtual_link, robot_elem.firstChild)

    # Use minidom to format the string with line breaks and indentation
    formatted_xml = urdf_dom.toprettyxml(indent="    ")

    # Remove extra newlines that minidom adds after each tag
    formatted_xml = "\n".join([line for line in formatted_xml.splitlines() if line.strip()])

    return formatted_xml


def write_mujoco_scene(scene_inputs, output_filepath):
    from xml.dom.minidom import Document, Node

    dom = Document()

    root = dom.createElement("mujoco")
    root.setAttribute("model", "scene")
    dom.appendChild(root)

    # Add an <include> tag for the mujoco description
    include_node = dom.createElement("include")
    include_node.setAttribute("file", "mujoco_description_formatted.xml")
    root.appendChild(include_node)

    if scene_inputs:
        scene_node = None
        if scene_inputs.tagName == "scene":
            scene_node = scene_inputs
        else:
            for child in scene_inputs.childNodes:
                if child.nodeType == Node.ELEMENT_NODE and child.tagName == "scene":
                    scene_node = child
                    break

        # If a <scene> node was found, import all of its child nodes
        if scene_node:
            for child in scene_node.childNodes:
                if child.nodeType == Node.TEXT_NODE and not child.data.strip():
                    continue
                imported_node = dom.importNode(child, True)  # deep copy
                root.appendChild(imported_node)

    with open(output_filepath + "scene.xml", "w") as file:
        file.write(dom.toprettyxml(indent="    "))


def main(args=None):

    parser = argparse.ArgumentParser(description="Convert a full URDF to MJCF for use in Mujoco")
    parser.add_argument("-u", "--urdf", required=False, default=None, help="Optionally pass an existing URDF file")
    parser.add_argument(
        "-r", "--robot_description", required=False, help="Optionally pass the robot description string"
    )
    parser.add_argument(
        "-m",
        "--mujoco_inputs",
        required=False,
        default=None,
        help="Optionally specify a defaults xml for default settings, actuators, options, and additional sensors",
    )
    parser.add_argument("-o", "--output", default="mjcf_data", help="Generated output path")
    parser.add_argument(
        "-p",
        "--publish_topic",
        required=False,
        default=None,
        help="Optionally specify the topic to publish the MuJoCo model",
    )
    parser.add_argument("-c", "--convert_stl_to_obj", action="store_true", help="If we should convert .stls to .objs")
    parser.add_argument(
        "-s",
        "--save_only",
        action="store_true",
        help="Save files permanently on disk; without this flag, files go to a temporary directory",
    )
    parser.add_argument(
        "-f",
        "--add_free_joint",
        action="store_true",
        help="Adds a free joint before the root link of the robot in the urdf before conversion",
    )
    parser.add_argument(
        "-a",
        "--asset_dir",
        required=False,
        default=None,
        help="Optionally pass an existing folder with pre-generated OBJ meshes.",
    )
    parser.add_argument("--scene", required=False, default=None, help="Optionally pass an existing xml for the scene")

    # remove ros args to make argparser happy
    args_without_filename = sys.argv[1:]
    while "--ros-args" in args_without_filename:
        args_without_filename.remove("--ros-args")

    parsed_args = parser.parse_args(args_without_filename)

    # Load URDF from file, string, or topic
    urdf_path = None
    if parsed_args.urdf:
        urdf = get_xml_from_file(parsed_args.urdf)
        urdf_path = parsed_args.urdf
    else:
        if parsed_args.robot_description:
            urdf = parsed_args.robot_description
        else:
            urdf = get_urdf_from_rsp(args)
        # Create a tempfile and store the URDF
        tmp = tempfile.NamedTemporaryFile()
        with open(tmp.name, "w") as f:
            f.write(urdf)
        urdf_path = tmp.name

    request_add_free_joint = parsed_args.add_free_joint

    convert_stl_to_obj = parsed_args.convert_stl_to_obj

    output_filepath = parsed_args.output
    if not os.path.isabs(parsed_args.output) and parsed_args.publish_topic:
        output_filepath = os.path.join(os.getcwd(), parsed_args.output)
    # Determine the path of the output directory
    if parsed_args.save_only:
        # Grab the output directory and ensure it ends with '/'
        output_filepath = os.path.join(output_filepath, "")
    elif parsed_args.publish_topic:
        temp_dir = tempfile.TemporaryDirectory()
        output_filepath = os.path.join(temp_dir.name, "")
    else:
        raise ValueError("You must specify at least one of the following options: " "--publish_topic or --save_only.")

    # Check if outputpath exists; if not, create it
    if not os.path.exists(output_filepath):
        os.makedirs(output_filepath, exist_ok=True)

    # Use provided MuJoCo input or scene XML files if given; otherwise use the URDF.
    mujoco_inputs_file = parsed_args.mujoco_inputs or urdf_path
    mujoco_scene_file = parsed_args.scene or urdf_path

    raw_inputs, processed_inputs = parse_inputs_xml(mujoco_inputs_file)

    scene_inputs = None
    if parsed_args.publish_topic or (parsed_args.save_only and not parsed_args.scene):
        scene_inputs = parse_scene_xml(mujoco_scene_file)

    # Copy the scene tags from URDF to a separate xml il not publishing
    if not parsed_args.publish_topic and parsed_args.save_only and scene_inputs:
        print("Copying scene tags from URDF to a separate xml")
        write_mujoco_scene(scene_inputs, output_filepath)
        scene_inputs = None

    decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_inputs)

    if parsed_args.asset_dir:
        assets_filepath = parsed_args.asset_dir
        if not os.path.isabs(parsed_args.asset_dir):
            assets_filepath = os.path.join(os.getcwd(), parsed_args.asset_dir)
        if output_filepath + "assets" in assets_filepath:
            raise ValueError("Output folder must be different from (or not inside) the assets folder")

    # Add a free joint to the urdf
    if request_add_free_joint:
        urdf = add_urdf_free_joint(urdf)

    print(f"Using destination directory: {output_filepath}")

    # Add required mujoco tags to the starting URDF
    xml_data = add_mujoco_info(urdf, output_filepath, parsed_args.publish_topic)

    # get rid of collision data, assuming the visual data is much better resolution.
    # not sure if this is the best move...
    xml_data = remove_tag(xml_data, "collision")

    xml_data = replace_package_names(xml_data)
    mesh_info_dict = extract_mesh_info(xml_data, parsed_args.asset_dir, decompose_dict)
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
    fix_mujoco_description(
        output_filepath,
        mesh_info_dict,
        raw_inputs,
        scene_inputs,
        urdf,
        decompose_dict,
        cameras_dict,
        modify_element_dict,
        lidar_dict,
        request_add_free_joint,
    )

    # Publish the MuJoCo model to the specified topic if provided
    if parsed_args.publish_topic:
        publish_model_on_topic(parsed_args.publish_topic, output_filepath, args)

    # Copy the existing scene.xml to the output folder if not publishing
    if not parsed_args.publish_topic and parsed_args.save_only and parsed_args.scene:
        shutil.copy2(f"{parsed_args.scene}", output_filepath)


if __name__ == "__main__":
    main()
