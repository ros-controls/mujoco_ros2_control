#!/usr/bin/env python3
#
# Copyright (c) 2026, United States Government, as represented by the
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
# under the License

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

from urdf_parser_py.urdf import URDF

from ament_index_python.packages import get_package_share_directory
from xml.dom import minidom


def add_mujoco_info(raw_xml, output_filepath, publish_topic, fuse=True):
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

    if not fuse:
        # Prevents merging of static bodies (like the fixed root link)
        compiler_element.setAttribute("fusestatic", "false")

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
