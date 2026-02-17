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
