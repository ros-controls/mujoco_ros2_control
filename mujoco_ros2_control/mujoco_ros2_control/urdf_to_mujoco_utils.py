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
