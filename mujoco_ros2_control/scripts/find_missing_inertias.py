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
import xml.etree.ElementTree as ET
import sys
from collections import deque, defaultdict

# Prints a list of all moveable links in an URDF that are missing inertial data.
# Mujoco requires that all moveable bodies have a non-zero inertia and mass, which boils
# down to having at least one statically attached object to every non-fixed joint's children.
# This is a little helper script to identify joints/links that are missing required data
# because it can be really annoying to identify what is missing when converting URDFs,
# and the errors from Mujoco's XML loader do not give info.


def parse_urdf(urdf_fp):
    """
    Parse the URDF and return a list of joints { name, parent, child, type }
    """
    try:
        tree = ET.parse(urdf_fp)
    except ET.ParseError as e:
        sys.exit(f"Failed to parse URDF: {e}")
    root = tree.getroot()
    joints = []
    for j in root.findall("joint"):
        joints.append(
            {
                "name": j.get("name", "<unnamed>"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "type": j.get("type", "fixed"),
            }
        )
    return root, joints


def collect_valid_inertial_links(root):
    """
    Gets a list of all the joints in the URDF that have valid inertial properties.
    """
    inertia_attrs = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}
    valid = set()
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = inertial.find("mass")
        if mass is None or "value" not in mass.attrib:
            continue
        inertia = inertial.find("inertia")
        if inertia is None or not inertia_attrs.issubset(inertia.attrib):
            continue
        valid.add(name)
    return valid


def has_static_inertial_descendant(start_link, child_map, valid_links):
    """
    Traverse over statically connected descedents and make sure at least one of them is in the
    list of links with inertia data.
    """
    q = deque([start_link])
    seen = {start_link}
    while q:
        link = q.popleft()
        # if this link itself has valid inertial, accept
        if link in valid_links:
            return True, []
        # traverse only fixed‐type joints
        for child, jtype, _ in child_map.get(link, ()):
            if jtype == "fixed" and child not in seen:
                seen.add(child)
                q.append(child)
    return False, seen


def check_urdf(urdf_file):
    root, joints = parse_urdf(urdf_file)
    valid_links = collect_valid_inertial_links(root)

    # Make a list of all joints
    child_map = defaultdict(list)
    for j in joints:
        child_map[j["parent"]].append((j["child"], j["type"], j["name"]))

    invalid_joints = []
    for j in joints:
        if j["type"] == "fixed":
            continue
        # for a moving joint, check its child link's static subtree
        valid, links = has_static_inertial_descendant(j["child"], child_map, valid_links)
        if not valid:
            invalid_joints.append((j["name"], links))

    if invalid_joints:
        for joint, links in invalid_joints:
            print(f"Invalid joint: {joint}")
            print("  links:")
            links_str = {", ".join(links)}
            print(f"    {links_str}")
        sys.exit(1)
    else:
        print("All moving joints have at least one static descendant with valid mass & inertia.")
        sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="For each moving joint, report if its static-descendant subtree lacks any valid mass+inertia link."
    )
    parser.add_argument("urdf_file", type=argparse.FileType("r"), help="Path to your URDF file")
    args = parser.parse_args()
    check_urdf(args.urdf_file)
