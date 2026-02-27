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
import shutil
import subprocess
import tempfile
import sys
import json

import numpy as np
import trimesh  # Added trimesh

from xml.dom import minidom
import mujoco_ros2_control as mrc


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
    os.makedirs(f"{directory}assets/{mrc.COMPOSED_PATH_NAME}", exist_ok=True)
    os.makedirs(f"{directory}assets/{mrc.DECOMPOSED_PATH_NAME}", exist_ok=True)

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
            assets_relative_filepath = f"{mrc.DECOMPOSED_PATH_NAME}/{filename_no_ext}/"
            os.makedirs(f"{directory}assets/{assets_relative_filepath}", exist_ok=True)
        else:
            assets_relative_filepath = f"{mrc.COMPOSED_PATH_NAME}/"
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
            image_files = mrc.get_images_from_dae(full_filepath)
            # keep track of the files that were copied to tmp directory to delete
            copied_image_files = []

            # set z axis to up in the dae file because that is how MuJoCo expects it
            z_up_dae_txt = mrc.set_up_axis_to_z_up(full_filepath)

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
                # element, assign default glossiness and specular but keep the rgba values.
                # We also ignore this process if there are any image files so that we don't
                # accidentally overwrite those. Technically we could have cases where daes have
                # both images, and materials but I think the logic gets exceedingly complex to
                # handle that.
                if not image_files:
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
                mrc.rename_material_textures(dir_path=os.path.dirname(output_path), modifier=mtl_modifier)

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


def run_obj2mjcf(output_filepath, decompose_dict):
    # remove the folders in the asset directory so that we are clean to run obj2mjcf
    with os.scandir(f"{output_filepath}assets/{mrc.COMPOSED_PATH_NAME}") as entries:
        for entry in entries:
            if entry.is_dir():
                shutil.rmtree(entry.path)

    # remove the folders in the asset directory so that we are clean to run obj2mjcf
    top_level_path = f"{output_filepath}assets/{mrc.DECOMPOSED_PATH_NAME}"
    for item in os.listdir(top_level_path):
        first_level_path = os.path.join(top_level_path, item)
        if os.path.isdir(first_level_path):
            # Now check inside this first-level directory
            for sub_item in os.listdir(first_level_path):
                second_level_path = os.path.join(first_level_path, sub_item)
                if os.path.isdir(second_level_path):
                    shutil.rmtree(second_level_path)

    # run obj2mjcf to generate folders of processed objs
    cmd = ["obj2mjcf", "--obj-dir", f"{output_filepath}assets/{mrc.COMPOSED_PATH_NAME}", "--save-mjcf"]
    subprocess.run(cmd)

    thresholds_file = os.path.join(f"{output_filepath}assets/{mrc.DECOMPOSED_PATH_NAME}", "metadata.json")

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
            f"{output_filepath}assets/{mrc.DECOMPOSED_PATH_NAME}/{mesh_name}",
            "--save-mjcf",
            "--decompose",
            "--coacd-args.threshold",
            threshold,
        ]
        subprocess.run(cmd)

        thresholds_data[mesh_name] = float(threshold)

    with open(thresholds_file, "w") as f:
        json.dump(thresholds_data, f, indent=4)


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
    mrc.copy_pre_generated_meshes(output_filepath, mesh_info_dict, decompose_dict)

    # Parse the DAE file
    dom = minidom.parse(full_filepath)

    if request_add_free_joint:
        dom = mrc.add_free_joint(dom, urdf)

    # Update and add the new fixed assets
    dom = mrc.update_obj_assets(dom, output_filepath, mesh_info_dict)
    dom = mrc.update_non_obj_assets(dom, output_filepath)

    # Add the MuJoCo input elements
    dom = mrc.add_mujoco_inputs(dom, raw_inputs, scene_inputs)

    # Add links as sites
    dom = mrc.add_links_as_sites(urdf, dom, request_add_free_joint)

    # Add cameras based on site names
    dom = mrc.add_cameras_from_sites(dom, cameras_dict)

    # Add replicates based on site names
    dom = mrc.add_lidar_from_sites(dom, lidar_dict)

    # modify elements based on modify_element tags
    dom = mrc.add_modifiers(dom, modify_element_dict)

    # Write the updated file
    with open(destination_file, "w") as file:
        # Convert the DOM back to a string
        modified_data = dom.toprettyxml(indent="  ")
        modified_data = "\n".join([line for line in modified_data.splitlines() if line.strip()])

        # remove the first line bc MuJoCo doesn't like the xml tag at the beginning
        modified_lines = modified_data.splitlines(True)
        modified_lines.pop(0)
        modified_data = "".join(modified_lines)
        file.write(modified_data)


def main(args=None):

    parser = argparse.ArgumentParser(description="Convert a full URDF to MJCF for use in MuJoCo")
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
        "--fuse",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Allows MuJoCo to merge static bodies. Use --no-fuse to prevent merging.",
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
        urdf = mrc.get_xml_from_file(parsed_args.urdf)
        urdf_path = parsed_args.urdf
    else:
        if parsed_args.robot_description:
            urdf = parsed_args.robot_description
        else:
            urdf = mrc.get_urdf_from_rsp(args)
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

    raw_inputs, processed_inputs = mrc.parse_inputs_xml(mujoco_inputs_file)

    scene_inputs = None
    if parsed_args.publish_topic or (parsed_args.save_only and not parsed_args.scene):
        scene_inputs = mrc.parse_scene_xml(mujoco_scene_file)

    # Copy the scene tags from URDF to a separate xml il not publishing
    if not parsed_args.publish_topic and parsed_args.save_only and scene_inputs:
        print("Copying scene tags from URDF to a separate xml")
        mrc.write_mujoco_scene(scene_inputs, output_filepath)
        scene_inputs = None

    decompose_dict, cameras_dict, modify_element_dict, lidar_dict = mrc.get_processed_mujoco_inputs(processed_inputs)

    if parsed_args.asset_dir:
        assets_filepath = parsed_args.asset_dir
        if not os.path.isabs(parsed_args.asset_dir):
            assets_filepath = os.path.join(os.getcwd(), parsed_args.asset_dir)
        if output_filepath + "assets" in assets_filepath:
            raise ValueError("Output folder must be different from (or not inside) the assets folder")

    # Add a free joint to the urdf
    # Only add the virtual link structure if we allow fusing.
    # If no-fuse is on, we skip this and add the freejoint directly in mrc.add_free_joint.
    if request_add_free_joint and parsed_args.fuse:
        urdf = mrc.add_urdf_free_joint(urdf)

    print(f"Using destination directory: {output_filepath}")

    # Add required MuJoCo tags to the starting URDF
    xml_data = mrc.add_mujoco_info(urdf, output_filepath, parsed_args.publish_topic, parsed_args.fuse)

    # get rid of collision data, assuming the visual data is much better resolution.
    # not sure if this is the best move...
    xml_data = mrc.remove_tag(xml_data, "collision")

    xml_data = mrc.replace_package_names(xml_data)
    mesh_info_dict, xml_data = mrc.extract_mesh_info(xml_data, parsed_args.asset_dir, decompose_dict)
    xml_data = convert_to_objs(mesh_info_dict, output_filepath, xml_data, convert_stl_to_obj, decompose_dict)

    print("writing data to robot_description_formatted.urdf")
    robot_description_filename = "robot_description_formatted.urdf"
    with open(output_filepath + "robot_description_formatted.urdf", "w") as file:
        # Remove extra newlines that minidom adds after each tag
        xml_data = "\n".join([line for line in xml_data.splitlines() if line.strip()])
        file.write(xml_data)

    model = mujoco.MjModel.from_xml_path(f"{output_filepath}{robot_description_filename}")
    mujoco.mj_saveLastXML(f"{output_filepath}mujoco_description.xml", model)

    # Converts objs for use in MuJoCo, adds tags, inputs, sites, and sensors to the final xml
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
        mrc.publish_model_on_topic(parsed_args.publish_topic, output_filepath, args)

    # Copy the existing scene.xml to the output folder if not publishing
    if not parsed_args.publish_topic and parsed_args.save_only and parsed_args.scene:
        shutil.copy2(f"{parsed_args.scene}", output_filepath)


if __name__ == "__main__":
    main()
