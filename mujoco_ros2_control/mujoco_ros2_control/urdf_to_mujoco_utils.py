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
import PyKDL
import json
import math

import numpy as np

from urdf_parser_py.urdf import URDF

from ament_index_python.packages import get_package_share_directory
from xml.dom import minidom

# Hardcoded relative paths for MuJoCo asset outputs
DECOMPOSED_PATH_NAME = "decomposed"
COMPOSED_PATH_NAME = "full"
# Plain visual meshes (rendering only) live here; they are referenced as-is and never
# decomposed. Collision meshes go through DECOMPOSED_PATH_NAME (obj2mjcf --decompose).
VISUAL_PATH_NAME = "visual"

# coacd threshold used when a collision mesh is not named in a decompose_mesh input.
# Matches obj2mjcf's coacd default.
DEFAULT_DECOMPOSE_THRESHOLD = "0.05"

# Explicit attributes written onto classified geoms so visual/collision separation does not
# depend on the user supplying a <default class="..."> block via mujoco_inputs.
# Visuals are render-only (contype=0 -> never collide) and live in group 2; collisions do the
# colliding (contype/conaffinity=1), live in group 3, and are tinted COLLISION_MATERIAL_NAME
# so they can be inspected in the viewer.
COLLISION_MATERIAL_NAME = "bright_orange"
VISUAL_GEOM_ATTRS = {"contype": "0", "conaffinity": "0", "group": "2", "density": "0"}
COLLISION_GEOM_ATTRS = {
    "group": "3",
    "contype": "1",
    "conaffinity": "1",
    "material": COLLISION_MATERIAL_NAME,
}


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


def add_missing_collisions(xml_string):
    """
    Ensures every link that can be rendered can also collide, while respecting any
    collision geometry the URDF author already provided.

    For each <link> that has at least one <visual> but no <collision>, a <collision>
    is synthesized for every visual by copying that visual's <geometry> and <origin>.
    Links that already declare a collision are left untouched (the authored collision
    drives physics), and links without any visual are left untouched.

    Establishing this fallback at the URDF level - before MuJoCo conversion and any
    static-body fusion - keeps the visual->collision fallback correct per link: the
    visual meshes are used purely for rendering while the collision geometry (authored
    when present, copied from the visual otherwise) is used for physics.

    :param xml_string: the URDF as a string
    :returns: the URDF string with synthesized collisions added where they were missing
    """
    dom = minidom.parseString(xml_string)

    for link in dom.getElementsByTagName("link"):
        visuals = [c for c in link.childNodes if c.nodeType == c.ELEMENT_NODE and c.tagName == "visual"]
        collisions = [c for c in link.childNodes if c.nodeType == c.ELEMENT_NODE and c.tagName == "collision"]

        # Respect authored collisions and skip links with nothing to render.
        if collisions or not visuals:
            continue

        for visual in visuals:
            collision = dom.createElement("collision")
            # Copy the geometry and origin (collisions carry no material in URDF).
            for child in visual.childNodes:
                if child.nodeType == child.ELEMENT_NODE and child.tagName in ("geometry", "origin"):
                    collision.appendChild(child.cloneNode(deep=True))
            link.appendChild(collision)

    return dom.toprettyxml()


def extract_mesh_info(raw_xml, asset_dir, decompose_dict):
    """
    Builds a dictionary of all unique meshes in the URDF (from both <visual> and
    <collision> tags) and rewrites the raw_xml so every mesh filename points at a
    disambiguated path. Visuals are processed first so their <material> color wins;
    a collision that reuses a visual's mesh file shares that asset, while a distinct
    collision mesh gets its own entry (defaulting to white, as collisions carry no
    material). There are some gotchas:

    - Different URDF meshes can share a filename stem, so colliding stems get an
      "__N" suffix so each resolves to its own asset directory. For example, if
      running multiple UR types there will be a shoulder/ and shoulder__1/ directory.
    - This happens BEFORE the pregen lookup. So if the asset_dir already
      contains shoulder/ and shoulder__1/ from a prior run, the first source
      will grab shoulder and the second source will grab shoulder__1.
    - stem_to_original_uri tracks which source uri claimed each stem slot, in
      an effort to avoid unnecessary copying.
    - A subset-equality check on (is_pre_generated, filename, scale, color) dedupes
      entries that genuinely match based on those criteria.

    Returns (mesh_info_dict, rewritten raw_xml). Each entry will have
    carries {is_pre_generated, filename (source path), scale, color, and
    new_filepath}.
    """

    robot = URDF.from_xml_string(raw_xml)
    mesh_info_dict = {}
    stem_to_original_uri = {}

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

    # Map each source mesh file to the entry (stem) it produced, so a collision that
    # reuses a visual's mesh (the fallback case) shares the single converted source
    # instead of duplicating it. Visuals are processed first so their material color
    # wins; a shared entry is then flagged for both visual and collision use, and the
    # downstream pipeline emits a whole visual mesh and decomposed collision pieces from
    # that one source.
    uri_to_stem = {}

    def mark_usage(stem_key, is_collision):
        entry = mesh_info_dict[stem_key]
        if is_collision:
            entry["used_as_collision"] = True
        else:
            entry["used_as_visual"] = True

    def process_geometry(element, is_collision):
        nonlocal raw_xml
        geom = element.geometry
        if not (geom and hasattr(geom, "filename")):
            return

        uri = geom.filename  # full URI

        # A collision reusing a visual's mesh file just records the extra usage on the
        # shared entry (one converted source, reused for both classes).
        if is_collision and uri in uri_to_stem:
            mark_usage(uri_to_stem[uri], is_collision)
            return

        original_stem = pathlib.Path(uri).stem  # NEW: remember original for disambiguation
        stem = original_stem
        counter = 0
        while stem in stem_to_original_uri and stem_to_original_uri[stem] != uri:
            counter += 1
            stem = f"{original_stem}__{counter}"
        stem_to_original_uri[stem] = uri

        # A collision mesh is only decomposed when its link was named in a decompose_mesh
        # input; otherwise (and for all visual meshes) it is used directly as a plain mesh.
        decompose_this = is_collision and (original_stem in decompose_dict)

        # Select the mesh file: use a pre-generated OBJ if available and valid; otherwise use the original
        is_pre_generated = False
        new_uri = uri  # default fallback

        if asset_dir:
            if decompose_this:
                # Decomposed collision mesh: reuse a pre-generated OBJ only if its coacd
                # threshold matches the requested one.
                mesh_file = f"{asset_dir}/{DECOMPOSED_PATH_NAME}/{stem}/{stem}/{stem}.obj"
                settings_file = f"{asset_dir}/{DECOMPOSED_PATH_NAME}/metadata.json"
                required_threshold = float(decompose_dict[original_stem])

                if os.path.exists(mesh_file) and os.path.exists(settings_file):
                    try:
                        with open(settings_file) as f:
                            data = json.load(f)
                            used_threshold = float(data.get(f"{stem}"))
                    except (FileNotFoundError, PermissionError, json.JSONDecodeError) as e:
                        print(f"Warning: could not read thresholds for {stem}: {e}")
                        used_threshold = None
                    # Use existing decomposed object only if it has the same threshold, otherwise regenerate it.
                    if used_threshold is not None and math.isclose(used_threshold, required_threshold, rel_tol=1e-9):
                        new_uri = mesh_file
                        is_pre_generated = True
                        raw_xml = raw_xml.replace(geom.filename, new_uri)
                    else:
                        print(
                            f"Existing decomposed obj for {stem} has different threshold {used_threshold} "
                            f"than required {required_threshold}. Regenerating..."
                        )
            else:
                # Plain mesh (visual, or a collision used directly): reuse a pre-generated
                # asset (.obj or copied .stl) from the appropriate plain dir.
                plain_dir = VISUAL_PATH_NAME if not is_collision else COMPOSED_PATH_NAME
                for ext in (".obj", ".stl"):
                    candidate = f"{asset_dir}/{plain_dir}/{stem}{ext}"
                    if os.path.exists(candidate):
                        new_uri = candidate
                        is_pre_generated = True
                        raw_xml = raw_xml.replace(geom.filename, new_uri)
                        break

        scale = " ".join(f"{v}" for v in geom.scale) if geom.scale else "1.0 1.0 1.0"
        # Collision elements carry no <material>, so they default to white.
        rgba = (1.0, 1.0, 1.0, 1.0) if is_collision else resolve_color(element)

        mesh_dict_value = {
            "is_pre_generated": is_pre_generated,
            "filename": new_uri,
            "scale": scale,
            "color": rgba,
        }

        # check to see if the values we are trying to add already exist
        existing_identifier = None
        for key, value in mesh_info_dict.items():
            if mesh_dict_value.items() <= value.items():
                existing_identifier = key
                break

        # an identical mesh already exists: reuse it and just record this usage
        if existing_identifier is not None:
            uri_to_stem.setdefault(uri, existing_identifier)
            mark_usage(existing_identifier, is_collision)
            return

        # get the name of the new file so that we can reference it later, but grab correct
        # pre generated asset if it exists
        path_obj = pathlib.Path(new_uri)
        if is_pre_generated and decompose_this:
            # pre-generated decomposed layout: .../<stem>/<stem>/<stem>.obj
            new_filepath = str(path_obj.parent.parent / stem / (stem + path_obj.suffix))
        else:
            # plain asset (visual or direct collision), or a freshly-sourced mesh
            new_filepath = str(path_obj.parent / (stem + path_obj.suffix))

        # add the unique name to the dictionary
        mesh_info_dict[stem] = {
            "is_pre_generated": is_pre_generated,
            "filename": new_uri,
            "scale": scale,
            "color": rgba,
            "new_filepath": new_filepath,
            # which classes reference this source; a shared source carries both
            "used_as_visual": not is_collision,
            "used_as_collision": is_collision,
        }
        uri_to_stem.setdefault(uri, stem)

        # if we changed the identifier, make sure we update it in the underlying file
        if stem != pathlib.Path(new_uri).stem:
            raw_xml = raw_xml.replace(new_uri, new_filepath)

    # First pass: visual meshes (their material color wins). Second pass: collision
    # meshes, which only add genuinely distinct meshes thanks to processed_uris.
    for link in robot.links:
        for vis in link.visuals:
            process_geometry(vis, is_collision=False)
    for link in robot.links:
        for col in link.collisions:
            process_geometry(col, is_collision=True)

    return mesh_info_dict, raw_xml


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
    """
    Expands each decomposed collision mesh into its obj2mjcf convex pieces.

    Only meshes that were decomposed (collision meshes, found under DECOMPOSED_PATH_NAME)
    are processed. For each such mesh, the collision geoms referencing it (the no-contype
    geoms) are replaced with the decomposed sub-geoms in the ``collision`` class. Visual
    geoms (contype-bearing) are left untouched referencing the whole mesh, and when the
    mesh is shared by a visual the whole ``<mesh>`` asset is kept so that visual still
    resolves. Visual-only meshes live in VISUAL_PATH_NAME, are not decomposed, and are
    left as plain single references.
    """
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

    full_decomposed_path = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}"
    decomposed_dirs = [
        name for name in os.listdir(full_decomposed_path) if os.path.isdir(os.path.join(full_decomposed_path, name))
    ]

    # obj2mjcf emits its meshes WITHOUT a name attribute, so MuJoCo derives the name from
    # the file stem. We dedupe on that "effective name" (explicit name, else file stem):
    # this skips obj2mjcf's whole/visual mesh when it collides with the kept whole mesh,
    # while still letting every uniquely-named collision piece through.
    def effective_mesh_name(m):
        name = m.getAttribute("name")
        return name if name else pathlib.Path(m.getAttribute("file")).stem

    existing_mesh_names = {effective_mesh_name(m) for m in asset_element.getElementsByTagName("mesh")}
    existing_material_names = {m.getAttribute("name") for m in asset_element.getElementsByTagName("material")}
    existing_texture_names = {t.getAttribute("name") for t in asset_element.getElementsByTagName("texture")}

    for mesh in meshes:
        mesh_name = mesh.getAttribute("name")

        # only collision meshes are decomposed; visual-only plain references are skipped
        if mesh_name not in decomposed_dirs:
            continue

        # This should definitely be there, otherwise something is horribly wrong
        scale = mesh_info_dict[mesh_name]["scale"]
        used_as_visual = mesh_info_dict[mesh_name].get("used_as_visual", False)

        mesh_path = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}/{mesh_name}/{mesh_name}/{mesh_name}.xml"
        if not os.path.exists(mesh_path):
            continue

        sub_dom = minidom.parse(mesh_path)
        sub_asset_element = sub_dom.getElementsByTagName("asset")[0]

        # If a visual also uses this mesh (shared / fallback), keep the whole <mesh> asset
        # so the visual geom still resolves; otherwise the undecomposed mesh is unused.
        if not used_as_visual:
            asset_element.removeChild(mesh)
            existing_mesh_names.discard(mesh_name)

        # bring in the decomposed sub-mesh assets, skipping any effective name we already
        # have (notably obj2mjcf's nameless visual mesh, which maps to mesh_name)
        for sub_mesh in sub_asset_element.getElementsByTagName("mesh"):
            eff = effective_mesh_name(sub_mesh)
            if eff in existing_mesh_names:
                continue
            sub_mesh_file = sub_mesh.getAttribute("file")
            sub_mesh.setAttribute("file", f"{DECOMPOSED_PATH_NAME}/{mesh_name}/{mesh_name}/{sub_mesh_file}")
            if scale:
                sub_mesh.setAttribute("scale", scale)
            asset_element.appendChild(sub_mesh)
            existing_mesh_names.add(eff)

        # bring in any materials/textures (collision usually has none, kept for safety),
        # also de-duplicating by name
        for sub_material in sub_asset_element.getElementsByTagName("material"):
            name = sub_material.getAttribute("name")
            if name in existing_material_names:
                continue
            asset_element.appendChild(sub_material)
            existing_material_names.add(name)
        for sub_texture in sub_asset_element.getElementsByTagName("texture"):
            if sub_texture.hasAttribute("file"):
                name = sub_texture.getAttribute("name")
                if name in existing_texture_names:
                    continue
                sub_texture_file = sub_texture.getAttribute("file")
                sub_texture.setAttribute("file", f"{DECOMPOSED_PATH_NAME}/{mesh_name}/{mesh_name}/{sub_texture_file}")
                asset_element.appendChild(sub_texture)
                existing_texture_names.add(name)

        body_element = sub_dom.getElementsByTagName("body")[0]
        sub_geoms = body_element.getElementsByTagName("geom")

        # Replace each collision geom (no contype) referencing this mesh with the decomposed
        # convex pieces. Visual geoms (contype) are left referencing the whole mesh.
        for geom_element in list(worldbody_geoms):
            if geom_element.getAttribute("mesh") != mesh_name:
                continue
            if geom_element.hasAttribute("contype"):
                continue

            pos = geom_element.getAttribute("pos")
            quat = geom_element.getAttribute("quat")
            parent = geom_element.parentNode
            parent.removeChild(geom_element)
            for sub_geom in sub_geoms:
                # skip obj2mjcf's own visual geom (it references the whole mesh); only the
                # convex collision pieces should become collision geoms
                if sub_geom.getAttribute("class") == "visual" or sub_geom.getAttribute("mesh") == mesh_name:
                    continue
                sub_geom_local = sub_geom.cloneNode(False)
                if pos:
                    sub_geom_local.setAttribute("pos", pos)
                if quat:
                    sub_geom_local.setAttribute("quat", quat)
                sub_geom_local.setAttribute("class", "collision")
                if sub_geom_local.hasAttribute("rgba"):
                    sub_geom_local.removeAttribute("rgba")
                parent.appendChild(sub_geom_local)

    return dom


def update_non_obj_assets(dom, output_filepath, mesh_info_dict=None):
    """
    Classifies the geoms that MuJoCo imported from the URDF into the "visual" and
    "collision" default classes. Because the source URDF now always carries both a
    <visual> and a <collision> per renderable link (the latter synthesized from the
    visual when missing, see add_missing_collisions), MuJoCo emits a separate geom for
    each, and we simply tag them rather than duplicating a single geom.

    When ``mesh_info_dict`` is provided, a visual mesh geom is also given an ``rgba`` from
    the mesh's URDF color, so plain (non-obj2mjcf) visual meshes keep their solid color.

    MuJoCo imports a URDF <visual> as a geom that still carries its raw import
    attributes, e.g.
        <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="finger_v6"/>
    and a <collision> as a plain collidable geom with no contype. So:

    - A geom WITH a contype attribute is a visual: it becomes
        <geom mesh="finger_v6" class="visual" rgba="0.2 0.2 0.2 1" .../>
      keeping its rgba but dropping the raw import attributes (contype, conaffinity,
      group, density).
    - A geom WITHOUT a contype attribute is a collision: it becomes
        <geom mesh="finger_v6" class="collision" .../>
      and its rgba (if any) is dropped since collisions are not rendered.

    Geoms that already carry a class attribute (e.g. those expanded by
    update_obj_assets) are left untouched.
    """

    # Find the <worldbody> element
    worldbody = dom.getElementsByTagName("worldbody")
    worldbody_element = worldbody[0]

    # get all of the geom elements in the worldbody element
    worldbody_geoms = worldbody_element.getElementsByTagName("geom")

    # raw import attributes that should not survive on a classified visual geom
    remove_attributes = ["contype", "conaffinity", "group", "density"]

    for geom in worldbody_geoms:
        # already classified upstream (e.g. obj-decomposed meshes); leave as is
        if geom.hasAttribute("class"):
            continue

        if geom.hasAttribute("contype"):
            # visual geom: keep rgba, strip the raw import attributes
            geom.setAttribute("class", "visual")
            for attribute in remove_attributes:
                if geom.hasAttribute(attribute):
                    geom.removeAttribute(attribute)
            # apply the URDF color so plain visual meshes (no obj2mjcf material) render
            if mesh_info_dict:
                mesh_name = geom.getAttribute("mesh")
                if mesh_name in mesh_info_dict and "color" in mesh_info_dict[mesh_name]:
                    rgba = mesh_info_dict[mesh_name]["color"]
                    geom.setAttribute("rgba", " ".join(str(v) for v in rgba))
        else:
            # collision geom: ensure a type, drop rgba (not rendered)
            if not geom.hasAttribute("type"):
                geom.setAttribute("type", "sphere")
            geom.setAttribute("class", "collision")
            if geom.hasAttribute("rgba"):
                geom.removeAttribute("rgba")

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
            if angle_increment <= 0:
                raise ValueError("'angle_increment' must be greater than zero for a 'lidar' tag!")

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

    print(f"Parsing MuJoCo elements from: {filename}")

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
                if raw_inputs is not None:
                    raise ValueError("Multiple 'raw_inputs' tags found in 'mujoco_inputs'")
                raw_inputs = child
            elif child.tagName == "processed_inputs":
                if processed_inputs is not None:
                    raise ValueError("Multiple 'processed_inputs' tags found in 'mujoco_inputs'")
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
                if raw_inputs is not None:
                    raise ValueError("Multiple 'raw_inputs' tags found in 'mujoco_inputs'")
                raw_inputs = child
            elif child.tagName == "processed_inputs":
                if processed_inputs is not None:
                    raise ValueError("Multiple 'processed_inputs' tags found in 'mujoco_inputs'")
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

    print(f"Parsing MuJoCo scene from: {filename}")

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
        return dom

    # Try to find the virtual_base_joint (standard behavior with fuse=True)
    # Locate the one with name="virtual_base_joint" of type="free"
    joints = dom.getElementsByTagName("joint")
    for joint in joints:
        if joint.getAttribute("name") == "virtual_base_joint" and joint.getAttribute("type") == "free":
            # Create the new freejoint element
            new_joint = dom.createElement("freejoint")
            new_joint.setAttribute("name", joint_name)
            # Replace the old joint with the new one
            joint.parentNode.replaceChild(new_joint, joint)
            return dom

    # Fallback: If no virtual joint found (likely because --no-fuse is on),
    #    find the root body and insert the freejoint directly.
    bodies = dom.getElementsByTagName("body")
    for body in bodies:
        if body.getAttribute("name") == root_link:
            print(f"Adding free joint directly to root body: {root_link}")
            new_joint = dom.createElement("freejoint")
            new_joint.setAttribute("name", joint_name)

            # Insert at the top of the body
            if body.hasChildNodes():
                body.insertBefore(new_joint, body.firstChild)
            else:
                body.appendChild(new_joint)
            return dom

    raise ValueError("Did not find virtual_base_joint nor a body matching the URDF root to add a free joint.")


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
            # MuJoCo goes to the end of the joint
            # transform = link_tf*transform
            return (link, transform, False)

    for link in robot.links:
        results[link.name] = get_parent_chain(link.name)

    return results


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert an Euler RPY angles to a quaternion, [w, x, y, z]
    """
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)

    return [qw, qx, qy, qz]


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
        # change x y z w to w x y z for ros to MuJoCo convention
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

    # remove the first line bc MuJoCo doesn't like the xml tag at the beginning
    modified_lines = modified_data.splitlines(True)
    modified_lines.pop(0)
    modified_data = "".join(modified_lines)

    return dom


def add_cameras_from_sites(dom, cameras_dict):
    """
    Adds cameras to the sites listed in the cameras_dict. We make the assumption that any
    link/site with that name is the color frame for a physical camera matching the REP 103 standard for
    optical frames. As noted in the ROS image message,
    https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg#L7

        > +x should point to the right in the image
        > +y should point down in the image
        > +z should point into to plane of the image

    MuJoCo Cameras are oriented slightly differently, https://mujoco.readthedocs.io/en/latest/modeling.html#cameras

        > Cameras look towards the negative Z axis of the camera frame, while positive X and Y correspond
        > to right and up in the image plane, respectively.

    Therefore, given the site name, the camera is added at the same position but it is rotated pi radians about the
    x axis to ensure the simulated camera's images match those from the URDF.
    """

    x_rotation = [0.0, 1.0, 0.0, 0.0]  # Rotation by pi around x axis

    matched_sites = set()

    # Construct all cameras for relevant sites in xml and add them as children to the same parent
    for node in dom.getElementsByTagName("site"):
        site_name = node.getAttribute("name")
        if site_name in cameras_dict:
            matched_sites.add(site_name)
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

    unmatched = set(cameras_dict.keys()) - matched_sites
    if unmatched:
        raise ValueError(f"Camera site(s) not found in the MJCF: {', '.join(sorted(unmatched))}")

    return dom


def add_lidar_from_sites(dom, lidar_dict):
    """
    Creates a replicates tag from MuJoCo inputs below the specified site name and lidar tag.

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
    matched_sites = set()

    # Construct all lidar sensor bodies for relevant sites in xml and add them as children to the same parent
    for node in dom.getElementsByTagName("site"):
        site_name = node.getAttribute("name")
        if site_name in lidar_dict:
            matched_sites.add(site_name)
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

    unmatched = set(lidar_dict.keys()) - matched_sites
    if unmatched:
        raise ValueError(f"Lidar site(s) not found in the MJCF: {', '.join(sorted(unmatched))}")

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
    Copies pre-generated meshes (from an --asset_dir) into the final MJCF assets
    structure. Collision meshes are copied as decomposed folders under
    DECOMPOSED_PATH_NAME; plain visual meshes are copied as single files under
    VISUAL_PATH_NAME.
    """
    thresholds_file = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}/metadata.json"
    thresholds_data = {}
    if os.path.exists(thresholds_file):
        try:
            with open(thresholds_file) as f:
                thresholds_data = json.load(f)
        except json.JSONDecodeError:
            thresholds_data = {}

    for mesh_name in mesh_info_dict:
        mesh_item = mesh_info_dict[mesh_name]
        if not mesh_item["is_pre_generated"]:
            continue

        full_path = mesh_item["filename"]
        # decomposed only when the collision mesh was explicitly requested via decompose_mesh
        decompose_this = mesh_item.get("used_as_collision", False) and (mesh_name in decompose_dict)

        if decompose_this:
            # decomposed collision mesh: copy the .../<stem>/<stem>/ folder of pieces
            threshold = decompose_dict[mesh_name]
            src_dir = os.path.dirname(full_path)
            dst_base = f"{output_filepath}assets/{DECOMPOSED_PATH_NAME}/{mesh_name}/{mesh_name}/"
            shutil.copytree(src_dir, dst_base, dirs_exist_ok=True)
            thresholds_data[mesh_name] = float(threshold)
        else:
            # plain mesh used directly: copy the single file into its plain dir
            plain_dir = VISUAL_PATH_NAME if mesh_item.get("used_as_visual", False) else COMPOSED_PATH_NAME
            os.makedirs(f"{output_filepath}assets/{plain_dir}", exist_ok=True)
            ext = os.path.splitext(full_path)[1]
            shutil.copy2(full_path, f"{output_filepath}assets/{plain_dir}/{mesh_name}{ext}")

    if thresholds_data:
        with open(thresholds_file, "w") as f:
            json.dump(thresholds_data, f, indent=4)


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
    Adds a free joint to the top of the urdf. This makes MuJoCo create a
    floating joint so that a base is free to move, like on an AMR.
    """

    # get the old root link
    robot = URDF.from_xml_string(urdf)
    old_root = robot.get_root()

    if old_root == "world":
        print("Not adding a free joint because world is the URDF root")
        return urdf

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

    # Add an <include> tag for the MuJoCo description
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
