# Copyright (c) 2026 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import unittest
import math
import json
import PyKDL
from unittest.mock import patch

from xml.dom import minidom

from mujoco_ros2_control import (
    add_mujoco_info,
    remove_tag,
    get_images_from_dae,
    rename_material_textures,
    set_up_axis_to_z_up,
    multiply_quaternion,
    euler_to_quaternion,
    replace_package_names,
    update_obj_assets,
    update_non_obj_assets,
    add_mujoco_inputs,
    get_processed_mujoco_inputs,
    DECOMPOSED_PATH_NAME,
    COMPOSED_PATH_NAME,
    write_mujoco_scene,
    add_urdf_free_joint,
    get_xml_from_file,
    add_modifiers,
    add_lidar_from_sites,
    add_cameras_from_sites,
    add_links_as_sites,
    get_urdf_transforms,
    add_free_joint,
    parse_inputs_xml,
    parse_scene_xml,
    extract_mesh_info,
    copy_pre_generated_meshes,
)


def get_child_elements(dom) -> list:
    return [node.nodeName for node in dom.childNodes if node.nodeType == node.ELEMENT_NODE]


class TestUrdfToMjcfUtils(unittest.TestCase):
    def test_multiply_quaternion_identity(self):
        q1 = [1.0, 0.0, 0.0, 0.0]
        q2 = [1.0, 0.0, 0.0, 0.0]
        result = multiply_quaternion(q1, q2)
        self.assertAlmostEqual(result[0], 1.0, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_multiply_quaternion_90deg_rotation(self):
        q1 = [math.sqrt(2) / 2, math.sqrt(2) / 2, 0.0, 0.0]
        q2 = [math.sqrt(2) / 2, 0.0, math.sqrt(2) / 2, 0.0]
        result = multiply_quaternion(q1, q2)
        w, x, y, z = result
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        self.assertAlmostEqual(norm, 1.0, places=6)

    def test_euler_to_quaternion_identity(self):
        result = euler_to_quaternion(0.0, 0.0, 0.0)
        self.assertAlmostEqual(result[0], 1.0, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_euler_to_quaternion_90deg_roll(self):
        result = euler_to_quaternion(math.pi / 2, 0.0, 0.0)
        self.assertAlmostEqual(result[0], math.sqrt(2) / 2, places=6)
        self.assertAlmostEqual(result[1], math.sqrt(2) / 2, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_euler_to_quaternion_90deg_pitch(self):
        result = euler_to_quaternion(0.0, math.pi / 2, 0.0)
        self.assertAlmostEqual(result[0], math.sqrt(2) / 2, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], math.sqrt(2) / 2, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_euler_to_quaternion_90deg_yaw(self):
        result = euler_to_quaternion(0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(result[0], math.sqrt(2) / 2, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], math.sqrt(2) / 2, places=6)

    def test_remove_tag_basic(self):
        xml_string = "<root><tag_to_remove>content</tag_to_remove><other>value</other></root>"
        result = remove_tag(xml_string, "tag_to_remove")
        assert "tag_to_remove" not in result
        assert "<other>value</other>" in result

    def test_remove_tag_nonexistent(self):
        xml_string = "<root><other>value</other></root>"
        result = remove_tag(xml_string, "nonexistent")
        assert "<other>value</other>" in result

    def test_remove_tag_multiple(self):
        xml_string = "<root><tag>first</tag><other>value</other><tag>second</tag></root>"
        result = remove_tag(xml_string, "tag")
        assert "<tag>" not in result
        assert "<other>value</other>" in result

    def test_add_mujoco_info_basic(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = True

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic)

        assert "<mujoco>" in result
        assert "<compiler" in result
        assert 'assetdir="' in result
        assert 'balanceinertia="true"' in result
        assert "<robot" in result

    def test_add_mujoco_info_no_publish_topic(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = False

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic)

        assert 'assetdir="assets"' in result

    def test_add_mujoco_info_with_publish_topic(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = True

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic)

        assert 'assetdir="/tmp/test/assets"' in result

    def test_add_mujoco_info_no_fuse(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = True
        fuse = False

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic, fuse=fuse)

        assert 'fusestatic="false"' in result

    def test_get_images_from_dae_basic(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_images>
    <image id="texture1" name="texture1">
      <init_from>image.png</init_from>
    </image>
  </library_images>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".dae", delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = get_images_from_dae(dae_path)
            self.assertEqual(len(result), 1)
            self.assertTrue(result[0].endswith("image.png"))
        finally:
            os.unlink(dae_path)

    def test_get_images_from_dae_multiple_images(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_images>
    <image id="texture1" name="texture1">
      <init_from>image1.png</init_from>
    </image>
    <image id="texture2" name="texture2">
      <init_from>image2.jpg</init_from>
    </image>
  </library_images>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".dae", delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = get_images_from_dae(dae_path)
            self.assertEqual(len(result), 2)
        finally:
            os.unlink(dae_path)

    def test_get_images_from_dae_no_images(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_geometries>
    <geometry id="mesh">
      <mesh>
      </mesh>
    </geometry>
  </library_geometries>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".dae", delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = get_images_from_dae(dae_path)
            self.assertEqual(len(result), 0)
        finally:
            os.unlink(dae_path)

    def test_get_images_from_dae_relative_path(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_images>
    <image id="texture1" name="texture1">
      <init_from>../textures/image.png</init_from>
    </image>
  </library_images>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".dae", delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = get_images_from_dae(dae_path)
            self.assertEqual(len(result), 1)
            self.assertTrue(os.path.isabs(result[0]))
        finally:
            os.unlink(dae_path)

    def test_rename_material_textures_basic(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            test_files = [
                "material_0.png",
                "material_1.jpg",
                "material_2.jpeg",
                "other_file.png",
            ]

            for fname in test_files:
                filepath = os.path.join(tmpdir, fname)
                with open(filepath, "w") as f:
                    f.write("test")

            rename_material_textures(tmpdir, "test_modifier")

            expected = [
                "material_test_modifier_0.png",
                "material_test_modifier_1.jpg",
                "material_test_modifier_2.jpeg",
                "other_file.png",
            ]

            for fname in expected:
                filepath = os.path.join(tmpdir, fname)
                self.assertTrue(os.path.exists(filepath), f"Expected {fname} to exist")

    def test_rename_material_textures_no_match(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "other_file.png")
            with open(filepath, "w") as f:
                f.write("test")

            rename_material_textures(tmpdir, "modifier")

            self.assertTrue(os.path.exists(os.path.join(tmpdir, "other_file.png")))

    def test_set_up_axis_to_z_up_create(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_visual_scenes>
  </library_visual_scenes>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".dae", delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = set_up_axis_to_z_up(dae_path)
            assert "<up_axis>" in result
            assert "Z_UP" in result
        finally:
            os.unlink(dae_path)

    def test_set_up_axis_to_z_up_update(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <up_axis>X_UP</up_axis>
  </asset>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode="w", suffix=".dae", delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = set_up_axis_to_z_up(dae_path)
            assert "Z_UP" in result
        finally:
            os.unlink(dae_path)

    def test_replace_package_names_minimal(self):
        xml_data = (
            '<?xml version="1.0"?><robot><link><visual><geometry><mesh '
            'filename="package://mujoco_ros2_control/model.dae"/></geometry>'
            "</visual></link></robot>"
        )
        result = replace_package_names(xml_data)
        assert "package://mujoco_ros2_control/" not in result
        dom = minidom.parseString(result)
        self.assertTrue(dom.getElementsByTagName("mesh")[0].getAttribute("filename").endswith("model.dae"))
        self.assertTrue(os.path.isdir(os.path.dirname(dom.getElementsByTagName("mesh")[0].getAttribute("filename"))))

    @patch("mujoco_ros2_control.urdf_to_mujoco_utils.get_package_share_directory")
    def test_replace_package_names_basic(self, mock_get_package):
        mock_get_package.return_value = "/opt/ros/rolling/share/test_package"
        xml_data = (
            '<?xml version="1.0"?><robot><link><visual><geometry><mesh '
            'filename="package://test_package/meshes/model.dae"/></geometry>'
            "</visual></link></robot>"
        )
        result = replace_package_names(xml_data)
        assert "package://test_package/" not in result
        mock_get_package.assert_called_once_with("test_package")

    def test_replace_package_names_no_package(self):
        xml_data = (
            '<?xml version="1.0"?><robot><link><visual><geometry><mesh '
            'filename="/absolute/path/model.dae"/></geometry></visual></link></robot>'
        )
        result = replace_package_names(xml_data)
        assert "/absolute/path/model.dae" in result

    def test_replace_package_names_file_prefix_removed(self):
        xml_data = (
            '<?xml version="1.0"?><robot><link><visual><geometry>'
            '<mesh filename="file:///some/path/model.dae"/></geometry></visual></link></robot>'
        )
        result = replace_package_names(xml_data)
        assert "file://" not in result
        assert "/some/path/model.dae" in result

    @patch("mujoco_ros2_control.urdf_to_mujoco_utils.get_package_share_directory")
    def test_replace_package_names_multiple_packages(self, mock_get_package):
        mock_get_package.side_effect = lambda pkg: f"/opt/ros/rolling/share/{pkg}"
        xml_data = (
            '<?xml version="1.0"?><robot><link><visual><geometry><mesh '
            'filename="package://pkg_a/mesh_a.dae"/></geometry></visual></link><link>'
            '<visual><geometry><mesh filename="package://pkg_b/mesh_b.dae"/></geometry>'
            "</visual></link></robot>"
        )
        result = replace_package_names(xml_data)
        assert "package://pkg_a/" not in result
        assert "package://pkg_b/" not in result
        dom = minidom.parseString(result)
        self.assertTrue(dom.getElementsByTagName("mesh")[0].getAttribute("filename").endswith("mesh_a.dae"))
        self.assertTrue(dom.getElementsByTagName("mesh")[1].getAttribute("filename").endswith("mesh_b.dae"))

    def test_update_obj_assets_no_assets(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody><body name="test"/></worldbody></mujoco>'
        dom = minidom.parseString(xml_string)
        result_dom = update_obj_assets(dom, "/tmp/output/", {})
        self.assertIsNotNone(result_dom)

    def test_update_obj_assets_no_matching_dirs(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            os.makedirs(os.path.join(tmpdir, "assets", DECOMPOSED_PATH_NAME))
            os.makedirs(os.path.join(tmpdir, "assets", COMPOSED_PATH_NAME))
            xml_string = (
                '<?xml version="1.0"?><mujoco><asset><mesh name="unknown_mesh" '
                'file="test.obj"/></asset><worldbody><body name="test"><geom mesh="unknown_mesh" '
                'pos="0 0 0" quat="1 0 0 0"/></body></worldbody></mujoco>'
            )
            dom = minidom.parseString(xml_string)
            mesh_info_dict = {
                "unknown_mesh": {
                    "is_pre_generated": False,
                    "filename": "/some/path.obj",
                    "scale": "1.0 1.0 1.0",
                    "color": (1.0, 1.0, 1.0, 1.0),
                }
            }
            result_dom = update_obj_assets(dom, tmpdir + "/", mesh_info_dict)
            self.assertIsNotNone(result_dom)

    def test_update_non_obj_assets_basic(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="test">
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.2 0.2 0.2 1" mesh="finger_v6"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        result_dom = update_non_obj_assets(dom, "/tmp/output/")
        result_xml = result_dom.toxml()
        assert 'class="collision"' in result_xml
        assert 'class="visual"' in result_xml
        assert "contype" not in result_xml

    def test_update_non_obj_assets_no_contype(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="test">
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        result_dom = update_non_obj_assets(dom, "/tmp/output/")
        result_xml = result_dom.toxml()
        assert "<geom" in result_xml

    def test_update_non_obj_assets_multiple_geoms(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="test">
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" mesh="mesh1"/>
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" mesh="mesh2"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        result_dom = update_non_obj_assets(dom, "/tmp/output/")
        result_xml = result_dom.toxml()
        self.assertEqual(result_xml.count('class="collision"'), 2)
        self.assertEqual(result_xml.count('class="visual"'), 2)
        self.assertEqual(result_xml.count("contype"), 0)
        self.assertEqual(result_xml.count("conaffinity"), 0)
        self.assertEqual(result_xml.count('group="1"'), 0)
        self.assertEqual(result_xml.count('density="0"'), 0)
        self.assertRegex(result_xml, r'<geom[^>]*mesh="mesh1"[^>]*class="visual"[^>]*>')
        self.assertRegex(result_xml, r'<geom[^>]*mesh="mesh2"[^>]*class="collision"[^>]*>')

    def test_add_mujoco_inputs_both_none(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody/></mujoco>'
        dom = minidom.parseString(xml_string)
        result_dom = add_mujoco_inputs(dom, None, None)
        self.assertIsNotNone(result_dom)

    def test_add_mujoco_inputs_with_raw_inputs(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody/></mujoco>'
        raw_xml = '<?xml version="1.0"?><raw_inputs><option integrator="implicitfast"/></raw_inputs>'
        raw_dom = minidom.parseString(raw_xml)
        raw_inputs = raw_dom.getElementsByTagName("raw_inputs")[0]

        dom = minidom.parseString(xml_string)
        result_dom = add_mujoco_inputs(dom, raw_inputs, None)
        result_xml = result_dom.toxml()
        assert "integrator" in result_xml

    def test_add_mujoco_inputs_with_scene_inputs(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody/></mujoco>'
        scene_xml = '<?xml version="1.0"?><scene><light name="test" diffuse="1 1 1"/></scene>'
        scene_dom = minidom.parseString(scene_xml)
        scene_inputs = scene_dom.getElementsByTagName("scene")[0]

        dom = minidom.parseString(xml_string)
        result_dom = add_mujoco_inputs(dom, None, scene_inputs)
        result_xml = result_dom.toxml()
        assert "light" in result_xml

    def test_add_mujoco_inputs_both_inputs(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody/></mujoco>'
        raw_xml = '<?xml version="1.0"?><raw_inputs><option integrator="implicitfast"/></raw_inputs>'
        raw_dom = minidom.parseString(raw_xml)
        raw_inputs = raw_dom.getElementsByTagName("raw_inputs")[0]

        scene_xml = '<?xml version="1.0"?><scene><light name="test"/></scene>'
        scene_dom = minidom.parseString(scene_xml)
        scene_inputs = scene_dom.getElementsByTagName("scene")[0]

        dom = minidom.parseString(xml_string)
        result_dom = add_mujoco_inputs(dom, raw_inputs, scene_inputs)
        result_xml = result_dom.toxml()
        assert "integrator" in result_xml
        assert "light" in result_xml

    def test_get_processed_mujoco_inputs_none_element(self):
        result = get_processed_mujoco_inputs(None)
        self.assertEqual(len(result), 4)
        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = result
        self.assertEqual(decompose_dict, {})
        self.assertEqual(cameras_dict, {})
        self.assertEqual(modify_element_dict, {})
        self.assertEqual(lidar_dict, {})

    def test_get_processed_mujoco_inputs_decompose_mesh(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <decompose_mesh mesh_name="test_mesh" threshold="0.03"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        assert "test_mesh" in decompose_dict
        self.assertEqual(decompose_dict["test_mesh"], "0.03")

    def test_get_processed_mujoco_inputs_decompose_mesh_default_threshold(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <decompose_mesh mesh_name="test_mesh"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        self.assertEqual(decompose_dict["test_mesh"], "0.05")

    def test_get_processed_mujoco_inputs_camera(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <camera site="camera_site" name="test_camera" fovy="58" mode="fixed"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        assert "camera_site" in cameras_dict
        self.assertEqual(cameras_dict["camera_site"].getAttribute("name"), "test_camera")

    def test_get_processed_mujoco_inputs_lidar(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <lidar ref_site="lidar_site" sensor_name="rf" min_angle="0" max_angle="1.57" angle_increment="0.025"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        assert "lidar_site" in lidar_dict

    def test_get_processed_mujoco_inputs_modify_element(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <modify_element name="test_body" type="body" pos="1 2 3"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        key = ("body", "test_body")
        assert key in modify_element_dict
        self.assertEqual(modify_element_dict[key]["pos"], "1 2 3")

    def test_get_processed_mujoco_inputs_modify_element_missing_attrs(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <modify_element name="test_body"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        with self.assertRaises(ValueError) as context:
            get_processed_mujoco_inputs(processed_element)
        assert "'name' and 'type'" in str(context.exception)

    def test_get_processed_mujoco_inputs_multiple_elements(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <decompose_mesh mesh_name="mesh1" threshold="0.01"/>
  <decompose_mesh mesh_name="mesh2"/>
  <camera site="site1" name="cam1" fovy="60"/>
  <modify_element name="body1" type="body" pos="0 0 0"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        self.assertEqual(len(decompose_dict), 2)
        assert "mesh1" in decompose_dict
        assert "mesh2" in decompose_dict
        assert "site1" in cameras_dict
        assert ("body", "body1") in modify_element_dict

    def test_write_mujoco_scene_none(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            write_mujoco_scene(None, tmpdir + "/")
            output_file = os.path.join(tmpdir, "scene.xml")
            self.assertTrue(os.path.exists(output_file))
            with open(output_file) as f:
                content = f.read()
            assert "<mujoco" in content
            assert 'model="scene"' in content
            assert "<include" in content

            dom = minidom.parseString(content)
            self.assertEqual(dom.documentElement.tagName, "mujoco")
            self.assertEqual(dom.documentElement.getAttribute("model"), "scene")
            include_tags = dom.getElementsByTagName("include")
            self.assertEqual(len(include_tags), 1)
            assert "file" in include_tags[0].attributes
            child_nodes = [node for node in dom.documentElement.childNodes if node.nodeType == node.ELEMENT_NODE]
            self.assertEqual(len(child_nodes), 1)  # Only the include tag should be present
            self.assertEqual(child_nodes[0].tagName, "include")

    def test_write_mujoco_scene_with_scene_tag(self):
        scene_xml = '<?xml version="1.0"?><scene><light name="test_light" diffuse="1 1 1"/></scene>'
        scene_dom = minidom.parseString(scene_xml)
        scene_inputs = scene_dom.getElementsByTagName("scene")[0]

        with tempfile.TemporaryDirectory() as tmpdir:
            write_mujoco_scene(scene_inputs, tmpdir + "/")
            output_file = os.path.join(tmpdir, "scene.xml")
            self.assertTrue(os.path.exists(output_file))
            with open(output_file) as f:
                content = f.read()
            assert "<mujoco" in content
            assert "<include" in content
            assert "light" in content
            assert 'name="test_light"' in content

            dom = minidom.parseString(content)
            lights = dom.getElementsByTagName("light")
            self.assertEqual(len(lights), 1)
            self.assertEqual(lights[0].getAttribute("name"), "test_light")
            self.assertEqual(lights[0].getAttribute("diffuse"), "1 1 1")
            child_nodes = [node for node in dom.documentElement.childNodes if node.nodeType == node.ELEMENT_NODE]
            self.assertEqual(len(child_nodes), 2)  # 1 light + 1 include
            for node in child_nodes:
                assert node.tagName in ["light", "include"]

    def test_write_mujoco_scene_with_muojco_inputs(self):
        xml_string = """<?xml version="1.0"?>
<mujoco_inputs>
  <scene>
    <light name="scene_light" diffuse="0.5 0.5 0.5"/>
  </scene>
</mujoco_inputs>"""
        scene_dom = minidom.parseString(xml_string)
        scene_inputs = scene_dom.getElementsByTagName("mujoco_inputs")[0]

        with tempfile.TemporaryDirectory() as tmpdir:
            write_mujoco_scene(scene_inputs, tmpdir + "/")
            output_file = os.path.join(tmpdir, "scene.xml")
            self.assertTrue(os.path.exists(output_file))
            with open(output_file) as f:
                content = f.read()
            assert "light" in content
            assert 'name="scene_light"' in content
            assert 'diffuse="0.5 0.5 0.5"' in content

            dom = minidom.parseString(content)
            lights = dom.getElementsByTagName("light")
            self.assertEqual(len(lights), 1)
            self.assertEqual(lights[0].getAttribute("name"), "scene_light")
            self.assertEqual(lights[0].getAttribute("diffuse"), "0.5 0.5 0.5")
            child_nodes = [node for node in dom.documentElement.childNodes if node.nodeType == node.ELEMENT_NODE]
            self.assertEqual(len(child_nodes), 2)  # 1 light + 1 include
            for node in child_nodes:
                assert node.tagName in ["light", "include"]

    def test_write_mujoco_scene_with_multiple_elements(self):
        xml_string = """<?xml version="1.0"?>
<scene>
  <light name="light1" diffuse="1 1 1"/>
  <light name="light2" diffuse="0 0 1"/>
  <global><ambient>0.5 0.5 0.5</ambient></global>
</scene>"""
        scene_dom = minidom.parseString(xml_string)
        scene_inputs = scene_dom.getElementsByTagName("scene")[0]

        with tempfile.TemporaryDirectory() as tmpdir:
            write_mujoco_scene(scene_inputs, tmpdir + "/")
            output_file = os.path.join(tmpdir, "scene.xml")
            self.assertTrue(os.path.exists(output_file))
            with open(output_file) as f:
                content = f.read()
            assert "light1" in content
            assert "light2" in content
            assert "global" in content
            # Also check that nothing more exists beyond the expected tags using xml parsing
            dom = minidom.parseString(content)
            lights = dom.getElementsByTagName("light")
            self.assertEqual(len(lights), 2)
            globals_ = dom.getElementsByTagName("global")
            self.assertEqual(len(globals_), 1)
            # Check that the DOMElement only has the expected children
            child_nodes = [node for node in dom.documentElement.childNodes if node.nodeType == node.ELEMENT_NODE]
            self.assertEqual(len(child_nodes), 4)  # 2 lights + 1 global + 1 include
            for node in child_nodes:
                assert node.tagName in ["light", "global", "include"]

    def test_add_urdf_free_joint_basic(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        result = add_urdf_free_joint(urdf)
        assert 'name="virtual_base"' in result
        assert 'name="virtual_base_joint"' in result
        assert 'type="floating"' in result
        assert 'parent link="virtual_base"' in result
        assert 'child link="base_link"' in result

        # check the number of links are equal to 2 (virtual_base + base_link)
        self.assertEqual(result.count("<link"), 2)
        # check the number of joints are equal to 1 (virtual_base_joint)
        self.assertEqual(result.count("<joint"), 1)

    def test_add_urdf_free_joint_world_root(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link2"/>
  </joint>
</robot>"""
        result = add_urdf_free_joint(urdf)
        # Should return the URDF unchanged as the world link is present
        self.assertEqual(result, urdf)

    def test_add_urdf_free_joint_preserves_existing_content(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link2"/>
  </joint>
  <link name="link2"/>
</robot>"""
        result = add_urdf_free_joint(urdf)
        assert 'name="base_link"' in result
        assert 'name="joint1"' in result
        assert 'name="link2"' in result
        assert 'name="virtual_base"' in result
        assert 'name="virtual_base_joint"' in result

        self.assertEqual(result.count("<link"), 3)  # virtual_base + base_link + link2
        self.assertEqual(result.count("<joint"), 2)  # virtual_base_joint + joint1

    def test_add_urdf_free_joint_origin_attributes(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        result = add_urdf_free_joint(urdf)
        assert 'xyz="0 0 0"' in result
        assert 'rpy="0 0 0"' in result

    def test_get_xml_from_file_basic(self):
        urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf_content)
            urdf_path = f.name

        try:
            result = get_xml_from_file(urdf_path)
            assert '<robot name="test_robot">' in result
            assert '<link name="base_link"/>' in result
            self.assertEqual(result, urdf_content)
        finally:
            os.unlink(urdf_path)

    def test_get_xml_from_file_nonexistent(self):
        with self.assertRaises(FileNotFoundError):
            get_xml_from_file("/nonexistent/path/robot.urdf")

    def test_add_modifiers_basic(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base_link">
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        modify_element_dict = {("body", "base_link"): {"pos": "1 2 3", "quat": "0 0 0 1"}}
        result_dom = add_modifiers(dom, modify_element_dict)
        result_xml = result_dom.toxml()
        print(result_xml)
        assert 'name="base_link"' in result_xml
        assert 'pos="1 2 3"' in result_xml
        assert 'quat="0 0 0 1"' in result_xml

        # Make sure the original attributes are preserved
        # check per element attributes to ensure no unintended modifications
        self.assertEqual(len(result_dom.getElementsByTagName("body")), 1)  # only one body element should be present
        self.assertEqual(len(result_dom.getElementsByTagName("geom")), 1)  # only one geom element should be present

        self.assertEqual(result_dom.getElementsByTagName("body")[0].getAttribute("name"), "base_link")
        self.assertEqual(result_dom.getElementsByTagName("body")[0].getAttribute("pos"), "1 2 3")
        self.assertEqual(result_dom.getElementsByTagName("body")[0].getAttribute("quat"), "0 0 0 1")
        self.assertNotEqual(
            result_dom.getElementsByTagName("body")[0].getAttribute("size"), "1 1 1"
        )  # size should not be modified
        # make sure there is no other attribute added to the body element
        self.assertEqual(len(result_dom.getElementsByTagName("body")[0].attributes), 3)  # name, pos, quat

        self.assertEqual(result_dom.getElementsByTagName("geom")[0].getAttribute("type"), "box")
        self.assertEqual(result_dom.getElementsByTagName("geom")[0].getAttribute("size"), "1 1 1")
        self.assertEqual(len(result_dom.getElementsByTagName("geom")[0].attributes), 2)  # type, size

        self.assertEqual(
            len(result_dom.getElementsByTagName("worldbody")[0].attributes), 0
        )  # worldbody should have no attributes

        self.assertEqual(
            len(result_dom.getElementsByTagName("mujoco")[0].attributes), 0
        )  # mujoco should have no attributes

        # Number of child links
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[0])), 1
        )  # body should have one child geom

    def test_add_modifiers_multiple_elements(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="link1">
      <joint name="joint1"/>
    </body>
    <body name="link2">
      <joint name="joint2"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        modify_element_dict = {("body", "link1"): {"pos": "0 0 1"}, ("joint", "joint2"): {"damping": "0.5"}}
        result_dom = add_modifiers(dom, modify_element_dict)
        result_xml = result_dom.toxml()
        assert 'pos="0 0 1"' in result_xml
        assert 'damping="0.5"' in result_xml

        self.assertEqual(len(result_dom.getElementsByTagName("body")), 2)  # two body elements should be present
        self.assertEqual(len(result_dom.getElementsByTagName("joint")), 2)  # two joint elements should be present

        self.assertEqual(result_dom.getElementsByTagName("body")[0].getAttribute("name"), "link1")
        self.assertEqual(result_dom.getElementsByTagName("body")[1].getAttribute("name"), "link2")
        self.assertEqual(result_dom.getElementsByTagName("body")[0].getAttribute("pos"), "0 0 1")

        self.assertEqual(result_dom.getElementsByTagName("joint")[0].getAttribute("name"), "joint1")
        self.assertEqual(result_dom.getElementsByTagName("joint")[1].getAttribute("name"), "joint2")
        self.assertEqual(result_dom.getElementsByTagName("joint")[1].getAttribute("damping"), "0.5")

        # Make sure no unnecessary modifications
        self.assertEqual(len(result_dom.getElementsByTagName("body")[0].attributes), 2)  # name and pos for link1
        self.assertEqual(
            len(result_dom.getElementsByTagName("body")[1].attributes), 1
        )  # only name for link2, no pos added
        self.assertEqual(
            len(result_dom.getElementsByTagName("joint")[0].attributes), 1
        )  # only name for joint1, no damping added
        self.assertEqual(len(result_dom.getElementsByTagName("joint")[1].attributes), 2)  # name and damping for joint2

        self.assertEqual(
            len(result_dom.getElementsByTagName("worldbody")[0].attributes), 0
        )  # worldbody should have no attributes

        self.assertEqual(
            len(result_dom.getElementsByTagName("mujoco")[0].attributes), 0
        )  # mujoco should have no attributes

        # Validate the number of children for each body element
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[0])), 1
        )  # link1 should have one child joint
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[1])), 1
        )  # link2 should have one child joint
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("joint")[0])), 0
        )  # joint1 should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("joint")[1])), 0
        )  # joint2 should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("worldbody")[0])), 2
        )  # worldbody should have two child bodies
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("mujoco")[0])), 1
        )  # mujoco should have one child worldbody

    def test_add_modifiers_overwrite_existing(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base_link" pos="0 0 0">
      <geom type="box"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        modify_element_dict = {("body", "base_link"): {"pos": "1 2 3"}}
        result_dom = add_modifiers(dom, modify_element_dict)
        result_xml = result_dom.toxml()
        assert 'pos="1 2 3"' in result_xml
        assert 'pos="0 0 0"' not in result_xml

    def test_add_modifiers_empty_dict(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base_link"/>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        result_dom = add_modifiers(dom, {})
        result_xml = result_dom.toxml()
        self.assertEqual(result_xml, dom.toxml())

    def test_add_lidar_from_sites_basic(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <site name="lidar_site" pos="0 0 0.5" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        lidar_dict = {}
        lidar_elem = minidom.parseString('<lidar name="lidar" url="file://sensor.xml"/>').documentElement
        lidar_elem.setAttribute("min_angle", "0")
        lidar_dict["lidar_site"] = lidar_elem

        with patch("builtins.print"):
            result_dom = add_lidar_from_sites(dom, lidar_dict)

        result_xml = result_dom.toxml()
        assert 'name="lidar_site_lidar_body"' in result_xml
        assert "<lidar" in result_xml

    def test_add_lidar_from_sites_no_matching_sites(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <site name="camera_site" pos="0 0 0.5" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        lidar_dict = {}
        lidar_elem = minidom.parseString('<lidar name="lidar"/>').documentElement
        lidar_elem.setAttribute("min_angle", "0")
        lidar_dict["other_site"] = lidar_elem

        result_dom = add_lidar_from_sites(dom, lidar_dict)
        result_xml = result_dom.toxml()
        assert "lidar_body" not in result_xml

    def test_add_lidar_from_sites_removes_min_angle(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <site name="lidar_site" pos="0 0 0.5" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        lidar_dict = {}
        lidar_elem = minidom.parseString('<lidar name="lidar" min_angle="0" max_angle="3.14"/>').documentElement
        lidar_elem.setAttribute("min_angle", "0")
        lidar_dict["lidar_site"] = lidar_elem

        with patch("builtins.print"):
            result_dom = add_lidar_from_sites(dom, lidar_dict)

        result_xml = result_dom.toxml()
        assert 'name="lidar"' in result_xml
        assert 'max_angle="3.14"' in result_xml

    def test_add_cameras_from_sites_check_attributes(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <site name="camera_site" pos="0 0 0.5" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        cameras_dict = {}
        camera_elem = minidom.parseString('<camera name="camera" fovy="60"/>').documentElement
        cameras_dict["camera_site"] = camera_elem

        with patch("builtins.print"):
            result_dom = add_cameras_from_sites(dom, cameras_dict)

        result_xml = result_dom.toxml()
        print(result_xml)
        assert 'name="camera"' in result_xml
        assert 'fovy="60"' in result_xml

        # Check the result XML has all the information as XML string
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("camera")[0])), 0
        )  # camera should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("worldbody")[0])), 1
        )  # worldbody should have one child body
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("mujoco")[0])), 1
        )  # mujoco should have one child worldbody
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[0])), 2
        )  # base body should have site and camera body as children

        self.assertEqual(len(result_dom.getElementsByTagName("camera")), 1)  # only one camera element should be present
        self.assertEqual(result_dom.getElementsByTagName("camera")[0].getAttribute("name"), "camera")
        self.assertEqual(result_dom.getElementsByTagName("camera")[0].getAttribute("fovy"), "60")
        self.assertEqual(result_dom.getElementsByTagName("camera")[0].getAttribute("pos"), "0 0 0.5")
        self.assertEqual(result_dom.getElementsByTagName("camera")[0].getAttribute("quat"), "0.0 1.0 0.0 0.0")
        self.assertEqual(
            len(result_dom.getElementsByTagName("worldbody")[0].attributes), 0
        )  # worldbody should have no attributes
        self.assertEqual(
            len(result_dom.getElementsByTagName("mujoco")[0].attributes), 0
        )  # mujoco should have no attributes
        self.assertEqual(
            len(result_dom.getElementsByTagName("body")), 1
        )  # there should still be a single body element (base)
        self.assertEqual(
            sorted(["camera", "site"]), sorted(get_child_elements(result_dom.getElementsByTagName("body")[0]))
        )  # base body should have site and camera body as children

    def test_add_cameras_from_sites_no_matching_sites(self):
        xml_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base">
      <site name="other_site" pos="0 0 0.5" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(xml_string)
        cameras_dict = {}
        camera_elem = minidom.parseString('<camera name="camera"/>').documentElement
        cameras_dict["camera_site"] = camera_elem

        result_dom = add_cameras_from_sites(dom, cameras_dict)
        result_xml = result_dom.toxml()
        assert 'name="camera"' not in result_xml
        self.assertEqual(dom.toxml(), result_dom.toxml())  # DOM should be unchanged

    def test_add_links_as_sites_basic(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link2"/>
  </joint>
  <link name="link2"/>
</robot>"""
        mjcf_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base_link">
      <joint name="joint1" type="revolute"/>
      <geom type="box" size="1 1 1"/>
    </body>
    <body name="link2">
      <geom type="sphere" size="0.5"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(mjcf_string)
        result_dom = add_links_as_sites(urdf, dom, True)
        result_xml = result_dom.toxml()

        sites = result_dom.getElementsByTagName("site")
        site_names = [site.getAttribute("name") for site in sites]
        assert "base_link" in site_names
        assert "link2" in site_names
        print(result_xml)

        # Check that the site elements have all attributes same as mjcf_string but only sites are extra
        self.assertEqual(
            len(result_dom.getElementsByTagName("worldbody")), 1
        )  # only one worldbody element should be present
        self.assertEqual(len(result_dom.getElementsByTagName("mujoco")), 1)  # only one mujoco element should be present
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("mujoco")[0])), 1
        )  # mujoco should have one child worldbody
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("worldbody")[0])), 2
        )  # worldbody should have two child bodies (base_link and link2)
        self.assertEqual(len(result_dom.getElementsByTagName("body")), 2)  # two body elements should be present
        self.assertEqual(
            len(result_dom.getElementsByTagName("geom")), 2
        )  # two geom elements should be present (2 from original)
        self.assertEqual(
            len(result_dom.getElementsByTagName("joint")), 1
        )  # two joint elements should be present (joint1)
        self.assertEqual(
            len(result_dom.getElementsByTagName("site")), 2
        )  # two site elements should be present (one for each link)
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[0])), 3
        )  # base_link body should have joint, geom, and site as children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[1])), 2
        )  # link2 body should have geom and site as children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("joint")[0])), 0
        )  # joint1 should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("geom")[0])), 0
        )  # base_link geom should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("geom")[1])), 0
        )  # link2 geom should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("site")[0])), 0
        )  # base_link site should have no children
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("site")[1])), 0
        )  # link2 site should have no children

    def test_add_links_as_sites_with_world_root(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link1"/>
</robot>"""
        mjcf_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="link1">
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(mjcf_string)
        result_dom = add_links_as_sites(urdf, dom, True)

        sites = result_dom.getElementsByTagName("site")
        site_names = [site.getAttribute("name") for site in sites]
        assert "world" in site_names
        assert "link1" in site_names

    #     def test_add_links_as_sites_urdf_has_more_links_than_mjcf(self):
    #         urdf = """<?xml version="1.0"?>
    # <robot name="test_robot">
    #   <link name="base_link"/>
    #   <joint name="joint1" type="revolute">
    #     <parent link="base_link"/>
    #     <child link="link2"/>
    #   </joint>
    #   <link name="link2"/>
    #   <joint name="joint2" type="revolute">
    #     <parent link="link2"/>
    #     <child link="link3"/>
    #   </joint>
    #   <link name="link3"/>
    #   <joint name="joint3" type="revolute">
    #     <parent link="link3"/>
    #     <child link="link4"/>
    #   </joint>
    #   <link name="link4"/>
    # </robot>"""
    #         mjcf_string = """<?xml version="1.0"?>
    # <mujoco>
    #   <worldbody>
    #     <body name="base_link">
    #       <joint name="joint1" type="revolute"/>
    #       <geom type="box" size="1 1 1"/>
    #     </body>
    #   </worldbody>
    # </mujoco>"""
    #         dom = minidom.parseString(mjcf_string)
    #         result_dom = add_links_as_sites(urdf, dom, True)
    #         result_xml = result_dom.toxml()

    #         sites = result_dom.getElementsByTagName("site")
    #         print(result_xml)
    #         site_names = [site.getAttribute("name") for site in sites]
    #         assert "base_link" in site_names
    #         assert "link2" in site_names
    #         assert "link3" in site_names
    #         assert "link4" in site_names

    def test_add_links_as_sites_no_free_joint(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        mjcf_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base_link">
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(mjcf_string)
        result_dom = add_links_as_sites(urdf, dom, False)

        sites = result_dom.getElementsByTagName("site")
        site_names = [site.getAttribute("name") for site in sites]
        assert "base_link" in site_names

    def test_get_urdf_transforms_single_link(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        result = get_urdf_transforms(urdf)
        assert "base_link" in result
        link, transform, is_root = result["base_link"]
        self.assertEqual(link, "base_link")
        self.assertTrue(is_root)
        self.assertEqual(transform, PyKDL.Frame.Identity())

    def test_get_urdf_transforms_multiple_links(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link2"/>
  </joint>
  <link name="link2"/>
  <joint name="joint2" type="revolute">
    <origin xyz="1 2 3" rpy="0 0 1.57"/>
    <parent link="link2"/>
    <child link="link3"/>
  </joint>
  <link name="link3"/>
</robot>"""
        result = get_urdf_transforms(urdf)
        assert "base_link" in result
        assert "link2" in result
        assert "link3" in result

        link, transform, is_root = result["base_link"]
        self.assertEqual(link, "base_link")
        self.assertTrue(is_root)
        self.assertEqual(transform, PyKDL.Frame.Identity())

        link, transform, is_root = result["link2"]
        self.assertEqual(link, "link2")
        self.assertFalse(is_root)
        # Identity as it is a revolute joint, only works for fixed joints
        self.assertEqual(transform, PyKDL.Frame.Identity())

    def test_get_urdf_transforms_with_fixed_joint(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="link2"/>
    <origin xyz="1 2 3" rpy="0 0 1.57"/>
  </joint>
  <link name="link2"/>
</robot>"""
        result = get_urdf_transforms(urdf)
        assert "base_link" in result
        assert "link2" in result
        link, transform, is_root = result["link2"]
        self.assertEqual(link, "base_link")
        self.assertTrue(is_root)
        expected_transform = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 1.57), PyKDL.Vector(1, 2, 3))
        self.assertEqual(transform, expected_transform)

    def test_get_urdf_transforms_with_world_root(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link"/>
</robot>"""
        result = get_urdf_transforms(urdf)
        assert "world" in result
        assert "base_link" in result

        link, transform, is_root = result["world"]
        self.assertEqual(link, "world")
        self.assertTrue(is_root)
        self.assertEqual(transform, PyKDL.Frame.Identity())

    def test_add_free_joint_with_virtual_base_joint(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        mjcf_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="virtual_base">
      <joint name="virtual_base_joint" type="free"/>
      <body name="base_link">
        <geom type="box" size="1 1 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(mjcf_string)
        result_dom = add_free_joint(dom, urdf)
        result_xml = result_dom.toxml()

        assert "<freejoint" in result_xml
        self.assertNotIn('name="virtual_base_joint"', result_xml)
        self.assertEqual(dom.toxml(), result_dom.toxml())  # DOM should be unchanged

    def test_add_free_joint_world_root(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link"/>
</robot>"""
        mjcf_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="base_link">
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(mjcf_string)
        result_dom = add_free_joint(dom, urdf)

        self.assertEqual(dom.toxml(), result_dom.toxml())  # DOM should be unchanged

    def test_add_free_joint_custom_name(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        mjcf_string = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="virtual_base">
      <joint name="virtual_base_joint" type="free"/>
      <body name="base_link">
        <geom type="box" size="1 1 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""
        dom = minidom.parseString(mjcf_string)
        result_dom = add_free_joint(dom, urdf, joint_name="my_custom_joint")
        result_xml = result_dom.toxml()

        print(result_xml)
        self.assertEqual(
            len(result_dom.getElementsByTagName("freejoint")), 1
        )  # only one freejoint element should be present
        self.assertEqual(result_xml.count("<freejoint"), 1)  # only one freejoint element should be present
        self.assertEqual(result_dom.getElementsByTagName("freejoint")[0].getAttribute("name"), "my_custom_joint")

        # Make sure the other data did not change and only the free joint name is modified
        self.assertEqual(len(result_dom.getElementsByTagName("body")), 2)  # virtual_base and base_link
        self.assertEqual(
            len(result_dom.getElementsByTagName("geom")), 1
        )  # only one geom element should be present (from original)
        self.assertEqual(
            len(result_dom.getElementsByTagName("joint")), 0
        )  # only one joint element should be present (the free joint)
        # Checking the names of the elements
        self.assertEqual(result_dom.getElementsByTagName("body")[0].getAttribute("name"), "virtual_base")
        self.assertEqual(result_dom.getElementsByTagName("body")[1].getAttribute("name"), "base_link")
        self.assertEqual(result_dom.getElementsByTagName("geom")[0].getAttribute("type"), "box")
        self.assertEqual(result_dom.getElementsByTagName("geom")[0].getAttribute("size"), "1 1 1")
        self.assertEqual(
            len(result_dom.getElementsByTagName("worldbody")[0].attributes), 0
        )  # worldbody should have no attributes
        self.assertEqual(
            len(result_dom.getElementsByTagName("mujoco")[0].attributes), 0
        )  # mujoco should have no attributes
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[0])), 2
        )  # virtual_base should have two child elements (freejoint and base_link)
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("body")[1])), 1
        )  # base_link should have geom as child
        self.assertEqual(
            len(get_child_elements(result_dom.getElementsByTagName("freejoint")[0])), 0
        )  # freejoint should have no children

    def test_parse_inputs_xml_none(self):
        result = parse_inputs_xml(None)
        self.assertIsNone(result[0])
        self.assertIsNone(result[1])

    def test_parse_inputs_xml_standalone(self):
        inputs_xml = """<?xml version="1.0"?>
<mujoco_inputs>
  <raw_inputs>
    <option integrator="implicitfast"/>
  </raw_inputs>
  <processed_inputs>
    <decompose_mesh mesh_name="test_mesh"/>
  </processed_inputs>
</mujoco_inputs>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(inputs_xml)
            inputs_path = f.name

        try:
            raw_inputs, processed_inputs = parse_inputs_xml(inputs_path)
            self.assertIsNotNone(raw_inputs)
            self.assertIsNotNone(processed_inputs)
            self.assertEqual(raw_inputs.tagName, "raw_inputs")
            self.assertEqual(raw_inputs.getElementsByTagName("option")[0].getAttribute("integrator"), "implicitfast")
            self.assertEqual(processed_inputs.tagName, "processed_inputs")
        finally:
            os.unlink(inputs_path)

    def test_parse_inputs_xml_in_urdf(self):
        urdf_with_inputs = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <mujoco_inputs>
    <raw_inputs>
      <option integrator="implicitfast"/>
    </raw_inputs>
    <processed_inputs>
      <camera site="camera_site" name="camera" fovy="58"/>
    </processed_inputs>
  </mujoco_inputs>
</robot>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf_with_inputs)
            urdf_path = f.name

        try:
            raw_inputs, processed_inputs = parse_inputs_xml(urdf_path)
            self.assertIsNotNone(raw_inputs)
            self.assertIsNotNone(processed_inputs)
            self.assertEqual(raw_inputs.tagName, "raw_inputs")
            self.assertEqual(
                len(raw_inputs.getElementsByTagName("option")), 1
            )  # only one option element should be present
            self.assertEqual(raw_inputs.getElementsByTagName("option")[0].getAttribute("integrator"), "implicitfast")
            self.assertEqual(processed_inputs.tagName, "processed_inputs")
            self.assertEqual(
                len(processed_inputs.getElementsByTagName("camera")), 1
            )  # only one camera element should be present
            self.assertEqual(processed_inputs.getElementsByTagName("camera")[0].getAttribute("name"), "camera")
        finally:
            os.unlink(urdf_path)

    def test_parse_inputs_xml_invalid_root(self):
        invalid_xml = """<?xml version="1.0"?>
<invalid_root>
  <raw_inputs/>
</invalid_root>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(invalid_xml)
            invalid_path = f.name

        try:
            with self.assertRaises(ValueError) as context:
                parse_inputs_xml(invalid_path)
            assert "Root tag" in str(context.exception)
        finally:
            os.unlink(invalid_path)

    def test_parse_scene_xml_none(self):
        result = parse_scene_xml(None)
        self.assertIsNone(result)

    def test_parse_scene_xml_standalone_mujoco(self):
        scene_xml = """<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <light name="test_light" diffuse="1 1 1"/>
  </worldbody>
</mujoco>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(scene_xml)
            scene_path = f.name

        try:
            result = parse_scene_xml(scene_path)
            self.assertIsNotNone(result)
            self.assertEqual(result.tagName, "mujoco")
            self.assertEqual(len(result.getElementsByTagName("light")), 1)  # only one light element should be present
            self.assertEqual(result.getElementsByTagName("light")[0].getAttribute("name"), "test_light")
            self.assertEqual(result.getElementsByTagName("light")[0].getAttribute("diffuse"), "1 1 1")
            self.assertEqual(
                len(get_child_elements(result.getElementsByTagName("worldbody")[0])), 1
            )  # worldbody should have one child light element
            self.assertEqual(
                len(result.getElementsByTagName("worldbody")), 1
            )  # only one worldbody element should be present
        finally:
            os.unlink(scene_path)

    def test_parse_scene_xml_in_urdf(self):
        urdf_with_scene = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <mujoco_inputs>
    <scene>
      <light name="test_light" diffuse="1 1 1"/>
    </scene>
  </mujoco_inputs>
</robot>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf_with_scene)
            urdf_path = f.name

        try:
            result = parse_scene_xml(urdf_path)
            self.assertIsNotNone(result)
            self.assertEqual(result.tagName, "scene")
            self.assertEqual(len(result.getElementsByTagName("light")), 1)  # only one light element should be present
            self.assertEqual(result.getElementsByTagName("light")[0].getAttribute("name"), "test_light")
            self.assertEqual(result.getElementsByTagName("light")[0].getAttribute("diffuse"), "1 1 1")
            self.assertEqual(len(get_child_elements(result)), 1)  # scene should have one child light element
        finally:
            os.unlink(urdf_path)

    def test_parse_scene_xml_urdf_without_scene(self):
        urdf_without_scene = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf_without_scene)
            urdf_path = f.name

        try:
            result = parse_scene_xml(urdf_path)
            self.assertIsNone(result)
        finally:
            os.unlink(urdf_path)

    def test_parse_scene_xml_invalid_root(self):
        invalid_xml = """<?xml version="1.0"?>
<invalid_root>
  <scene/>
</invalid_root>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(invalid_xml)
            invalid_path = f.name

        try:
            with self.assertRaises(ValueError) as context:
                parse_scene_xml(invalid_path)
            assert "Root tag" in str(context.exception)
        finally:
            os.unlink(invalid_path)

    def test_extract_mesh_info_basic(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test_package/meshes/model.dae"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        result, updated_xml = extract_mesh_info(urdf, None, {})
        self.assertEqual(len(result), 1)
        assert "model" in result
        self.assertEqual(result["model"]["filename"], "package://test_package/meshes/model.dae")
        self.assertEqual(result["model"]["scale"], "1.0 1.0 1.0")
        self.assertEqual(result["model"]["color"], (1.0, 1.0, 1.0, 1.0))
        self.assertFalse(result["model"]["is_pre_generated"])
        assert "package://test_package/meshes/model.dae" in updated_xml

    def test_extract_mesh_info_with_material_color(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <material name="red_material">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test_package/meshes/model.dae"/>
      </geometry>
      <material name="red_material"/>
    </visual>
  </link>
</robot>"""
        result, updated_xml = extract_mesh_info(urdf, None, {})
        self.assertEqual(len(result), 1)
        self.assertEqual(result["model"]["filename"], "package://test_package/meshes/model.dae")
        self.assertEqual(result["model"]["color"], (1.0, 0.0, 0.0, 1.0))
        self.assertEqual(result["model"]["scale"], "1.0 1.0 1.0")
        self.assertFalse(result["model"]["is_pre_generated"])
        assert "package://test_package/meshes/model.dae" in updated_xml

    def test_extract_mesh_info_with_scale(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test_package/meshes/model.stl" scale="2.0 3.0 4.0"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        result, updated_xml = extract_mesh_info(urdf, None, {})
        self.assertEqual(result["model"]["scale"], "2.0 3.0 4.0")
        self.assertFalse(result["model"]["is_pre_generated"])
        self.assertEqual(result["model"]["color"], (1.0, 1.0, 1.0, 1.0))
        self.assertEqual(result["model"]["filename"], "package://test_package/meshes/model.stl")
        assert "package://test_package/meshes/model.stl" in updated_xml

    def test_extract_mesh_info_no_mesh_geometry(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        result, updated_xml = extract_mesh_info(urdf, None, {})
        self.assertEqual(len(result), 0)
        self.assertEqual(updated_xml, urdf)

    def test_extract_mesh_info_with_decompose_dict(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test_package/meshes/complex_mesh.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        with tempfile.TemporaryDirectory() as tmpdir:
            decomposed_dir = os.path.join(tmpdir, DECOMPOSED_PATH_NAME, "complex_mesh", "complex_mesh")
            os.makedirs(decomposed_dir)
            mesh_file = os.path.join(decomposed_dir, "complex_mesh.obj")
            with open(mesh_file, "w") as f:
                f.write("# OBJ file")

            metadata_file = os.path.join(tmpdir, DECOMPOSED_PATH_NAME, "metadata.json")
            with open(metadata_file, "w") as f:
                json.dump({"complex_mesh": "0.05"}, f)

            result, updated_xml = extract_mesh_info(urdf, tmpdir, {"complex_mesh": "0.05"})
            self.assertEqual(len(result), 1)
            assert "complex_mesh" in result
            self.assertTrue(result["complex_mesh"]["is_pre_generated"])
            self.assertEqual(result["complex_mesh"]["scale"], "1.0 1.0 1.0")
            self.assertEqual(result["complex_mesh"]["color"], (1.0, 1.0, 1.0, 1.0))
            assert "complex_mesh.obj" in result["complex_mesh"]["filename"]

            # The updated_xml differs from urdf only for the new path
            reverted_xml = updated_xml.replace(
                result["complex_mesh"]["filename"], "package://test_package/meshes/complex_mesh.stl"
            )
            self.assertEqual(urdf, reverted_xml)

    def test_extract_mesh_different_threshold(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test_package/meshes/complex_mesh.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        with tempfile.TemporaryDirectory() as tmpdir:
            decomposed_dir = os.path.join(tmpdir, DECOMPOSED_PATH_NAME, "complex_mesh", "complex_mesh")
            os.makedirs(decomposed_dir)
            mesh_file = os.path.join(decomposed_dir, "complex_mesh.obj")
            with open(mesh_file, "w") as f:
                f.write("# OBJ file")

            metadata_file = os.path.join(tmpdir, DECOMPOSED_PATH_NAME, "metadata.json")
            with open(metadata_file, "w") as f:
                json.dump({"complex_mesh": "0.05"}, f)

            # The pregenerated object has a different threshold than the one required
            result, updated_xml = extract_mesh_info(urdf, tmpdir, {"complex_mesh": "0.02"})
            self.assertEqual(len(result), 1)
            assert "complex_mesh" in result
            self.assertFalse(result["complex_mesh"]["is_pre_generated"])
            self.assertEqual(result["complex_mesh"]["scale"], "1.0 1.0 1.0")
            self.assertEqual(result["complex_mesh"]["color"], (1.0, 1.0, 1.0, 1.0))
            assert "complex_mesh.stl" in result["complex_mesh"]["filename"]

            self.assertEqual(urdf, updated_xml)

    def test_extract_mesh_with_compose_dict(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test_package/meshes/complex_mesh.stl"/>
      </geometry>
    </visual>
  </link>
</robot>"""
        with tempfile.TemporaryDirectory() as tmpdir:
            composed_dir = os.path.join(tmpdir, COMPOSED_PATH_NAME, "complex_mesh")
            os.makedirs(composed_dir)
            mesh_file = os.path.join(composed_dir, "complex_mesh.obj")
            with open(mesh_file, "w") as f:
                f.write("# OBJ file")

            result, updated_xml = extract_mesh_info(urdf, tmpdir, {})
            self.assertEqual(len(result), 1)
            assert "complex_mesh" in result
            self.assertTrue(result["complex_mesh"]["is_pre_generated"])
            self.assertEqual(result["complex_mesh"]["scale"], "1.0 1.0 1.0")
            self.assertEqual(result["complex_mesh"]["color"], (1.0, 1.0, 1.0, 1.0))
            assert "complex_mesh.obj" in result["complex_mesh"]["filename"]

            # The updated_xml differs from urdf only for the new path
            reverted_xml = updated_xml.replace(
                result["complex_mesh"]["filename"], "package://test_package/meshes/complex_mesh.stl"
            )
            self.assertEqual(urdf, reverted_xml)

    def test_copy_pre_generated_meshes_decomposed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            source_dir = os.path.join(tmpdir, "source")
            output_dir = os.path.join(tmpdir, "output") + "/"
            os.makedirs(source_dir)

            mesh_source_dir = os.path.join(source_dir, "mesh_name", "mesh_name")
            os.makedirs(mesh_source_dir)
            mesh_file = os.path.join(mesh_source_dir, "mesh_name.obj")
            with open(mesh_file, "w") as f:
                f.write("# OBJ content")

            mesh_info_dict = {
                "mesh_name": {
                    "is_pre_generated": True,
                    "filename": os.path.join(source_dir, "mesh_name", "mesh_name", "mesh_name.obj"),
                    "scale": "1.0 1.0 1.0",
                    "color": (1.0, 1.0, 1.0, 1.0),
                }
            }
            decompose_dict = {"mesh_name": "0.05"}

            copy_pre_generated_meshes(output_dir, mesh_info_dict, decompose_dict)

            expected_dst = os.path.join(
                output_dir, "assets", DECOMPOSED_PATH_NAME, "mesh_name", "mesh_name", "mesh_name.obj"
            )
            self.assertTrue(os.path.exists(expected_dst))
            with open(expected_dst) as f:
                self.assertEqual(f.read(), "# OBJ content")

    def test_copy_pre_generated_meshes_composed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            source_dir = os.path.join(tmpdir, "source")
            output_dir = os.path.join(tmpdir, "output") + "/"
            os.makedirs(source_dir)

            mesh_source_dir = os.path.join(source_dir, "mesh_name")
            os.makedirs(mesh_source_dir)
            mesh_file = os.path.join(mesh_source_dir, "mesh_name.obj")
            with open(mesh_file, "w") as f:
                f.write("# COMPOSED OBJ content")

            mesh_info_dict = {
                "mesh_name": {
                    "is_pre_generated": True,
                    "filename": os.path.join(source_dir, "mesh_name", "mesh_name.obj"),
                    "scale": "1.0 1.0 1.0",
                    "color": (1.0, 1.0, 1.0, 1.0),
                }
            }
            decompose_dict = {}

            copy_pre_generated_meshes(output_dir, mesh_info_dict, decompose_dict)

            expected_dst = os.path.join(output_dir, "assets", COMPOSED_PATH_NAME, "mesh_name", "mesh_name.obj")
            self.assertTrue(os.path.exists(expected_dst))
            with open(expected_dst) as f:
                self.assertEqual(f.read(), "# COMPOSED OBJ content")

    def test_copy_pre_generated_meshes_not_pre_generated(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = os.path.join(tmpdir, "output")

            mesh_info_dict = {
                "mesh_name": {
                    "is_pre_generated": False,
                    "filename": "/some/path/mesh.obj",
                    "scale": "1.0 1.0 1.0",
                    "color": (1.0, 1.0, 1.0, 1.0),
                }
            }
            decompose_dict = {}

            copy_pre_generated_meshes(output_dir, mesh_info_dict, decompose_dict)

            assets_dir = os.path.join(output_dir, "assets")
            self.assertFalse(os.path.exists(assets_dir))


if __name__ == "__main__":
    unittest.main()
