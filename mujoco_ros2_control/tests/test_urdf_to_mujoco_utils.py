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

from xml.dom import minidom

from mujoco_ros2_control import (
    add_mujoco_info,
    remove_tag,
    get_images_from_dae,
    rename_material_textures,
    set_up_axis_to_z_up,
    multiply_quaternion,
    euler_to_quaternion,
    update_non_obj_assets,
    add_mujoco_inputs,
    get_processed_mujoco_inputs,
    DECOMPOSED_PATH_NAME,
    COMPOSED_PATH_NAME,
    write_mujoco_scene,
    add_urdf_free_joint,
    get_xml_from_file,
)


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
        self.assertNotIn("tag_to_remove", result)
        self.assertIn("<other>value</other>", result)

    def test_remove_tag_nonexistent(self):
        xml_string = "<root><other>value</other></root>"
        result = remove_tag(xml_string, "nonexistent")
        self.assertIn("<other>value</other>", result)

    def test_remove_tag_multiple(self):
        xml_string = "<root><tag>first</tag><other>value</other><tag>second</tag></root>"
        result = remove_tag(xml_string, "tag")
        self.assertNotIn("<tag>", result)
        self.assertIn("<other>value</other>", result)

    def test_add_mujoco_info_basic(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = True

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic)

        self.assertIn("<mujoco>", result)
        self.assertIn("<compiler", result)
        self.assertIn('assetdir="', result)
        self.assertIn('balanceinertia="true"', result)
        self.assertIn("<robot", result)

    def test_add_mujoco_info_no_publish_topic(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = False

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic)

        self.assertIn('assetdir="assets"', result)

    def test_add_mujoco_info_with_publish_topic(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = True

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic)

        self.assertIn('assetdir="/tmp/test/assets"', result)

    def test_add_mujoco_info_no_fuse(self):
        raw_xml = '<?xml version="1.0"?><robot name="test"></robot>'
        output_filepath = "/tmp/test/"
        publish_topic = True
        fuse = False

        result = add_mujoco_info(raw_xml, output_filepath, publish_topic, fuse=fuse)

        self.assertIn('fusestatic="false"', result)

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
            self.assertIn("<up_axis>", result)
            self.assertIn("Z_UP", result)
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
            self.assertIn("Z_UP", result)
        finally:
            os.unlink(dae_path)


    def test_update_obj_assets_no_assets(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody><body name="test"/></worldbody></mujoco>'
        dom = minidom.parseString(xml_string)
        result_dom = update_obj_assets(dom, "/tmp/output/", {})
        self.assertIsNotNone(result_dom)

    def test_update_obj_assets_no_matching_dirs(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            os.makedirs(os.path.join(tmpdir, "assets", DECOMPOSED_PATH_NAME))
            os.makedirs(os.path.join(tmpdir, "assets", COMPOSED_PATH_NAME))
            xml_string = '<?xml version="1.0"?><mujoco><asset><mesh name="unknown_mesh" file="test.obj"/></asset><worldbody><body name="test"><geom mesh="unknown_mesh" pos="0 0 0" quat="1 0 0 0"/></body></worldbody></mujoco>'
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
        self.assertIn('class="collision"', result_xml)
        self.assertIn('class="visual"', result_xml)
        self.assertNotIn("contype", result_xml)

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
        self.assertIn("<geom", result_xml)

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
        self.assertIn("integrator", result_xml)

    def test_add_mujoco_inputs_with_scene_inputs(self):
        xml_string = '<?xml version="1.0"?><mujoco><worldbody/></mujoco>'
        scene_xml = '<?xml version="1.0"?><scene><light name="test" diffuse="1 1 1"/></scene>'
        scene_dom = minidom.parseString(scene_xml)
        scene_inputs = scene_dom.getElementsByTagName("scene")[0]

        dom = minidom.parseString(xml_string)
        result_dom = add_mujoco_inputs(dom, None, scene_inputs)
        result_xml = result_dom.toxml()
        self.assertIn("light", result_xml)

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
        self.assertIn("integrator", result_xml)
        self.assertIn("light", result_xml)

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
        self.assertIn("test_mesh", decompose_dict)
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
        self.assertIn("camera_site", cameras_dict)
        self.assertEqual(cameras_dict["camera_site"].getAttribute("name"), "test_camera")

    def test_get_processed_mujoco_inputs_lidar(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <lidar ref_site="lidar_site" sensor_name="rf" min_angle="0" max_angle="1.57" angle_increment="0.025"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        self.assertIn("lidar_site", lidar_dict)

    def test_get_processed_mujoco_inputs_modify_element(self):
        xml_string = """<?xml version="1.0"?>
<processed_inputs>
  <modify_element name="test_body" type="body" pos="1 2 3"/>
</processed_inputs>"""
        dom = minidom.parseString(xml_string)
        processed_element = dom.getElementsByTagName("processed_inputs")[0]

        decompose_dict, cameras_dict, modify_element_dict, lidar_dict = get_processed_mujoco_inputs(processed_element)
        key = ("body", "test_body")
        self.assertIn(key, modify_element_dict)
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
        self.assertIn("'name' and 'type'", str(context.exception))

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
        self.assertIn("mesh1", decompose_dict)
        self.assertIn("mesh2", decompose_dict)
        self.assertIn("site1", cameras_dict)
        self.assertIn(("body", "body1"), modify_element_dict)

    def test_write_mujoco_scene_none(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            write_mujoco_scene(None, tmpdir + "/")
            output_file = os.path.join(tmpdir, "scene.xml")
            self.assertTrue(os.path.exists(output_file))
            with open(output_file) as f:
                content = f.read()
            self.assertIn("<mujoco", content)
            self.assertIn('model="scene"', content)
            self.assertIn("<include", content)

            dom = minidom.parseString(content)
            self.assertEqual(dom.documentElement.tagName, "mujoco")
            self.assertEqual(dom.documentElement.getAttribute("model"), "scene")
            include_tags = dom.getElementsByTagName("include")
            self.assertEqual(len(include_tags), 1)
            self.assertIn("file", include_tags[0].attributes)
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
            self.assertIn("<mujoco", content)
            self.assertIn("<include", content)
            self.assertIn("light", content)
            self.assertIn('name="test_light"', content)

            dom = minidom.parseString(content)
            lights = dom.getElementsByTagName("light")
            self.assertEqual(len(lights), 1)
            self.assertEqual(lights[0].getAttribute("name"), "test_light")
            self.assertEqual(lights[0].getAttribute("diffuse"), "1 1 1")
            child_nodes = [node for node in dom.documentElement.childNodes if node.nodeType == node.ELEMENT_NODE]
            self.assertEqual(len(child_nodes), 2)  # 1 light + 1 include
            for node in child_nodes:
                self.assertIn(node.tagName, ["light", "include"])

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
            self.assertIn("light", content)
            self.assertIn('name="scene_light"', content)
            self.assertIn('diffuse="0.5 0.5 0.5"', content)

            dom = minidom.parseString(content)
            lights = dom.getElementsByTagName("light")
            self.assertEqual(len(lights), 1)
            self.assertEqual(lights[0].getAttribute("name"), "scene_light")
            self.assertEqual(lights[0].getAttribute("diffuse"), "0.5 0.5 0.5")
            child_nodes = [node for node in dom.documentElement.childNodes if node.nodeType == node.ELEMENT_NODE]
            self.assertEqual(len(child_nodes), 2)  # 1 light + 1 include
            for node in child_nodes:
                self.assertIn(node.tagName, ["light", "include"])

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
            self.assertIn("light1", content)
            self.assertIn("light2", content)
            self.assertIn("global", content)
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
                self.assertIn(node.tagName, ["light", "global", "include"])

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
        self.assertIn('name="virtual_base"', result)
        self.assertIn('name="virtual_base_joint"', result)
        self.assertIn('type="floating"', result)
        self.assertIn('parent link="virtual_base"', result)
        self.assertIn('child link="base_link"', result)

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
        self.assertIn('name="base_link"', result)
        self.assertIn('name="joint1"', result)
        self.assertIn('name="link2"', result)
        self.assertIn('name="virtual_base"', result)
        self.assertIn('name="virtual_base_joint"', result)

        self.assertEqual(result.count("<link"), 3)  # virtual_base + base_link + link2
        self.assertEqual(result.count("<joint"), 2)  # virtual_base_joint + joint1

    def test_add_urdf_free_joint_origin_attributes(self):
        urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>"""
        result = add_urdf_free_joint(urdf)
        self.assertIn('xyz="0 0 0"', result)
        self.assertIn('rpy="0 0 0"', result)

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
            self.assertIn('<robot name="test_robot">', result)
            self.assertIn('<link name="base_link"/>', result)
            self.assertEqual(result, urdf_content)
        finally:
            os.unlink(urdf_path)

    def test_get_xml_from_file_nonexistent(self):
        with self.assertRaises(FileNotFoundError):
            get_xml_from_file("/nonexistent/path/robot.urdf")


if __name__ == "__main__":
    unittest.main()
