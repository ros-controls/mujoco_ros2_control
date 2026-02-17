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
        q1 = [math.sqrt(2)/2, math.sqrt(2)/2, 0.0, 0.0]
        q2 = [math.sqrt(2)/2, 0.0, math.sqrt(2)/2, 0.0]
        result = multiply_quaternion(q1, q2)
        w, x, y, z = result
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        self.assertAlmostEqual(norm, 1.0, places=6)

    def test_euler_to_quaternion_identity(self):
        result = euler_to_quaternion(0.0, 0.0, 0.0)
        self.assertAlmostEqual(result[0], 1.0, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_euler_to_quaternion_90deg_roll(self):
        result = euler_to_quaternion(math.pi/2, 0.0, 0.0)
        self.assertAlmostEqual(result[0], math.sqrt(2)/2, places=6)
        self.assertAlmostEqual(result[1], math.sqrt(2)/2, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_euler_to_quaternion_90deg_pitch(self):
        result = euler_to_quaternion(0.0, math.pi/2, 0.0)
        self.assertAlmostEqual(result[0], math.sqrt(2)/2, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], math.sqrt(2)/2, places=6)
        self.assertAlmostEqual(result[3], 0.0, places=6)

    def test_euler_to_quaternion_90deg_yaw(self):
        result = euler_to_quaternion(0.0, 0.0, math.pi/2)
        self.assertAlmostEqual(result[0], math.sqrt(2)/2, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)
        self.assertAlmostEqual(result[3], math.sqrt(2)/2, places=6)

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

        with tempfile.NamedTemporaryFile(mode='w', suffix='.dae', delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = get_images_from_dae(dae_path)
            self.assertEqual(len(result), 1)
            self.assertTrue(result[0].endswith('image.png'))
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

        with tempfile.NamedTemporaryFile(mode='w', suffix='.dae', delete=False) as f:
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

        with tempfile.NamedTemporaryFile(mode='w', suffix='.dae', delete=False) as f:
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

        with tempfile.NamedTemporaryFile(mode='w', suffix='.dae', delete=False) as f:
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
                'material_0.png',
                'material_1.jpg',
                'material_2.jpeg',
                'other_file.png',
            ]

            for fname in test_files:
                filepath = os.path.join(tmpdir, fname)
                with open(filepath, 'w') as f:
                    f.write('test')

            rename_material_textures(tmpdir, 'test_modifier')

            expected = [
                'material_test_modifier_0.png',
                'material_test_modifier_1.jpg',
                'material_test_modifier_2.jpeg',
                'other_file.png',
            ]

            for fname in expected:
                filepath = os.path.join(tmpdir, fname)
                self.assertTrue(os.path.exists(filepath), f"Expected {fname} to exist")

    def test_rename_material_textures_no_match(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, 'other_file.png')
            with open(filepath, 'w') as f:
                f.write('test')

            rename_material_textures(tmpdir, 'modifier')

            self.assertTrue(os.path.exists(os.path.join(tmpdir, 'other_file.png')))

    def test_set_up_axis_to_z_up_create(self):
        dae_content = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_visual_scenes>
  </library_visual_scenes>
</COLLADA>"""

        with tempfile.NamedTemporaryFile(mode='w', suffix='.dae', delete=False) as f:
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

        with tempfile.NamedTemporaryFile(mode='w', suffix='.dae', delete=False) as f:
            f.write(dae_content)
            dae_path = f.name

        try:
            result = set_up_axis_to_z_up(dae_path)
            self.assertIn("Z_UP", result)
        finally:
            os.unlink(dae_path)


if __name__ == '__main__':
    unittest.main()
