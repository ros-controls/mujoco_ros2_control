# Mujoco To ROS Tools

> **WARNING**: These tools are highly experimental. Expect things to be broken.

As Mujoco does not ingest URDFs, we have written a helper tool for converting URDF to MJCF to assist with converting a robot description to an MJCF.

As noted in the warning above, but reiterating here, these tools are highly experimental.
They are intended to be used for assistance and getting started, but do not expect things to work for all possible inputs, nor to work immediately out of the box.

## Usage

The current tool that is available is `make_mjcf_from_robot_description`, which is runnable with:

```bash
ros2 run mujoco_ros2_simulation make_mjcf_from_robot_description.py
```

By default, the tool will pull a URDF from the `/robot_description` topic.
However, this is configurable at execution time.
A complete list of options is available from the argument parser:

```bash
$ ros2 run mujoco_ros2_simulation make_mjcf_from_robot_description.py --help
usage: make_mjcf_from_robot_description.py [-h] [-u URDF] [-r ROBOT_DESCRIPTION] [-m MUJOCO_INPUTS] [-o OUTPUT] [-c] [-f]

Convert a full URDF to MJCF for use in Mujoco

options:
  -h, --help            show this help message and exit
  -u URDF, --urdf URDF  Optionally pass an existing URDF file
  -r ROBOT_DESCRIPTION, --robot_description ROBOT_DESCRIPTION
                        Optionally pass the robot description string
  -m MUJOCO_INPUTS, --mujoco_inputs MUJOCO_INPUTS
                        Optionally specify a defaults xml for default settings, actuators, options, and additional sensors
  -o OUTPUT, --output OUTPUT
                        Generated output path
  -c, --convert_stl_to_obj
                        If we should convert .stls to .objs
  -f, --add_free_joint  Adds a free joint as the base link for mobile robots
```

A sample URDF and inputs file is provided in [test_robot.urdf](test/test_robot.urdf).
To convert the URDF, run the following from the repo root

```bash
ros2 run mujoco_ros2_simulation make_mjcf_from_robot_description.py -u test/resources/test_robot.urdf  -m test/resources/test_inputs.xml -o /tmp/output/
```

The `/tmp/output/` directory will contain all necessary assets and MJCF files that can be copied into the relevant locations in a config package.
They can also be adjusted as needed after the fact.

```bash
"${MUJOCO_DIR}"/bin/simulate /tmp/output/scene.xml
```

## Conversion of CKR

We generally recommend using the `view_robot.launch.py` from description packages to run a conversion for one of our robots.
Though it can be as long as there is an active `/robot_description` topic with a fully processed URDF.
For example, to convert the ChonkUR robot, we can use the inputs from the `clr_mujoco_config` package:

```bash
# Publish the robot description
ros2 launch clr_description view_robot.launch.py

# Then process the urdf, note you MUST break down the .objs by passing `-c`.
ros2 run mujoco_ros2_simulation make_mjcf_from_robot_description.py -c -m $(ros2 pkg prefix clr_mujoco_config --share)/description/mujoco_inputs.xml -o /tmp/clr_output/
```

Like above, this will dump all necessary data to the `/tmp/clr_output/` directory, which can then be copied or modified as needed.

To view the results of the conversion, you can use Mujoco's `simulate` tool.
This will bringup the standard Mujoco user panel with the converted Chonkur:

```bash
"${MUJOCO_DIR}"/bin/simulate /tmp/clr_output/scene.xml
```

## Notes

**NOTE** This has some heave non-ROS dependencies that could probably be cleaned up:

* bpy - blender python api
* mujoco python api
* xml.etree (not sure if this is already available)
* xml.dom (not sure if this is already available)

A rough outline of the automated process to convert a URDF:

* reads a robot descriptiong URDF
* add in mujoco tag that provides necessary info for conversion
* replace package names from `package://` to absolute filepaths
* read absolute filepaths of all meshes and convert either dae or stl to obj using blender python api
  * put all of these meshes into an `assets/` folder under `mjcf_data/` relative to current working dir
  * modify filepaths again in urdf to point to `assets/` folder
  * (NOTE) blender dae conversion is kind of broken, need to get this to do better orientation
* publish the new formatted robot description xml file that can be used for conversion
* convert the new robot description urdf file
* run the mujoco conversion tool to get the mjcf version
* copy in a default scene.xml file which gives some better camera and scene info
* add remaining sites and items
