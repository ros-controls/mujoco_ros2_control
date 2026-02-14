# URDF to MJCF Conversion


> [!WARNING]
> This tool is hacky and _highly_ experimental!
> Expect things to be broken.

As MuJoCo does not ingest URDFs, we have written a helper tool for converting URDF to MJCF to assist with converting a robot description to an MJCF.
This can either be done offline or at runtime, refer to [demo 2](../../mujoco_ros2_control_demos/launch/02_mjcf_generation.launch.py) for an example.

As noted in the warning above, but reiterating here, these tools are highly experimental!
They are intended to be used for assistance and getting started, but do not expect things to work for all possible inputs, nor to work immediately out of the box.

Additional cleanup, documentation, and tips and tricks are a work in progress.

## Usage

The current tool that is available is `make_mjcf_from_robot_description`, which is runnable with:

```bash
ros2 run mujoco_ros2_control make_mjcf_from_robot_description.py
```

(or)

```bash
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh
```

When `robot_description_to_mjcf.sh` is first executed, it creates a Python virtual environment at `$ROS_HOME/ros2_control` and installs all necessary dependencies. Once set up, the script sources the environment and runs `make_mjcf_from_robot_description.py`. On subsequent runs, it reuses the existing virtual environment.

By default, the tool will pull a URDF from the `/robot_description` topic.
However, this is configurable at execution time.
A complete list of options is available from the argument parser:

```bash
$ ros2 run mujoco_ros2_control make_mjcf_from_robot_description.py --help
usage: make_mjcf_from_robot_description.py [-h] [-u URDF] [-r ROBOT_DESCRIPTION] [-m MUJOCO_INPUTS] [-o OUTPUT] [-p PUBLISH_TOPIC] [-c] [-s] [-f] [-a ASSET_DIR] [--scene SCENE]

Convert a full URDF to MJCF for use in MuJoCo

options:
  -h, --help            show this help message and exit
  -u URDF, --urdf URDF  Optionally pass an existing URDF file
  -r ROBOT_DESCRIPTION, --robot_description ROBOT_DESCRIPTION
                        Optionally pass the robot description string
  -m MUJOCO_INPUTS, --mujoco_inputs MUJOCO_INPUTS
                        Optionally specify a defaults xml for default settings, actuators, options, and additional sensors
  -o OUTPUT, --output OUTPUT
                        Generated output path
  -p PUBLISH_TOPIC, --publish_topic PUBLISH_TOPIC
                        Optionally specify the topic to publish the MuJoCo model
  -c, --convert_stl_to_obj
                        If we should convert .stls to .objs
  -s, --save_only       Save files permanently on disk; without this flag, files go to a temporary directory
  -f, --add_free_joint  Adds a free joint before the root link of the robot in the urdf before conversion
  -a ASSET_DIR, --asset_dir ASSET_DIR
                        Optionally pass an existing folder with pre-generated OBJ meshes.
  --scene SCENE         Optionally pass an existing xml for the scene
```

A sample URDF and inputs file are provided in [test_robot.urdf](../../mujoco_ros2_control_tests/test/test_resources/test_robot.urdf) and [test_inputs.xml](../../mujoco_ros2_control_tests/test/test_resources/test_inputs.xml).

<!-- TODO: Updates test paths -->

To convert the URDF, run the following from the repo root

```bash
# Dependencies are installed on the fly, if needed
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh \
  --save_only \
  -u mujoco_ros2_control_demos/demo_resources/test_robot.urdf \
  -m mujoco_ros2_control_demos/demo_resources/test_inputs.xml \
  -o /tmp/output/
```

The `/tmp/output/` directory will contain all necessary assets and MJCF files that can be copied into the relevant locations in a config package.
They can also be adjusted as needed after the fact.

```bash
/opt/ros/${ROS_DISTRO}/opt/mujoco_vendor/bin/simulate /tmp/output/mujoco_description_formatted.xml
```

Of note, the test robot has a good chunk of supported functionality, and we recommend using it as a guide.

> [!NOTE]
> The `make_mjcf_from_robot_description.py` script requires `trimesh`, `mujoco`, and `obj2mjcf`. These must either be installed system-wide or available within a virtual environment that is sourced before running the command.

## Notes

> [!NOTE]
>  This has some heavy non-ROS dependencies that could probably be cleaned up:

* MuJoCo Python API
* trimesh - Python library for loading and using triangular meshes.
* obj2mjcf - A tool for converting Wavefront OBJ files to multiple MuJoCo meshes grouped by material.
* xml.dom (not sure if this is already available)

A rough outline of the automated process to convert a URDF:

* reads a robot descriptiong URDF
* add in MuJoCo tag that provides necessary info for conversion
* replace package names from `package://` to absolute filepaths
* read absolute filepaths of all meshes and convert either dae or stl to obj using trimesh
  * put all of these meshes into an `assets/` folder under `mjcf_data/` relative to current working dir
  * modify filepaths again in urdf to point to `assets/` folder
  * decomposes large meshes into multiple components to ensure convex hulls
* publish the new formatted robot description xml file that can be used for conversion
* convert the new robot description urdf file
* run the MuJoCo conversion tool to get the mjcf version
* copy in a default scene.xml file which gives some better camera and scene info
* add remaining sites and items and any other custom inputs
