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
$ ros2 run  mujoco_ros2_control make_mjcf_from_robot_description.py -h
usage: make_mjcf_from_robot_description.py [-h] [-u URDF] [-r ROBOT_DESCRIPTION] [-m MUJOCO_INPUTS] [-o OUTPUT] [-p PUBLISH_TOPIC] [-c] [-s]
                                           [-f] [--fuse | --no-fuse] [-a ASSET_DIR] [--scene SCENE]

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
  --fuse, --no-fuse     Allows MuJoCo to merge static bodies. Use --no-fuse to prevent merging (Default: --fuse).
  -a ASSET_DIR, --asset_dir ASSET_DIR
                        Optionally pass an existing folder with pre-generated OBJ meshes.
  --scene SCENE         Optionally pass an existing xml for the scene
```

A sample URDF and inputs file are provided in [test_robot.urdf](../../mujoco_ros2_control_demos/demo_resources/test_robot.urdf) and [test_inputs.xml](../../mujoco_ros2_control_demos/demo_resources/test_inputs.xml).

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

### A rough outline of the automated process to convert a URDF:

* reads a robot descriptiong URDF
* add in mujoco tag that provides necessary info for conversion
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

## Embedding MuJoCo inputs inside URDF (schema)

You can embed MuJoCo-specific information directly inside a URDF (typically inside a xacro)
so the URDF -> MJCF conversion script can pick it up and inject the corresponding tags into
the generated MJCF. See [mujoco_ros2_control_demos/demo_resources/test_robot.urdf](../../mujoco_ros2_control_demos/demo_resources/test_robot.urdf) for a complete example.

### Top-level container
- Use a `<mujoco_inputs>` element inside your xacro/URDF. The converter looks for this element
  and copies or processes its children into the MJCF.

#### Main sub-elements
- `raw_inputs`: arbitrary MJCF XML fragments that will get copied into the generated
  MJCF. Use this for elements that don't require conversion (for example `option`, `default`,
  `actuator`, `tendon`, `equality`, simple `sensor` definitions, etc.). See `test_robot.urdf`.

- `processed_inputs`: convenience tags that the converter understands and processes into valid
  MJCF entries. Use these when the converter must transform (or) generate MJCF elements (for
  example, cameras, lidar rangefinders, mesh decomposition hints, or targeted modifications).
  Common processed tags (supported by the demo converter):
  - `decompose_mesh` (attributes: `mesh_name`, `threshold`) — request mesh decomposition when
    running `obj2mjcf`/decompose step; useful when large meshes must be split for more robust
    collision hull generation.
  - `camera` (attributes: `site`, `name`, `fovy`, `mode`, `resolution`) — instructs the
    converter to add a camera in the MJCF attached to the given URDF site (frame). The
    converter will fill position/quaternion from the URDF link pose. `resolution` is two
    integers separated by a space (e.g. `640 480`). The `name` must match the `sensor` name
    declared in the `ros2_control` block if you plan to publish images to ROS topics.
  - `lidar` (attributes: `ref_site`, `sensor_name`, `min_angle`, `max_angle`, `angle_increment`)
    — generates a set of MJCF `rangefinder` sensors placed around `ref_site`. The converter
    rotates rangefinders about the replicate frame's Y axis between `min_angle` and
    `max_angle` at the step size `angle_increment`. The generated rangefinders will be named
    by the `sensor_name` base (e.g. `rf-01`, `rf-02`, ...); ROS-facing `sensor` entries in the
    `ros2_control` section should use the same base name.
  - `modify_element` (attributes: `type`, `name`, ...any MJCF attributes...) — finds the
    generated MJCF element by `type` (for example `joint` or `body`) and `name` and set or
    overwrite the provided attributes. This is useful to tweak physics properties like
    `frictionloss`, `damping`, `gravcomp`, etc.

- `scene`: scene-level MJCF fragments such as `asset`, `worldbody`, `visual` and small scene
  parameters. If present the converter will merge/insert it into the MJCF scene (camera
  lighting, ground textures, skybox definitions, etc.). Example: the `scene` block in
  `test_robot.urdf` adds a `groundplane` texture and a light in the MJCF. On the other hand, `scene.xml` can be parsed to the script using `--scene` arg to the script, inorder to generate the model including the scene configuration.

### Sensor and ROS mapping notes
- Define ROS-facing sensors inside the `ros2_control` tag (or anywhere in the URDF that your
  robot description consumers expect). For cameras the `sensor` name in the URDF must match the
  `camera` `name` you placed in `processed_inputs` so the plugin can map the MJCF camera to a
  ROS topic (see `test_robot.urdf`).
- Lidar: the converter produces multiple MJCF `rangefinder` sensors. The URDF `sensor` for
  the lidar should provide `angle_increment`, `min_angle`, `max_angle`, `range_min`, `range_max` and `laserscan_topic` parameters. The hardware interface will combine the set of generated
  rangefinders into a single ROS `LaserScan` message.

### Practical tips and conventions
- Use URDF/xacro frames (links) as reference `site` locations for cameras and sensors; the
  converter will read the link pose and attach MJCF objects accordingly.
- Keep `raw_inputs` minimal and prefer `processed_inputs` for things that need conversion or
  generation (meshes, cameras, rangefinders) — processed inputs document intent and are
  easier to maintain than dropping raw MJCF fragments into the URDF.
- When matching sensors: make sure MJCF sensor names and URDF `sensor` names align — this is
  how runtime mapping and topics are resolved.

**Where to look**
- Example usage in repo: [mujoco_ros2_control_demos/demo_resources/test_robot.urdf](../../mujoco_ros2_control_demos/demo_resources/test_robot.urdf).
- The conversion inputs file used by the demo: [mujoco_ros2_control_demos/demo_resources/mjcf_generation/test_inputs.xml](../../mujoco_ros2_control_demos/demo_resources/mjcf_generation/test_inputs.xml).


### Minimal example (camera + lidar)

Insert the following snippet inside a xacro/URDF where you want to describe MuJoCo inputs. It
demonstrates a small `mujoco_inputs` block plus the matching `ros2_control` sensor entries that
map MJCF sensors to ROS topics.

```xml
  <!-- MuJoCo inputs embedded in URDF/xacro -->
  <mujoco_inputs>
    <raw_inputs>
      <!-- simple actuator copied verbatim into MJCF -->
      <actuator>
        <position name="joint1" joint="joint1" kp="1000"/>
      </actuator>
    </raw_inputs>

    <processed_inputs>
      <!-- add a camera attached to the URDF frame 'camera_frame' -->
      <camera site="camera_frame" name="camera" fovy="58" mode="fixed" resolution="640 480"/>

      <!-- generate a set of rangefinders attached to 'lidar_frame' -->
      <lidar ref_site="lidar_frame" sensor_name="rf" min_angle="-0.3" max_angle="0.3" angle_increment="0.025"/>
    </processed_inputs>
  </mujoco_inputs>

  <!-- ROS-facing sensor mappings (example inside ros2_control or globally) -->
  <ros2_control name="MujocoSystem" type="system">
    <!-- camera sensor: name must match processed_inputs camera 'name' -->
    <sensor name="camera">
      <param name="frame_name">camera_color_mujoco_frame</param>
      <param name="image_topic">/camera/color/image_raw</param>
      <param name="info_topic">/camera/color/camera_info</param>
    </sensor>

    <!-- lidar sensor: base name must match processed_inputs 'sensor_name' -->
    <sensor name="lidar">
      <param name="frame_name">lidar_frame</param>
      <param name="angle_increment">0.025</param>
      <param name="min_angle">-0.3</param>
      <param name="max_angle">0.3</param>
      <param name="range_min">0.05</param>
      <param name="range_max">10.0</param>
      <param name="laserscan_topic">/scan</param>
    </sensor>
  </ros2_control>
```

### Processed inputs attribute reference

The following lists the supported `processed_inputs` tags and their attributes. These are the
attributes the demo converter recognizes; converters may extend this list.

- `decompose_mesh`
  - Required: `mesh_name` (string)
  - Optional: `threshold` (float) — convex decomposition threshold; smaller values produce
    finer decomposition. Example: `<decompose_mesh mesh_name="shoulder_link" threshold="0.05"/>`.

- `camera`
  - Required: `site` (string) — URDF link/frame name to attach the camera to.
  - Required: `name` (string) — camera name in MJCF; must match URDF `sensor` name used for
    ROS mapping.
  - Optional: `fovy` (int/float) — vertical field of view in degrees (example: `58`).
  - Optional: `mode` (string) — MJCF camera mode (commonly `fixed`).
  - Optional: `resolution` (string) — two integers `"<width> <height>"` (example: `640 480`).
  - Notes: Converter fills transform (position + quaternion) from the URDF link pose.

- `lidar`
  - Required: `ref_site` (string) — URDF frame used as reference for placing rangefinders.
  - Required: `sensor_name` (string) — base name for generated MJCF rangefinders (e.g.
    `rf` will produce `rf-01`, `rf-02`, ...). URDF `sensor` entries and ROS mapping should
    reference the same base name.
  - Optional: `min_angle` (float) — start angle in radians (default depends on converter).
  - Optional: `max_angle` (float) — end angle in radians.
  - Optional: `angle_increment` (float) — angular step between generated rangefinders.
  - Notes: The converter creates multiple MJCF `rangefinder` sensors across the angle range;
    the hardware interface merges them into a single ROS LaserScan.

- `modify_element`
  - Required: `type` (string) — MJCF element type to target (e.g. `joint`, `body`).
  - Required: `name` (string) — name attribute of the MJCF element to modify.
  - Additional attributes: any MJCF attributes you want to set or overwrite (for example
    `frictionloss`, `damping`, `gravcomp`, `solimp`, `solref`, ...).
  - Example: `<modify_element type="joint" name="joint1" frictionloss="1.0" damping="2.0"/>`.
