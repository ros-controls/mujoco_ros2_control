# MuJoCo ROS 2 Simulation

This package contains a ROS 2 control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/3.3.4/overview.html).
It was originally written for simulating robot hardware in NASA Johnson's [iMETRO facility](https://ntrs.nasa.gov/citations/20230015485).

The system interface wraps MuJoCo's [Simulate App](https://github.com/google-deepmind/mujoco/tree/3.3.4/simulate) to provide included functionality.
Because the app is not bundled as a library, we compile it directly from a local install of MuJoCo.

Parts of this library are also based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

## Installation

This interface has only been tested against ROS 2 jazzy and MuJoCo `3.3.4`.
It should also be compatible with kilted and rolling, but we do not actively maintain those.
We assume all required ROS dependencies have been installed either manually or with `rosdep`.

For configuring MuJoCo, the included [CMakeLists.txt](./CMakeLists.txt) will download and install the tarfile automatically.
As long as users have a good network connection there should not be an issue.

However, a local install of MuJoCo can be used to build the application by setting the following environment variables,

```bash
# The tested version
MUJOCO_VERSION=3.3.4

# Wherever it was installed and extracted on your machine
MUJOCO_INSTALL_DIR=/opt/mujoco/mujoco-3.3.4
```

From there the library can be compiled with `colcon build ...`, as normal.

## URDF Model Conversion

Mujoco does not support the full feature set of xacro/URDFs in the ROS 2 ecosystem.
As such, users are required to convert any existing robot description files to an MJCF format.
This includes adding actuators, sensors, and cameras as needed to the MJCF XML.

We have built a *highly experimental tool to automate URDF conversion.
For more information refer to the [documentation](./docs/TOOLS.md).

## Hardware Interface Setup

### Plugin

This application is shipped as a ros2_control hardware interface, and can be configured as such.
Just specify the plugin and point to a valid MJCF on launch:

```xml
  <ros2_control name="MujocoSystem" type="system">
    <hardware>
      <plugin>mujoco_ros2_simulation/MujocoSystemInterface</plugin>
      <param name="mujoco_model">$(find my_description)/description/scene.xml</param>

      <!--
       Optional parameter to override the speed scaling parameters from the Simulate App window
       and just attempt to run at whatever the desired rate is here. This allows users to run the simulation
       faster than real time. For example, at 500% speed as set here. If this param is omitted or set to
       a value <0, then the simulation will run using the slowdown requested from the App.
      -->
      <param name="sim_speed_factor">5.0</param>

      <!--
        Optional parameter to use the keyframe from a provided file as the starting configuration. This is mutually exclusive with
        the initial_value that can be used for state interfaces. This is intended to provide an alternative method to load an entire
        mujoco model state from a configuration that was saved by clicking 'Copy state' in the simulate window, and pasted into a
        config file. Expected use cases are to work on a specific part of an application that involves the environment being in a
        very specific starting configuration. If this parameter is an empty string, it will be ignored.
      -->
      <param name="override_start_position_file">$(find my_description)/config/start_positions.xml</param>

      <!--
        Optional parameter to update the simulated camera's color and depth image publish rates. If no
        parameter is set then all cameras will publish at 5 hz. Note that all cameras in the sim currently
        publish at the same intervals.
      -->
      <param name="camera_publish_rate">6.0</param>

      <!--
        Optional parameter to update the simulated lidar sensor's scan message publish rates.
        All lidar sensors in the simulation will be configured to publish these scan messages at the same rate.
      -->
      <param name="lidar_publish_rate">10.0</param>
    </hardware>
  ...
```

Due to compatibility issues, we use a [slightly modified ROS 2 control node](./src/mujoco_ros2_control_node.cpp).
It is the same executable and parameters as the upstream, but requires updating the launchfile:

```python
    control_node = Node(
        # Specify the control node from this package!
        package="mujoco_ros2_simulation",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters,
        ],
    )
```

> **_NOTE_**: We can remove the the ROS 2 control node after the next ros2_control upstream release,
as the simulation requires [this PR](https://github.com/ros-controls/ros2_control/pull/2654) to run.
The hardware interface _should_ then be compatible with `humble`, `jazzy`, and `kilted`.

### Joints

Joints in the ros2_control interface are mapped to actuators defined in the MJCF.
For now, we rely on Mujoco's PD level `ctrl` input for all actuator control.
Refer to Mujoco's [actuation model](https://mujoco.readthedocs.io/en/stable/computation/index.html#geactuation) for more information.
Of note, only one type of actuator per-joint can be controllable at a time, and the type CANNOT be switched during runtime (ie, switching from position to effort control is not supported).
Users are required to manually adjust actuator types and command interfaces to ensure that they are compatible.

For example a position controlled joint on the mujoco

```xml
  <actuator>
    <position joint="joint1" name="joint1" kp="25000" dampratio="1.0" ctrlrange="0.0 2.0"/>
  </actuator>
```

Could map to the following hardware interface:

```xml
  <joint name="joint1">
    <command_interface name="position"/>
    <!-- Initial values for state interfaces can be specified, but default to 0 if they are not. -->
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
```

Switching actuator/control types on the fly is an [open issue](#13).

### Sensors

The hardware interfaces supports force-torque sensors (FTS) and inertial measurement units (IMUs).
Mujoco does not support modeling complete FTS and IMUs out of the box, so we combine supported MJCF constructs to map to a ros2_control sensor.
The types and other parameters can be specified in the ros2_control xacro, as noted below.

For FTS, we model the `force` and `torque` sensors individually in the MJFC.
For a sensor named `fts_sensor`, we suffix each entry accordingly as `fts_sensor_force` and `fts_sensor_torque`.
For example, in the MJCF,

```xml
  <sensor>
    <force name="fts_sensor_force" site="ft_frame"/>
    <torque name="fts_sensor_torque" site="ft_frame"/>
  </sensor>
```

In the corresponding ros2_control xacro, this becomes a single sensor:

```xml
  <sensor name="fts_sensor">
    <param name="mujoco_type">fts</param>
    <!-- There is no requirement for the mujoco_sensor_name to match the ros2_control sensor name -->
    <param name="mujoco_sensor_name">fts_sensor</param>
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
  </sensor>
```

Similarly, for an IMU, we simulate a `framequat`, `gyro`, and `accelerometer` as a single IMU.

```xml
  <sensor>
      <framequat name="imu_sensor_quat" objtype="site" objname="imu_sensor" />
      <gyro name="imu_sensor_gyro" site="imu_sensor" />
      <accelerometer name="imu_sensor_accel" site="imu_sensor" />
  </sensor>
```

Which then map to the corresponding ros2_control sensor:

```xml
  <sensor name="imu_sensor">
    <param name="mujoco_type">imu</param>
    <!-- There is no requirement for the mujoco_sensor_name to match the ros2_control sensor name -->
    <param name="mujoco_sensor_name">imu_sensor</param>
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
    <state_interface name="linear_acceleration.x"/>
    <state_interface name="linear_acceleration.y"/>
    <state_interface name="linear_acceleration.z"/>
  </sensor>
```

These sensor state interfaces can then be used out of the box with the standard broadcasters.

### Cameras

Any `camera` included in the MJCF will automatically have its RGB-D images and info published to ROS topics.
Currently all images are published at a fixed 5hz rate.

Cameras must include a string `<name>`, which sets defaults for the frame and topic names.
By default, the ROS 2 wrapper assumes the camera is attached to a frame named `<name>_frame`.
Additionally camera_info, color, and depth images will be published to topics called `<name>/camera_info`, `<name>/color`, and `<name>/depth`, respectively.
Also note that MuJuCo's conventions for cameras are different than ROS's, and which must be accounted for.
Refer to the documentation for more information.

For example,

```xml
<camera name="wrist_mounted_camera" fovy="58" mode="fixed" resolution="640 480" pos="0 0 0" quat="0 0 0 1"/>
```

Will publish the following topics:

```bash
$ ros2 topic info /wrist_mounted_camera/camera_info
Type: sensor_msgs/msg/CameraInfo
$ ros2 topic info /wrist_mounted_camera/color
Type: sensor_msgs/msg/Image
$ ros2 topic info /wrist_mounted_camera/depth
Type: sensor_msgs/msg/Image
```

The frame and topic names are also configurable from the ros2_control xacro.
Default parameters can be overridden with:

```xml
  <!-- For cameras, the sensor name _must_ match the camera name in the MJCF -->
  <sensor name="wrist_mounted_camera">
    <param name="frame_name">wrist_mounted_camera_mujoco_frame</param>
    <param name="info_topic">/wrist_mounted_camera/color/camera_info</param>
    <param name="image_topic">/wrist_mounted_camera/color/image_raw</param>
    <param name="depth_topic">/wrist_mounted_camera/aligned_depth_to_color/image_raw</param>
  </sensor>
```

### Lidar

MuJoCo does not include native support for lidar sensors.
However, this package offers a ROS 2-like lidar implementation by wrapping sets of [rangefinders](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-rangefinder) together.

MuJoCo rangefinders measure the distance to the nearest surface along the positive `Z` axis of the sensor site.
The ROS 2 lidar wrapper uses the standard defined in [LaserScan](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg#L10) messages.
In particular, the first rangefinder's `Z` axis (e.g. `rf-00`) must align with the ROS 2 lidar sensor's positive `X` axis.

In the MJCF, use the `replicate` tag along with a `-` separator to add N sites to attach sensors to.
For example, the following will add 12 sites named `rf-00` to `rf-11` each at a 0.025 radian offset from each other:

```xml
  <replicate count="12" sep="-" offset="0 0 0" euler="0 0.025 0">
    <site name="rf" size="0.01" pos="0.0 0.0 0.0" quat="0.0 0.0 0.0 1.0"/>
  </replicate>
```

Then a set of rangefinders can be attached to each site with:

```xml
  <sensor>
    <!-- We require a sensor name be provided -->
    <rangefinder name="lidar" site="rf" />
  </sensor>
```

The lidar sensor is then configurable through ROS 2 control xacro with:

```xml
    <!-- Lidar sensors are matched to a set of rangefinder sensors in the MJCF, which should be -->
    <!-- generated with "replicate" and will generally be of the form "<sensor_name>-01". -->
    <!-- We assume the lidar sensor starts at angle 0, increments by the specified `angle_increment`, and -->
    <!-- that there are exactly `num_rangefinders` all named from <sensor_name>-000 to the max -->
    <!-- <sensor_name>-<num_rangefinders> -->
    <sensor name="lidar">
      <param name="frame_name">lidar_sensor_frame</param>
      <param name="angle_increment">0.025</param>
      <param name="num_rangefinders">12</param>
      <param name="range_min">0.05</param>
      <param name="range_max">10</param>
      <param name="laserscan_topic">/scan</param>
    </sensor>
  </ros2_control>
```

## Docker Development Workflow

This project includes a [compose](./docker-compose.yml) and [Dockerfile](./.docker/Dockerfile) for development and testing in an isolated environment.

**Note**: you may need to give docker access to xhost with `xhost +local:docker` to ensure the container has access to the host UI.

For users on arm64 machines, be sure to specify the `CPU_ARCH` variable in your environment when building.

```bash
docker compose build
```

The service can be started with:

```bash
# Start the service in one shell (or start detached)
docker compose up

# Connect to it in another
docker compose exec dev bash
```

This will launch a container with the source code mounted in a colcon workspace.
From there the source can be modified, built, tested, or otherwise used as normal.
For example, launch the included test scene with,

```bash
# Evaluate using the included mujoco simulate application
${MUJOCO_DIR}/bin/simulate ${ROS_WS}/src/mujoco_ros2_simulation/test/test_resources/scene.xml

# Or launch the test ROS control interface
ros2 launch mujoco_ros2_simulation test_robot.launch.py
```

> **_NOTE:_** Rendering contexts in containers can be tricky.
Users may need to tweak the compose file to support their specific host OS or GPUs.
For more information refer to the comments in the compose file.

## Pixi Development Workflow

A [pixi](https://pixi.sh/latest/installation/) and [robostack](https://robostack.github.io) workflow is also provided.
The environment is currently only compatible with Jazzy.

To run ensure pixi is installed.
Then,

```bash
# Setup the build environment
pixi run setup-colcon

# Build the package
pixi run build

# Run tests
pixi run test

# Launch an interactive shell and source the install
pixi shell
source install/setup.bash
```

### Test Robot System

While examples are limited, we maintain a functional example 2-dof robot system in the [test examples](./test/test_resources/test_robot.urdf) space.
We generally recommend looking there for examples and recommended workflows.

For now, built the drivers with testing enabled, then the test robot system can be launched with:

```bash
# Brings up the hardware drivers and mujoco interface, along with a single position controller
ros2 launch mujoco_ros2_simulation test_robot.launch.py

# Launch an rviz2 window with the provided configuration
rviz2 -d $(ros2 pkg prefix --share mujoco_ros2_simulation)/config/test_robot.rviz
```

From there, command joints to move with,

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.25, 0.75]" --once
```
