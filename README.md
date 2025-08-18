# MuJoCo ROS 2 Simulation

This package contains a ROS 2 control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/3.3.4/overview.html).
It was originally written for simulating robot hardware in NASA Johnson's [iMETRO facility](https://ntrs.nasa.gov/citations/20230015485).

The system interface wraps MuJoCo's [Simulate App](https://github.com/google-deepmind/mujoco/tree/3.3.4/simulate) to provide included functionality.
Because the app is not bundled as a library, we compile it directly from a local install of MuJoCo.

Parts of this library are also based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

## Installation

This interface has only been tested against ROS 2 humble and MuJoCo `3.3.4`.
We assume all required ROS dependencies have been installed either manually or with `rosdep`.

A local install of MuJoCo is required to build the application, this package will not handle it for you.
Refer to their documentation for installation procedures.

Once installed, set the following environment variables:

```bash
MUJOCO_VERSION=3.3.4
MUJOCO_DIR=/opt/mujoco/mujoco-3.3.4
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
        Optional parameter to update the simulated camera's color and depth image publish rates. If no
        parameter is set then all cameras will publish at 5 hz. Note that all cameras in the sim currently
        publish at the same intervals.
      -->
      <param name="camera_publish_rate">6.0</param>
    </hardware>
  ...
```

### Joints

Joints in the ros2_control interface are mapped to actuators defined in the MJCF.
For now, we rely on Mujoco's PD level joint control for position and velocity command interfaces, and write directly to `qfrc_applied` for effort command interfaces.
Refer to Mujoco's [actuation model](https://mujoco.readthedocs.io/en/stable/computation/index.html#geactuation) for more information.

```xml
  <actuator>
    <position joint="joint1" name="joint1" kp="25000" dampratio="1.0" ctrlrange="0.0 2.0"/>
  </actuator>
```

From there, joints can be configured with position, velocity, and effort command and state interfaces.
Initial values can be
For example,

```xml
  <joint name="join1">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
```

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

## Docker Development Workflow

This project includes a [Dockerfile](./.docker/Dockerfile) for development and testing in an isolated environment.
Use the provided scripts to build and run the images.

**Note**: you may need to give docker access to xhost with `xhost +local:docker` to ensure the container has access to the host UI.

First, build a new image with default settings:

```bash
./docker/build.sh
```

The image can be started in a new shell with:

```bash
./docker/run.sh
```

This will launch you into a container with the source code mounted in a colcon workspace.
From there the source can be modified, built, tested, or otherwise used as normal.
For example, launch the included test scene with,

```bash
# Evaluate using the included mujoco simulate application
${MUJOCO_DIR}/bin/simulate ${ROS_WS}/src/mujoco_ros2_simulation/test/resources/scene.xml

# Or launch the test ROS control interface
ros2 launch mujoco_ros2_simulation test_robot.launch.py
```
