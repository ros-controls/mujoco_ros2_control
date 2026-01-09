# MuJoCo ros2_control Simulation

This package contains a ros2_control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/3.4.0/overview.html).
It was originally written for simulating robot hardware in NASA Johnson's [iMETRO facility](https://ntrs.nasa.gov/citations/20230015485).

The system interface wraps MuJoCo's [Simulate App](https://github.com/google-deepmind/mujoco/tree/3.4.0/simulate) to provide included functionality.
Because the app is not bundled as a library, we compile it directly from a local install of MuJoCo.

Parts of this library are also based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

## Installation

This interface has only been tested against ROS 2 jazzy and MuJoCo `3.4.0`.
It should also be compatible with kilted and rolling, but we do not actively maintain those.
We assume all required ROS dependencies have been installed either manually or with `rosdep`.

For configuring MuJoCo, the included [CMakeLists.txt](./CMakeLists.txt) will download and install the tarfile automatically.
As long as users have a good network connection there should not be an issue.

However, a local install of MuJoCo can be used to build the application by setting the following environment variables,

```bash
# The tested version
MUJOCO_VERSION=3.4.0

# Wherever it was installed and extracted on your machine
MUJOCO_INSTALL_DIR=/opt/mujoco/mujoco-3.4.0
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
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">$(find my_description)/description/scene.xml</param>

      <!--
       Optional parameter to load the PIDs that can be used with the actuators loaded with the MuJoCo model.
       The velocity actuator supports position mode with the PID gains, and the rest of the actuation models
       support both position and velocity mode provided the corresponding PID gains. The gains should be in ROS
       parameters format to be loaded by the control_toolbox::PidROS class.
        -->
      <param name="pids_config_file">$(find my_description)/config/pids.yaml</param>

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
        Optional parameter to choose a topic name from which it can subscribe and get the MJCF contents
        to load the model. This is only used, when the mujoco_model parameter is not set.
        By default, this parameter is initialized to /mujoco_robot_description
      -->
      <param name="mujoco_model_topic">/mujoco_robot_description</param>

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

      <!--
        The parameter headless can be used to choose whether to launch the MuJoCo simulator in headless mode or not.
        By default, it is set to false
      -->
      <param name="headless">false</param>
    </hardware>
  ...
```

Due to compatibility issues, we use a [slightly modified ROS 2 control node](./src/mujoco_ros2_control_node.cpp).
It is the same executable and parameters as the upstream, but requires updating the launchfile:

```python
    control_node = Node(
        # Specify the control node from this package!
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters,
        ],
    )
```

> [!NOTE]
> We can remove the the ROS 2 control node after the next ros2_control upstream release,
> as the simulation requires [this PR](https://github.com/ros-controls/ros2_control/pull/2654) to run.
> The hardware interface _should_ then be compatible with `humble`, `jazzy`, and `kilted`.

### Joints

Joints in the ros2_control interface are mapped to actuators defined in the MJCF.
The system supports different joint control modes based on the actuator type and available command interfaces.

We rely on MuJoCo's PD-level ctrl input for direct position, velocity, or effort control.
For velocity, motor, or custom actuators, a position or velocity PID is created if specified using ROS parameters to enable accurate control.
Incompatible actuator-interface combinations trigger an error.

Refer to Mujoco's [actuation model](https://mujoco.readthedocs.io/en/stable/computation/index.html#geactuation) for more information.

Of note, only one type of MuJoCo actuator per-joint can be controllable at a time, and the type CANNOT be switched during runtime (i.e., switching from position to motor actuator is not supported).
However, the active command interface can be switched dynamically, allowing control to shift between position, velocity, or effort as supported by the actuator type.
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

**Supported modes between MuJoCo actuators and ros2_control command interfaces:**

<table>
  <thead>
    <tr>
      <th rowspan="2" style="border:none;"></th>
      <th style="border:none;"></th>
      <th colspan="3" style="text-align:center;">MuJoCo Actuators</th>
    </tr>
    <tr>
      <th style="border:none;"></th>
      <th>position</th>
      <th>velocity</th>
      <th>motor, general, etc </th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th rowspan="3" style="text-align:center;">ros2 control<br>command<br>interfaces</th>
      <th>position</th>
      <td style="background:#c6efce;">Native support</td>
      <td style="background:#ffeb9c;">Supported using PIDs</td>
      <td style="background:#ffeb9c;">Supported using PIDs</td>
    </tr>
    <tr>
      <th>velocity</th>
      <td style="background:#ffc7ce;">Not supported</td>
      <td style="background:#c6efce;">Native support</td>
      <td style="background:#ffeb9c;">Supported using PIDs</td>
    </tr>
    <tr>
      <th>effort</th>
      <td style="background:#ffc7ce;">Not supported</td>
      <td style="background:#ffc7ce;">Not supported</td>
      <td style="background:#c6efce;">Native support</td>
    </tr>
  </tbody>
</table>

> [!NOTE]
> The `torque` and `force` command/state interfaces are semantically equivalent to `effort`, and map to the same underlying data in the sim.

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
    <!-- Default value of force_mjcf_suffix is '_force' -->
    <param name="force_mjcf_suffix">_force</param>
    <!-- Default value of torque_mjcf_suffix is '_torque' -->
    <param name="torque_mjcf_suffix">_torque</param>
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
    <!-- Default value of orientation_mjcf_suffix is '_quat' -->
    <param name="orientation_mjcf_suffix">_quat</param>
    <!-- Default value of angular_velocity_mjcf_suffix is '_gyro' -->
    <param name="angular_velocity_mjcf_suffix">_gyro</param>
    <!-- Default value of linear_acceleration_mjcf_suffix is '_accel' -->
    <param name="linear_acceleration_mjcf_suffix">_accel</param>
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

## Test Robot System

While examples are limited, we maintain a functional example 2-dof robot system in the [test examples](./test/test_resources/test_robot.urdf) space.
We generally recommend looking there for examples and recommended workflows.

For now, built the drivers with testing enabled, then the test robot system can be launched with:

```bash
# Brings up the hardware drivers and mujoco interface, along with a single position controller
ros2 launch mujoco_ros2_control test_robot.launch.py

# Or optionally include the PID controller as mentioned above
ros2 launch mujoco_ros2_control test_robot.launch.py use_pid:=true

# Launch an rviz2 window with the provided configuration
rviz2 -d $(ros2 pkg prefix --share mujoco_ros2_control)/config/test_robot.rviz
```

From there, command joints to move with,

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.25, 0.75]" --once
```

> [!TIP]
> UI panels can be toggled with `Tab` or `Shift+Tab`.
> All standard MuJoCo keyboard shortcuts are available.
> To see a short list, press `F1`.

## Development

More information is provided in the [developers guide](./docs/DEVELOPMENT.md) document.
