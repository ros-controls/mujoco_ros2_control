# MuJoCo ros2_control Simulation

This package contains a ros2_control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/3.4.0/overview.html).
It was originally written for simulating robot hardware in NASA Johnson's [iMETRO facility](https://ntrs.nasa.gov/citations/20230015485).

The system interface wraps MuJoCo's [Simulate App](https://github.com/google-deepmind/mujoco/tree/3.4.0/simulate) to provide included functionality.
Because the app is not bundled as a library, we compile it directly from a local install of MuJoCo.

Parts of this library are also based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

## URDF Model Conversion

MuJoCo does not support the full feature set of xacro/URDFs in the ROS 2 ecosystem.
Users are required to convert any existing robot description files to an MJCF format.
We provide a *highly experimental* tool to automate URDF conversion — refer to the [URDF to MJCF conversion documentation](docs/tools.rst) for details.

## Hardware Interface Setup

The MuJoCo hardware interface is shipped as a `ros2_control` plugin. Specify it in your URDF and point to a valid MJCF:

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
    <param name="mujoco_model">$(find my_description)/description/scene.xml</param>
  </hardware>
  ...
```

A custom `ros2_control` node is required due to compatibility requirements:

```python
control_node = Node(
    package="mujoco_ros2_control",
    executable="ros2_control_node",
    output="both",
    parameters=[
        {"use_sim_time": True},
        controller_parameters,
    ],
)
```

For the full plugin parameter reference, joint control modes, gripper/mimic joint setup, sensors (FTS, IMU), cameras, and lidar configuration, see the [hardware interface documentation](docs/hardware_interface.rst).

## Simulation Topics and Services

The simulator exposes ROS 2 topics and services for interacting with the simulation at runtime (pause, reset, step-by-step control, etc.).
See the [simulation topics and services documentation](docs/hardware_interface.rst#simulation-topics-and-services) for details.

## Test Robot System

While examples are limited, we maintain a functional example 2-dof robot system in the demos space (see `mujoco_ros2_control_demos` package).
We generally recommend looking there for examples and recommended workflows.

## Development

More information is provided in the [developers guide](../doc/development.rst).

## Further Documentation

| Document | Description |
|---|---|
| [Hardware Interface](docs/hardware_interface.rst) | Plugin params, joints, sensors, cameras, lidar, topics, services, debugging |
| [URDF to MJCF Conversion](docs/tools.rst) | Conversion tool usage and MJCF schema reference |
| [Modeling Tips](docs/modeling_tips.rst) | Tips for modeling complex geometries in MuJoCo |
| [Developers Guide](../doc/development.rst) | Development workflows (Docker, pixi) |
