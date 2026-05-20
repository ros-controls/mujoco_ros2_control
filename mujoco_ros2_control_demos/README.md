# mujoco_ros2_control_demos

Demonstration tutorials for the `mujoco_ros2_control` package.
This package provides ready-to-use tutorials that show how to use MuJoCo with ros2_control.

## Documentation

Full tutorial documentation is maintained in RST format:

- **[Demos and Tutorials](doc/tutorials.rst)** — step-by-step tutorials covering basic robot setup, MJCF generation, PID control, and transmissions.

## Quick Start

```bash
# Tutorial 1 – Basic Robot
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py

# Tutorial 2 – MJCF Generation at Runtime
ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py

# Tutorial 3 – PID Control
ros2 launch mujoco_ros2_control_demos 03_pid_control.launch.py

# Tutorial 4 – Transmissions
ros2 launch mujoco_ros2_control_demos 04_transmissions.launch.py
```

All tutorials support `headless:=true` to run without the MuJoCo viewer.

For the full list of launch arguments, features, controlling the robot, and package structure, see [`doc/tutorials.rst`](doc/tutorials.rst).

## See Also

- Main package: [mujoco_ros2_control](../mujoco_ros2_control/)
- Tests: [mujoco_ros2_control_tests](../mujoco_ros2_control_tests/)
