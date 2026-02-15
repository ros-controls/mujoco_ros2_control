# MuJoCo ros2_control

[![Rdev](https://build.ros2.org/job/Rdev__mujoco_ros2_control__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mujoco_ros2_control__ubuntu_noble_amd64/) [![Kdev](https://build.ros2.org/job/Kdev__mujoco_ros2_control__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mujoco_ros2_control__ubuntu_noble_amd64/) [![Jdev](https://build.ros2.org/job/Jdev__mujoco_ros2_control__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mujoco_ros2_control__ubuntu_noble_amd64/) [![CI](https://github.com/ros-controls/mujoco_ros2_control/actions/workflows/ci.yaml/badge.svg)](https://github.com/ros-controls/mujoco_ros2_control/actions/workflows/ci.yaml) ![License](https://img.shields.io/github/license/ros-controls/mujoco_ros2_control) [![Codecov](https://codecov.io/gh/ros-controls/mujoco_ros2_control/branch/main/graph/badge.svg)](https://codecov.io/gh/ros-controls/mujoco_ros2_control)

This repository provides a ros2_control system interface and supporting packages to run ROS 2 controllers against the MuJoCo physics simulator.

This project wraps MuJoCo as a hardware/system interface so you can use the ros2_control stack (controller manager, controllers, controller interfaces) against simulated robots based on MJCF or generated from URDF.

### Contents

- `mujoco_ros2_control` - core system interface plugin and resources
- `mujoco_ros2_control_msgs` - message/service definitions used by the plugin
- `mujoco_ros2_control_demos` - demo launch files, configs and example robots
- `mujoco_ros2_control_tests` - integration / launch tests and simple examples
- `docker/` - Dockerfiles and scripts to run CI/containers

### Key features

- Full ros2_control SystemInterface plugin for MuJoCo
- MJCF/URDF conversion utilities to auto-generate MuJoCo models
- Example demos showing basic control, PID and transmission setups

## Quick start

There are two common ways to get running: build from source (recommended)
or install prebuilt binaries (if available for your distribution).

- Build from source (recommended)

  1. Install required dependencies manually or from rosdep, including the `mujoco_vendor` package, if available. Otherwise MuJoCo will will be downloaded at build time.

  2. Build the workspace (example with a sourced ROS 2 installation):

  ```bash
  # from workspace root (this repository is typically inside a ROS 2 workspace)
  colcon build --symlink-install --packages-select mujoco_ros2_control* \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

  3. Source the workspace and run a demo:

  ```bash
  source install/setup.bash
  ros2 launch mujoco_ros2_control_demos demo.launch.py
  ```

- Install prebuilt binaries (if available)

  If your ROS 2 distribution or your OS package index provides prebuilt
  packages for `mujoco_ros2_control`, you can install those instead of
  compiling from source. Check your distribution's package repositories or
  the project's GitHub releases for available binary artifacts.

  Example (Debian/Ubuntu with ROS packages — replace `<distro>`):

  ```bash
  sudo apt update
  sudo apt install ros-<distro>-mujoco-ros2-control
  ```

  After installing binaries, source your ROS install and run a demo:

  ```bash
  source /opt/ros/<distro>/setup.bash
  ros2 launch mujoco_ros2_control_demos demo.launch.py
  ```

See `mujoco_ros2_control/README.md` for detailed usage, configuration examples and mappings between MJCF actuators/sensors and ros2_control interfaces.

Supported ROS 2 distributions
- The project is developed and tested against multiple ROS 2 distributions.
  This README includes basic notes for: `Humble`, `Kilted`, `Jazzy` and
  `Rolling`.

### Support matrix

| Distribution | Status |
| --- | --- |
| Humble | Supported |
| Kilted | Supported |
| Jazzy | Supported |
| Rolling | Supported (development) |

### Contributing

- Contributions, bug reports and feature requests are welcome. Please follow standard ROS Controls project workflows: open issues, send PRs against the   `main` branch and respect the repository code style using `pre-commit`.

### License & maintainers

- This repository is distributed under the terms of the LICENSE file (`LICENSE`). Maintainers and authors are listed in the Git history and package manifests (`package.xml`) inside each package.
