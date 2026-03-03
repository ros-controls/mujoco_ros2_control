# mujoco_ros2_control_plugins

This package provides a plugin interface for extending the functionality of `mujoco_ros2_control`.

## Overview

The `mujoco_ros2_control_plugins` package is designed to contain plugins that extend the capabilities of the main `mujoco_ros2_control` package. This separation allows for modular development and optional features without adding complexity to the core package.

## Dependencies

- `mujoco_vendor`: Provides the MuJoCo physics simulator library

## Building

This package is part of the `mujoco_ros2_control` workspace. Build it using:

```bash
colcon build --packages-select mujoco_ros2_control_plugins
```

## Usage

To create a new plugin:

1. Add your plugin header file to `include/mujoco_ros2_control_plugins/`
2. Add your plugin implementation to `src/`
3. Update the CMakeLists.txt to build your plugin
4. If creating a pluginlib plugin, add the appropriate plugin description XML file

## License

Apache License 2.0
