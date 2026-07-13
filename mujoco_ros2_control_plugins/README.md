# mujoco_ros2_control_plugins

This package provides a plugin interface for extending the functionality of `mujoco_ros2_control`.

## Documentation

Full documentation is maintained in RST format:

- **[MuJoCo ROS 2 Control Plugins](doc/plugins.rst)** — available plugins, usage, configuration, and instructions for writing your own plugin.

## Available Plugins

| Plugin | Description |
|---|---|
| `HeartbeatPublisherPlugin` | Publishes a heartbeat string to `/mujoco_heartbeat` at 1 Hz |
| `ExternalWrenchPlugin` | Applies external wrenches to MuJoCo bodies via a ROS 2 service |

## Quick Start

Load plugins by passing a parameters file to the `mujoco_ros2_control` node:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      heart_beat_plugin:
        type: "mujoco_ros2_control_plugins/HeartbeatPublisherPlugin"
```

For full usage details, service/topic interfaces, parameters, and a guide to writing your own
plugin, see [`doc/plugins.rst`](doc/plugins.rst).

## See Also

- Main package: [mujoco_ros2_control](../mujoco_ros2_control/)
- Demos: [mujoco_ros2_control_demos](../mujoco_ros2_control_demos/)
