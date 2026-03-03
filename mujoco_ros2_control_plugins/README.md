# mujoco_ros2_control_plugins

This package provides a plugin interface for extending the functionality of `mujoco_ros2_control`.

## Overview

The `mujoco_ros2_control_plugins` package is designed to contain plugins that extend the capabilities of the main `mujoco_ros2_control` package. This separation allows for modular development and optional features without adding complexity to the core package.

> [!NOTE]
> This interface provides flexibility for accessing information from the MuJoCo model and data.
> Users are responsible for handling that data correctly and avoiding changes to critical information.

## Available Plugins

### HeartbeatPublisherPlugin

A simple demonstration plugin that publishes a heartbeat message every second to the `/mujoco_heartbeat` topic.

**Topic**: `mujoco_heartbeat` (std_msgs/String)
**Rate**: 1 Hz (every 1 second)
**Message Format**: "MuJoCo ROS2 Control Heartbeat #N | Simulation time: Xs"

## Dependencies

- `mujoco_vendor`: Provides the MuJoCo physics simulator library
- `rclcpp`: ROS 2 C++ client library
- `pluginlib`: Plugin loading framework
- `std_msgs`: Standard ROS message types

## Building

This package is part of the `mujoco_ros2_control` workspace. Build it using:

```bash
colcon build --packages-select mujoco_ros2_control_plugins
```

## Usage

Plugins are loaded from ROS 2 parameters under `mujoco_plugins`.

Each plugin must have:

- A unique key (for example `heart_beat_plugin`)
- A `type` field with the pluginlib class name

Use a parameters file like this:

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      heart_beat_plugin:
        type: "mujoco_ros2_control_plugins/HeartbeatPublisherPlugin"
        update_rate: 1.0
```

Then pass that file to the `mujoco_ros2_control` node (for example with `ParameterFile(...)` in your launch file).

> [!NOTE]
> In this repository, `mujoco_ros2_control_demos/launch/01_basic_robot.launch.py` already loads
> `mujoco_ros2_control_demos/config/mujoco_ros2_control_plugins.yaml`.

### Example: Monitoring the Heartbeat

```bash
# Terminal 1: Launch your mujoco_ros2_control simulation
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py

# Terminal 2: Echo the heartbeat messages
ros2 topic echo /mujoco_heartbeat
```

## Creating Your Own Plugin

### 1. Create Plugin Header

Create a header file that inherits from `MuJoCoROS2ControlPluginBase`:

```cpp
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace my_namespace
{

class MyCustomPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  // Your member variables
};

}  // namespace my_namespace
```

### 2. Implement Plugin Methods

```cpp
#include "my_custom_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace my_namespace
{

bool MyCustomPlugin::init(
  rclcpp::Node::SharedPtr node,
  const mjModel* model,
  mjData* data)
{
  // Initialize your plugin
  return true;
}

void MyCustomPlugin::update(const mjModel* model, mjData* data)
{
  // Called every control loop iteration
}

void MyCustomPlugin::cleanup()
{
  // Clean up resources
}

}  // namespace my_namespace

PLUGINLIB_EXPORT_CLASS(
  my_namespace::MyCustomPlugin,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
)
```

### 3. Create Plugin XML Descriptor

Create `my_plugins.xml`:

```xml
<library path="my_plugin_library">
  <class name="my_namespace/MyCustomPlugin"
         type="my_namespace::MyCustomPlugin"
         base_class_type="mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase">
    <description>
      Description of what your plugin does.
    </description>
  </class>
</library>
```

### 4. Update CMakeLists.txt

```cmake
find_package(mujoco_ros2_control_plugins REQUIRED)

add_library(my_plugin_library SHARED
  src/my_custom_plugin.cpp
)

target_link_libraries(my_plugin_library
  ${mujoco_ros2_control_plugins_TARGETS}
  # ... other dependencies
)

pluginlib_export_plugin_description_file(
  mujoco_ros2_control_plugins
  my_plugins.xml
)
```

## Plugin Lifecycle

1. **Initialization** (`init`): Called once when the hardware interface is initialized
2. **Update** (`update`): Called every control loop iteration
3. **Cleanup** (`cleanup`): Called when shutting down
