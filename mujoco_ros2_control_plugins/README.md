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

| | |
|---|---|
| **Topic** | `mujoco_heartbeat` (`std_msgs/String`) |
| **Rate** | 1 Hz |
| **Message format** | `"MuJoCo ROS2 Control Heartbeat #N \| Simulation time: Xs"` |

---

### ConstraintViolationsPublisherPlugin

Monitors active MuJoCo equality constraints every simulation step and publishes the maximum absolute position-level residual (`efc_pos`) for each one.

| | |
|---|---|
| **Topic** | `~/constraint_violations` (`mujoco_ros2_control_msgs/ConstraintViolations`) |
| **Rate** | Configurable via parameter `publish_rate` (Hz, default 50 Hz) |

Each message entry contains:
- `name` — constraint name from MJCF, or synthesised as `<obj1>_<obj2>_{joint\|tendon\|weld\|connect}_constraint`
- `type` — one of `joint`, `tendon`, `weld`, `connect`, `unknown`
- `violation` — maximum absolute `efc_pos` across all rows of that constraint

**Parameters**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate` | `double` | `50.0` | Publish rate in Hz |

**Example configuration**

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      constraint_monitor:
        type: "mujoco_ros2_control_plugins/ConstraintViolationsPublisherPlugin"
        publish_rate: 100.0
```

---

### ExternalWrenchPlugin

Applies an external wrench (force + torque) to a named MuJoCo body for a configurable duration via a ROS 2 service.  Multiple concurrent wrenches on different — or the same — bodies are supported; each expires independently.

| | |
|---|---|
| **Service** | `~/apply_wrench` (`mujoco_ros2_control_msgs/srv/ApplyExternalWrench`) |
| **Topic** | `~/wrench_markers` (`visualization_msgs/msg/MarkerArray`) |

**Service request fields**

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Stamp and frame ID (informational) |
| `link_name` | `string` | MuJoCo body name (must match the MJCF `<body name="...">`) |
| `wrench.force` | `geometry_msgs/Vector3` | Linear force [N] in the **body (link) frame** |
| `wrench.torque` | `geometry_msgs/Vector3` | Angular moment [N·m] in the **body (link) frame** |
| `application_point` | `geometry_msgs/Point` | Force application point in the **body (link) frame** (relative to body frame origin, metres). Zero → apply at the body frame origin. |
| `duration` | `builtin_interfaces/Duration` | How long the wrench remains active. Zero → single simulation step. |
| `ramp_down_duration` | `builtin_interfaces/Duration` | Duration over which the wrench linearly ramps from full magnitude to zero at the end of `duration`. Zero → no ramp-down. |

**Service response fields**

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | `false` if the body name was not found in the model |
| `message` | `string` | Human-readable status or error description |

**Example: apply a 10 N push along X for 2 seconds at a 10 cm offset, with a 0.5 s ramp-down**

This applies constant force of 10N for 1.5 seconds and then this force decays linearly over the next 0.5 seconds.

```bash
ros2 service call /mujoco_ros2_control/my_plugin/apply_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{
    link_name: 'base_link',
    wrench: {
      force:  {x: 10.0, y: 0.0, z: 0.0},
      torque: {x:  0.0, y: 0.0, z: 0.0}
    },
    application_point: {x: 0.1, y: 0.0, z: 0.0},
    duration: {sec: 2, nanosec: 0},
    ramp_down_duration: {sec: 0, nanosec: 500000000}
  }"
```

> [!NOTE]
> The service call **blocks** until the full `duration` has elapsed and then returns the response. For long-duration wrenches, call the service from a separate terminal or use an async client.

**Visualization**

While a wrench is active, arrow markers are published to `~/wrench_markers` for display in RViz:
- **Red arrow** — force vector, originating at the application point (body frame)
- **Cyan arrow** — torque vector, originating at the application point (body frame)

Each marker is published in the body's TF frame (i.e. `header.frame_id = link_name`), so RViz will correctly follow the body as it moves. Add a `MarkerArray` display in RViz pointed at the topic and ensure the body's TF frame is being broadcast.

**Parameters**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `force_arrow_scale` | `double` | `0.01` | Arrow length per unit force [m/N]. A 100 N force → 1 m arrow. |
| `torque_arrow_scale` | `double` | `0.1` | Arrow length per unit torque [m/(N·m)]. A 10 N·m torque → 1 m arrow. |
| `marker_frame_id` | `string` | `"base_link"` | TF frame ID used in the marker DELETEALL message sent when all wrenches expire. Active wrench markers always use the body's own TF frame (`link_name`). |

**Example configuration**

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      external_wrench:
        type: "mujoco_ros2_control_plugins/ExternalWrenchPlugin"
        force_arrow_scale: 0.01      # 100 N  → 1 m arrow
        torque_arrow_scale: 0.1      # 10 N·m → 1 m arrow
        marker_frame_id: "base_link"
```

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

1. **Initialization** (`init`): Called once when the plugin is loaded. Use this to read parameters and set up publishers, subscribers, and services.
2. **Update** (`update`): Called every simulation step at the **end of the `read` loop**, before the controller update and `write` loops. Changes to `mjData` here are visible to controllers and affect the next simulation step. This runs in a real-time thread — avoid blocking operations.
3. **Cleanup** (`cleanup`): Called when shutting down. Release any resources acquired in `init`.
