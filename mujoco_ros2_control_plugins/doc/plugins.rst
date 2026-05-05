MuJoCo ROS 2 Control Plugins
=============================

The ``mujoco_ros2_control_plugins`` package provides a plugin interface for extending the
functionality of ``mujoco_ros2_control``.
This separation allows for modular, optional features without adding complexity to the core package.

.. note::

   This interface provides flexibility for accessing information from the MuJoCo model and data.
   Users are responsible for handling that data correctly and avoiding changes to critical information.


Available Plugins
-----------------

HeartbeatPublisherPlugin
~~~~~~~~~~~~~~~~~~~~~~~~

A simple demonstration plugin that publishes a heartbeat message every second to the
``/mujoco_heartbeat`` topic.

.. list-table::
   :widths: 25 75
   :header-rows: 0

   * - **Topic**
     - ``mujoco_heartbeat`` (``std_msgs/String``)
   * - **Rate**
     - 1 Hz
   * - **Message format**
     - ``"MuJoCo ROS2 Control Heartbeat #N | Simulation time: Xs"``

**Example: monitoring the heartbeat**

.. code-block:: bash

   # Terminal 1: launch your mujoco_ros2_control simulation
   ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py

   # Terminal 2: echo the heartbeat messages
   ros2 topic echo /mujoco_heartbeat


ExternalWrenchPlugin
~~~~~~~~~~~~~~~~~~~~~

Applies one or more external wrenches (force + torque) to named MuJoCo bodies for configurable
durations via a ROS 2 service.
Multiple wrenches can be submitted in a single call and each expires independently.

.. list-table::
   :widths: 25 75
   :header-rows: 0

   * - **Service**
     - ``~/apply_wrench`` (``mujoco_ros2_control_msgs/srv/ApplyExternalWrench``)
   * - **Topic**
     - ``~/wrench_markers`` (``visualization_msgs/msg/MarkerArray``)

Service Request
^^^^^^^^^^^^^^^

The request contains a single ``wrenches`` field of type
``mujoco_ros2_control_msgs/ExternalWrenchArray``, which holds an array of ``ExternalWrench``
messages.
All wrenches in the array are validated atomically — if any body name is unknown the entire
request is rejected and nothing is applied.

Each ``ExternalWrench`` in the array has:

.. list-table::
   :widths: 30 30 40
   :header-rows: 1

   * - Field
     - Type
     - Description
   * - ``wrench.header.frame_id``
     - ``string``
     - MuJoCo body name (must match the MJCF ``<body name="...">``)
   * - ``wrench.wrench.force``
     - ``geometry_msgs/Vector3``
     - Linear force [N] expressed in the **body (link) frame**. Rotates with the body every
       simulation step.
   * - ``wrench.wrench.torque``
     - ``geometry_msgs/Vector3``
     - Angular moment [N·m] expressed in the **body (link) frame**. Rotates with the body every
       simulation step.
   * - ``application_point``
     - ``geometry_msgs/Point``
     - Force application point in the **body (link) frame** (relative to body frame origin,
       metres). Zero → apply at the body frame origin.
   * - ``duration``
     - ``builtin_interfaces/Duration``
     - How long the wrench remains active. Zero → single simulation step.
   * - ``ramp_down_duration``
     - ``builtin_interfaces/Duration``
     - Duration over which the wrench linearly ramps from full magnitude to zero at the end of
       ``duration``. Zero → no ramp-down.

Service Response
^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 20 20 60
   :header-rows: 1

   * - Field
     - Type
     - Description
   * - ``success``
     - ``bool``
     - ``false`` if any body name was not found in the model
   * - ``message``
     - ``string``
     - Human-readable status or error description

.. note::

   The service call **blocks** until the longest ``duration`` in the array has elapsed, then
   returns the response.
   For long-duration wrenches, call the service from a separate terminal or use an async client.

**Example: apply a 10 N push along X for 2 seconds at a 10 cm offset, with a 0.5 s ramp-down**

This applies a constant force of 10 N for 1.5 seconds and then decays linearly over the next
0.5 seconds.

.. code-block:: bash

   ros2 service call /external_wrench/apply_wrench \
     mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
     "{
       wrenches: {
         external_wrenches: [
           {
             wrench: {
               header: {frame_id: 'base_link'},
               wrench: {
                 force:  {x: 10.0, y: 0.0, z: 0.0},
                 torque: {x:  0.0, y: 0.0, z: 0.0}
               }
             },
             application_point: {x: 0.1, y: 0.0, z: 0.0},
             duration: {sec: 2, nanosec: 0},
             ramp_down_duration: {sec: 0, nanosec: 500000000}
           }
         ]
       }
     }"

**Example: apply two simultaneous wrenches in a single call**

.. code-block:: bash

   ros2 service call /mujoco_ros2_control/my_plugin/apply_wrench \
     mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
     "{
       wrenches: {
         external_wrenches: [
           {
             wrench: {header: {frame_id: 'link_a'}, wrench: {force: {x: 5.0, y: 0.0, z: 0.0}}},
             duration: {sec: 1, nanosec: 0}
           },
           {
             wrench: {header: {frame_id: 'link_b'}, wrench: {force: {x: 0.0, y: -3.0, z: 0.0}}},
             duration: {sec: 1, nanosec: 0}
           }
         ]
       }
     }"

Visualization
^^^^^^^^^^^^^

While wrenches are active, arrow markers are published to ``~/wrench_markers`` for display in RViz:

- **Red arrow** (``external_wrench/force`` namespace) — force vector, originating at the
  application point (body frame)
- **Cyan arrow** (``external_wrench/torque`` namespace) — torque vector, originating at the
  application point (body frame)

Each marker uses the body's TF frame as ``header.frame_id``, so RViz correctly follows the body
as it moves.
Add a ``MarkerArray`` display in RViz pointed at the topic and ensure the body's TF frames are
being broadcast.

Parameters
^^^^^^^^^^

.. list-table::
   :widths: 25 15 15 45
   :header-rows: 1

   * - Parameter
     - Type
     - Default
     - Description
   * - ``force_arrow_scale``
     - ``double``
     - ``0.01``
     - Arrow length per unit force [m/N]. A 100 N force → 1 m arrow.
   * - ``torque_arrow_scale``
     - ``double``
     - ``0.1``
     - Arrow length per unit torque [m/(N·m)]. A 10 N·m torque → 1 m arrow.

**Example configuration**

.. code-block:: yaml

   /**:
     ros__parameters:
       mujoco_plugins:
         external_wrench:
           type: "mujoco_ros2_control_plugins/ExternalWrenchPlugin"
           force_arrow_scale: 0.01      # 100 N  → 1 m arrow
           torque_arrow_scale: 0.1      # 10 N·m → 1 m arrow


Usage
-----

Plugins are loaded from ROS 2 parameters under ``mujoco_plugins``.
Each plugin entry requires:

- A unique key (e.g. ``heart_beat_plugin``)
- A ``type`` field with the pluginlib class name

.. code-block:: yaml

   /**:
     ros__parameters:
       mujoco_plugins:
         heart_beat_plugin:
           type: "mujoco_ros2_control_plugins/HeartbeatPublisherPlugin"
           update_rate: 1.0

Pass this file to the ``mujoco_ros2_control`` node via ``ParameterFile(...)`` in your launch file.

.. note::

   In this repository, ``mujoco_ros2_control_demos/launch/01_basic_robot.launch.py`` already loads
   ``mujoco_ros2_control_demos/config/mujoco_ros2_control_plugins.yaml``.


Creating Your Own Plugin
------------------------

1. Create the Plugin Header
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a header that inherits from ``MuJoCoROS2ControlPluginBase``:

.. code-block:: cpp

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

2. Implement the Plugin Methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

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

3. Create the Plugin XML Descriptor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create ``my_plugins.xml``:

.. code-block:: xml

   <library path="my_plugin_library">
     <class name="my_namespace/MyCustomPlugin"
            type="my_namespace::MyCustomPlugin"
            base_class_type="mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase">
       <description>
         Description of what your plugin does.
       </description>
     </class>
   </library>

4. Update CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cmake

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


Plugin Lifecycle
----------------

1. **Initialization** (``init``): Called once when the plugin is loaded. Use this to read
   parameters and set up publishers, subscribers, and services.
2. **Update** (``update``): Called every simulation step at the **end of the** ``read`` **loop**,
   before the controller update and ``write`` loops. Changes to ``mjData`` here are visible to
   controllers and affect the next simulation step. This runs in a real-time thread — avoid
   blocking operations.
3. **Cleanup** (``cleanup``): Called when shutting down. Release any resources acquired in
   ``init``.


Building
--------

This package is part of the ``mujoco_ros2_control`` workspace:

.. code-block:: bash

   colcon build --packages-select mujoco_ros2_control_plugins
