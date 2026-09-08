MuJoCo ROS 2 Control Plugins
============================

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

.. _camera_plugin:

CameraPlugin
~~~~~~~~~~~~

Using the camera plugin will ensure that any ``camera`` included in the MJCF will automatically have its RGB-D images and camera info published to ROS topics.

The camera ``name`` attribute sets the defaults for the frame and topic names:

- Frame: ``<name>_frame``
- Topics: ``<name>/camera_info``, ``<name>/color``, ``<name>/depth``

For example, in an MJCF:

.. code-block:: xml

   <camera name="camera" fovy="58" mode="fixed" resolution="640 480" pos="0 0 0" quat="0 0 0 1"/>

Then including the plugin will publish the following topics:

.. code-block:: bash

   $ ros2 topic info /camera/camera_info
   Type: sensor_msgs/msg/CameraInfo
   $ ros2 topic info /camera/color
   Type: sensor_msgs/msg/Image
   $ ros2 topic info /camera/depth
   Type: sensor_msgs/msg/Image

Frame and topic names can be overridden using the yaml configuration.
Note that any number of cameras can be configured in the plugin configuration.

**Example configuration**

.. code-block:: yaml

    mujoco_camera_plugin:
      type: "mujoco_ros2_control_plugins/CameraPlugin"
      # Note all cameras are published at the same rate
      camera_publish_rate: 5.0
      camera:
        frame_name: camera_color_optical_frame
        info_topic: /camera_topic/color/camera_info
        image_topic: /camera_topic/color/image_raw
        depth_topic: /camera_topic/aligned_depth_to_color/image_raw


.. note::

   MuJoCo's camera coordinate conventions differ from ROS.
   Refer to the MuJoCo documentation for details.

Headless Rendering
^^^^^^^^^^^^^^^^^^

Camera rendering is supported in headless environments (without a display).
The system automatically detects whether a display is available:

* With display: Uses GLFW for OpenGL context creation (default behavior)
* Without display: Falls back to EGL for GPU-accelerated headless rendering

This allows camera topics to be published even when running in headless mode (e.g., on a server, in Docker containers, or in CI environments).

.. note::
   EGL requires proper GPU drivers and EGL libraries to be installed (e.g., libegl1-mesa on Ubuntu).
   If both GLFW and EGL fail to initialize, camera publishing will be disabled with a warning.


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

ExternalWrench Visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While wrenches are active, arrow markers are published to ``~/wrench_markers`` for display in RViz:

- **Red arrow** (``external_wrench/force`` namespace) — force vector, originating at the
  application point (body frame)
- **Cyan arrow** (``external_wrench/torque`` namespace) — torque vector, originating at the
  application point (body frame)

Each marker uses the body's TF frame as ``header.frame_id``, so RViz correctly follows the body
as it moves.
Add a ``MarkerArray`` display in RViz pointed at the topic and ensure the body's TF frames are
being broadcast.

ExternalWrench Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^

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

BaseVelocityPlugin
~~~~~~~~~~~~~~~~~~

Drives a mobile or floating-base robot from a commanded planar body velocity (``vx``, ``vy``,
yaw-rate) received on a ``cmd_vel``-style topic, without relying on wheel-ground contact.

Wheel-terrain friction/slip is often unreliable enough to make it a poor foundation for testing
navigation stacks. This plugin instead requests a hard **kinematic override** of the base body's
free-joint velocity every cycle: the (optionally clamped) commanded planar velocity is written
directly into the joint's ``qvel``, bypassing force/mass dynamics entirely for the driven DOFs.
There is no gain to tune and no convergence delay — the measured body velocity on the driven axes
is exactly the commanded velocity on the very next simulation step.

.. warning::

   Because the override is kinematic, it outranks MuJoCo's contact solver. **Colliding with a
   wall or obstacle will not slow the base down** on the driven axes — the commanded velocity is
   reasserted every cycle regardless of what any contact computed in between. If you need
   physically realistic collision response while driving the base, this plugin is not the right
   tool; the trade-off it makes is exact, disturbance-immune velocity tracking in exchange for
   giving up momentum-conserving contacts on the driven DOFs.

Only the planar degrees of freedom are driven: body-frame linear x/y and yaw-rate (about body z).
Vertical motion and roll/pitch are left entirely to gravity and contacts, so the base settles onto
the ground normally.

.. list-table::
   :widths: 25 75
   :header-rows: 0

   * - **Topic**
     - ``cmd_vel`` (``geometry_msgs/msg/Twist``, or ``TwistStamped`` if ``use_stamped_twist`` is
       set)

Velocity Override Behavior
^^^^^^^^^^^^^^^^^^^^^^^^^^

Each cycle, the commanded body-frame ``vx``/``vy`` is clamped to ``max_linear_velocity`` (preserving
direction) and rotated into the world frame using the body's current orientation, since a free
joint's linear ``qvel`` is expressed in the world frame. The commanded yaw-rate is clamped to
``max_yaw_rate`` and used as-is, since a free joint's rotational ``qvel`` is already expressed in
the body-local frame. The result is written directly into ``data->qvel`` during ``pre_step()``,
which runs on the physics thread immediately before every ``mj_step``, which will happen per physics
step!

A command that hasn't been refreshed within ``cmd_timeout`` seconds is treated as zero (safety
stop) rather than left to coast on the last commanded velocity.

BaseVelocity Parameters
^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 15 15 45
   :header-rows: 1

   * - Parameter
     - Type
     - Default
     - Description
   * - ``body``
     - ``string``
     - *(required)*
     - MJCF body name of the base. Must carry a ``<freejoint/>`` — ``init()`` fails otherwise,
       since there is no ``qvel`` to override without one.
   * - ``cmd_vel_topic``
     - ``string``
     - ``cmd_vel``
     - Command topic name.
   * - ``use_stamped_twist``
     - ``bool``
     - ``false``
     - Subscribe to ``geometry_msgs/TwistStamped`` instead of ``geometry_msgs/Twist`` (e.g. for
       Nav2, which publishes stamped twists by default in some configurations). Even when the param
       is set to ``true``, the header info is internally not used and only the time at which the
       message received internal to the plugin takes precedence.
   * - ``max_linear_velocity``
     - ``double``
     - ``+inf``
     - Clamps the commanded planar speed ``sqrt(vx^2+vy^2)`` [m/s]; unset (the default) passes
       the command through unclamped.
   * - ``max_yaw_rate``
     - ``double``
     - ``+inf``
     - Clamps the commanded yaw-rate [rad/s]; unset (the default) passes the command through
       unclamped.
   * - ``cmd_timeout``
     - ``double``
     - ``0.5``
     - Seconds since the last received command after which it is treated as zero.

.. note::

   ``rclcpp::Node::create_sub_node()`` (used to give this plugin its own topic/service
   namespace) does not namespace *parameters* -- they're always node-level. So the plugin's own
   parameters are declared under an explicitly-built ``mujoco_plugins.<instance_name>.`` prefix
   (matching the plugin's actual instance key in your YAML), rather than relying on the
   sub-node's namespace the way topics/services do. This mirrors the pattern used by
   ``CameraPlugin``/``RangefinderLidarPlugin``/``Mujoco3dLidarPlugin``.

**Example configuration**

.. code-block:: yaml

   /**:
     ros__parameters:
       mujoco_plugins:
         base_velocity_plugin:
           type: "mujoco_ros2_control_plugins/BaseVelocityPlugin"
           body: base_link
           cmd_vel_topic: /cmd_vel

**Example: teleop from the command line**

.. code-block:: bash

   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.5}, angular: {z: 0.2}}" --rate 10

.. note::

   Odometry for the floating base is published independently by ``mujoco_ros2_control`` itself
   (parameter ``odom_free_joint_name``, default topic ``/simulator/floating_base_state``) — this
   plugin only drives the base, it does not publish odometry.

FreeJointStatePublisherPlugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Publishes the pose and velocity of every MuJoCo free-joint body (loose objects, unattached
links, etc.) or a user-selected subset to a single topic, in a user-selectable reference
frame.

.. list-table::
   :widths: 25 75
   :header-rows: 0

   * - **Topic**
     - ``free_joint_states`` (``mujoco_ros2_control_msgs/msg/FreeJointStateArray``), configurable
       via the ``topic`` parameter

Each published entry uses the same ``FreeJointState`` layout as the ``~/set_free_joint_state``
service (see :ref:`simulation_topics_and_services`), so a received message's ``free_joints``
field can be fed straight into a ``SetFreeJointState`` request to reproduce the snapshotted
state.

Frame semantics
^^^^^^^^^^^^^^^

When ``frame_id`` is empty (the default), poses and twists are expressed in the **world** frame.
When it names another MuJoCo body, poses are expressed relative to that body's current world
pose, and twists are rotated into that body's current world orientation — the reference body's
own velocity is **not** subtracted, exactly mirroring how ``~/set_free_joint_state`` interprets a
non-empty ``frame_id``. This means a message published in frame ``X`` can be sent straight back
to ``~/set_free_joint_state`` with the same ``frame_id`` to recover the identical world-frame
state.

If ``frame_id`` names an unknown body, the plugin logs an error and falls back to the world frame
(published entries then carry an empty ``frame_id``, reflecting the frame actually used).

FreeJointStatePublisher Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 15 15 45
   :header-rows: 1

   * - Parameter
     - Type
     - Default
     - Description
   * - ``frame_id``
     - ``string``
     - ``""``
     - Name of the MuJoCo body every published pose/twist is expressed relative to. Empty means
       the world frame.
   * - ``body_names``
     - ``string[]``
     - ``[]``
     - Names of the free-joint bodies to publish. Empty means every free-joint body in the model.
       An unknown or non-free-joint name here fails plugin initialization.
   * - ``topic``
     - ``string``
     - ``free_joint_states``
     - Output topic name.
   * - ``publish_rate``
     - ``double``
     - ``50.0``
     - Publish frequency in Hz.

**Example configuration**

.. code-block:: yaml

   /**:
     ros__parameters:
       mujoco_plugins:
         free_joint_state_publisher:
           type: "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin"
           frame_id: ""              # world frame; set to a body name to publish relative poses
           body_names: []            # empty = all free-joint bodies
           topic: "free_joint_states"
           publish_rate: 50.0

**Example: monitoring free-joint bodies**

.. code-block:: bash

   ros2 topic echo /mujoco_ros2_control_node/free_joint_state_publisher/free_joint_states

.. _rangefinder_lidar_plugin:

RangefinderLidarPlugin
~~~~~~~~~~~~~~~~~~~~~~

.. warning::

   This plugin is included to support legacy implementations of rangefinder based lidar sensors.
   We do not recommend using this, and instead would direct users to the 3d lidar plugin for improved
   features and performance.

MuJoCo 3D Lidar Plugin
~~~~~~~~~~~~~~~~~~~~~~

MuJoCo does not include native lidar support.
This package implements lidar through a custom MuJoCo sensor extension in ``mujoco_extensions`` (``mujoco.plugin.lidar``) that uses
`mj_multiRay <https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-multiray>`_ to cast rays each simulation step.
Refer to the extension package for more information about the computation.

The ``Mujoco3dLidarPlugin`` wraps the underlying sensor to convert the raw data to relevant messages and publish them to ROS topics.
Specifically, the data for 2D (single-row) and 3D (multi-row) will be published as
`LaserScan <https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg>`_ or
`PointCloud2 <https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg>`_ messages respectively.

When using the ``Mujoco3dLidarPlugin``, every ``mujoco.plugin.lidar`` sensor will have its data published.

3D Lidar Parameters
^^^^^^^^^^^^^^^^^^^

Each sensor is individually configurable by name in the plugin's yaml.
The available parameters are:

.. list-table::
   :widths: 25 15 15 45
   :header-rows: 1

   * - Parameter
     - Type
     - Default
     - Description
   * - ``<sensor_name>.frame_name``
     - ``string``
     - ``<sensor_site_name>``
     - The frame name of the sensor in the URDF. Defaults to the site name from the MJCF.
   * - ``<sensor_name>.topic``
     - ``string``
     - ``/scan`` or ``/points``
     - Topic name to publish messages. Defaults to ``/scan`` for 2D sensors and ``/points`` for 3D sensors.

.. code-block:: yaml

  /**:
    ros__parameters:
        mujoco_3d_lidar_plugin:
          type: "mujoco_ros2_control_plugins/Mujoco3dLidarPlugin"
          2d_lidar:
            frame_name: "lidar_sensor_frame"
            topic: "/lidar_scan_2d"
          3d_lidar:
            frame_name: "3d_lidar_sensor_frame"
            topic: "/lidar_points_3d"

3D Lidar Usage
--------------

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

     // Override whichever of these you need -- both have a no-op default, see
     // "Plugin Lifecycle" below for how they differ.
     void update(const mjModel* model, mjData* data) override;
     void pre_step(mjData* data) override;

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
     // Called once per ros2_control write() cycle, on the control thread.
   }

   void MyCustomPlugin::pre_step(mjData* data)
   {
     // Called on the physics thread, immediately before every mj_step.
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
     pluginlib::pluginlib
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
2. **Update** (``update``, optional): Called once per ``ros2_control`` ``write()`` cycle, on the
   control thread. ``data`` is a recent snapshot, not the live simulation data. Use this for
   anything that doesn't need to run on exactly one physics step: publishing sensor data,
   servicing a trigger, etc. Most plugins in this package (``CameraPlugin``, the lidar plugins,
   ``HeartbeatPublisherPlugin``, ``FreeJointStatePublisherPlugin``) use only this hook.
3. **Pre-step** (``pre_step``, optional): Called on the physics thread, immediately before every
   ``mj_step`` -- including multiple times per outer iteration when the loop batches steps to
   catch up. ``data`` is the live simulation data: read and write it directly, with no separate
   command buffer, so an untouched entry keeps its last value. Use this for anything that must
   hold for exactly one physics step, such as ``BaseVelocityPlugin``'s kinematic velocity
   override. Runs with the simulation mutex held, so blocking here stalls the physics loop and
   native viewer too.
4. **Cleanup** (``cleanup``): Called when shutting down. Release any resources acquired in
   ``init``.

Both hooks default to doing nothing, so implement whichever fits (or both, or neither). See
``MuJoCoROS2ControlPluginBase``'s class doc comment in ``mujoco_ros2_control_plugins_base.hpp``
for the authoritative reference.


Building
--------

This package is part of the ``mujoco_ros2_control`` workspace:

.. code-block:: bash

   colcon build --packages-select mujoco_ros2_control_plugins
