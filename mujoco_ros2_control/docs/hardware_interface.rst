Hardware Interface Configuration
=================================

Plugin
------

The MuJoCo hardware interface is shipped as a ``ros2_control`` plugin.
Specify it in your URDF and point to a valid MJCF on launch:

.. code-block:: xml

   <ros2_control name="MujocoSystem" type="system">
     <hardware>
       <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>

       <!-- Path to the MuJoCo scene XML -->
       <param name="mujoco_model">$(find my_description)/description/scene.xml</param>

       <!-- Optional: load PID gains from a ROS parameters YAML file.
            Enables position/velocity PID control for velocity, motor, and custom actuators. -->
       <param name="pids_config_file">$(find my_description)/config/pids.yaml</param>

       <!-- Optional: override the Simulate App speed scaling.
            Allows running faster than real time (e.g. 5.0 = 500%). Omit or set <0
            to use the rate requested from the App window. -->
       <param name="sim_speed_factor">5.0</param>

       <!-- Optional: apply a named keyframe from the MJCF as the initial state.
            Has no effect if override_start_position_file is also set. -->
       <param name="initial_keyframe">optional_frame</param>

       <!-- Optional: load the full MuJoCo model state from a file saved via
            'Copy state' in the Simulate window. Mutually exclusive with initial_value
            on state interfaces. Ignored if the string is empty. -->
       <param name="override_start_position_file">$(find my_description)/config/start_positions.xml</param>

       <!-- Optional: topic from which the MJCF XML is read when mujoco_model is not set.
            Defaults to /mujoco_robot_description. -->
       <param name="mujoco_model_topic">/mujoco_robot_description</param>

       <!-- Optional: camera RGB-D image publish rate in Hz (all cameras share one rate).
            Defaults to 5 Hz. -->
       <param name="camera_publish_rate">6.0</param>

       <!-- Optional: lidar LaserScan publish rate in Hz (all lidar sensors share one rate). -->
       <param name="lidar_publish_rate">10.0</param>

       <!-- Optional: run the simulator without a GUI window. Defaults to false. -->
       <param name="headless">false</param>

       <!-- Optional: name of a MuJoCo free joint whose odometry is published as a
            nav_msgs/Odometry message. -->
       <param name="odom_free_joint_name">floating_base_joint</param>
       <param name="odom_topic">/simulator/floating_base_state</param>
     </hardware>
   ...

Due to compatibility requirements, a slightly modified ``ros2_control`` node is required.
It is the same executable and accepts the same parameters as the upstream node:

.. code-block:: python

   control_node = Node(
       # Use the node from this package
       package="mujoco_ros2_control",
       executable="ros2_control_node",
       output="both",
       parameters=[
           {"use_sim_time": True},
           controller_parameters,
       ],
   )

.. note::

   The custom node can be removed after the next ``ros2_control`` upstream release, which will include
   the required changes.

Joints
------

Joints in the ``ros2_control`` interface are mapped to actuators defined in the MJCF, either directly or as transmission interfaces.
The system supports different joint control modes based on the actuator type and available command interfaces.

MuJoCo's PD-level ``ctrl`` input is used for direct position, velocity, or effort control.
For velocity, motor, or custom actuators, a position or velocity PID is created if specified using ROS parameters.
Incompatible actuator-interface combinations trigger an error at startup.

Refer to MuJoCo's `actuation model <https://mujoco.readthedocs.io/en/stable/computation/index.html#geactuation>`_ for more information.

Only one type of MuJoCo actuator per-joint can be controllable at a time, and the type **cannot** be switched at runtime.
However, the active command interface can be switched dynamically, allowing control to shift between position, velocity, or effort as supported by the actuator type.

For example, a position-controlled joint in MJCF:

.. code-block:: xml

   <actuator>
     <position joint="joint1" name="joint1" kp="25000" dampratio="1.0" ctrlrange="0.0 2.0"/>
   </actuator>

Maps to the following ``ros2_control`` hardware interface:

.. code-block:: xml

   <joint name="joint1">
     <command_interface name="position"/>
     <!-- Initial values for state interfaces default to 0 if not specified -->
     <state_interface name="position">
       <param name="initial_value">0.0</param>
     </state_interface>
     <state_interface name="velocity"/>
     <state_interface name="effort"/>
   </joint>

**Supported modes between MuJoCo actuators and ros2_control command interfaces:**

.. list-table::
   :header-rows: 1
   :stub-columns: 1

   * - Command Interface
     - MuJoCo ``position``
     - MuJoCo ``velocity``
     - MuJoCo ``motor``, ``general``, etc.
   * - **position**
     - Native support
     - Supported using PIDs
     - Supported using PIDs
   * - **velocity**
     - Not supported
     - Native support
     - Supported using PIDs
   * - **effort**
     - Not supported
     - Not supported
     - Native support

.. note::

   The ``torque`` and ``force`` command/state interfaces are semantically equivalent to ``effort`` and map to the same underlying data in the sim.

Grippers and Mimic Joints
--------------------------

Many robot grippers include mimic joints, where a single actuator drives the state of multiple joints.
In the current implementation, drivers require a motor-type actuator for joint control and state information.
Tendons and other non-standard joint types in an MJCF are not directly controllable through the drivers.

For parallel jaw mechanisms and similar mimic joints, we recommend combining tendon actuators with an equality constraint.
For example, from the test robot:

.. code-block:: xml

   <actuator>
     <position tendon="split" name="gripper_left_finger_joint" kp="1000" dampratio="3.0" ctrlrange="-0.09 0.005"/>
   </actuator>
   <tendon>
     <fixed name="split">
       <joint joint="gripper_left_finger_joint" coef="0.5"/>
       <joint joint="gripper_right_finger_joint" coef="-0.5"/>
     </fixed>
   </tendon>
   <equality>
     <joint joint1="gripper_left_finger_joint" joint2="gripper_right_finger_joint" polycoef="0 -1 0 0 0" solimp="0.95 0.99 0.001" solref="0.005 1"/>
   </equality>

The tendon name matches the controllable joint in the ``ros2_control`` configuration.
The drivers expose control and state for that single joint, while the simulation enforces the mimic constraint internally.

Sensors
-------

The hardware interface supports force-torque sensors (FTS) and inertial measurement units (IMUs).
MuJoCo does not model complete FTS and IMUs natively, so we combine supported MJCF sensor constructs to map to a single ``ros2_control`` sensor.

Force-Torque Sensors
~~~~~~~~~~~~~~~~~~~~

Model ``force`` and ``torque`` sensors separately in the MJCF, suffixed with ``_force`` and ``_torque``:

.. code-block:: xml

   <sensor>
     <force name="fts_sensor_force" site="ft_frame"/>
     <torque name="fts_sensor_torque" site="ft_frame"/>
   </sensor>

Map them to a single ``ros2_control`` sensor:

.. code-block:: xml

   <sensor name="fts_sensor">
     <param name="mujoco_type">fts</param>
     <!-- mujoco_sensor_name does not need to match the ros2_control sensor name -->
     <param name="mujoco_sensor_name">fts_sensor</param>
     <!-- Defaults: _force and _torque -->
     <param name="force_mjcf_suffix">_force</param>
     <param name="torque_mjcf_suffix">_torque</param>
     <state_interface name="force.x"/>
     <state_interface name="force.y"/>
     <state_interface name="force.z"/>
     <state_interface name="torque.x"/>
     <state_interface name="torque.y"/>
     <state_interface name="torque.z"/>
   </sensor>

IMU
~~~

Simulate a ``framequat``, ``gyro``, and ``accelerometer`` as a single IMU:

.. code-block:: xml

   <sensor>
     <framequat name="imu_sensor_quat" objtype="site" objname="imu_sensor"/>
     <gyro name="imu_sensor_gyro" site="imu_sensor"/>
     <accelerometer name="imu_sensor_accel" site="imu_sensor"/>
   </sensor>

Map to the corresponding ``ros2_control`` sensor:

.. code-block:: xml

   <sensor name="imu_sensor">
     <param name="mujoco_type">imu</param>
     <!-- mujoco_sensor_name does not need to match the ros2_control sensor name -->
     <param name="mujoco_sensor_name">imu_sensor</param>
     <!-- Defaults: _quat, _gyro, _accel -->
     <param name="orientation_mjcf_suffix">_quat</param>
     <param name="angular_velocity_mjcf_suffix">_gyro</param>
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

These sensor state interfaces work out of the box with the standard ROS 2 broadcasters.

Cameras
-------

Any ``camera`` included in the MJCF will automatically have its RGB-D images and camera info published to ROS topics.

The camera ``name`` attribute sets the defaults for the frame and topic names:

- Frame: ``<name>_frame``
- Topics: ``<name>/camera_info``, ``<name>/color``, ``<name>/depth``

For example:

.. code-block:: xml

   <camera name="wrist_mounted_camera" fovy="58" mode="fixed" resolution="640 480" pos="0 0 0" quat="0 0 0 1"/>

Publishes the following topics:

.. code-block:: bash

   $ ros2 topic info /wrist_mounted_camera/camera_info
   Type: sensor_msgs/msg/CameraInfo
   $ ros2 topic info /wrist_mounted_camera/color
   Type: sensor_msgs/msg/Image
   $ ros2 topic info /wrist_mounted_camera/depth
   Type: sensor_msgs/msg/Image

Frame and topic names can be overridden via ``ros2_control`` xacro:

.. code-block:: xml

   <!-- The sensor name must match the camera name in the MJCF -->
   <sensor name="wrist_mounted_camera">
     <param name="frame_name">wrist_mounted_camera_mujoco_frame</param>
     <param name="info_topic">/wrist_mounted_camera/color/camera_info</param>
     <param name="image_topic">/wrist_mounted_camera/color/image_raw</param>
     <param name="depth_topic">/wrist_mounted_camera/aligned_depth_to_color/image_raw</param>
   </sensor>

.. note::

   MuJoCo's camera coordinate conventions differ from ROS. Refer to the MuJoCo documentation for details.

Lidar
-----

MuJoCo does not include native lidar support.
This package implements a ROS 2-like lidar by wrapping sets of
`rangefinders <https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-rangefinder>`_ together.

MuJoCo rangefinders measure the distance to the nearest surface along the positive ``Z`` axis of the sensor site.
The first rangefinder's ``Z`` axis (e.g. ``rf-00``) must align with the ROS 2 lidar sensor's positive ``X`` axis,
consistent with the `LaserScan <https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg#L10>`_ convention.

Use the ``replicate`` tag to add N sites at regular angular offsets:

.. code-block:: xml

   <replicate count="12" sep="-" offset="0 0 0" euler="0 0.025 0">
     <site name="rf" size="0.01" pos="0.0 0.0 0.0" quat="0.0 0.0 0.0 1.0"/>
   </replicate>

Attach rangefinder sensors to each site:

.. code-block:: xml

   <sensor>
     <rangefinder name="lidar" site="rf"/>
   </sensor>

Configure the lidar through ``ros2_control`` xacro:

.. code-block:: xml

   <sensor name="lidar">
     <param name="frame_name">lidar_sensor_frame</param>
     <param name="angle_increment">0.025</param>
     <param name="min_angle">-0.3</param>
     <param name="max_angle">0.3</param>
     <param name="range_min">0.05</param>
     <param name="range_max">10</param>
     <param name="laserscan_topic">/scan</param>
   </sensor>

Simulation Topics and Services
================================

Topics
------

``/mujoco_actuators_states`` (``sensor_msgs/msg/JointState``)
   Provides information on all internal MuJoCo joints, regardless of whether their interfaces are exposed via ``ros2_control``.

``/clock`` (``rosgraph_msgs/msg/Clock``)
   Contains the internal physics clock tracked by each MuJoCo simulation step.

Services
--------

``~/set_pause`` (``mujoco_ros2_control_msgs/srv/SetPause``)
   Pauses or resumes the simulation.

   - Set ``paused`` to ``true`` to pause, or ``false`` to resume.
   - Returns immediately with no blocking. Returns ``success = true`` even if already in the requested state.
   - When resuming, the physics loop automatically re-syncs its wall-clock reference so no catch-up steps are executed.

   .. code-block:: bash

      # Pause the simulation
      ros2 service call /ros2_control_node/set_pause mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"

      # Resume the simulation
      ros2 service call /ros2_control_node/set_pause mujoco_ros2_control_msgs/srv/SetPause "{paused: false}"

``~/reset_world`` (``mujoco_ros2_control_msgs/srv/ResetWorld``)
   Resets the simulation state.

   - If the optional ``keyframe`` string field is empty, the simulation is restored to the state captured at startup (initial joint positions, velocities, and control values).
   - If a ``keyframe`` name is provided, that named keyframe from the MJCF is applied instead.
   - Returns ``success`` and a human-readable ``message``.

   .. code-block:: bash

      # Reset to startup state
      ros2 service call /ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld "{}"

      # Reset to a named MJCF keyframe
      ros2 service call /ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld "{keyframe: 'home'}"

   .. important::

      If controllers are active during the service call, the robot may reset to the initial state and then immediately
      snap back to its previous commanded position. Deactivate any active joint controllers before calling this service.

``~/step_simulation`` (``mujoco_ros2_control_msgs/srv/StepSimulation``)
   Advances the paused simulation by an exact number of physics steps and blocks until all steps have completed.

   - ``steps`` (``uint32``): number of physics steps to execute. Must be ≥ 1.
   - Blocks until all requested steps finish, the simulation diverges, or a timeout is reached.
   - Returns ``success`` and a human-readable ``message``.
   - **The simulation must be paused** before calling this service. Returns ``success = false`` immediately if the simulation is running.
   - Timeout: whichever is larger — 30 s, or 10 ms × ``steps``.

   .. code-block:: bash

      # Step the simulation forward by 100 physics steps
      ros2 service call /ros2_control_node/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 100}"

Debugging
=========

The simulator provides several mechanisms for pausing execution and advancing it in a controlled, step-by-step fashion.
This is useful for inspecting robot state, verifying controller output, or reproducing intermittent issues.

Pausing the Simulation
----------------------

Click the **Pause** button in the MuJoCo Simulate window (or press **Space**) to pause the physics loop.
When paused, the simulation clock stops advancing and no physics steps are executed until explicitly requested.

Single-Stepping via the Keyboard
---------------------------------

While the simulation window is focused and the simulation is **paused**, press the **right arrow key** (``→``) to
advance the simulation by exactly one physics step.
Holding the key down advances the simulation continuously one step at a time, allowing slow frame-by-frame inspection.

The status overlay in the top-right corner of the simulation window shows the current state (``Running`` / ``Paused``)
and the total number of physics steps executed.

Single-Stepping via ROS 2 Service
-----------------------------------

The ``~/step_simulation`` service allows programmatic step-by-step control from the command line or from test/debug scripts.
This is particularly useful for automated testing scenarios where a reproducible sequence of physics steps is needed.

.. code-block:: bash

   # Pause the simulation first (from the UI or via the set_pause service), then:

   # Advance by a single physics step
   ros2 service call /ros2_control_node/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 1}"

   # Advance by 500 steps (blocks until complete)
   ros2 service call /ros2_control_node/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 500}"

The service call blocks until all requested steps have been executed, the simulation diverges, or a timeout is reached.
This makes it safe to pipeline service calls sequentially without additional synchronisation (send a command → step N times → read state → repeat).

.. note::

   ``~/step_simulation`` requires the simulation to be **paused**. Calling it while the simulation is running
   returns ``success: false`` immediately without executing any steps.
