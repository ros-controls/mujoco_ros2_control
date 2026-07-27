Demos and Tutorials
===================

The ``mujoco_ros2_control_demos`` package provides ready-to-run tutorials that demonstrate how to use MuJoCo with ``ros2_control``.
Each tutorial builds on the previous one and introduces a new concept.

.. note::

   All tutorials support a ``headless:=true`` argument to run without the MuJoCo visualiser window:

   .. code-block:: bash

      ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py headless:=true

.. tip::

   Inside the MuJoCo viewer, UI panels can be toggled with :kbd:`Tab` or :kbd:`Shift+Tab`.
   All standard MuJoCo keyboard shortcuts are available; press :kbd:`F1` for a short reference.


Tutorial 1: Basic Robot
-----------------------

The simplest setup — launches a two-link arm with position controllers using a pre-defined MJCF model.

.. code-block:: bash

   ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py

**Key concepts:** Pre-defined MJCF loading, basic ``ros2_control`` integration, position control.

**Resources:** ``demo_resources/scenes/scene.xml``, ``demo_resources/robot/test_robot.xml``


Tutorial 2: MJCF Generation at Runtime
---------------------------------------

Demonstrates generating MJCF models from URDF at runtime using the conversion script.

.. code-block:: bash

   # Using external input files
   ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py

   # Using mujoco_inputs embedded in the URDF
   ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py use_urdf_inputs:=true

**Key concepts:** Runtime URDF→MJCF conversion, ``<mujoco_inputs>`` tags, external input files.

**Resources:** ``demo_resources/mjcf_generation/test_inputs.xml``, ``demo_resources/scenes/scene_info.xml``

See :doc:`../../mujoco_ros2_control/docs/tools` for full documentation of the URDF-to-MJCF conversion tool.


Tutorial 3: PID Control
------------------------

Demonstrates PID controllers with motor actuators for velocity/effort control modes.

.. code-block:: bash

   ros2 launch mujoco_ros2_control_demos 03_pid_control.launch.py

**Key concepts:** PID gain configuration, motor actuators, velocity/effort control.

**Resources:** ``demo_resources/pid_control/test_robot_pid.xml``, ``config/mujoco_pid.yaml``

Refer to :doc:`../../mujoco_ros2_control/docs/hardware_interface` for the full set of hardware-interface
parameters, including ``pids_config_file``.


Tutorial 4: Transmissions
--------------------------

Demonstrates ``ros2_control`` transmissions with mechanical reduction ratios.

.. code-block:: bash

   ros2 launch mujoco_ros2_control_demos 04_transmissions.launch.py

**Key concepts:** ``DifferentialTransmission`` interface, mechanical reduction, actuator-to-joint mapping.

**Resources:** ``demo_resources/robot/test_robot.urdf`` with ``use_transmissions:=true``


Tutorial 5: Base Velocity Plugin
---------------------------------

Demonstrates driving a mobile/floating-base robot with ``BaseVelocityPlugin``: a free-floating,
wheeled chassis (MJCF ``<freejoint>``) carrying a 1-DOF arm, driven from a ``cmd_vel`` topic via a
hard velocity override applied directly to the chassis's free-joint ``qvel`` every cycle. The
wheels and ground have zero friction, which is now largely cosmetic -- propulsion no longer goes
through wheel-ground contact at all, so it does not depend on friction either way.

.. warning::

   Because the override is kinematic, it outranks the contact solver: the wall in the scene will
   **not** stop the base -- it will push through or climb it rather than being stopped by contact,
   since the commanded velocity is reasserted every cycle regardless of collisions. This is the
   trade-off for exact, disturbance-immune velocity tracking; see the ``BaseVelocityPlugin``
   documentation for details.

.. code-block:: bash

   ros2 launch mujoco_ros2_control_demos 05_base_velocity_plugin.launch.py

   # In another terminal, drive the base:
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" --rate 10

**Disturbance immunity**: swinging the mounted arm creates a reaction force/torque on the base
through the joint, unrelated to ground friction, since it acts directly between the two bodies.
Because ``BaseVelocityPlugin`` overrides the base's driven DOFs directly rather than servoing
against them, this reaction has *no* effect on the base's velocity at all -- not approximately,
exactly none -- so with no ``cmd_vel`` active the base stays exactly in place while the arm moves.
The arm's joint is range-limited to a back-and-forth sweep rather than a continuous spin:

.. code-block:: bash

   ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.7]"
   ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.7]"

**Key concepts:** ``BaseVelocityPlugin`` free-joint velocity override, driving a MJCF
``<freejoint>`` body, immunity to reaction forces from a moving mounted joint, floating-base
odometry (``odom_free_joint_name``).

**Resources:** ``demo_resources/scenes/scene_mobile_base.xml``, ``demo_resources/mobile_base/mobile_base.xml``,
``demo_resources/mobile_base/mobile_base.urdf``, ``config/mujoco_ros2_control_plugins_base_velocity.yaml``

See :doc:`../../mujoco_ros2_control_plugins/doc/plugins` for the full list of ``BaseVelocityPlugin`` parameters.


Combined Demo
-------------

The ``demo.launch.py`` file combines multiple features and is retained for backwards compatibility and
integration testing:

.. code-block:: bash

   ros2 launch mujoco_ros2_control_demos demo.launch.py
   ros2 launch mujoco_ros2_control_demos demo.launch.py use_pid:=true
   ros2 launch mujoco_ros2_control_demos demo.launch.py use_mjcf_from_topic:=true
   ros2 launch mujoco_ros2_control_demos demo.launch.py test_transmissions:=true


Controlling the Robot
---------------------

Once any tutorial is running you can send commands and inspect state with standard ROS 2 CLI tools:

.. code-block:: bash

   # Set joint positions (joint1, joint2)
   ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]"

   # Control the gripper
   ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.02]"

   # Monitor joint states
   ros2 topic echo /joint_states


Package Structure
-----------------

.. code-block:: text

   mujoco_ros2_control_demos/
   ├── launch/
   │   ├── 01_basic_robot.launch.py         # Tutorial 1
   │   ├── 02_mjcf_generation.launch.py     # Tutorial 2
   │   ├── 03_pid_control.launch.py         # Tutorial 3
   │   ├── 04_transmissions.launch.py       # Tutorial 4
   │   ├── 05_base_velocity_plugin.launch.py # Tutorial 5
   │   └── demo.launch.py                  # Combined demo
   ├── config/
   │   ├── controllers.yaml                        # Controller configuration
   │   ├── mujoco_pid.yaml                         # PID gains (Tutorial 3)
   │   ├── controllers_base_velocity.yaml          # joint_state_broadcaster + arm_position_controller (Tutorial 5)
   │   └── mujoco_ros2_control_plugins_base_velocity.yaml  # BaseVelocityPlugin config (Tutorial 5)
   └── demo_resources/
       ├── robot/
       │   ├── test_robot.urdf          # Shared URDF description
       │   └── test_robot.xml           # MJCF robot model
       ├── scenes/
       │   ├── scene.xml                # Basic scene (Tutorial 1, 4)
       │   ├── scene_pid.xml            # PID scene (Tutorial 3)
       │   ├── scene_info.xml           # Scene generation info
       │   └── scene_mobile_base.xml    # Mobile base + wall obstacle (Tutorial 5)
       ├── mjcf_generation/
       │   └── test_inputs.xml          # MJCF conversion inputs (Tutorial 2)
       ├── pid_control/
       │   └── test_robot_pid.xml       # Robot with motor actuators
       └── mobile_base/
           ├── mobile_base.xml          # MJCF chassis with freejoint (Tutorial 5)
           └── mobile_base.urdf         # Minimal URDF, no ros2_control joints (Tutorial 5)
