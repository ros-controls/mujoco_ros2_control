:github_url: https://github.com/ros-controls/mujoco_ros2_control/blob/{REPOS_FILE_BRANCH}/doc/index.rst

.. _mujoco_ros2_control:

=====================
mujoco_ros2_control
=====================

This is a ROS 2 package for integrating the *ros2_control* controller architecture with the `MuJoCo <https://mujoco.readthedocs.io>`__ physics simulator.

This package provides a *ros2_control* system interface plugin that wraps MuJoCo's `Simulate App <https://github.com/google-deepmind/mujoco/tree/main/simulate>`__ to expose hardware interfaces, allowing robots described in MJCF or converted from URDF to be controlled via the standard ROS 2 control stack.

Key features:

* Full *ros2_control* ``SystemInterface`` plugin for MuJoCo
* MJCF/URDF conversion utilities to auto-generate MuJoCo models
* Optional plugin system for extending simulation with custom publishers and services
* Force-torque sensor, IMU, camera, and lidar support
* Step-by-step simulation control via ROS 2 services
* Example demos showing basic control, PID and transmission setups

Installation
============

Binary packages
---------------

``mujoco_ros2_control`` is released for ROS 2 {DISTRO} on Ubuntu. To use it, install the ``ros-{DISTRO}-mujoco-ros2-control`` package:

.. code-block:: shell

  sudo apt install ros-{DISTRO}-mujoco-ros2-control ros-{DISTRO}-mujoco-ros2-control-demos

Building from source
--------------------

To use the latest features or build against a custom MuJoCo version, build from source.
The package requires the ``mujoco_vendor`` package which provides the base MuJoCo install.

.. code-block:: shell

  mkdir -p ~/mujoco_ros2_control_ws/src
  cd ~/mujoco_ros2_control_ws/src
  git clone https://github.com/ros-controls/mujoco_ros2_control -b main
  git clone https://github.com/pal-robotics/mujoco_vendor -b master
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
  cd ~/mujoco_ros2_control_ws
  colcon build

Quick Start
===========

After installation, source the workspace and run a demo:

.. code-block:: shell

  source install/setup.bash
  ros2 launch mujoco_ros2_control_demos demo.launch.py

The demo launches the MuJoCo Simulate window with a test robot and the *ros2_control* stack connected.

Add ros2_control tag to a URDF
================================

To use *ros2_control* with your robot in MuJoCo, add a ``<ros2_control>`` tag to your URDF
pointing to the ``MujocoSystemInterface`` plugin and your MJCF scene file:

.. code-block:: xml

  <ros2_control name="MujocoSystem" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">$(find my_description)/description/scene.xml</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

Due to compatibility requirements, a slightly modified ``ros2_control`` node from this package must be used:

.. code-block:: python

  control_node = Node(
      package="mujoco_ros2_control",
      executable="ros2_control_node",
      output="both",
      parameters=[
          {"use_sim_time": True},
          controller_parameters,
      ],
  )

For the full plugin parameter reference, joint control modes, gripper/mimic joint setup,
sensors, cameras, and lidar configuration, see the `Hardware Interface` documentation linked below.

URDF to MJCF Conversion
========================

MuJoCo does not support the full feature set of xacro/URDFs. A *highly experimental* conversion
tool is provided to assist with converting robot description files to MJCF format.

See the `URDF to MJCF Conversion` documentation linked below for usage details, parameter reference, and examples.

mujoco_ros2_control_demos
==========================

Example launch files are provided in the ``mujoco_ros2_control_demos`` package.

.. code-block:: shell

  # Basic robot demo with multiple control interfaces
  ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py

  # MJCF generation from URDF at runtime
  ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py

  # PID control demo
  ros2 launch mujoco_ros2_control_demos 03_pid_control.launch.py

  # Transmission interface demo
  ros2 launch mujoco_ros2_control_demos 04_transmissions.launch.py

Support Matrix
==============

.. list-table::
   :header-rows: 1

   * - ROS 2 Distribution
     - Status
   * - Humble
     - Supported
   * - Jazzy
     - Supported
   * - Kilted
     - Supported
   * - Rolling
     - Supported (development)

Further Documentation
=====================

.. toctree::
   :titlesonly:

   Hardware Interface <../mujoco_ros2_control/docs/hardware_interface>
   URDF to MJCF Conversion <../mujoco_ros2_control/docs/tools>
   Modeling Tips <../mujoco_ros2_control/docs/modeling_tips>
   Tutorials <../mujoco_ros2_control_demos/doc/tutorials>
   Plugins <../mujoco_ros2_control_plugins/doc/plugins>
   Developers Guide <development>
