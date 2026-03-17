^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mujoco_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2026-03-17)
------------------
* Update internal links for myst-parser readthedocs (`#148 <https://github.com/ros-controls/mujoco_ros2_control/issues/148>`_)
* Fix reset simulation service for robots using PID control (`#140 <https://github.com/ros-controls/mujoco_ros2_control/issues/140>`_)
* throw on failing init of mujoco_ros2_control_plugins (`#136 <https://github.com/ros-controls/mujoco_ros2_control/issues/136>`_)
* Cleanup duplicate files (`#146 <https://github.com/ros-controls/mujoco_ros2_control/issues/146>`_)
* Export mujoco_ros2_control_plugins (`#138 <https://github.com/ros-controls/mujoco_ros2_control/issues/138>`_)
* [URDF->MJCF] fixed path for urdf with test (`#127 <https://github.com/ros-controls/mujoco_ros2_control/issues/127>`_)
* Set MuJoCo install include dirs (`#135 <https://github.com/ros-controls/mujoco_ros2_control/issues/135>`_)
* [Feature] MuJoCo ros2 control plugins (`#133 <https://github.com/ros-controls/mujoco_ros2_control/issues/133>`_)
* Use pixi-build-ros as the backend for pixi builds (`#130 <https://github.com/ros-controls/mujoco_ros2_control/issues/130>`_)
* Rename default node to mujoco_ros2_control_node (`#132 <https://github.com/ros-controls/mujoco_ros2_control/issues/132>`_)
* Changes necessary to also work for MuJoCo 3.5.0 (`#123 <https://github.com/ros-controls/mujoco_ros2_control/issues/123>`_)
* Update license name to SPDX standard (`#129 <https://github.com/ros-controls/mujoco_ros2_control/issues/129>`_)
* Add minor fixes to the conversion methods (`#125 <https://github.com/ros-controls/mujoco_ros2_control/issues/125>`_)
* Update RPATH of the mujoco_vendor libraries (`#122 <https://github.com/ros-controls/mujoco_ros2_control/issues/122>`_)
* Improve testing for URDF->MJCF tooling (`#119 <https://github.com/ros-controls/mujoco_ros2_control/issues/119>`_)
* Add git dependency (`#121 <https://github.com/ros-controls/mujoco_ros2_control/issues/121>`_)
* Contributors: Christoph Fröhlich, Erik Holum, Julia Jia, Ortisa, Sai Kishor Kothakota, Nathan Dunkelberger, Jordan Palacios

0.0.1 (2026-02-24)
------------------
* Update documentation of URDF <-> MJCF tool (`#117 <https://github.com/ros-controls/mujoco_ros2_control/issues/117>`_)
* Add proper logging for the mimic joints (`#118 <https://github.com/ros-controls/mujoco_ros2_control/issues/118>`_)
* Fix mardown link checker CI + fix local references (`#114 <https://github.com/ros-controls/mujoco_ros2_control/issues/114>`_)
* Updates docs and comments (`#113 <https://github.com/ros-controls/mujoco_ros2_control/issues/113>`_)
* Cleanup of Docs and Dev Guides (`#111 <https://github.com/ros-controls/mujoco_ros2_control/issues/111>`_)
* Update other ROS Controls maintainers (`#112 <https://github.com/ros-controls/mujoco_ros2_control/issues/112>`_)
* Update README.md across repository (`#110 <https://github.com/ros-controls/mujoco_ros2_control/issues/110>`_)
* Conditioning visual fixes on if there are images or not for daes (`#109 <https://github.com/ros-controls/mujoco_ros2_control/issues/109>`_)
* Cleanup the non-vendor mujoco install (`#105 <https://github.com/ros-controls/mujoco_ros2_control/issues/105>`_)
* Feature: Add `--no-fuse` arg to preserve body hierarchy (`#93 <https://github.com/ros-controls/mujoco_ros2_control/issues/93>`_)
* Add `mujoco_vendor` integration (`#6 <https://github.com/ros-controls/mujoco_ros2_control/issues/6>`_)
* Containerize pixi in CI (`#99 <https://github.com/ros-controls/mujoco_ros2_control/issues/99>`_)
* Add ResetWorld service to reset to a specific defined keyframe (`#95 <https://github.com/ros-controls/mujoco_ros2_control/issues/95>`_)
* Separate tests to mujoco_ros2_control_tests package (`#90 <https://github.com/ros-controls/mujoco_ros2_control/issues/90>`_)
* Support for starting from a specific declared keyframe  (`#89 <https://github.com/ros-controls/mujoco_ros2_control/issues/89>`_)
* Add `reset_world` service functionality (`#88 <https://github.com/ros-controls/mujoco_ros2_control/issues/88>`_)
* Fix the issue with the PIDs applying to wrong variable (`#87 <https://github.com/ros-controls/mujoco_ros2_control/issues/87>`_)
* Move the main contents of mujoco_ros2_control to subfolder (`#76 <https://github.com/ros-controls/mujoco_ros2_control/issues/76>`_)
* Contributors: Erik Holum, Louis LE LAY, Nathan Dunkelberger, Sai Kishor Kothakota, Sergi de las Muelas
