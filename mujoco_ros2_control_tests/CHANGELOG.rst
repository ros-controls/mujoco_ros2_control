^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mujoco_ros2_control_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-08-18)
------------------
* Add support for Magnetometer sensor (`#257 <https://github.com/ros-controls/mujoco_ros2_control/issues/257>`_)
* Add missing dependencies (`#250 <https://github.com/ros-controls/mujoco_ros2_control/issues/250>`_)
* Fix transmissions integration working with multiple actuators (`#241 <https://github.com/ros-controls/mujoco_ros2_control/issues/241>`_)
  Co-authored-by: Daniel Costanzi <daniel.costanzi@pal-robotics.com>
* Add service to set the free joint body state (`#233 <https://github.com/ros-controls/mujoco_ros2_control/issues/233>`_)
* Add pose sensor test to launch tests (`#232 <https://github.com/ros-controls/mujoco_ros2_control/issues/232>`_)
  Co-authored-by: Sebastian Castro <sebastian.a.castrofernandez@nasa.gov>
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Switch to isolated launch tests (`#234 <https://github.com/ros-controls/mujoco_ros2_control/issues/234>`_)
* remove shebangs from launch files (`#227 <https://github.com/ros-controls/mujoco_ros2_control/issues/227>`_)
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Add a 3-D Lidar Extension (`#205 <https://github.com/ros-controls/mujoco_ros2_control/issues/205>`_)
  Co-authored-by: Nathan Dunkelberger <138718889+ndunkelb-nasa@users.noreply.github.com>
* Add support for "site" transmission type (`#154 <https://github.com/ros-controls/mujoco_ros2_control/issues/154>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Add support for pose sensors (`#220 <https://github.com/ros-controls/mujoco_ros2_control/issues/220>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* fixing plugin export so that downstream plugins don't fail (`#215 <https://github.com/ros-controls/mujoco_ros2_control/issues/215>`_)
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Re-add descoped ResetWorld functional test (`#206 <https://github.com/ros-controls/mujoco_ros2_control/issues/206>`_)
* Isolate flakey functional tests and update pixi (`#204 <https://github.com/ros-controls/mujoco_ros2_control/issues/204>`_)
* Support/cameras rendering/headless (`#197 <https://github.com/ros-controls/mujoco_ros2_control/issues/197>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Increase timeout for mjcf_generation* tests (`#201 <https://github.com/ros-controls/mujoco_ros2_control/issues/201>`_)
* Contributors: Christian Rauch, Christoph Fröhlich, Erik Holum, Nathan Dunkelberger, Sai Kishor Kothakota, Sebastian Castro, msavchen-nasa

0.0.3 (2026-05-01)
------------------
* Switch to forward_command_controller dependency (`#173 <https://github.com/ros-controls/mujoco_ros2_control/issues/173>`_)
* Add epsilon to avoid floating point comparison (`#166 <https://github.com/ros-controls/mujoco_ros2_control/issues/166>`_)
* Ensure the clock messages are flushed before time checking (`#153 <https://github.com/ros-controls/mujoco_ros2_control/issues/153>`_)
* [Feature] Add ability to run the simulation progress by steps upon pause  (`#139 <https://github.com/ros-controls/mujoco_ros2_control/issues/139>`_)
* Contributors: Christoph Fröhlich, Erik Holum, Sai Kishor Kothakota

0.0.2 (2026-03-17)
------------------
* Fix reset simulation service for robots using PID control (`#140 <https://github.com/ros-controls/mujoco_ros2_control/issues/140>`_)
* Use pixi-build-ros as the backend for pixi builds (`#130 <https://github.com/ros-controls/mujoco_ros2_control/issues/130>`_)
* Improve integration test reliability and speed for CI (`#131 <https://github.com/ros-controls/mujoco_ros2_control/issues/131>`_)
* Rename default node to mujoco_ros2_control_node (`#132 <https://github.com/ros-controls/mujoco_ros2_control/issues/132>`_)
* Contributors: Erik Holum, Sai Kishor Kothakota

0.0.1 (2026-02-24)
------------------
* Updates docs and comments (`#113 <https://github.com/ros-controls/mujoco_ros2_control/issues/113>`_)
* Add mujoco_ros2_control_demos package and retarget tests (`#100 <https://github.com/ros-controls/mujoco_ros2_control/issues/100>`_)
* Containerize pixi in CI (`#99 <https://github.com/ros-controls/mujoco_ros2_control/issues/99>`_)
* Adjust pixi CI job and bump versions (`#96 <https://github.com/ros-controls/mujoco_ros2_control/issues/96>`_)
* Add ResetWorld service to reset to a specific defined keyframe (`#95 <https://github.com/ros-controls/mujoco_ros2_control/issues/95>`_)
* Re-enable rolling workflow (`#91 <https://github.com/ros-controls/mujoco_ros2_control/issues/91>`_)
* Separate tests to mujoco_ros2_control_tests package (`#90 <https://github.com/ros-controls/mujoco_ros2_control/issues/90>`_)
* Contributors: Erik Holum, Louis LE LAY, Sai Kishor Kothakota
