^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mujoco_ros2_control_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-08-18)
------------------
* Add base twist plugin for commanding the mobile base (`#254 <https://github.com/ros-controls/mujoco_ros2_control/issues/254>`_)
* Add support for Magnetometer sensor (`#257 <https://github.com/ros-controls/mujoco_ros2_control/issues/257>`_)
* Add missing dependencies (`#250 <https://github.com/ros-controls/mujoco_ros2_control/issues/250>`_)
* MuJoCo free joints state publisher plugin (`#247 <https://github.com/ros-controls/mujoco_ros2_control/issues/247>`_)
* Fix transmissions integration working with multiple actuators (`#241 <https://github.com/ros-controls/mujoco_ros2_control/issues/241>`_)
  Co-authored-by: Daniel Costanzi <daniel.costanzi@pal-robotics.com>
* Add service to set the free joint body state (`#233 <https://github.com/ros-controls/mujoco_ros2_control/issues/233>`_)
* Add pose sensor test to launch tests (`#232 <https://github.com/ros-controls/mujoco_ros2_control/issues/232>`_)
  Co-authored-by: Sebastian Castro <sebastian.a.castrofernandez@nasa.gov>
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Fix state passed into pose sensors (`#231 <https://github.com/ros-controls/mujoco_ros2_control/issues/231>`_)
* remove shebangs from launch files (`#227 <https://github.com/ros-controls/mujoco_ros2_control/issues/227>`_)
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Add support for pose sensors (`#220 <https://github.com/ros-controls/mujoco_ros2_control/issues/220>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Support polling, streaming, and disabled cameras in camera plugin (`#217 <https://github.com/ros-controls/mujoco_ros2_control/issues/217>`_)
* Refactor Rangefinder base Lidar as a Plugin (`#214 <https://github.com/ros-controls/mujoco_ros2_control/issues/214>`_)
* Refactor RGB-D Cameras to a Plugin (`#211 <https://github.com/ros-controls/mujoco_ros2_control/issues/211>`_)
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
* Migrate documentation to rst format for control.ros.org (`#188 <https://github.com/ros-controls/mujoco_ros2_control/issues/188>`_)
* Contributors: Christian Rauch, Erik Holum, Sai Kishor Kothakota, Sebastian Castro, msavchen-nasa

0.0.3 (2026-05-01)
------------------
* Switch to forward_command_controller dependency (`#173 <https://github.com/ros-controls/mujoco_ros2_control/issues/173>`_)
* Contributors: Christoph Fröhlich

0.0.2 (2026-03-17)
------------------
* Fix reset simulation service for robots using PID control (`#140 <https://github.com/ros-controls/mujoco_ros2_control/issues/140>`_)
* Cleanup duplicate files (`#146 <https://github.com/ros-controls/mujoco_ros2_control/issues/146>`_)
* Remove deprecated JointGroupPositionController with ForwardCommandController (`#147 <https://github.com/ros-controls/mujoco_ros2_control/issues/147>`_)
* [URDF->MJCF] fixed path for urdf with test (`#127 <https://github.com/ros-controls/mujoco_ros2_control/issues/127>`_)
* [Feature] MuJoCo ros2 control plugins (`#133 <https://github.com/ros-controls/mujoco_ros2_control/issues/133>`_)
* Use pixi-build-ros as the backend for pixi builds (`#130 <https://github.com/ros-controls/mujoco_ros2_control/issues/130>`_)
* Contributors: Christoph Fröhlich, Erik Holum, Ortisa, Sai Kishor Kothakota

0.0.1 (2026-02-24)
------------------
* Updates docs and comments (`#113 <https://github.com/ros-controls/mujoco_ros2_control/issues/113>`_)
* Cleanup of Docs and Dev Guides (`#111 <https://github.com/ros-controls/mujoco_ros2_control/issues/111>`_)
* Add mujoco_ros2_control_demos package and retarget tests (`#100 <https://github.com/ros-controls/mujoco_ros2_control/issues/100>`_)
* Contributors: Erik Holum, Louis LE LAY, Sai Kishor Kothakota
