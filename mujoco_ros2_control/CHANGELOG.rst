^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mujoco_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-08-18)
------------------
* Add a pre-step callback to the simulation (`#282 <https://github.com/ros-controls/mujoco_ros2_control/issues/282>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Fix control mode selection (`#281 <https://github.com/ros-controls/mujoco_ros2_control/issues/281>`_)
* Support intvelocity actuators (`#271 <https://github.com/ros-controls/mujoco_ros2_control/issues/271>`_)
* Add base twist plugin for commanding the mobile base (`#254 <https://github.com/ros-controls/mujoco_ros2_control/issues/254>`_)
* Refine reset world overrides functionality (`#263 <https://github.com/ros-controls/mujoco_ros2_control/issues/263>`_)
* Support joint overrides in reset world service (`#256 <https://github.com/ros-controls/mujoco_ros2_control/issues/256>`_)
* Support multiple sets of state interfaces from multiple sensor tags with same name (`#258 <https://github.com/ros-controls/mujoco_ros2_control/issues/258>`_)
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
* [Fix] Don't overwrite transmission-driven joint states in the direct-copy fallback (`#259 <https://github.com/ros-controls/mujoco_ros2_control/issues/259>`_)
* [doc] Fix undefined label errors (`#260 <https://github.com/ros-controls/mujoco_ros2_control/issues/260>`_)
* Add support for Magnetometer sensor (`#257 <https://github.com/ros-controls/mujoco_ros2_control/issues/257>`_)
* [Feature] modify elements of geom attribute (`#244 <https://github.com/ros-controls/mujoco_ros2_control/issues/244>`_)
* Add missing dependencies (`#250 <https://github.com/ros-controls/mujoco_ros2_control/issues/250>`_)
* [CI] Fix dependency declarations for bloom (`#226 <https://github.com/ros-controls/mujoco_ros2_control/issues/226>`_)
* Fix transmissions integration working with multiple actuators (`#241 <https://github.com/ros-controls/mujoco_ros2_control/issues/241>`_)
  Co-authored-by: Daniel Costanzi <daniel.costanzi@pal-robotics.com>
* Add service to set the free joint body state (`#233 <https://github.com/ros-controls/mujoco_ros2_control/issues/233>`_)
* Fix links in interface docs, use auto label generator like the upstream (`#238 <https://github.com/ros-controls/mujoco_ros2_control/issues/238>`_)
* Fix state passed into pose sensors (`#231 <https://github.com/ros-controls/mujoco_ros2_control/issues/231>`_)
* Fix controller manager rate collapse by decoupling control and physics locks (`#224 <https://github.com/ros-controls/mujoco_ros2_control/issues/224>`_)
* Use updated ament_index_cpp interfaces, re-add rolling (`#229 <https://github.com/ros-controls/mujoco_ros2_control/issues/229>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Add a 3-D Lidar Extension (`#205 <https://github.com/ros-controls/mujoco_ros2_control/issues/205>`_)
  Co-authored-by: Nathan Dunkelberger <138718889+ndunkelb-nasa@users.noreply.github.com>
* Add support for "site" transmission type (`#154 <https://github.com/ros-controls/mujoco_ros2_control/issues/154>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Add support for pose sensors (`#220 <https://github.com/ros-controls/mujoco_ros2_control/issues/220>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Fix deprecations in ament_index_cpp (`#145 <https://github.com/ros-controls/mujoco_ros2_control/issues/145>`_)
  Co-authored-by: Julia Jia <juliajster@gmail.com>
* Fix race condition on sim-display overlay text (`#222 <https://github.com/ros-controls/mujoco_ros2_control/issues/222>`_)
* Display the sim real time factor on the native viewer (`#189 <https://github.com/ros-controls/mujoco_ros2_control/issues/189>`_)
* Refactor Rangefinder base Lidar as a Plugin (`#214 <https://github.com/ros-controls/mujoco_ros2_control/issues/214>`_)
* Refactor RGB-D Cameras to a Plugin (`#211 <https://github.com/ros-controls/mujoco_ros2_control/issues/211>`_)
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
* Add 'lyrical' to ros_distro matrix in CI workflow (`#213 <https://github.com/ros-controls/mujoco_ros2_control/issues/213>`_)
* Fix scipy for openblas missing dependency (`#212 <https://github.com/ros-controls/mujoco_ros2_control/issues/212>`_)
* Isolate flakey functional tests and update pixi (`#204 <https://github.com/ros-controls/mujoco_ros2_control/issues/204>`_)
* Support/cameras rendering/headless (`#197 <https://github.com/ros-controls/mujoco_ros2_control/issues/197>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Fix glfw initialization on headless run (`#200 <https://github.com/ros-controls/mujoco_ros2_control/issues/200>`_)
* Add changes for mujoco vendor bump (`#202 <https://github.com/ros-controls/mujoco_ros2_control/issues/202>`_)
* Control and plugin data input cleanup (`#191 <https://github.com/ros-controls/mujoco_ros2_control/issues/191>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Migrate documentation to rst format for control.ros.org (`#188 <https://github.com/ros-controls/mujoco_ros2_control/issues/188>`_)
* Refactor the core mujoco simulation to be in its own container (`#175 <https://github.com/ros-controls/mujoco_ros2_control/issues/175>`_)
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
* Handle topic model loading failures with nullptr instead of exit (`#183 <https://github.com/ros-controls/mujoco_ros2_control/issues/183>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Improve LiDAR validation in URDF-to-MJCF conversion (`#182 <https://github.com/ros-controls/mujoco_ros2_control/issues/182>`_)
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Fix Sphinx xref_missing reference (`#186 <https://github.com/ros-controls/mujoco_ros2_control/issues/186>`_)
* Handle duplicate mesh file names in the conversion tool (`#171 <https://github.com/ros-controls/mujoco_ros2_control/issues/171>`_)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Contributors: Christian Rauch, Christoph Fröhlich, Erik Holum, Lang Qinglin, Marq Rasmussen, Ortisa, Sai Kishor Kothakota, Sebastian Castro, Tianlin Zhang, danielcostanzi18, msavchen-nasa

0.0.3 (2026-05-01)
------------------
* Add more useful plugins for the mujoco_ros2_control (`#165 <https://github.com/ros-controls/mujoco_ros2_control/issues/165>`_)
* Explicit scipy version for numpy >2.0 compatibility (`#177 <https://github.com/ros-controls/mujoco_ros2_control/issues/177>`_)
* Use pkg install dir for requirements in conversion script (`#170 <https://github.com/ros-controls/mujoco_ros2_control/issues/170>`_)
* Documentation on Generating MJCFs (`#167 <https://github.com/ros-controls/mujoco_ros2_control/issues/167>`_)
* Use requirements.txt to install python dependencies (`#151 <https://github.com/ros-controls/mujoco_ros2_control/issues/151>`_)
* Allow freejoint without name (`#163 <https://github.com/ros-controls/mujoco_ros2_control/issues/163>`_)
* Replace stream logging with compiled log statements (`#160 <https://github.com/ros-controls/mujoco_ros2_control/issues/160>`_)
* Increase timeout on sim being ready to 10s (`#161 <https://github.com/ros-controls/mujoco_ros2_control/issues/161>`_)
* improve debug logs (`#159 <https://github.com/ros-controls/mujoco_ros2_control/issues/159>`_)
* Search delimiter from back to support joint names with '/' (`#158 <https://github.com/ros-controls/mujoco_ros2_control/issues/158>`_)
* Use right arrow key to advance the simulation instead of S key (`#150 <https://github.com/ros-controls/mujoco_ros2_control/issues/150>`_)
* [Feature] Add ability to run the simulation progress by steps upon pause  (`#139 <https://github.com/ros-controls/mujoco_ros2_control/issues/139>`_)
* disable site visuals by default in the visualizer and cameras (`#149 <https://github.com/ros-controls/mujoco_ros2_control/issues/149>`_)
* Contributors: Christian Rauch, Emily Sheetz, Erik Holum, Nathan Dunkelberger, Sai Kishor Kothakota, Óscar Martínez Martínez, Mathias Lüdtke

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
