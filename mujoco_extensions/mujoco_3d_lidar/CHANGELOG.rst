^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mujoco_3d_lidar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-08-18)
------------------
* Make CI tests more robust (`#239 <https://github.com/ros-controls/mujoco_ros2_control/issues/239>`_)
  * Protect lidar timestamps from uninitialized plugin data
  * Different software and llvmpipe for tests
  * Actually wait for rendering and skip the tests if it's broken
  * Actually fail explicitly for uninitialized contexts
  * Remove magic numbers, standardize sleep, fix comments
* Add Plugin for the 3D Lidar Extension (`#207 <https://github.com/ros-controls/mujoco_ros2_control/issues/207>`_)
  * Add a 3D Lidar ROS 2 plugin
  * Fix merge issues and updated plugin build
  * Support async lidar
  * More ament build fun for rolling
  * Format fixes
  * fat fingered format fix
  * Ensure the plugin is available for tests
  * Clear out extra file
  * Self review, and minor cleanup
  * Make example crazy
  * Consolidate branches
  * Add doc
  * Add test, more logs
  * More test cleanup
  * Start times at 0
  * Possibly fix mujoco_vendor
  * Lamdas, so hot right now
  * Docs and comment nits
* Update Pixi packages with pixi.lock (`#223 <https://github.com/ros-controls/mujoco_ros2_control/issues/223>`_)
  * Update Pixi packages with pixi.lock
  * Conditional plugin init macro for diff mujoco versions
  * Formatting
  * One more lock update for good luck
  * dupe ns
  * Pin pytest to compatible version
  ---------
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Add a 3-D Lidar Extension (`#205 <https://github.com/ros-controls/mujoco_ros2_control/issues/205>`_)
  * Add standalone mujoco 3d lidar plugin
  * Add note about removing update rate to the readme
  * Rename to extensions
  * Add docs
  * Add extension docs
  * Update demo robots
  * fix path
  * Use ament for plugin identification
  * add todo
  * Fix pid xml
  * PR comments
  * Remove extraneous paths in lidar
  * mj_multiRay hasn't changed
  * Add a do nothing extension, can revert
  * Update mujoco_extensions/mujoco_3d_lidar/CMakeLists.txt
  Co-authored-by: Nathan Dunkelberger <138718889+ndunkelb-nasa@users.noreply.github.com>
  * source for ament before testing
  * Revert "Add a do nothing extension, can revert"
  This reverts commit 1f64aacfe715f916765f806f02d0d42326b6bc36.
  * fix typo
  * Update deps
  * Update docs
  * Remove the lidar extension from the demos for now
  * Update include and build
  * SebC nits
  * Fix out of bounds access
  * PR comments
  ---------
  Co-authored-by: Nathan Dunkelberger <138718889+ndunkelb-nasa@users.noreply.github.com>
* Contributors: Erik Holum, Sai Kishor Kothakota

* Make CI tests more robust (`#239 <https://github.com/ros-controls/mujoco_ros2_control/issues/239>`_)
  * Protect lidar timestamps from uninitialized plugin data
  * Different software and llvmpipe for tests
  * Actually wait for rendering and skip the tests if it's broken
  * Actually fail explicitly for uninitialized contexts
  * Remove magic numbers, standardize sleep, fix comments
* Add Plugin for the 3D Lidar Extension (`#207 <https://github.com/ros-controls/mujoco_ros2_control/issues/207>`_)
  * Add a 3D Lidar ROS 2 plugin
  * Fix merge issues and updated plugin build
  * Support async lidar
  * More ament build fun for rolling
  * Format fixes
  * fat fingered format fix
  * Ensure the plugin is available for tests
  * Clear out extra file
  * Self review, and minor cleanup
  * Make example crazy
  * Consolidate branches
  * Add doc
  * Add test, more logs
  * More test cleanup
  * Start times at 0
  * Possibly fix mujoco_vendor
  * Lamdas, so hot right now
  * Docs and comment nits
* Update Pixi packages with pixi.lock (`#223 <https://github.com/ros-controls/mujoco_ros2_control/issues/223>`_)
  * Update Pixi packages with pixi.lock
  * Conditional plugin init macro for diff mujoco versions
  * Formatting
  * One more lock update for good luck
  * dupe ns
  * Pin pytest to compatible version
  ---------
  Co-authored-by: Erik Holum <erik.holum@nasa.gov>
* Add a 3-D Lidar Extension (`#205 <https://github.com/ros-controls/mujoco_ros2_control/issues/205>`_)
  * Add standalone mujoco 3d lidar plugin
  * Add note about removing update rate to the readme
  * Rename to extensions
  * Add docs
  * Add extension docs
  * Update demo robots
  * fix path
  * Use ament for plugin identification
  * add todo
  * Fix pid xml
  * PR comments
  * Remove extraneous paths in lidar
  * mj_multiRay hasn't changed
  * Add a do nothing extension, can revert
  * Update mujoco_extensions/mujoco_3d_lidar/CMakeLists.txt
  Co-authored-by: Nathan Dunkelberger <138718889+ndunkelb-nasa@users.noreply.github.com>
  * source for ament before testing
  * Revert "Add a do nothing extension, can revert"
  This reverts commit 1f64aacfe715f916765f806f02d0d42326b6bc36.
  * fix typo
  * Update deps
  * Update docs
  * Remove the lidar extension from the demos for now
  * Update include and build
  * SebC nits
  * Fix out of bounds access
  * PR comments
  ---------
  Co-authored-by: Nathan Dunkelberger <138718889+ndunkelb-nasa@users.noreply.github.com>
* Contributors: Erik Holum, Sai Kishor Kothakota

0.0.3 (2026-05-01)
------------------

0.0.2 (2026-03-17)
------------------

0.0.1 (2026-02-24)
------------------
