.. _custom_mujoco_extensions:

Custom MuJoCo Extensions
========================

MuJoCo includes support for `custom extensions <https://mujoco.readthedocs.io/en/stable/programming/extension.html>`_ that can be loaded directly into the application at runtime.
This gives developers the ability to interact with the simulation at a very low level, and to connect custom applications that work directly with the ``Simulate`` application.
To build custom extensions, follow the guide in the upstream documentation.

Adding Extensions
-----------------

``mujoco_ros2_control`` automatically discovers and loads MuJoCo plugin libraries at runtime using two mechanisms:

1. **Vendor plugins**: Any ``.so`` files in the ``mujoco_plugin/`` directory next to the ``ros2_control_node`` executable (these come from ``mujoco_vendor``).
2. **Ament resource index**: Any ROS 2 package that registers itself as a ``mujoco_plugins`` resource provider.

To add a new MuJoCo plugin to the system, create a standard ament package that builds a shared library and register it with the ament resource index.
The ``mujoco_ros2_control`` node will find and load it automatically at startup.

CMakeLists.txt
~~~~~~~~~~~~~~

A minimal ``CMakeLists.txt`` for a MuJoCo plugin package:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.22)
    project(my_mujoco_plugin)

    find_package(ament_cmake REQUIRED)
    find_package(mujoco_vendor REQUIRED)

    add_library(my_plugin SHARED src/my_plugin.cpp src/register.cpp)
    target_link_libraries(my_plugin PRIVATE mujoco::mujoco)

    # Install the plugin .so into a known location
    install(TARGETS my_plugin
      LIBRARY DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
    )

    # Register with the ament resource index so mujoco_ros2_control discovers it
    ament_index_register_resource(mujoco_plugins
      CONTENT "lib/${PROJECT_NAME}/mujoco_plugin")

    ament_export_dependencies(mujoco_vendor)
    ament_package()


Using with ``simulate``
~~~~~~~~~~~~~~~~~~~~~~~

The standalone ``simulate`` application only loads plugins from the ``mujoco_plugin/`` directory next to its own binary.
Users can attempt to install the plugin to that directory by adding a best-effort copy to ``CMakeLists.txt``:

.. code-block:: cmake

    find_program(mujoco_SIMULATE NAMES simulate mujoco-simulate)
    if(mujoco_SIMULATE)
      cmake_path(SET mujoco_BIN_PATH NORMALIZE ${mujoco_SIMULATE}/..)
      install(CODE "
        execute_process(
          COMMAND cp -f \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mujoco_plugin/libmy_plugin.so\"
                        \"${mujoco_BIN_PATH}/mujoco_plugin/\"
          RESULT_VARIABLE _err)
        if(_err)
          message(WARNING \"Could not install plugin next to simulate! Manually copy it to ${mujoco_BIN_PATH}/mujoco_plugin/\")
        endif()
      ")
    endif()

If the copy fails due to permissions, the warning message will include the exact ``cp`` command to run manually (e.g., with ``sudo``).

Supported Extensions
--------------------

3D Lidar (``mujoco.plugin.lidar``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This sensor uses ray casting to simulate lidar.

A ``lidar`` sensor is associated with a site and finds the nearest collision points from the site along a set of vectors.
The vectors are determined by the size and field of view parameters.

- ``resolution``: Horizontal and vertical ray count (e.g., ``"360 1"`` for a 2D scanner, ``"24 10"`` for 3D).
- ``azimuth_range``: Min and max horizontal angles in radians (e.g., ``"-3.14159 3.14159"``).
- ``elevation_range``: Min and max vertical angles in radians. For 2D sensors, a single value is sufficient (e.g., ``"0.0"``).
- ``max_range``: Maximum detection distance in meters.
- ``min_range``: Minimum detection distance in meters (optional, defaults to 0).
- ``update_rate``: Sensor update frequency in Hz (optional, defaults to every timestep).

    * Setting an ``update_rate`` can drastically save time when calling ``mj_step``.
    * This may be removed in future versions after the `interval attribute <https://mujoco.readthedocs.io/en/stable/modeling.html#sensors>`_ is available for sensors.

- ``async``: Whether or not to do raycasting synchronously or in a background thread.

    * For high density sensors, background processing will help avoid blocking ``mj_step`` for long periods.
    * Async processing requires a copy of the entirety of ``mjData``, so consumers should decide if the tradeoff is worth it.

Note that rays that hit nothing or fall outside the min/max range return ``-1``.

Example MJCF for a 2D lidar sensor:

.. code-block:: xml

   <extension>
     <plugin plugin="mujoco.plugin.lidar">
       <instance name="2d_lidar">
         <config key="resolution" value="24 1"/>
         <config key="azimuth_range" value="-0.3 0.3"/>
         <config key="elevation_range" value="0.0"/>
         <config key="max_range" value="10.0"/>
         <config key="min_range" value="0.0"/>
         <config key="update_rate" value="10.0"/>
         <config key="async" value="0"/>
       </instance>
     </plugin>
   </extension>

   <sensor>
     <plugin name="2d_lidar" instance="2d_lidar" objtype="site" objname="lidar_sensor_frame"/>
   </sensor>

For a 3D lidar, increase the vertical resolution and specify an elevation range.

.. code-block:: xml

   <extension>
     <plugin plugin="mujoco.plugin.lidar">
       <instance name="3d_lidar">
         <config key="resolution" value="24 50"/>
         <config key="azimuth_range" value="-0.3 0.3"/>
         <config key="elevation_range" value="-1.0 1.0"/>
         <config key="max_range" value="10.0"/>
         <config key="min_range" value="0.0"/>
         <config key="update_rate" value="1.0"/>
         <config key="async" value="0"/>
       </instance>
     </plugin>
   </extension>

   <sensor>
     <plugin name="3d_lidar" instance="3d_lidar" objtype="site" objname="3d_lidar_sensor_frame"/>
   </sensor>
