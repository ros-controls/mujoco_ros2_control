# Developers Guide

This package can be built "normally" in a colcon workspace on any compatible system.
However, we also include two workflows that enable developers to work in a completely isolated system environment.
This ensure consistency with the supported workflows, and obviates the need to install any specific ROS, apt, or pip dependencies locally.

## Pixi Development Workflow

A [pixi](https://pixi.sh/latest/installation/) and [robostack](https://robostack.github.io) workflow is also provided.
The environment is currently only compatible with Jazzy.

To run ensure pixi is installed.
Then,

```bash
# Setup the build environment
pixi run setup-colcon

# Build the package
pixi run build

# Run tests
pixi run test
```

pixi also provides an interactive shell that sources the installed package environment.

```bash
# Launch an interactive shell environment and run things as usual
pixi shell

# Build things as normal
colcon build

# And source and launch the test application
source install/setup.bash
ros2 launch mujoco_ros2_control test_robot.launch.py
```

For more information on pixi and ROS refer to the documentation or this excellent [blog post](https://jafarabdi.github.io/blog/2025/ros2-pixi-dev/).

## Docker Development Workflow

This project includes a [compose](./../docker-compose.yml) and [Dockerfile](./../docker/Dockerfile) for development and testing in an isolated environment.

> [!NOTE]
> You may need to give docker access to xhost with `xhost +local:docker` to ensure the container has access to the host UI.

For users on arm64 machines, be sure to specify the `CPU_ARCH` variable in your environment when building.

```bash
docker compose build
```

The service can be started with:

```bash
# Start the service in one shell (or start detached)
docker compose up

# Connect to it in another
docker compose exec dev bash
```

This will launch a container with the source code mounted in a colcon workspace.
From there the source can be modified, built, tested, or otherwise used as normal.
For example, launch the included test scene with,

```bash
# Evaluate using the included mujoco simulate application
${MUJOCO_DIR}/bin/simulate ${ROS_WS}/src/mujoco_ros2_control/test/test_resources/scene.xml

# Or launch the test ROS control interface
ros2 launch mujoco_ros2_control test_robot.launch.py
```

> [!NOTE]
> Rendering contexts in containers can be tricky.

Users may need to tweak the compose file to support their specific host OS or GPUs.
For more information refer to the comments in the compose file.

## Development Workflow within ROS 2 Workspace

This package can be built in a standard ROS 2 workspace on Linux/Ubuntu. This workflow is recommended for integrating with existing ROS 2 workspaces or for developers working with rolling builds from `ros2_control` or `ros2_controllers` projects. For isolated development environments, consider using the [Docker Development Workflow](#docker-development-workflow) or [Pixi Development Workflow](#pixi-development-workflow) instead.

> [!WARNING]
> Special care is needed when building in a ROS 2 workspace because ROS 2 packages from system installations or rolling builds may not be compatible with the versions pinned in `pixi.lock`. Version mismatches can cause ABI incompatibilities leading to segmentation faults (SIGSEGV) during runtime, such as `ros2_control_node` crashing with exit code -11 during hardware initialization. Ensure that your ROS 2 workspace uses compatible package versions to avoid build conflicts and runtime crashes. See [issue #30](https://github.com/ros-controls/mujoco_ros2_control/issues/30) for more details.

### Dependencies

Install required system dependencies:

```bash
# Install ROS 2 dependencies via rosdep
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src --ignore-src -r -y

# Install additional build dependencies
sudo apt-get install libglfw3-dev sccache mold
```

**Note:** `mold` is a Linux-only linker optimization. On macOS/Windows, it will be automatically skipped during the build.

### Build

From your ROS 2 workspace root:

```bash
# Source ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Build the package
colcon build --symlink-install --packages-select mujoco_ros2_control

# Source the workspace
source install/setup.bash
```

The build system will:
- Automatically download MuJoCo 3.3.4 if not found locally
- Use `pkg-config` to find GLFW (system package) or fall back to CMake config (conda/pixi environments)
- Require `sccache` for compiler caching (cross-platform)
- Require `mold` linker on Linux for faster linking (skipped on other platforms)

If any required dependency is missing, CMake will provide clear error messages with installation instructions.
