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
