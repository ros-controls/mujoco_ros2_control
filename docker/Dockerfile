ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO} AS mujoco_ros

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Defaults for the MuJoCo install, CPU_ARCH may be set to aarch64 for arm cpus
ARG MUJOCO_VERSION=3.3.4
ARG CPU_ARCH=x86_64

ARG DEBIAN_FRONTEND=noninteractive

# Configure base dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
      libglfw3-dev \
      ros-${ROS_DISTRO}-xacro\
      wget

# Configure and install MuJoCo
ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
RUN mkdir -p ${MUJOCO_DIR}

RUN wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
RUN mkdir -p /home/mujoco
RUN tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}")

# Copy source
ENV ROS_WS="/opt/mujoco/ws"
RUN mkdir -p ${ROS_WS}/src/mujoco_ros2_simulation
WORKDIR ${ROS_WS}
COPY . src/mujoco_ros2_simulation/

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install -iry --from-paths src/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DBUILD_TESTING=ON

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo "source /entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
