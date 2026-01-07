ARG ROS_DISTRO=jazzy

# This layer grabs package manifests from the src directory for preserving rosdep installs.
# This can significantly speed up rebuilds for the base package when src contents have changed.
FROM alpine:latest AS package-manifests

# Copy in the src directory, then remove everything that isn't a manifest or an ignore file.
COPY . /src/
RUN find /src -type f ! -name "package.xml" ! -name "COLCON_IGNORE" -delete && \
    find /src -type d -empty -delete

FROM ros:${ROS_DISTRO} AS mujoco_ros

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Defaults for the MuJoCo install, CPU_ARCH may be set to aarch64 for arm cpus
ARG MUJOCO_VERSION=3.4.0
ARG CPU_ARCH=x86_64

ARG DEBIAN_FRONTEND=noninteractive

# Configure base dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
      gdb \
      libglfw3-dev \
      libvirglrenderer1 \
      libgl1-mesa-dri \
      libglx-mesa0 \
      libxi6 \
      libxkbcommon0 \
      mesa-utils \
      python3-pip \
      tmux \
      wget \
      xterm \
      vim

# Install extra python dependencies
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pip install --ignore-installed mujoco obj2mjcf

# There's no build for arm64 on linux, so just ignore failures here if that's the case
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pip install bpy==4.0.0 --extra-index-url https://download.blender.org/pypi/ || true

# Configure and install MuJoCo
ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
ENV MUJOCO_INSTALL_DIR=${MUJOCO_DIR}
RUN mkdir -p ${MUJOCO_DIR}

RUN wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
RUN mkdir -p /home/mujoco
RUN tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}")

# Setup a ROS workspace
ENV ROS_WS="/opt/mujoco/ws"
RUN mkdir -p ${ROS_WS}/src/mujoco_ros2_control
WORKDIR ${ROS_WS}

# Copy package manifests for installing rosdeps
COPY --from=package-manifests /src/ ${ROS_WS}/src/mujoco_ros2_control

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install -iry --from-paths src/

# Copy remaining source
COPY . ${ROS_WS}/src/mujoco_ros2_control/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
      --symlink-install \
      --event-handlers console_direct+ \
      --cmake-args "-DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo"

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo "source /entrypoint.sh" >> ~/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
