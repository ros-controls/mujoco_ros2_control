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

ARG DEBIAN_FRONTEND=noninteractive

# Configure base dependencies
# NOTE: A dist-update is required due to recently broken rolling images.
# https://openrobotics.zulipchat.com/#narrow/channel/526029-ROS-PMC-.28Restricted-posting.29/topic/2026-01-22.20Rolling.20Docker.20image.20out-of-date/near/569502119
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get dist-upgrade -y && \
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

# Install MuJoCo specific pip dependencies, ignoring the conflict in kilted and rolling images
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip install --ignore-installed typing-extensions && \
    pip install mujoco obj2mjcf trimesh pycollada

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
