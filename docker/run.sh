#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="mujoco_ros2_simulation"
TAG="humble"

pushd "${SCRIPT_DIR}/.." > /dev/null || exit

# Run the container and mount the source into the workspace directory,
# set other defaults to avoid spamming networks.
docker run --rm \
           -it \
           --network host \
           -e DISPLAY \
           -e QT_X11_NO_MITSHM=1 \
           -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}" \
           --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,ro \
           --mount type=bind,src=.,dst="/opt/mujoco/ws/src/mujoco_ros2_simulation" \
           --name ${IMAGE_NAME} \
           ${IMAGE_NAME}:${TAG} \
           bash

popd > /dev/null || exit
