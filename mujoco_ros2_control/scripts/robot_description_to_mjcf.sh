#!/bin/bash

# Copyright 2025 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This script checks if the dependencies trimesh, mujoco and obj2mjcf are installed in a virtual environment inside the ROS_HOME/ros2_control/.venv.
# If it is present, then sources it and runs the make_mjcf_from_robot_description.py script.
# If it is not present, it will create it and install the dependencies and the source it before running the make_mjcf_from_robot_description.py script.

export PYTHONKEYRING_BACKEND=keyring.backends.null.Keyring
export PYTHONUNBUFFERED=1

# Check if the ROS_HOME env is set, if not set it to $HOME/.ros
if [ -z "$ROS_HOME" ]; then
    ROS_HOME=$HOME/.ros
fi

# Check if the virtual environment is present
function check_virtual_env() {
    if [ ! -d "$ROS_HOME/ros2_control/.venv" ]; then
        echo "Virtual environment not found. Creating virtual environment in '$ROS_HOME/ros2_control/.venv'...."
        python3 -m venv --system-site-packages "$ROS_HOME/ros2_control/.venv"
    fi
    echo "Sourcing virtual environment from '$ROS_HOME/ros2_control/.venv'...."
    # shellcheck source=/dev/null
    source "$ROS_HOME/ros2_control/.venv/bin/activate"
}

# Check if the dependencies trimesh, mujoco and obj2mjcf are installed, if not install them
function check_dependencies() {
    if [ ! -f "$ROS_HOME/ros2_control/.venv/bin/activate" ]; then
        echo "Virtual environment : '$ROS_HOME/ros2_control/.venv' not found. Something went wrong. Aborting...."
        exit 1
    fi
    # check if the dependencies trimesh, mujoco, obj2mjcf and scipy are installed, if not install them
    if ! python3 -c "import trimesh; import mujoco; import obj2mjcf" &> /dev/null; then
        echo "Dependencies not found. Installing trimesh, mujoco, obj2mjcf, scipy...."
        start_time=$(date +%s)
        pip3 install --no-input --no-cache-dir --disable-pip-version-check trimesh mujoco obj2mjcf
        end_time=$(date +%s)
        echo "Successfully installed dependencies in $((end_time - start_time)) seconds."
    fi
}

check_virtual_env
#kill any running make_mjcf_from_robot_description.py processes
pkill -f make_mjcf_from_robot_description.py || true
check_dependencies

# check if the user just wants to install the dependencies or run the script
if [[ "$*" == *"--install-only"* ]]; then
    exit 0
fi

# Get all the arguments of the bash script and then forward it to the make_mjcf_from_robot_description.py script
exec python3 "$(dirname "$0")/make_mjcf_from_robot_description.py" "$@"
