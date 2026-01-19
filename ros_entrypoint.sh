#!/usr/bin/env bash
set -e

# Source ROS base
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source workspace overlay if it exists (after you build it)
if [ -f "/home/${USERNAME}/ros2_ws/install/setup.bash" ]; then
  source "/home/${USERNAME}/ros2_ws/install/setup.bash"
fi

exec "$@"
