#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

# Preserve DISPLAY for X11 forwarding
export DISPLAY=${DISPLAY:-:0}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-rosdev}

# Only build if needed (check if install directory exists)
if [ ! -d "/home/rosdev/ros2_ws/install" ]; then
    echo "Building workspace for the first time..."
    cd /home/rosdev/ros2_ws/
    rm -rf build install log
    source /opt/ros/"$ROS_DISTRO"/setup.bash
    colcon build --symlink-install
fi

# Source the setup files
source /opt/ros/"$ROS_DISTRO"/setup.bash
source /home/rosdev/ros2_ws/install/setup.bash

# Add sourcing to .bashrc (only if not already there)
grep -q "source '/opt/ros/$ROS_DISTRO/setup.bash'" ~/.bashrc || echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> ~/.bashrc
grep -q "source '/home/rosdev/ros2_ws/install/setup.bash'" ~/.bashrc || echo "source '/home/rosdev/ros2_ws/install/setup.bash'" >> ~/.bashrc

# Install ipykernel for Jupyter notebooks (only if not already installed)
python3 -m ipykernel install --user --name=ros2_workspace --display-name "ROS 2 Workspace" 2>/dev/null || true

exec "$@"