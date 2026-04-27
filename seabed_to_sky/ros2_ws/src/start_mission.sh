#!/bin/bash
# start_mission.sh
# Launches the full seabed-to-sky pipeline inside a tmux session.
# Each node gets its own window so you can monitor or kill them independently.
#
# Usage (inside the container):
#   chmod +x start_mission.sh   # only needed once
#   ./start_mission.sh
#
# Attach to the session:
#   tmux attach -t mission
#
# Kill everything:
#   tmux kill-session -t mission

SESSION="mission"
SETUP="source /opt/ros/humble/setup.bash && source /home/rosdev/ros2_ws/install/setup.bash"

# If a session already exists, kill it and start fresh
tmux kill-session -t $SESSION 2>/dev/null

# Create session — first window: TF static transforms (must be up first)
tmux new-session  -d -s $SESSION -n "tf_static"
tmux send-keys    -t $SESSION:tf_static "$SETUP && ros2 launch tf_static_publisher tf_static_launch.py" Enter

# Window 1: LiDAR odometry (spark_fast_lio)
tmux new-window   -t $SESSION -n "lio"
tmux send-keys    -t $SESSION:lio "$SETUP && ros2 launch spark_fast_lio os2.launch.yaml" Enter

# Window 2: Sonar 3D reconstruction
tmux new-window   -t $SESSION -n "sonar3d"
tmux send-keys    -t $SESSION:sonar3d "$SETUP && ros2 launch sonar3d_reconstruction acoustic3d_launch.py" Enter

# Window 3: Sonar map
tmux new-window   -t $SESSION -n "sonar_map"
tmux send-keys    -t $SESSION:sonar_map "$SETUP && ros2 launch sonar_map sonar_mapping.launch.py" Enter

# Window 4: spare terminal (ros2 topic echo, service calls, etc.)
tmux new-window   -t $SESSION -n "shell"
tmux send-keys    -t $SESSION:shell "$SETUP" Enter

# Land on the first window
tmux select-window -t $SESSION:tf_static

echo "Mission launched — attaching to tmux session '$SESSION'"
echo "Windows: tf_static | lio | sonar3d | sonar_map | shell"
echo "Detach with Ctrl-b d  |  Kill all: tmux kill-session -t $SESSION"
sleep 1

tmux attach -t $SESSION
