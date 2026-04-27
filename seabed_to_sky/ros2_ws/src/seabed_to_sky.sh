#!/bin/bash
# seabed_to_sky.sh
# ================================================================================
# Master launch script for maritime robotics pipelines on Jetson + Ubuntu ARM
# Now with tmux support for parallel execution!
#
# Usage:
#   source seabed_to_sky.sh
#   launch_lio              # Launch LIO only
#   launch_sonar3d          # Launch sonar 3D only
#   launch_sonar_map        # Launch sonar mapping only
#   launch_all              # Launch all three in PARALLEL (tmux)
#   launch_all_sequential   # Launch all three one after another (old behavior)
#   launch_all_separate     # Launch all three in separate terminals (GUI)
#
# ================================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ================================================================================
# Configuration (Edit these paths if needed)
# ================================================================================

# LIO launch file
LIO_LAUNCH="spark_fast_lio os2.launch.yaml"
LIO_PACKAGE="spark_fast_lio"

# Sonar 3D reconstruction launch file
SONAR3D_LAUNCH="sonar3d_reconstruction acoustic3d.launch.py"
SONAR3D_PACKAGE="sonar3d_reconstruction"

# Sonar mapping launch file
SONAR_MAP_LAUNCH="sonar_map sonar_mapping.launch.py"
SONAR_MAP_PACKAGE="sonar_map"

# ================================================================================
# Helper Functions
# ================================================================================

_print_header() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

_print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

_print_error() {
    echo -e "${RED}✗ $1${NC}"
}

_print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

_check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        _print_error "ROS2 not found. Please source your ROS2 setup file first."
        _print_info "Example: source /opt/ros/humble/setup.bash"
        return 1
    fi
    return 0
}

_check_package() {
    local pkg=$1
    if ! ros2 pkg prefix "$pkg" &> /dev/null 2>&1; then
        _print_error "Package '$pkg' not found in ROS2 workspace"
        _print_info "Make sure you've built and sourced your workspace: colcon build && source install/setup.bash"
        return 1
    fi
    return 0
}

_check_tmux() {
    if ! command -v tmux &> /dev/null; then
        _print_error "tmux not found. Install it first:"
        _print_info "Ubuntu: sudo apt install tmux"
        _print_info "Then try again: launch_all"
        return 1
    fi
    return 0
}

# ================================================================================
# Launch Functions
# ================================================================================

launch_lio() {
    _print_header "🚀 Launching LIO (SPARK-FastLIO2)"
    
    _check_ros2 || return 1
    _check_package "$LIO_PACKAGE" || return 1
    
    _print_info "Starting: ros2 launch $LIO_LAUNCH"
    ros2 launch $LIO_LAUNCH
}

launch_sonar3d() {
    _print_header "🚀 Launching Sonar 3D Reconstruction"
    
    _check_ros2 || return 1
    _check_package "$SONAR3D_PACKAGE" || return 1
    
    _print_info "Starting: ros2 launch $SONAR3D_LAUNCH"
    ros2 launch $SONAR3D_LAUNCH
}

launch_sonar_map() {
    _print_header "🚀 Launching Sonar Mapping"
    
    _check_ros2 || return 1
    _check_package "$SONAR_MAP_PACKAGE" || return 1
    
    _print_info "Starting: ros2 launch $SONAR_MAP_LAUNCH"
    ros2 launch $SONAR_MAP_LAUNCH
}

launch_all() {
    _print_header "🚀 Launching All Pipelines (PARALLEL with tmux)"
    
    _check_ros2 || return 1
    _check_tmux || return 1
    _check_package "$LIO_PACKAGE" || return 1
    _check_package "$SONAR3D_PACKAGE" || return 1
    _check_package "$SONAR_MAP_PACKAGE" || return 1
    
    # Session name with timestamp to avoid conflicts
    local session_name="seabed_$(date +%s)"
    
    _print_info "Creating tmux session: $session_name"
    _print_info "All pipelines will run simultaneously in background"
    
    # Create tmux session with first window (LIO)
    tmux new-session -d -s "$session_name" -c "$(pwd)"
    
    # Window 1: LIO
    tmux send-keys -t "$session_name" "source ~/.bashrc; ros2 launch $LIO_LAUNCH" C-m
    sleep 2
    
    # Window 2: Sonar 3D
    tmux new-window -t "$session_name" -n "sonar3d"
    tmux send-keys -t "$session_name:sonar3d" "source ~/.bashrc; ros2 launch $SONAR3D_LAUNCH" C-m
    sleep 2
    
    # Window 3: Sonar Mapping
    tmux new-window -t "$session_name" -n "sonar_map"
    tmux send-keys -t "$session_name:sonar_map" "source ~/.bashrc; ros2 launch $SONAR_MAP_LAUNCH" C-m
    
    _print_success "All three pipelines launched in tmux session: $session_name"
    
    echo ""
    _print_info "Available commands:"
    echo "  tmux attach -t $session_name              # Attach to tmux session"
    echo "  tmux list-windows -t $session_name        # List all windows"
    echo "  tmux select-window -t $session_name:0     # Switch to LIO window"
    echo "  tmux select-window -t $session_name:1     # Switch to Sonar3D window"
    echo "  tmux select-window -t $session_name:2     # Switch to Sonar Mapping window"
    echo "  tmux kill-session -t $session_name        # Stop all pipelines"
    echo ""
    _print_info "Or use the convenience functions below:"
    echo "  seabed_attach                 # Attach to latest seabed session"
    echo "  seabed_stop                   # Stop all pipelines"
    echo "  seabed_list                   # List all active seabed sessions"
}

launch_all_sequential() {
    _print_header "🚀 Launching All Pipelines (Sequential - OLD BEHAVIOR)"
    
    _check_ros2 || return 1
    _check_package "$LIO_PACKAGE" || return 1
    _check_package "$SONAR3D_PACKAGE" || return 1
    _check_package "$SONAR_MAP_PACKAGE" || return 1
    
    _print_info "Order: LIO → Sonar3D → Sonar Mapping"
    _print_info "Press Ctrl+C to stop each pipeline and continue to the next"
    
    echo ""
    _print_header "Stage 1/3: LIO (SPARK-FastLIO2)"
    ros2 launch $LIO_LAUNCH
    
    echo ""
    _print_header "Stage 2/3: Sonar 3D Reconstruction"
    ros2 launch $SONAR3D_LAUNCH
    
    echo ""
    _print_header "Stage 3/3: Sonar Mapping"
    ros2 launch $SONAR_MAP_LAUNCH
}

launch_all_separate() {
    _print_header "🚀 Launching All Pipelines (Separate Terminals - GUI)"
    
    _check_ros2 || return 1
    _check_package "$LIO_PACKAGE" || return 1
    _check_package "$SONAR3D_PACKAGE" || return 1
    _check_package "$SONAR_MAP_PACKAGE" || return 1
    
    _print_info "Starting LIO in new terminal..."
    gnome-terminal -- bash -c "source ~/.bashrc && ros2 launch $LIO_LAUNCH; exec bash" 2>/dev/null || \
    xfce4-terminal -e "bash -c 'source ~/.bashrc && ros2 launch $LIO_LAUNCH; exec bash'" 2>/dev/null || \
    xterm -e "bash -c 'source ~/.bashrc && ros2 launch $LIO_LAUNCH; exec bash'" 2>/dev/null || \
    _print_error "Could not open terminal. Try launch_all (uses tmux)"
    
    sleep 2
    
    _print_info "Starting Sonar 3D in new terminal..."
    gnome-terminal -- bash -c "source ~/.bashrc && ros2 launch $SONAR3D_LAUNCH; exec bash" 2>/dev/null || \
    xfce4-terminal -e "bash -c 'source ~/.bashrc && ros2 launch $SONAR3D_LAUNCH; exec bash'" 2>/dev/null || \
    xterm -e "bash -c 'source ~/.bashrc && ros2 launch $SONAR3D_LAUNCH; exec bash'" 2>/dev/null || \
    _print_error "Could not open terminal. Try launch_all (uses tmux)"
    
    sleep 2
    
    _print_info "Starting Sonar Mapping in new terminal..."
    gnome-terminal -- bash -c "source ~/.bashrc && ros2 launch $SONAR_MAP_LAUNCH; exec bash" 2>/dev/null || \
    xfce4-terminal -e "bash -c 'source ~/.bashrc && ros2 launch $SONAR_MAP_LAUNCH; exec bash'" 2>/dev/null || \
    xterm -e "bash -c 'source ~/.bashrc && ros2 launch $SONAR_MAP_LAUNCH; exec bash'" 2>/dev/null || \
    _print_error "Could not open terminal. Try launch_all (uses tmux)"
    
    _print_success "All pipelines launched in separate terminals"
}

# ================================================================================
# Advanced Launch Functions
# ================================================================================

launch_lio_with_args() {
    _print_header "🚀 Launching LIO with Custom Arguments"
    
    _check_ros2 || return 1
    _check_package "$LIO_PACKAGE" || return 1
    
    if [ $# -eq 0 ]; then
        _print_error "No arguments provided"
        _print_info "Usage: launch_lio_with_args <arg1:=value1> <arg2:=value2> ..."
        _print_info "Example: launch_lio_with_args config_path:=/path/to/config.yaml"
        return 1
    fi
    
    _print_info "Starting: ros2 launch $LIO_LAUNCH $@"
    ros2 launch $LIO_LAUNCH "$@"
}

launch_sonar_map_with_args() {
    _print_header "🚀 Launching Sonar Mapping with Custom Arguments"
    
    _check_ros2 || return 1
    _check_package "$SONAR_MAP_PACKAGE" || return 1
    
    if [ $# -eq 0 ]; then
        _print_error "No arguments provided"
        _print_info "Usage: launch_sonar_map_with_args <arg1:=value1> <arg2:=value2> ..."
        _print_info "Example: launch_sonar_map_with_args sonar_frame:=oculus"
        return 1
    fi
    
    _print_info "Starting: ros2 launch $SONAR_MAP_LAUNCH $@"
    ros2 launch $SONAR_MAP_LAUNCH "$@"
}

# ================================================================================
# tmux Management Functions
# ================================================================================

seabed_attach() {
    _print_header "Attaching to Latest Seabed Session"
    
    local latest_session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_" | tail -1)
    
    if [ -z "$latest_session" ]; then
        _print_error "No active seabed tmux sessions found"
        _print_info "Start pipelines with: launch_all"
        return 1
    fi
    
    _print_success "Attaching to: $latest_session"
    tmux attach -t "$latest_session"
}

seabed_stop() {
    _print_header "Stopping All Seabed Pipelines"
    
    local sessions=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_")
    
    if [ -z "$sessions" ]; then
        _print_error "No active seabed sessions found"
        return 1
    fi
    
    while IFS= read -r session; do
        _print_info "Killing session: $session"
        tmux kill-session -t "$session"
    done <<< "$sessions"
    
    _print_success "All seabed sessions stopped"
}

seabed_list() {
    _print_header "Active Seabed Sessions"
    
    local sessions=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_")
    
    if [ -z "$sessions" ]; then
        _print_error "No active seabed sessions"
        return 1
    fi
    
    echo ""
    while IFS= read -r session; do
        echo -e "${GREEN}Session: $session${NC}"
        tmux list-windows -t "$session" -F "  Window #{window_index}: #{window_name}"
    done <<< "$sessions"
    echo ""
}

seabed_switch_lio() {
    local session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_" | tail -1)
    [ -z "$session" ] && { _print_error "No active session"; return 1; }
    tmux select-window -t "$session:0"
    tmux attach -t "$session"
}

seabed_switch_sonar3d() {
    local session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_" | tail -1)
    [ -z "$session" ] && { _print_error "No active session"; return 1; }
    tmux select-window -t "$session:1"
    tmux attach -t "$session"
}

seabed_switch_sonar_map() {
    local session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_" | tail -1)
    [ -z "$session" ] && { _print_error "No active session"; return 1; }
    tmux select-window -t "$session:2"
    tmux attach -t "$session"
}

# ================================================================================
# Utility Functions
# ================================================================================

seabed_status() {
    _print_header "System Status"
    
    _check_ros2 || return 1
    
    echo ""
    _print_info "Active ROS2 nodes:"
    ros2 node list 2>/dev/null || _print_error "Could not list nodes. ROS2 daemon might not be running."
    
    echo ""
    _print_info "Active topics:"
    ros2 topic list 2>/dev/null | head -20 || _print_error "Could not list topics."
    
    echo ""
    _print_info "Active tmux seabed sessions:"
    tmux list-sessions -F "#{session_name}" 2>/dev/null | grep "^seabed_" || _print_error "No active sessions"
}

seabed_help() {
    _print_header "Seabed-to-Sky Launch Commands"
    
    cat << EOF

📦 MAIN LAUNCH FUNCTIONS:

  launch_all              Launch all 3 pipelines SIMULTANEOUSLY (tmux) ⭐ RECOMMENDED
  launch_all_sequential   Launch all 3 one after another (press Ctrl+C between each)
  launch_all_separate     Launch all 3 in separate GUI terminals
  launch_lio              Launch LIO only
  launch_sonar3d          Launch Sonar 3D only
  launch_sonar_map        Launch Sonar mapping only

🎮 TMUX SESSION MANAGEMENT:

  seabed_attach           Attach to latest seabed session (see output)
  seabed_stop             Kill all seabed pipelines
  seabed_list             List all active seabed sessions
  seabed_switch_lio       Switch to LIO window
  seabed_switch_sonar3d   Switch to Sonar3D window
  seabed_switch_sonar_map Switch to Sonar Mapping window

🔧 ADVANCED:

  launch_lio_with_args <args>        Launch LIO with custom arguments
  launch_sonar_map_with_args <args>  Launch sonar mapping with custom arguments

  Example:
    launch_sonar_map_with_args sonar_frame:=oculus sonar_cloud_topic:=/my_topic

📊 UTILITIES:

  seabed_status          Show active nodes, topics, and sessions
  seabed_help            Show this help message

⚙️  CONFIGURATION:

  Edit this file to change launch file paths:
    LIO_LAUNCH="spark_fast_lio os2.launch.yaml"
    SONAR3D_LAUNCH="sonar3d_reconstruction acoustic3d.launch.py"
    SONAR_MAP_LAUNCH="sonar_map sonar_mapping.launch.py"

🔗 REQUIREMENTS:

  1. Source ROS2 setup:
     source /opt/ros/humble/setup.bash

  2. Source your workspace:
     source ~/ros2_ws/install/setup.bash

  3. Source this script:
     source seabed_to_sky.sh

  4. Install tmux (for parallel launches):
     sudo apt install tmux

📝 QUICK START (RECOMMENDED):

  # Launch all three pipelines in parallel
  launch_all

  # Attach to the session (see windows 0, 1, 2)
  seabed_attach

  # Switch between pipelines
  Ctrl+b 0    (LIO window)
  Ctrl+b 1    (Sonar3D window)
  Ctrl+b 2    (Sonar Mapping window)

  # Detach from tmux
  Ctrl+b d

  # Stop all pipelines
  seabed_stop

💡 TIPS:

  • All three pipelines run simultaneously
  • They keep running even if you detach
  • You can check logs in different windows
  • Each window runs independently
  • Use Ctrl+b ? for tmux help

EOF
}

# ================================================================================
# Initialization
# ================================================================================

_print_success "Seabed-to-Sky launch environment loaded!"
_print_info "Type 'seabed_help' for available commands"

# EOF