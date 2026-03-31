#!/bin/bash
# =============================================================================
#  start_robot.sh — Full robot stack startup for Jetson (physical hardware)
# =============================================================================
#
#  Launches in order:
#    1. andr_bringup launch — micro-ROS, RSP, RPLIDAR, EKF, SLAM, Nav2
#    2. ANDR stack          — agent, tools, managers, web UI
#
#  The ROS 2 navigation stack (micro-ROS agent, robot_state_publisher,
#  RPLIDAR, EKF, SLAM Toolbox, Nav2) is launched via:
#    ros2 launch andr_bringup robot.launch.py
#
#  All processes are tracked.  Ctrl-C or SIGTERM cleanly stops everything.
#
#  Usage:
#    chmod +x start_robot.sh
#    ./start_robot.sh
#
#  To run on boot, install the systemd service:
#    sudo cp andr_bot.service /etc/systemd/system/
#    sudo systemctl enable andr_bot
#    sudo systemctl start andr_bot
#
# =============================================================================

set -e

# ── Configuration ─────────────────────────────────────────────────────────────

# Directory this script lives in (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 Humble install
ROS_DISTRO="humble"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# ROS workspace (built with colcon — contains andr_bringup, robot_localization,
# rplidar_ros, micro_ros_agent, etc.)
# Change this if your workspace is elsewhere.
WORKSPACE_SETUP="${HOME}/ros2_ws/install/setup.bash"

# micro-ROS serial port / baud — passed to the launch file
MICRO_ROS_PORT="/dev/ttyUSB0"
MICRO_ROS_BAUD="115200"

# RPLIDAR serial port / baud — passed to the launch file
LIDAR_SERIAL_PORT="/dev/rplidar"
LIDAR_BAUDRATE="115200"

# Log directory
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "${LOG_DIR}"

# ── Helpers ───────────────────────────────────────────────────────────────────

log() { echo "[$(date '+%H:%M:%S')] $*"; }

# Array of background PIDs to kill on exit
PIDS=()

cleanup() {
    log "Shutting down robot stack..."
    # Kill in reverse launch order
    for (( i=${#PIDS[@]}-1; i>=0; i-- )); do
        pid="${PIDS[$i]}"
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
        fi
    done
    # Give processes a moment then force-kill any stragglers
    sleep 2
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null
        fi
    done
    log "Stack stopped."
}

trap cleanup EXIT SIGINT SIGTERM

# ── Source ROS environment ────────────────────────────────────────────────────

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 ${ROS_DISTRO} not found at ${ROS_SETUP}"
    exit 1
fi
source "${ROS_SETUP}"

if [[ -f "${WORKSPACE_SETUP}" ]]; then
    source "${WORKSPACE_SETUP}"
else
    log "WARNING: workspace not found at ${WORKSPACE_SETUP} — skipping."
    log "         Run: cd ~/ros2_ws && colcon build --symlink-install"
fi

# ── 1. andr_bringup launch ──────────────────────────────────────────────────
# Launches the full ROS 2 navigation stack via a single launch file:
#   micro-ROS agent → robot_state_publisher → RPLIDAR → EKF → SLAM → Nav2
#
# The launch file reads ~/andr_maps/slam_config.json to determine whether to
# start in mapping or localization mode (and which map to load).

log "Starting andr_bringup (micro-ROS, RSP, lidar, EKF, SLAM, Nav2)..."

ros2 launch andr_bringup robot.launch.py \
    use_sim_time:=false \
    micro_ros_port:="${MICRO_ROS_PORT}" \
    micro_ros_baud:="${MICRO_ROS_BAUD}" \
    lidar_serial_port:="${LIDAR_SERIAL_PORT}" \
    lidar_baudrate:="${LIDAR_BAUDRATE}" \
    launch_rviz:=false \
    >> "${LOG_DIR}/andr_bringup.log" 2>&1 &

PIDS+=($!)

# Wait for the navigation stack to initialize before starting ANDR
log "Waiting for navigation stack to initialize..."
sleep 8

# ── 2. ANDR stack ────────────────────────────────────────────────────────────
# Launches (auto-discovered by start.py):
#   managers/map_server.py      — waypoint / map management
#   tools/walk.py, spin.py, navigate_to_point.py
#   inputs/web_ui.py            — web dashboard on :8080
#   runnables/                  — any standalone processes

log "Starting ANDR stack..."

cd "${SCRIPT_DIR}"
python3 start.py >> "${LOG_DIR}/andr.log" 2>&1 &
PIDS+=($!)

# ── Done ──────────────────────────────────────────────────────────────────────

log "Robot stack is up. Web UI: http://localhost:8080"
log "Logs: ${LOG_DIR}/"
log "Press Ctrl-C to stop."

# Wait for any process to exit unexpectedly
wait -n "${PIDS[@]}" 2>/dev/null || true
log "A process exited — shutting down."
