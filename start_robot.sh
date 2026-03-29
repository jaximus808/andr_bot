#!/bin/bash
# =============================================================================
#  start_robot.sh — Full robot stack startup for Jetson (physical hardware)
# =============================================================================
#
#  Launches in order:
#    1. micro-ROS agent    — bridges ESP32 serial (USB) to ROS 2
#    2. robot_state_publisher — publishes TF tree from URDF
#    3. SLAM Toolbox       — lidar-based mapping / localisation (map→odom TF)
#    4. Nav2               — path planning and navigation
#    5. ANDR stack         — agent, tools, lidar node, EKF, web UI
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

# ROS workspace (built with colcon — contains robot_localization, rplidar_ros, etc.)
# Change this if your workspace is elsewhere.
WORKSPACE_SETUP="${HOME}/ros2_ws/install/setup.bash"

# Serial port the ESP32 is connected to.
# Run: ls /dev/ttyUSB* /dev/ttyACM*  after plugging in, to confirm.
MICRO_ROS_PORT="/dev/ttyUSB0"
MICRO_ROS_BAUD="115200"

# URDF xacro path (used by robot_state_publisher)
XACRO_FILE="${SCRIPT_DIR}/robot/description/robot.urdf.xacro"

# Nav2 and SLAM config (real-hardware variants — use_sim_time: False)
NAV2_PARAMS="${SCRIPT_DIR}/robot/config/nav2_params_real.yaml"
SLAM_PARAMS="${SCRIPT_DIR}/robot/config/slam_toolbox_params_real.yaml"

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

wait_for_topic() {
    local topic="$1"
    local timeout="${2:-30}"
    log "Waiting for topic ${topic} (timeout ${timeout}s)..."
    for (( i=0; i<timeout; i++ )); do
        if ros2 topic info "${topic}" &>/dev/null; then
            log "Topic ${topic} is live."
            return 0
        fi
        sleep 1
    done
    log "WARNING: ${topic} not seen after ${timeout}s — continuing anyway."
}

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

# ── 1. micro-ROS agent ────────────────────────────────────────────────────────
# Bridges the ESP32 (micro-ROS) serial connection to ROS 2.
# Publishes: /odom, /imu
# Subscribes: /cmd_vel

log "Starting micro-ROS agent on ${MICRO_ROS_PORT} @ ${MICRO_ROS_BAUD} baud..."

ros2 run micro_ros_agent micro_ros_agent serial \
    --dev "${MICRO_ROS_PORT}" \
    --baudrate "${MICRO_ROS_BAUD}" \
    >> "${LOG_DIR}/micro_ros_agent.log" 2>&1 &

PIDS+=($!)
sleep 3   # give the agent time to open the serial port before the ESP32 connects

# ── 2. Robot state publisher ─────────────────────────────────────────────────
# Reads the URDF xacro and publishes all TF frames (base_link→lidar_link, etc.)
# Nav2 and SLAM Toolbox depend on this being up first.

log "Starting robot_state_publisher..."

ROBOT_DESC=$(xacro "${XACRO_FILE}")

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p "robot_description:=${ROBOT_DESC}" \
    -p "use_sim_time:=false" \
    >> "${LOG_DIR}/rsp.log" 2>&1 &

PIDS+=($!)
wait_for_topic /robot_description 15

# ── 3. SLAM Toolbox ───────────────────────────────────────────────────────────
# Builds and maintains the map using /scan.
# Publishes: /map, map→odom TF
# Depends on: /scan (from lidar runnable, started by ANDR), TF from RSP

log "Starting SLAM Toolbox (mapping mode)..."

ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args \
    --params-file "${SLAM_PARAMS}" \
    -p "use_sim_time:=false" \
    >> "${LOG_DIR}/slam_toolbox.log" 2>&1 &

PIDS+=($!)
sleep 2

# ── 4. Nav2 ───────────────────────────────────────────────────────────────────
# Path planning and navigation stack.
# Depends on: /odom/filtered (EKF), /scan, /map, TF tree

log "Starting Nav2..."

ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    params_file:="${NAV2_PARAMS}" \
    >> "${LOG_DIR}/nav2.log" 2>&1 &

PIDS+=($!)
sleep 3

# ── 5. ANDR stack ─────────────────────────────────────────────────────────────
# Launches (auto-discovered):
#   runnables/lidar.py          — RPLIDAR node → /scan
#   runnables/sensor_fusion.py  — robot_localization EKF → /odom/filtered
#   managers/map_server.py      — waypoint / map management
#   tools/walk.py, spin.py, navigate_to_point.py
#   inputs/web_ui.py            — web dashboard on :8080

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
