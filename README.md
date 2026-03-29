# andr_bot

A mobile robot built on the [ANDR](https://github.com/anthropics/andr) framework — an LLM-powered ROS 2 robotics toolkit where an AI agent controls the robot through a set of registered tools.

The robot runs on a **Jetson** (or any Linux/ROS 2 machine), drives hardware via an **ESP32** over USB serial (micro-ROS), uses an **RPLIDAR** for mapping and navigation, and is controlled through a web dashboard or natural language commands.

---

## How it works

### The ANDR framework

ANDR (`pip install andr`) provides the core AI stack. You never modify it — you only build on top of it by subclassing two base classes:

- **`BaseAgentTool`** — a capability the LLM agent can call (e.g. "walk forward")
- **`BaseInputSource`** — a bridge that sends tasks into the agent pipeline (e.g. the web UI)

When the robot starts, ANDR launches:

- **`tool_manager`** — a C++ registry that tools register with at startup
- **`agent_server`** — the LLM ReAct loop (Ollama/OpenAI backend)
- **`task_manager`** — the single entry point for all tasks
- **`task_brain`** — priority scheduler with preemption
- **`prompt_manager`** — system prompt versioning

### Task flow

```
Web UI → WebSocket → web_ui.py (BaseInputSource)
  → task_manager/execute
    → task_brain (priority/preemption)
      → agent_server (LLM ReAct loop)
        → tool_manager → tool (walk, spin, navigate_to_point…)
          → feedback streams back → Web UI
```

The agent discovers available tools at runtime from `tool_manager` — it has no hardcoded knowledge of what this robot can do.

### Physical hardware flow

```
RPLIDAR (/dev/ttyUSB0)
  → lidar.py runnable → /scan
    → SLAM Toolbox → /map, map→odom TF

ESP32 (/dev/ttyUSB1, micro-ROS)
  → micro-ROS agent → /odom, /imu
    → EKF (sensor_fusion.py) → /odom/filtered

agent → walk/spin tool → /cmd_vel → micro-ROS agent → ESP32 → motors
agent → navigate_to_point tool → Nav2 /navigate_to_pose → controller → /cmd_vel
```

---

## Project structure

```
andr_bot/
├── start.py                  # Entry point — auto-discovers and launches everything
├── start_robot.sh            # Full hardware stack startup (RSP, SLAM, Nav2, ANDR)
├── andr_bot.service          # systemd unit for boot-time autostart
├── andr.config.yaml          # LLM backend, model, agent and brain settings
│
├── tools/                    # Agent tools (auto-discovered)
│   ├── walk.py               # Move forward/backward via /cmd_vel
│   ├── spin.py               # Rotate in place via /cmd_vel
│   └── navigate_to_point.py  # Waypoint navigation via Nav2
│
├── managers/                 # Long-running ROS services (auto-discovered)
│   └── map_server.py         # Map, waypoint and SLAM management (SQLite)
│
├── inputs/                   # Input sources (auto-discovered)
│   └── web_ui.py             # FastAPI + WebSocket web dashboard
│
├── runnables/                # Standalone processes (auto-discovered)
│   ├── lidar.py              # RPLIDAR node → publishes /scan
│   └── sensor_fusion.py      # robot_localization EKF → /odom/filtered
│
├── robot/                    # ROS 2 robot package
│   ├── description/          # URDF/xacro robot model
│   ├── config/               # Nav2, SLAM Toolbox, EKF, RViz configs
│   └── launch/               # Launch files (sim + real hardware)
│
├── firmware/                 # ESP32 micro-ROS firmware
│   ├── src/main.cpp          # Publishes /odom, /imu — subscribes /cmd_vel
│   ├── platformio.ini        # PlatformIO build config
│   └── SETUP.md              # Firmware build and flash instructions
│
└── ui/static/                # Web dashboard frontend (served by web_ui.py)
    ├── index.html             # Main control panel
    └── rviz.html              # RViz visualization page
```

---

## Prerequisites

- ROS 2 Humble
- Python 3.10+
- `pip install andr`
- A ROS 2 workspace with: `rplidar_ros`, `slam_toolbox`, `nav2_bringup`, `robot_localization`, `robot_state_publisher`
- Ollama running locally with a model pulled (default: `llama3.2`)

---

## Running on hardware

### 1. Flash the ESP32

See [firmware/SETUP.md](firmware/SETUP.md) for full instructions. In short:

```bash
# Install PlatformIO (official installer only — not pip install)
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py

# Install micro-ROS build deps
~/.platformio/penv/bin/pip install catkin-pkg lark-parser colcon-common-extensions \
    importlib-resources pyyaml pytz "markupsafe==2.0.1" "empy==3.3.4" \
    --force-reinstall empy==3.3.4

# Build and flash
cd firmware/
pio run -t upload
```

The ESP32 firmware publishes `/odom` and `/imu`, and subscribes to `/cmd_vel`. The onboard LED turns on when the micro-ROS agent connects.

### 2. Start the full stack

```bash
./start_robot.sh
```

This launches in order:

1. **micro-ROS agent** — bridges ESP32 serial to ROS 2
2. **robot_state_publisher** — publishes the TF tree from the URDF
3. **SLAM Toolbox** — lidar mapping, produces `/map` and `map→odom` TF
4. **Nav2** — path planning and navigation
5. **ANDR stack** (`python start.py`) — agent, tools, lidar, EKF, web UI

Web UI is available at **http://localhost:8080** once the stack is up.

Logs are written to `logs/` in the project root.

### 3. Start on boot (systemd)

```bash
sudo cp andr_bot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable andr_bot
sudo systemctl start andr_bot
```

```bash
sudo systemctl status andr_bot     # check status
journalctl -u andr_bot -f          # follow logs
sudo systemctl restart andr_bot    # restart
```

---

## Configuration

Edit `andr.config.yaml` to change the LLM backend, model, or behaviour:

```yaml
llm:
  backend: ollama              # ollama | openai
  model: llama3.2
  host: http://localhost:11434
  temperature: 0.2

agent:
  max_iterations: 20

brain:
  enabled: true
  enable_wander: false         # idle task generation when no tasks pending
  resume_preempted: true       # resume interrupted tasks
```

---

## Tools

Tools are Python files in `tools/` that subclass `BaseAgentTool`. They auto-register with `tool_manager` on startup — no other changes needed to make the agent aware of them.

| Tool | What it does | Key params |
|---|---|---|
| `walk` | Move forward or backward at `/cmd_vel` | `direction`, `duration_s`, `speed` |
| `spin` | Rotate in place at `/cmd_vel` | `direction`, `duration_s`, `speed_deg_s` |
| `navigate_to_point` | Navigate to a named waypoint via Nav2 | `point_name` |

### Adding a tool

1. Create `tools/my_tool.py` subclassing `BaseAgentTool`
2. Define `TOOL_NAME`, `TOOL_DESCRIPTION`, `TOOL_PARAMETERS`
3. Implement `_execute(params, goal_handle)`
4. Done — `start.py` discovers and launches it automatically

---

## Managers

Managers are long-running ROS 2 nodes in `managers/` that expose services to tools and inputs.

**`map_server.py`** manages maps, waypoints, and SLAM state:
- SQLite database at `~/andr_maps/maps.db`
- Services: `save_map`, `get_maps`, `save_point`, `get_map_points`, `get_point_coordinates`, `set_slam_config`, `restart_slam`

The `navigate_to_point` tool calls `map_manager/get_point_coordinates` to resolve a named point like `"kitchen"` into `(x, y)` coordinates before sending a Nav2 goal.

---

## Runnables

Runnables in `runnables/` are standalone processes auto-launched by `start.py`.

**`lidar.py`** — runs `rplidar_ros rplidarNode`, publishing `/scan` at the configured baud rate. Edit `SERIAL_PORT` and `RPLIDAR_BAUDRATE` at the top of the file to match your hardware.

**`sensor_fusion.py`** — runs `robot_localization ekf_node` using `robot/config/ekf.yaml`, fusing wheel odometry and IMU to produce `/odom/filtered` and the `odom→base_link` TF.

---

## Simulation

To run in Gazebo instead of hardware:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch andr_sim robot.launch.py
```

Then in a separate terminal:

```bash
python start.py
```

The launch file reads `~/andr_maps/slam_config.json` to restore the last map/localization state from the UI.

---

## Sending tasks

Via the web UI at **http://localhost:8080**, or from the terminal:

```bash
andr task "walk forward for 3 seconds"
andr task "go to the kitchen"
andr task "spin right 90 degrees"
andr status
```
