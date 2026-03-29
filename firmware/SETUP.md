# ESP32 micro-ROS Firmware — Setup & Usage

Bare-bones micro-ROS node for ESP32 that connects to the ROS 2 graph
over USB serial. It creates a node called `esp32_node` and currently has
no subscribers or publishers — just a live connection ready to be extended.

The onboard LED (GPIO 2) turns **on** when connected to the agent and
**off** when disconnected.

---

## 1. Install PlatformIO

Use the official installer — **do not use `pip install platformio`** as it
won't create the penv that the micro-ROS build scripts depend on.

```bash
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```

After install, make sure `pio` is on your PATH:
```bash
pio --version
```

---

## 2. Install micro-ROS Build Dependencies

The micro-ROS library compiles from source and requires specific Python
packages in PlatformIO's internal penv. Install them before the first build:

```bash
~/.platformio/penv/bin/pip install \
  catkin-pkg \
  lark-parser \
  colcon-common-extensions \
  importlib-resources \
  pyyaml \
  pytz \
  "markupsafe==2.0.1" \
  "empy==3.3.4" \
  --force-reinstall empy==3.3.4
```

> **Note:** `empy` must be version 3.3.4. Version 4.x has a breaking API
> change that causes the ROS 2 code-generation tools to fail. The
> `--force-reinstall` flag ensures any existing newer version is replaced.

---

## 3. Build the Firmware

```bash
cd firmware/

# First build downloads the ESP32 toolchain + compiles micro-ROS (~10 min)
pio run
```

The first build takes a while because it compiles the entire micro-ROS
stack for ESP32. Subsequent builds are fast.

---

## 3. Flash the ESP32

Plug in the ESP32 via USB, then:

```bash
# Find the serial port
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Flash (auto-detects port, or specify with -t upload_port=/dev/ttyUSB0)
pio run -t upload
```

If you get permission errors:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

---

## 4. Install the micro-ROS Agent

The agent runs on your computer and bridges the ESP32's serial connection
into the ROS 2 network.

### Option A: Build from source (recommended for Humble)

```bash
# Create a workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

# Clone the agent
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git

# Build
cd ~/microros_ws
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

### Option B: Docker (quick start)

```bash
docker run -it --rm \
    --device /dev/ttyUSB0 \
    --net=host \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyUSB0 -b 115200
```

---

## 5. Run It

### Step 1: Start the micro-ROS agent

```bash
# If built from source:
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# If using Docker (see Option B above)
```

### Step 2: Reset the ESP32

Press the **EN** (reset) button on the ESP32, or unplug/replug USB.
The firmware waits for the agent on startup — resetting ensures a clean
handshake.

### Step 3: Verify the connection

In another terminal:
```bash
source /opt/ros/humble/setup.bash

# You should see "esp32_node" in the list
ros2 node list

# Check node info
ros2 node info /esp32_node
```

The onboard LED should be **on** when connected.

---

## 6. Monitor Serial Output

```bash
pio device monitor -b 115200
```

Note: the serial port is shared between micro-ROS transport and the
monitor. You can't run both at the same time. Use the monitor for
debugging only (when the agent is not running).

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `pio: command not found` | Add `~/.platformio/penv/bin` to PATH |
| Permission denied on `/dev/ttyUSB0` | `sudo usermod -a -G dialout $USER`, then re-login |
| ESP32 not detected | Try a different USB cable (some are charge-only) |
| Agent starts but no node appears | Press EN/reset on ESP32 after agent starts |
| Build fails on micro-ROS lib | Run `pio lib install` then `pio run` again |
| `RCCHECK` fails at runtime | Agent not running or wrong baud rate |

---

## What's Next

This firmware is a skeleton. Future additions:
- Subscribe to `/cmd_vel` → drive motors
- Publish encoder ticks → odometry
- Publish IMU data
- Publish battery voltage
- Expose GPIO as a tool via topics

Each of these just means adding subscribers/publishers to `main.cpp`
and bumping the executor handle count.
