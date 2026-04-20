# andr_bot — ESP32 Wiring + Brainless Bring-up Guide

Everything you need to go from a bag of parts to a running robot, derived
directly from `firmware/src/main.cpp`, `andr_bringup/launch/robot.launch.py`,
`start_robot.sh`, and `andr_bot.service`.

> **Target hardware (from the code)**
> - ESP32 dev board (`board = esp32dev`, Arduino framework)
> - L298N dual H-bridge motor driver
> - 2× DC motors with quadrature encoders (16 PPR → 64 counts/rev after 4×)
> - Adafruit BNO085 IMU (STEMMA QT, I²C addr `0x4A`)
> - RPLIDAR (A1/A2M7/A2M8 @ 115200, others see baud table)
> - Jetson (or any Linux + ROS 2 Humble host) running the ANDR stack

---

## 1. ESP32 Pinout (authoritative — from `firmware/src/main.cpp` lines 19–43)

| Function                | ESP32 GPIO | Connects to                                       |
|-------------------------|-----------:|---------------------------------------------------|
| **Left motor PWM**      | `25`       | L298N **ENA**                                     |
| **Left motor IN1**      | `26`       | L298N **IN1**                                     |
| **Left motor IN2**      | `27`       | L298N **IN2**                                     |
| **Right motor PWM**     | `32`       | L298N **ENB**                                     |
| **Right motor IN1**     | `33`       | L298N **IN3**                                     |
| **Right motor IN2**     | `14`       | L298N **IN4**                                     |
| **Left encoder A**      | `18`       | Left encoder channel A                            |
| **Left encoder B**      | `19`       | Left encoder channel B                            |
| **Right encoder A**     | `4`        | Right encoder channel A                           |
| **Right encoder B**     | `5`        | Right encoder channel B                           |
| **I²C SDA**             | `21`       | BNO085 **SDA** (STEMMA QT blue)                   |
| **I²C SCL**             | `22`       | BNO085 **SCL** (STEMMA QT yellow)                 |
| **Status LED**          | `2`        | On-board blue LED (most ESP32 dev boards)         |

### Wiring steps in order (do them exactly like this)

Power off everything before wiring. Build on a breadboard first if you can.

**Step A — Common ground first.** Tie together: ESP32 `GND`, L298N `GND`,
encoder `GND`, BNO085 `GND`, battery negative. Skipping this is the #1 source
of "it works for a second then resets" problems.

**Step B — L298N power.**
- L298N `+12V` (or `VS`) → battery positive (6–12 V typical for small DC
  motors). Motors live on this rail.
- Remove the L298N **5V_EN** jumper **only if** you're feeding >12 V, then
  supply ESP32 from its own USB or a separate 5 V BEC.
- If you keep the 5V_EN jumper, the L298N outputs 5 V on its `+5V` pin — you
  can use that to power the ESP32's `VIN`, but **do not power both VIN and
  USB simultaneously**.

**Step C — Left motor (L298N channel A).**
- ESP32 `GPIO 25` → L298N `ENA`
- ESP32 `GPIO 26` → L298N `IN1`
- ESP32 `GPIO 27` → L298N `IN2`
- L298N `OUT1`, `OUT2` → left motor terminals
- If the robot spins in circles when you command "forward", swap `OUT1`/`OUT2`
  on this motor.

**Step D — Right motor (L298N channel B).**
- ESP32 `GPIO 32` → L298N `ENB`
- ESP32 `GPIO 33` → L298N `IN3`
- ESP32 `GPIO 14` → L298N `IN4`
- L298N `OUT3`, `OUT4` → right motor terminals

**Step E — Left encoder.**
- Encoder `Vcc` → ESP32 `3V3` (most 3.3 V encoders) or `5V` if your encoder
  needs 5 V. Internal pull-ups are enabled in firmware (line 456–459), so no
  external pull-ups are required if the encoder is push-pull.
- Encoder `GND` → common ground.
- Encoder `A` → ESP32 `GPIO 18`
- Encoder `B` → ESP32 `GPIO 19`

**Step F — Right encoder.**
- `Vcc` → `3V3` / `5V` as above, `GND` → common ground.
- Encoder `A` → ESP32 `GPIO 4`
- Encoder `B` → ESP32 `GPIO 5`

> **Encoder polarity:** the firmware assumes positive counts when the wheel
> moves **forward**. If forward driving produces negative `/odom` X movement,
> swap the `A`/`B` wires of that encoder (don't patch the code — just the
> wires).

**Step G — BNO085 IMU (STEMMA QT / Qwiic).**
- BNO085 `Vin` → ESP32 `3V3` (the Adafruit board has a regulator; 3V3 is
  safe and what the I²C lines run at).
- BNO085 `GND` → common ground.
- BNO085 `SDA` → ESP32 `GPIO 21`
- BNO085 `SCL` → ESP32 `GPIO 22`
- Leave the `ADR` pad floating → I²C address `0x4A` (what the firmware
  expects via `BNO08x_I2CADDR_DEFAULT`, line 468).
- I²C pull-ups are on the Adafruit breakout; don't add more.

**Step H — ESP32 power and USB to the Jetson.**
- USB data cable from ESP32 → Jetson. This single cable carries:
  - 5 V power to the ESP32
  - micro-ROS serial at 115200 baud (see `platformio.ini`, line 13)
- Confirm with `ls /dev/ttyUSB*` on the Jetson — you should see `/dev/ttyUSB0`
  or `/dev/ttyUSB1` appear when you plug it in.

**Step I — RPLIDAR.**
- RPLIDAR USB → Jetson (separate USB cable).
- Power: A1/A2 are USB-powered; A3/S1/S2 need a separate 5 V input — check
  your unit.

### Pin collisions & ESP32 gotchas (heads-up)

- **GPIO 2** is also used as a boot strapping pin on many ESP32 dev boards.
  It has to be **LOW** (or floating) at boot for the ESP32 to enter flash
  mode. The firmware drives it high after connecting to the agent — that's
  fine post-boot, just don't connect anything that actively pulls it high
  during power-up.
- **GPIO 12** and **GPIO 15** are also boot-sensitive. The firmware doesn't
  use them — if you're adding your own peripherals, avoid those.
- **GPIO 34/35/36/39** are input-only — not used here, but good to know.
- **GPIO 4 and 5** (right encoder) are safe general-purpose pins.
- ISRs are registered on **CHANGE** for all four encoder channels — don't
  put encoders on pins without full interrupt support.

### What the firmware does with these pins

| Behavior                                  | File:line                      |
|-------------------------------------------|--------------------------------|
| LEDC PWM, 1 kHz, 8-bit, channels 0 & 1    | `main.cpp:70–73, 449–452`      |
| Direction via IN1/IN2 logic               | `main.cpp:210–223`             |
| cmd_vel → diff-drive, scaled by 0.5 m/s   | `main.cpp:76, 231–236`         |
| 4× quadrature decode in ISR               | `main.cpp:106–132`             |
| Odometry publish @ 20 Hz (`/odom`, `/imu`) | `main.cpp:92, 255–323`        |
| `/cmd_vel` timeout → stop motors at 500 ms | `main.cpp:84, 526–527`        |
| BNO085 fusion reports at 100 Hz           | `main.cpp:470–472`             |

---

## 2. Match the firmware's geometry to your robot

The firmware hard-codes these in `main.cpp:48–63`. If your hardware differs,
**update the firmware (re-flash)** — not the URDF alone — or odometry will
drift.

| Constant              | Firmware default | URDF match (`robot_core.xacro`)               |
|-----------------------|------------------|-----------------------------------------------|
| `WHEEL_RADIUS`        | `0.05` m         | `wheel_radius = 0.05`                         |
| `WHEEL_SEPARATION`    | `0.34` m         | `2 * wheel_offset_y = 2*(0.15 + 0.02) = 0.34` |
| `ENCODER_CPR`         | `64` counts/rev  | 16 PPR × 4 (quadrature decoding)              |
| `MAX_WHEEL_SPEED`     | `0.5` m/s        | Used to normalise `cmd_vel` → PWM duty        |

If your encoder is, say, 20 PPR instead of 16, change `ENCODER_CPR` to 80 and
re-flash. If your wheel diameter is 100 mm, `WHEEL_RADIUS = 0.05` is correct.

---

## 3. Brainless bring-up on the Jetson (do this once per machine)

This is the "I just want it to come up when I power the bot on" path. Every
step matters.

### 3.1 Prereqs on the Jetson

```bash
# ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base \
                    ros-humble-rplidar-ros \
                    ros-humble-slam-toolbox \
                    ros-humble-nav2-bringup \
                    ros-humble-robot-localization \
                    ros-humble-robot-state-publisher \
                    ros-humble-xacro

# Serial port access (re-login after)
sudo usermod -aG dialout "$USER"

# Python 3.10+ + ANDR
pip install andr

# Ollama + model (the LLM backend)
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2
```

### 3.2 Build the ROS 2 workspace

`start_robot.sh` expects the colcon workspace at `~/ros2_ws`, and the
`andr_bringup` package has to be inside it (it's a standard ament_cmake
package — see `andr_bringup/package.xml`).

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Your robot repo
git clone <this repo> andr_bot
ln -s ~/ros2_ws/src/andr_bot/andr_bringup .

# micro-ROS Agent (humble)
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

> **Readiness note:** `andr_bringup/CMakeLists.txt` installs a `worlds/`
> directory that does not exist. Either `mkdir andr_bringup/worlds` (empty
> dir) or delete `worlds` from the `install(DIRECTORY ...)` line before
> `colcon build`. See §5.

### 3.3 Build + flash the ESP32 firmware

```bash
# PlatformIO (official installer — not pip)
curl -fsSL -o get-platformio.py \
  https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py

# micro-ROS Python deps (ONE-TIME, see firmware/SETUP.md)
~/.platformio/penv/bin/pip install catkin-pkg lark-parser \
  colcon-common-extensions importlib-resources pyyaml pytz \
  "markupsafe==2.0.1" "empy==3.3.4" --force-reinstall empy==3.3.4

cd ~/ros2_ws/src/andr_bot/firmware
pio run -t upload          # first build is ~10 min
```

After flash, unplug/replug the ESP32. The blue LED should stay off until the
micro-ROS agent connects.

### 3.4 Pin serial ports with udev (stops `/dev/ttyUSB*` shuffling)

This is the biggest "brainless" win. Without it, the ESP32 and the RPLIDAR
fight over `/dev/ttyUSB0` depending on plug order.

```bash
# Find each device's USB vendor/product IDs
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
# Repeat for /dev/ttyUSB1 etc.
```

Create `/etc/udev/rules.d/99-andr-bot.rules`:

```
# ESP32 (adjust idVendor/idProduct to your board — CP2102: 10c4:ea60, CH340: 1a86:7523)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp32", MODE="0666"

# RPLIDAR (Slamtec CP2102 variant often shares 10c4:ea60 — pin by serial if so)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<your-rplidar-serial>", SYMLINK+="rplidar", MODE="0666"
```

Reload and replug:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/esp32 /dev/rplidar
```

Then edit `start_robot.sh` (lines 44–49) to use the stable names:

```bash
MICRO_ROS_PORT="/dev/esp32"
LIDAR_SERIAL_PORT="/dev/rplidar"
```

### 3.5 Configure the ANDR stack

Edit `andr.config.yaml`:

- `llm.model` — set to a model you've actually `ollama pull`-ed. The current
  value `gemma4:e4b` is not a real Ollama tag; use e.g. `llama3.2` or
  `gemma2:2b`. See §5.
- `ui.port` — change from `8080` if something else uses it.
- `brain.enable_wander: false` — keep off until you're confident.

### 3.6 Install the systemd service (the "brainless" part)

```bash
# Edit the service to match your username/paths (see §5 — it hardcodes "observer")
sudo cp andr_bot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable andr_bot
sudo systemctl start andr_bot

# Watch it come up
journalctl -u andr_bot -f
```

Power-cycle the Jetson. Within ~30 s you should:
- See the ESP32 blue LED turn on (agent connected).
- Be able to open `http://<jetson-ip>:8080/` in a browser.

That's brainless — no keyboard, no SSH needed after this.

---

## 4. Post-boot verification checklist

From any shell on the Jetson:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ESP32 node is up (its name is "esp32_robot" in main.cpp:369)
ros2 node list | grep esp32_robot

# Data flowing
ros2 topic hz /odom           # expect ~20 Hz
ros2 topic hz /imu            # expect ~20 Hz (only valid if IMU initialised)
ros2 topic hz /scan           # expect ~10 Hz

# Drive test (kills motors after 0.5 s of silence, per CMD_VEL_TIMEOUT_MS)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'

# End-to-end LLM test
andr task "walk forward for 2 seconds"
```

---

## 5. Readiness assessment (things I found while reading the code)

### Blockers (must fix before first boot)

1. **`andr_bringup/CMakeLists.txt` installs a nonexistent `worlds/` directory.**
   colcon build will fail with something like "directory does not exist".
   Fix: create `andr_bringup/worlds/` as an empty folder (add a `.gitkeep`),
   or remove `worlds` from the `install(DIRECTORY ... DESTINATION ...)` call
   (CMakeLists.txt:6–9).

2. **`andr.config.yaml` references `gemma4:e4b`**, which isn't an Ollama
   tag. The stack will start but the agent will fail on first prompt. Change
   to a real tag such as `llama3.2` or `gemma2:2b` and run `ollama pull`.

3. **`andr_bot.service` hardcodes `User=observer` and
   `WorkingDirectory=/home/observer/andr_bot`.** Change both to your actual
   user before installing the unit (file lines 27, 30, 33, 43). If you're
   not named `observer`, systemd will silently fail to start.

4. **`start_robot.sh` sources `~/ros2_ws/install/setup.bash` but also
   expects `andr_bringup` to be installed there.** If you clone into
   `~/andr_bot` but don't symlink `andr_bringup` into `~/ros2_ws/src` before
   building, `ros2 launch andr_bringup robot.launch.py` will fail. Mentioned
   in §3.2 above — don't forget the symlink.

### Inconsistencies (harmless but confusing)

5. **README port drift.** `README.md` says ESP32 lives on `/dev/ttyUSB1`,
   but `start_robot.sh` defaults to `/dev/ttyUSB0`. Use the udev rule in
   §3.4 and this stops mattering.

6. **Two lidar launch paths.** `andr_bringup/robot.launch.py` launches the
   RPLIDAR node itself, but `runnables/_lidar.py` (note the leading
   underscore) used to do it too. `start.py:38` skips modules starting with
   `_`, so `_lidar.py` is intentionally disabled — the bringup launch file
   is the live path. If you ever rename the file to `lidar.py` you'll get
   two `rplidar_node` instances fighting over the same `/dev/ttyUSB0`.

7. **`_sensor_fusion.py` is also underscored.** Same story — EKF is launched
   from `robot.launch.py` instead. Leave both files as `_`-prefixed.

8. **`firmware/SETUP.md` is out of date.** It says the ROS node is called
   `esp32_node`, but `main.cpp:369` now creates `esp32_robot`. Small docs
   fix — not a runtime bug.

9. **The `robot/` directory the README describes doesn't exist.** Everything
   has moved to `andr_bringup/`. The README's structure section is stale —
   not a bug, but don't go looking for `robot/description/` on disk.

### Things that look correct and are ready

- ESP32 firmware: pins consistent, encoder ISRs on CHANGE with pull-ups,
  cmd_vel timeout implemented, BNO085 optional (robot still works if IMU
  fails to init), connection state machine handles agent disconnect cleanly.
- URDF geometry matches firmware constants (wheel radius 0.05 m, separation
  0.34 m).
- EKF config (`ekf.yaml`) only fuses wheel `vx` + gyro `vyaw` — correct and
  safe for an indoor planar robot without a magnetometer.
- `robot.launch.py` delays Nav2 by 5 s so SLAM + EKF TFs settle first —
  this is the right pattern.
- `start.py`'s module auto-discovery cleanly skips `_`-prefixed files, so
  dead code doesn't run accidentally.

### Nice-to-haves you might want before shipping

- Battery voltage divider → another ADC pin → publish `/battery_state` so
  `scheduled_tasks` in `andr.config.yaml` can run the "check battery"
  patrol that's commented out in the config today.
- Bumper / cliff sensors feeding into `/cmd_vel` as a safety layer on the
  ESP32 (already has the state machine skeleton for it).
- A pre-flight check script in `scripts/` that verifies `/dev/esp32` and
  `/dev/rplidar` exist and that `ollama list` contains the configured
  model — run it from `start_robot.sh` before launching, fail fast with a
  clear message if anything is missing.

---

## 6. TL;DR wiring card (tape this to your chassis)

```
ESP32                                L298N
─────                                ─────
GPIO 25 ───PWM─────────► ENA
GPIO 26 ───DIR─────────► IN1         Left motor ► OUT1/OUT2
GPIO 27 ───DIR─────────► IN2
GPIO 32 ───PWM─────────► ENB
GPIO 33 ───DIR─────────► IN3         Right motor ► OUT3/OUT4
GPIO 14 ───DIR─────────► IN4

Left encoder  A→GPIO 18  B→GPIO 19   (3V3, GND, pull-ups internal)
Right encoder A→GPIO  4  B→GPIO  5

BNO085 (I²C, 0x4A)    SDA→GPIO 21   SCL→GPIO 22   (3V3, GND)

LED (status) GPIO 2 (on-board)

USB ───► Jetson  (serial @ 115200, powers ESP32)
Common GND: ESP32, L298N, encoders, IMU, battery−.
```

When the blue LED is steady on, you're connected. Open
`http://<jetson-ip>:8080/` and say hi.
