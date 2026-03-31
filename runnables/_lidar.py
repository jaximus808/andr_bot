#!/usr/bin/env python3
"""
lidar.py — RPLIDAR launcher
============================
Auto-discovered and launched by start.py as a standalone process.

Starts the rplidar_ros rplidarNode, which publishes:
  /scan   sensor_msgs/LaserScan   (consumed by SLAM Toolbox and Nav2 costmaps)

The frame_id is set to 'lidar_link' to match the URDF TF frame, so SLAM
Toolbox and Nav2 can correctly transform scan points into the robot frame.

── Model / baud rate reference ──────────────────────────────────────────────
  A1, A2M7, A2M8   →  115200   (RPLIDAR_BAUDRATE = 115200)
  A2M12, A3, S1    →  256000
  C1               →  460800
  S2, S3           →  1000000

Change RPLIDAR_BAUDRATE below to match your specific unit.
Change SERIAL_PORT if the device appears on a different port (check with
`ls /dev/ttyUSB*` or `ls /dev/ttyACM*` after plugging in).
─────────────────────────────────────────────────────────────────────────────

Requires:
  sudo apt install ros-humble-rplidar-ros
  sudo usermod -aG dialout $USER   # grant serial port access (re-login after)
"""

import subprocess
import sys

# ── Configuration — edit these to match your hardware ─────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
RPLIDAR_BAUDRATE = 115200        # see model/baud table in header above
FRAME_ID      = "lidar_link"     # must match URDF lidar link name
SCAN_TOPIC    = "/scan"          # consumed by SLAM Toolbox + Nav2 costmaps
# ──────────────────────────────────────────────────────────────────────────


def main():
    cmd = [
        "ros2", "run", "rplidar_ros", "rplidarNode",
        "--ros-args",
        "-p", f"serial_port:={SERIAL_PORT}",
        "-p", f"serial_baudrate:={RPLIDAR_BAUDRATE}",
        "-p", f"frame_id:={FRAME_ID}",
        "--remap", f"scan:={SCAN_TOPIC}",
    ]

    try:
        proc = subprocess.run(cmd)
        sys.exit(proc.returncode)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
