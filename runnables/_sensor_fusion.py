#!/usr/bin/env python3
"""
sensor_fusion.py — robot_localization EKF launcher
====================================================
Auto-discovered and launched by start.py as a standalone process.

Starts the robot_localization ekf_node, which fuses:
  /odom  (wheel encoders)  →  vx, vyaw
  /imu   (gyroscope)       →  vyaw

Output:
  /odom/filtered      nav_msgs/Odometry
  TF odom → base_link

Config: robot/config/ekf.yaml

Note on LiDAR: scan-to-scan correction is not needed here because
SLAM Toolbox already corrects absolute drift via the map→odom TF.
robot_localization only smooths the odom→base_link leg.

Requires:
  sudo apt install ros-humble-robot-localization
"""

import os
import subprocess
import sys


def main():
    config_path = os.path.join(
        os.path.dirname(__file__),
        "..", "robot", "config", "ekf.yaml",
    )
    config_path = os.path.abspath(config_path)

    cmd = [
        "ros2", "run", "robot_localization", "ekf_node",
        "--ros-args",
        "--params-file", config_path,
    ]

    try:
        proc = subprocess.run(cmd)
        sys.exit(proc.returncode)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
