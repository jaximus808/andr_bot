"""
robot.launch.py — Full robot bringup for Nav2 and high-level control
=====================================================================

Launches the complete hardware + navigation stack:
  1. micro-ROS agent     — bridges ESP32 serial (USB) to ROS 2
                           Publishes: /odom, /imu   Subscribes: /cmd_vel
  2. Robot State Publisher — TF tree from URDF (base_link → lidar_link, etc.)
  3. RPLIDAR node         — publishes /scan (LaserScan)
  4. EKF (robot_localization) — fuses /odom + /imu → /odom/filtered
  5. SLAM Toolbox         — mapping or localization mode (map → odom TF)
  6. Nav2                 — path planning, obstacle avoidance, recovery behaviors
  7. RViz2 (optional)     — visualization

Launch arguments:
  use_sim_time        — Use simulation clock (default: false)
  localization        — Run SLAM in localization mode (default: from slam_config.json)
  map_file            — Pose-graph path for localization mode (default: from slam_config.json)
  launch_nav2         — Launch Nav2 stack (default: true)
  launch_rviz         — Launch RViz2 (default: false)
  launch_micro_ros    — Launch micro-ROS agent (default: true)
  launch_lidar        — Launch RPLIDAR node (default: true)
  micro_ros_port      — Serial port for ESP32 (default: /dev/ttyUSB0)
  micro_ros_baud      — Baud rate for micro-ROS serial (default: 115200)
  lidar_serial_port   — Serial port for RPLIDAR (default: /dev/rplidar)
  lidar_baudrate      — RPLIDAR baud rate (default: 115200)

Usage:
  # Full hardware bringup (mapping mode)
  ros2 launch andr_bringup robot.launch.py

  # Localization mode with a saved map
  ros2 launch andr_bringup robot.launch.py localization:=true map_file:=/home/user/andr_maps/my_map

  # Skip hardware drivers (already running separately)
  ros2 launch andr_bringup robot.launch.py launch_micro_ros:=false launch_lidar:=false

  # Simulation mode
  ros2 launch andr_bringup robot.launch.py use_sim_time:=true launch_micro_ros:=false launch_lidar:=false
"""

import json
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _load_slam_config():
    """Read persisted SLAM config from ~/andr_maps/slam_config.json.

    Returns (map_file, localization_str) with safe defaults when the file
    is absent or malformed.
    """
    config_path = os.path.expanduser("~/andr_maps/slam_config.json")
    try:
        with open(config_path) as f:
            cfg = json.load(f)
        map_file = cfg.get("map_file", "") or ""
        localization = cfg.get("localization", False)
        return map_file, "true" if localization else "false"
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        return "", "false"


def generate_launch_description():
    pkg_share = get_package_share_directory("andr_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # Load persisted SLAM config so defaults match the last UI selection
    _cfg_map_file, _cfg_localization = _load_slam_config()

    # ── Launch configurations ─────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    map_file = LaunchConfiguration("map_file")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_micro_ros = LaunchConfiguration("launch_micro_ros")
    launch_lidar = LaunchConfiguration("launch_lidar")
    micro_ros_port = LaunchConfiguration("micro_ros_port")
    micro_ros_baud = LaunchConfiguration("micro_ros_baud")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")

    # ── 1. micro-ROS Agent ────────────────────────────────────────────────
    # Bridges ESP32 serial to ROS 2.
    # ESP32 publishes: /odom (wheel encoders), /imu (gyro/accel)
    # ESP32 subscribes: /cmd_vel (motor commands)
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=[
            "serial",
            "--dev", micro_ros_port,
            "-b", micro_ros_baud,
        ],
        condition=IfCondition(launch_micro_ros),
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────
    # Publishes the full TF tree from URDF (base_link, lidar_link, imu_link, etc.)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 3. RPLIDAR Node ──────────────────────────────────────────────────
    # Publishes: /scan (LaserScan, 360 deg)
    # Consumed by: SLAM Toolbox, Nav2 costmaps
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": lidar_serial_port,
            "serial_baudrate": lidar_baudrate,
            "frame_id": "lidar_link",
            "angle_compensate": True,
            "scan_mode": "",
        }],
        condition=IfCondition(launch_lidar),
    )

    # ── 4. EKF — Robot Localization ──────────────────────────────────────
    # Fuses /odom (wheel encoders: vx, vyaw) + /imu (gyro: vyaw)
    # Publishes: /odom/filtered, TF odom → base_link
    # Note: SLAM Toolbox handles map → odom TF (absolute localization)
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── 5a. SLAM Toolbox: mapping mode (default) ─────────────────────────
    slam_mapping_params = os.path.join(
        pkg_share, "config", "slam_toolbox_params_real.yaml"
    )

    slam_mapping = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_mapping_params,
            {"use_sim_time": use_sim_time},
        ],
        condition=UnlessCondition(localization),
    )

    # ── 5b. SLAM Toolbox: localization mode ──────────────────────────────
    slam_localization_params = os.path.join(
        pkg_share, "config", "slam_toolbox_localization_params_real.yaml"
    )

    slam_localization = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_localization_params,
            {
                "use_sim_time": use_sim_time,
                "map_file_name": map_file,
            },
        ],
        condition=IfCondition(localization),
    )

    # ── 6. Nav2 Navigation Stack ─────────────────────────────────────────
    # Full navigation: path planning, obstacle avoidance, recovery behaviors
    # Depends on: /odom/filtered (EKF), /scan (lidar), /map (SLAM), TF tree
    nav2_params = os.path.join(pkg_share, "config", "nav2_params_real.yaml")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    # ── 7. RViz2 (optional) ──────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, "config", "hardware.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    # ── Delayed launch for Nav2 ──────────────────────────────────────────
    # Nav2 needs EKF and SLAM TFs to be available before starting.
    # Delay Nav2 by 5 seconds to give upstream nodes time to initialize.
    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2],
    )

    # ── Build launch description ─────────────────────────────────────────
    return LaunchDescription([
        # Declare all launch arguments
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "localization", default_value=_cfg_localization,
            description=(
                "Run SLAM in localization mode instead of mapping "
                "(default read from ~/andr_maps/slam_config.json)"
            ),
        ),
        DeclareLaunchArgument(
            "map_file", default_value=_cfg_map_file,
            description=(
                "Absolute path to serialized pose graph (without extension) "
                "used when localization=true "
                "(default read from ~/andr_maps/slam_config.json)"
            ),
        ),
        DeclareLaunchArgument(
            "launch_nav2", default_value="true",
            description="Launch the Nav2 navigation stack",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false",
            description="Launch RViz2 for visualization",
        ),
        DeclareLaunchArgument(
            "launch_micro_ros", default_value="true",
            description="Launch the micro-ROS serial agent for ESP32",
        ),
        DeclareLaunchArgument(
            "launch_lidar", default_value="true",
            description="Launch the RPLIDAR driver node",
        ),
        DeclareLaunchArgument(
            "micro_ros_port", default_value="/dev/ttyUSB0",
            description="Serial port for the micro-ROS agent (ESP32)",
        ),
        DeclareLaunchArgument(
            "micro_ros_baud", default_value="115200",
            description="Baud rate for the micro-ROS serial connection",
        ),
        DeclareLaunchArgument(
            "lidar_serial_port", default_value="/dev/rplidar",
            description="Serial port for the RPLIDAR",
        ),
        DeclareLaunchArgument(
            "lidar_baudrate", default_value="115200",
            description=(
                "RPLIDAR baud rate "
                "(A1/A2M7/A2M8: 115200, A2M12/A3/S1: 256000, C1: 460800)"
            ),
        ),

        # Launch nodes in dependency order:
        # 1. Hardware drivers (micro-ROS agent, lidar)
        micro_ros_agent,
        rplidar_node,
        # 2. Robot state publisher (TF tree)
        rsp,
        # 3. Sensor fusion (EKF)
        ekf_node,
        # 4. SLAM (mapping or localization)
        slam_mapping,
        slam_localization,
        # 5. Nav2 (delayed to let TFs stabilize)
        delayed_nav2,
        # 6. Visualization
        rviz,
    ])
