import json
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    # Update this to your actual hardware bringup package name
    pkg_share = get_package_share_directory("andr_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # Load persisted SLAM config so defaults match the last UI selection
    _cfg_map_file, _cfg_localization = _load_slam_config()

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    map_file     = LaunchConfiguration("map_file")
    launch_nav2  = LaunchConfiguration("launch_nav2")

    # ── Robot State Publisher ─────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Sensor Drivers & Hardware Interface ───────────────────────────────
    # Depending on your setup, you'll want to launch your hardware drivers here
    # (e.g., LiDAR, micro-ROS agent for motor controllers/IMU, etc.)
    # 
    # hardware_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_share, "launch", "hardware.launch.py")
    #     )
    # )

    # ── Robot Localization (EKF) ──────────────────────────────────────────
    # Fuses IMU, Encoders (odom), and potentially SLAM pose data
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")
    
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config, 
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            # Remap outputs if necessary, e.g., mapping to a specific odom topic
            # ("odometry/filtered", "odom") 
        ]
    )

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, "config", "hardware.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── SLAM Toolbox: mapping mode (default) ──────────────────────────────
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
            {"use_sim_time": use_sim_time}
        ],
        condition=UnlessCondition(localization),
    )

    # ── SLAM Toolbox: localization mode ───────────────────────────────────
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

    # ── Map manager node (ANDR custom) ────────────────────────────────────
    map_server = Node(
        package="andr_nav",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "slam_params_mapping": os.path.join(
                pkg_share, "config", "slam_toolbox_params_real.yaml"
            ),
            "slam_params_localization": os.path.join(
                pkg_share, "config", "slam_toolbox_localization_params_real.yaml"
            ),
        }],
    )

    # ── Nav2 navigation stack ─────────────────────────────────────────────
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

    return LaunchDescription([
        # Default changed to false for real hardware
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

        rsp,
        ekf_node,
        slam_mapping,
        slam_localization,
        map_server,
        nav2,
        rviz,
    ])