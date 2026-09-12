"""Real-vehicle integration template; hardware outputs stay disabled by default."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("xycar_parking_nav"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_enabled = LaunchConfiguration("drive_enabled")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("autostart_mission", default_value="false"),
            DeclareLaunchArgument("start_odometry", default_value="false"),
            DeclareLaunchArgument("start_scan_filter", default_value="false"),
            DeclareLaunchArgument("publish_laser_tf", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("enable_preflight", default_value="false"),
            DeclareLaunchArgument("preflight_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("vesc_port", default_value=""),
            DeclareLaunchArgument("laser_x", default_value="0.0"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map", default_value=str(share / "maps" / "example_map.yaml")
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=str(share / "config" / "nav2_parking.yaml"),
            ),
            DeclareLaunchArgument(
                "mission_config",
                default_value=str(share / "config" / "parking_mission.yaml"),
            ),
            DeclareLaunchArgument(
                "manager_params",
                default_value=str(share / "config" / "mission_manager.yaml"),
            ),
            DeclareLaunchArgument(
                "odom_params",
                default_value=str(share / "config" / "vesc_imu_odom.yaml"),
            ),
            DeclareLaunchArgument(
                "adapter_params",
                default_value=str(share / "config" / "cmd_vel_adapter.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch" / "parking_navigation.launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "nav2_params": LaunchConfiguration("nav2_params"),
                    "mission_config": LaunchConfiguration("mission_config"),
                    "manager_params": LaunchConfiguration("manager_params"),
                    "odom_params": LaunchConfiguration("odom_params"),
                    "adapter_params": LaunchConfiguration("adapter_params"),
                    "use_sim_time": use_sim_time,
                    "drive_enabled": drive_enabled,
                    "autostart_mission": LaunchConfiguration("autostart_mission"),
                    "start_odometry": LaunchConfiguration("start_odometry"),
                    "start_scan_filter": LaunchConfiguration("start_scan_filter"),
                    "publish_laser_tf": LaunchConfiguration("publish_laser_tf"),
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                    "laser_x": LaunchConfiguration("laser_x"),
                    "laser_y": LaunchConfiguration("laser_y"),
                    "laser_z": LaunchConfiguration("laser_z"),
                    "laser_yaw": LaunchConfiguration("laser_yaw"),
                }.items(),
            ),
            Node(
                package="xycar_parking_nav",
                executable="parking_preflight",
                name="parking_preflight",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "timeout_sec": ParameterValue(
                            LaunchConfiguration("preflight_timeout_sec"),
                            value_type=float,
                        ),
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "lidar_device": "",
                        "imu_device": "",
                        "vesc_device": LaunchConfiguration("vesc_port"),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("enable_preflight")),
            ),
        ]
    )
