"""Run the public parking stack with the synthetic map and ray-cast sensors."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("xycar_parking_nav"))
    map_yaml = LaunchConfiguration("map")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=str(share / "maps" / "example_map.yaml")
            ),
            DeclareLaunchArgument("autostart_mission", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            Node(
                package="xycar_parking_nav",
                executable="parking_simulator",
                name="parking_simulator",
                output="screen",
                parameters=[
                    {
                        "map_yaml": map_yaml,
                        "motor_calibration_yaml": str(
                            share / "config" / "cmd_vel_adapter.yaml"
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch" / "parking_navigation.launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": "false",
                    "drive_enabled": "false",
                    "autostart_mission": LaunchConfiguration("autostart_mission"),
                    "start_odometry": "false",
                    "start_scan_filter": "true",
                    "publish_laser_tf": "true",
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                }.items(),
            ),
        ]
    )
