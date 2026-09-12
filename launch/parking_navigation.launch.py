"""Bring up Nav2, mission sequencing, and the safe vehicle adapter."""

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
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
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
            Node(
                package="xycar_parking_nav",
                executable="vesc_imu_odom",
                name="vesc_imu_odom",
                output="screen",
                parameters=[
                    LaunchConfiguration("odom_params"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                condition=IfCondition(LaunchConfiguration("start_odometry")),
            ),
            Node(
                package="xycar_parking_nav",
                executable="scan_filter",
                name="parking_scan_filter",
                output="screen",
                parameters=[
                    LaunchConfiguration("odom_params"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                condition=IfCondition(LaunchConfiguration("start_scan_filter")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="parking_base_to_laser",
                arguments=[
                    "--x", LaunchConfiguration("laser_x"),
                    "--y", LaunchConfiguration("laser_y"),
                    "--z", LaunchConfiguration("laser_z"),
                    "--yaw", LaunchConfiguration("laser_yaw"),
                    "--pitch", "0",
                    "--roll", "0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "laser_frame",
                ],
                condition=IfCondition(LaunchConfiguration("publish_laser_tf")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / "launch" / "bringup_launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("nav2_params"),
                    "use_sim_time": use_sim_time,
                    "slam": "False",
                    "autostart": "True",
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
            Node(
                package="xycar_parking_nav",
                executable="mission_manager",
                name="parking_mission_manager",
                output="screen",
                parameters=[
                    LaunchConfiguration("manager_params"),
                    {
                        "mission_config": LaunchConfiguration("mission_config"),
                        "behavior_tree": str(
                            share / "behavior_trees" / "ackermann_navigate_to_pose.xml"
                        ),
                        "autostart_mission": ParameterValue(
                            LaunchConfiguration("autostart_mission"), value_type=bool
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
            Node(
                package="xycar_parking_nav",
                executable="cmd_vel_adapter",
                name="parking_cmd_vel_adapter",
                output="screen",
                parameters=[
                    LaunchConfiguration("adapter_params"),
                    {
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "laser_x": ParameterValue(
                            LaunchConfiguration("laser_x"), value_type=float
                        ),
                        "laser_y": ParameterValue(
                            LaunchConfiguration("laser_y"), value_type=float
                        ),
                        "laser_yaw": ParameterValue(
                            LaunchConfiguration("laser_yaw"), value_type=float
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="parking_rviz",
                output="screen",
                arguments=["-d", str(share / "rviz" / "parking_nav.rviz")],
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
