#!/usr/bin/env python3
"""PGM ray-cast LiDAR and kinematic odometry for parking-stack verification."""

from __future__ import annotations

import math
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import TransformBroadcaster

from .map_core import load_occupancy_map
from .mission_core import Pose2D
from .command_core import MotorCalibration, motor_command_to_twist
from .sim_core import integrate_twist, raycast_range


def _quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class ParkingSimulator(Node):
    """Exercise the production Nav2 graph without Gazebo or motor output."""

    def __init__(self) -> None:
        super().__init__("parking_simulator")
        self._declare_parameters()
        share = Path(get_package_share_directory("xycar_parking_nav"))
        map_path = str(self.get_parameter("map_yaml").value).strip()
        if not map_path:
            map_path = str(share / "maps" / "example_map.yaml")
        self.occupancy_map = load_occupancy_map(map_path)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)
        self.laser_x = float(self.get_parameter("laser_x").value)
        self.laser_y = float(self.get_parameter("laser_y").value)
        self.laser_yaw = float(self.get_parameter("laser_yaw").value)
        self.range_min = float(self.get_parameter("range_min_m").value)
        self.range_max = float(self.get_parameter("range_max_m").value)
        self.beam_count = int(self.get_parameter("beam_count").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self.motion_rate = float(self.get_parameter("motion_rate_hz").value)
        self.scan_rate = float(self.get_parameter("scan_rate_hz").value)
        self.pose = Pose2D(
            float(self.get_parameter("initial_x").value),
            float(self.get_parameter("initial_y").value),
            float(self.get_parameter("initial_yaw").value),
        )
        calibration_path = str(self.get_parameter("motor_calibration_yaml").value).strip()
        if not calibration_path:
            calibration_path = str(share / "config" / "cmd_vel_adapter.yaml")
        with open(calibration_path, "r", encoding="utf-8") as stream:
            calibration_parameters = yaml.safe_load(stream)[
                "parking_cmd_vel_adapter"
            ]["ros__parameters"]
        self.motor_calibration = MotorCalibration(
            speed_gain_mps_per_command=float(
                calibration_parameters["speed_gain_mps_per_command"]
            ),
            minimum_moving_command=float(
                calibration_parameters["minimum_moving_command"]
            ),
            maximum_forward_command=float(
                calibration_parameters["maximum_forward_command"]
            ),
            maximum_reverse_command=float(
                calibration_parameters["maximum_reverse_command"]
            ),
            steering_commands=tuple(
                float(value)
                for value in calibration_parameters["steering_map_commands"]
            ),
            steering_curvatures=tuple(
                float(value)
                for value in calibration_parameters["steering_map_curvatures"]
            ),
        )
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.command_stamp_sec: float | None = None
        self.authorized = False
        self.last_motion_sec = self._now_sec()
        self.path = PathMessage()
        self.path.header.frame_id = "map"

        self.odom_publisher = self.create_publisher(Odometry, "/slam/odom", 20)
        self.scan_publisher = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/parking/sim_ground_truth", 10
        )
        self.path_publisher = self.create_publisher(
            PathMessage, "/parking/sim_ground_truth_path", 1
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Float32MultiArray,
            "/parking/xycar_motor_shadow",
            self._on_motor_command,
            20,
        )
        self.create_subscription(
            Bool, "/parking/drive_authorized", self._on_authorization, 20
        )
        self.create_timer(1.0 / self.motion_rate, self._on_motion_timer)
        self.create_timer(1.0 / self.scan_rate, self._on_scan_timer)
        self.get_logger().info(
            "주차 시뮬레이터가 준비되었습니다: 지도=%s, LiDAR 광선=%d개"
            % (map_path, self.beam_count)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("motor_calibration_yaml", "")
        self.declare_parameter("odom_frame", "slam_odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("laser_frame", "laser_frame")
        self.declare_parameter("initial_x", -1.5)
        self.declare_parameter("initial_y", -1.5)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("laser_x", 0.0)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)
        self.declare_parameter("range_min_m", 0.20)
        self.declare_parameter("range_max_m", 6.0)
        self.declare_parameter("beam_count", 360)
        self.declare_parameter("motion_rate_hz", 50.0)
        self.declare_parameter("scan_rate_hz", 12.5)
        self.declare_parameter("command_timeout_sec", 0.35)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_motor_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            self.linear_command = 0.0
            self.angular_command = 0.0
            self.command_stamp_sec = self._now_sec()
            return
        self.linear_command, self.angular_command = motor_command_to_twist(
            float(message.data[0]),
            float(message.data[1]),
            self.motor_calibration,
        )
        self.command_stamp_sec = self._now_sec()

    def _on_authorization(self, message: Bool) -> None:
        self.authorized = bool(message.data)

    def _active_velocity(self, now_sec: float) -> tuple[float, float]:
        if not self.authorized or self.command_stamp_sec is None:
            return 0.0, 0.0
        if now_sec - self.command_stamp_sec > self.command_timeout:
            return 0.0, 0.0
        return self.linear_command, self.angular_command

    def _on_motion_timer(self) -> None:
        now_sec = self._now_sec()
        dt_sec = max(0.0, min(0.10, now_sec - self.last_motion_sec))
        self.last_motion_sec = now_sec
        linear, angular = self._active_velocity(now_sec)
        self.pose = integrate_twist(self.pose, linear, angular, dt_sec)
        stamp = self.get_clock().now().to_msg()
        sine, cosine = _quaternion_from_yaw(self.pose.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation.z = sine
        odom.pose.pose.orientation.w = cosine
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose.x
        transform.transform.translation.y = self.pose.y
        transform.transform.rotation.z = sine
        transform.transform.rotation.w = cosine
        self.tf_broadcaster.sendTransform(transform)

        truth = PoseStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = "map"
        truth.pose = odom.pose.pose
        self.pose_publisher.publish(truth)
        if not self.path.poses or self._path_sample_due(truth):
            self.path.header.stamp = stamp
            self.path.poses.append(truth)
            if len(self.path.poses) > 4000:
                self.path.poses = self.path.poses[-4000:]
            self.path_publisher.publish(self.path)

    def _path_sample_due(self, current: PoseStamped) -> bool:
        previous = self.path.poses[-1].pose.position
        return math.hypot(
            current.pose.position.x - previous.x,
            current.pose.position.y - previous.y,
        ) >= 0.025

    def _on_scan_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        base_cosine = math.cos(self.pose.yaw)
        base_sine = math.sin(self.pose.yaw)
        laser_origin_x = (
            self.pose.x + base_cosine * self.laser_x - base_sine * self.laser_y
        )
        laser_origin_y = (
            self.pose.y + base_sine * self.laser_x + base_cosine * self.laser_y
        )
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / self.beam_count
        ranges = [
            raycast_range(
                self.occupancy_map,
                origin_x=laser_origin_x,
                origin_y=laser_origin_y,
                heading=(
                    self.pose.yaw
                    + self.laser_yaw
                    + angle_min
                    + index * angle_increment
                ),
                minimum_range_m=self.range_min,
                maximum_range_m=self.range_max,
            )
            for index in range(self.beam_count)
        ]
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.laser_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (self.beam_count - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.scan_time = 1.0 / self.scan_rate
        scan.time_increment = scan.scan_time / self.beam_count
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
