#!/usr/bin/env python3
"""Republish a reliable, range-limited scan for indoor SLAM."""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan

from .scan_filter_core import filter_ranges


class SlamScanFilterNode(Node):
    """Remove long reflective returns and normalize scan publisher QoS."""

    def __init__(self) -> None:
        super().__init__("slam_scan_filter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/slam/scan_filtered")
        self.declare_parameter("minimum_range_m", 0.20)
        self.declare_parameter("maximum_range_m", 6.0)
        self.minimum_range = float(
            self.get_parameter("minimum_range_m").value
        )
        self.maximum_range = float(
            self.get_parameter("maximum_range_m").value
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_topic").value),
            output_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.received = 0
        self.accepted = 0
        self.last_report_sec = time.monotonic()
        self.get_logger().info(
            "주차용 LiDAR 필터가 준비되었습니다: 사용 거리 %.2f~%.2f m"
            % (self.minimum_range, self.maximum_range)
        )

    def _on_scan(self, message: LaserScan) -> None:
        ranges = filter_ranges(
            message.ranges,
            minimum_range_m=self.minimum_range,
            maximum_range_m=self.maximum_range,
        )
        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = max(float(message.range_min), self.minimum_range)
        output.range_max = min(float(message.range_max), self.maximum_range)
        output.ranges = ranges
        output.intensities = message.intensities
        self.publisher.publish(output)

        self.received += len(ranges)
        self.accepted += sum(value == value for value in ranges)
        now = time.monotonic()
        if now - self.last_report_sec >= 5.0:
            ratio = self.accepted / max(1, self.received)
            self.get_logger().info(
                "주차용 LiDAR 유효 데이터 비율: %.1f%%" % (ratio * 100.0)
            )
            self.received = 0
            self.accepted = 0
            self.last_report_sec = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamScanFilterNode()
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
