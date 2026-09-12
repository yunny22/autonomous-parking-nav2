#!/usr/bin/env python3
"""Fail-closed Nav2 Twist to calibrated Xycar motor adapter."""

from __future__ import annotations

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from .command_core import (
    DirectionChangeGuard,
    Footprint,
    MotorCalibration,
    scan_points_in_base,
    slew,
    swept_footprint_collision,
    twist_to_motor_command,
)


GATE_REASON_KO = {
    "ok": "정상(주행 허용)",
    "not_authorized": "미션 주행 권한이 아직 없음",
    "no_cmd_vel": "Nav2 주행 명령(/cmd_vel)이 아직 없음",
    "stale_cmd_vel": "Nav2 주행 명령(/cmd_vel)이 끊김",
    "no_scan": "주차용 LiDAR 데이터가 아직 없음",
    "stale_scan": "주차용 LiDAR 데이터가 끊김",
    "insufficient_scan": "유효한 LiDAR 점 개수가 부족함",
    "non_finite_twist": "유효하지 않은 주행 명령을 받음",
    "stopped": "정지 명령",
    "rotate_in_place_rejected": "차량이 수행할 수 없는 제자리 회전 명령을 거부함",
    "lidar_swept_collision": "예상 주행 궤적에서 장애물을 감지함",
    "direction_change_dwell": "전진·후진 전환 전 안전 정지 중",
    "steering_settle": "목표 조향각 정렬 중",
    "shutdown": "노드 종료로 정지",
}


class CmdVelAdapter(Node):
    """Apply physical calibration and an independent LiDAR stop shield."""

    def __init__(self) -> None:
        super().__init__("parking_cmd_vel_adapter")
        self._declare_parameters()

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.motor_topic = str(self.get_parameter("motor_topic").value)
        self.shadow_topic = str(self.get_parameter("shadow_topic").value)
        self.authorization_topic = str(
            self.get_parameter("authorization_topic").value
        )
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.timer_period = float(self.get_parameter("timer_period_sec").value)
        self.maximum_steering_rate = float(
            self.get_parameter("maximum_steering_command_rate").value
        )
        self.steering_settle_tolerance = float(
            self.get_parameter("steering_settle_tolerance_command").value
        )

        self.calibration = MotorCalibration(
            speed_gain_mps_per_command=float(
                self.get_parameter("speed_gain_mps_per_command").value
            ),
            minimum_moving_command=float(
                self.get_parameter("minimum_moving_command").value
            ),
            maximum_forward_command=float(
                self.get_parameter("maximum_forward_command").value
            ),
            maximum_reverse_command=float(
                self.get_parameter("maximum_reverse_command").value
            ),
            steering_commands=tuple(
                float(value)
                for value in self.get_parameter("steering_map_commands").value
            ),
            steering_curvatures=tuple(
                float(value)
                for value in self.get_parameter("steering_map_curvatures").value
            ),
        )
        self.footprint = Footprint(
            minimum_x=float(self.get_parameter("footprint_minimum_x").value),
            maximum_x=float(self.get_parameter("footprint_maximum_x").value),
            half_width=float(self.get_parameter("footprint_half_width").value),
        )
        self.collision_margin = float(
            self.get_parameter("collision_margin_m").value
        )
        self.reaction_time = float(
            self.get_parameter("reaction_time_sec").value
        )
        self.braking_deceleration = float(
            self.get_parameter("braking_deceleration_mps2").value
        )
        self.minimum_projection = float(
            self.get_parameter("minimum_projection_m").value
        )
        self.maximum_projection = float(
            self.get_parameter("maximum_projection_m").value
        )
        self.projection_step = float(
            self.get_parameter("projection_sample_step_m").value
        )
        self.minimum_scan_points = int(
            self.get_parameter("minimum_scan_points").value
        )
        self.laser_x = float(self.get_parameter("laser_x").value)
        self.laser_y = float(self.get_parameter("laser_y").value)
        self.laser_yaw = float(self.get_parameter("laser_yaw").value)

        self.direction_guard = DirectionChangeGuard(
            float(self.get_parameter("direction_change_dwell_sec").value)
        )
        self.latest_twist: Twist | None = None
        self.latest_twist_time: float | None = None
        self.latest_scan_points: list[tuple[float, float]] = []
        self.latest_scan_time: float | None = None
        self.authorized = False
        self.applied_steering = 0.0
        self.applied_speed = 0.0
        self.last_timer_time = time.monotonic()
        self.last_reason = "startup"

        self.motor_publisher = self.create_publisher(
            Float32MultiArray, self.motor_topic, 10
        )
        self.shadow_publisher = self.create_publisher(
            Float32MultiArray, self.shadow_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, "~/status", 10
        )
        self.debug_publisher = self.create_publisher(
            Float32MultiArray, "~/debug", 10
        )
        self.create_subscription(Twist, self.input_topic, self._on_twist, 10)
        self.create_subscription(
            Bool, self.authorization_topic, self._on_authorization, 10
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_timer(self.timer_period, self._on_timer)
        self.get_logger().info(
            "주차 모터 명령기가 준비되었습니다: 실차출력=%s, 입력=%s, 모터=%s, "
            "주행명령=연속 ±%.1f"
            % (
                "켜짐" if self.drive_enabled else "꺼짐",
                self.input_topic,
                self.motor_topic,
                self.calibration.minimum_moving_command,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("shadow_topic", "/parking/xycar_motor_shadow")
        self.declare_parameter("authorization_topic", "/parking/drive_authorized")
        self.declare_parameter("scan_topic", "/slam/scan_filtered")
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("command_timeout_sec", 0.50)
        self.declare_parameter("scan_timeout_sec", 0.50)
        self.declare_parameter("timer_period_sec", 0.05)
        self.declare_parameter("maximum_steering_command_rate", 120.0)
        self.declare_parameter("steering_settle_tolerance_command", 3.0)
        self.declare_parameter("direction_change_dwell_sec", 0.40)

        self.declare_parameter("speed_gain_mps_per_command", 0.10)
        self.declare_parameter("minimum_moving_command", 1.0)
        self.declare_parameter("maximum_forward_command", 1.0)
        self.declare_parameter("maximum_reverse_command", 1.0)
        self.declare_parameter(
            "steering_map_commands",
            [-1.0, 0.0, 1.0],
        )
        self.declare_parameter(
            "steering_map_curvatures",
            [1.0, 0.0, -1.0],
        )

        # Front-wheel-center base frame, including the physical shell and tires.
        self.declare_parameter("footprint_minimum_x", -0.40)
        self.declare_parameter("footprint_maximum_x", 0.40)
        self.declare_parameter("footprint_half_width", 0.25)
        self.declare_parameter("collision_margin_m", 0.05)
        self.declare_parameter("reaction_time_sec", 0.20)
        self.declare_parameter("braking_deceleration_mps2", 1.0)
        self.declare_parameter("minimum_projection_m", 0.10)
        self.declare_parameter("maximum_projection_m", 0.80)
        self.declare_parameter("projection_sample_step_m", 0.025)
        self.declare_parameter("minimum_scan_points", 20)
        self.declare_parameter("laser_x", 0.0)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_twist(self, message: Twist) -> None:
        self.latest_twist = message
        self.latest_twist_time = self._now_sec()

    def _on_authorization(self, message: Bool) -> None:
        self.authorized = bool(message.data)
        if not self.authorized:
            self.direction_guard.reset()

    def _on_scan(self, message: LaserScan) -> None:
        points = scan_points_in_base(
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=max(float(message.range_min), 0.10),
            range_max=min(float(message.range_max), 6.0),
            laser_x=self.laser_x,
            laser_y=self.laser_y,
            laser_yaw=self.laser_yaw,
        )
        # A near return inside the known body is a self reflection. Keeping it
        # would permanently stop a 360-degree scanner mounted over the chassis.
        self.latest_scan_points = [
            (x, y)
            for x, y in points
            if not (
                self.footprint.minimum_x <= x <= self.footprint.maximum_x
                and abs(y) <= self.footprint.half_width
            )
        ]
        self.latest_scan_time = self._now_sec()

    def _stop(self, reason: str) -> None:
        self.applied_steering = 0.0
        self.applied_speed = 0.0
        self._publish(0.0, 0.0, 0.0, reason, live=self.drive_enabled)

    def _publish(
        self,
        steering: float,
        speed: float,
        curvature: float,
        reason: str,
        *,
        live: bool,
    ) -> None:
        command = Float32MultiArray(data=[float(steering), float(speed)])
        self.shadow_publisher.publish(command)
        if live:
            self.motor_publisher.publish(command)
        self.debug_publisher.publish(
            Float32MultiArray(
                data=[
                    float(steering),
                    float(speed),
                    float(curvature),
                    1.0 if self.authorized else 0.0,
                    float(len(self.latest_scan_points)),
                ]
            )
        )
        if reason != self.last_reason:
            self.status_publisher.publish(String(data=reason))
            if reason == "ok":
                self.get_logger().info("모터 안전조건: 정상, 주행 출력을 허용합니다")
            else:
                self.get_logger().warning(
                    "모터 안전조건: %s" % GATE_REASON_KO.get(reason, reason)
                )
            self.last_reason = reason

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        monotonic_now = time.monotonic()
        dt_sec = max(0.0, min(0.25, monotonic_now - self.last_timer_time))
        self.last_timer_time = monotonic_now

        if not self.authorized:
            self._stop("not_authorized")
            return
        if self.latest_twist is None or self.latest_twist_time is None:
            self._stop("no_cmd_vel")
            return
        if now_sec - self.latest_twist_time > self.command_timeout:
            self._stop("stale_cmd_vel")
            return
        if self.latest_scan_time is None:
            self._stop("no_scan")
            return
        if now_sec - self.latest_scan_time > self.scan_timeout:
            self._stop("stale_scan")
            return
        if len(self.latest_scan_points) < self.minimum_scan_points:
            self._stop("insufficient_scan")
            return

        desired = twist_to_motor_command(
            self.latest_twist.linear.x,
            self.latest_twist.angular.z,
            self.calibration,
        )
        if desired.reason != "ok":
            self._stop(desired.reason)
            return

        collision = swept_footprint_collision(
            self.latest_scan_points,
            # The real chassis runs every non-zero request at command 4, so
            # stopping distance must use that executable speed.
            speed_mps=(
                desired.speed_command
                * self.calibration.speed_gain_mps_per_command
            ),
            curvature=desired.curvature,
            footprint=self.footprint,
            margin_m=self.collision_margin,
            reaction_time_sec=self.reaction_time,
            braking_deceleration_mps2=self.braking_deceleration,
            minimum_projection_m=self.minimum_projection,
            maximum_projection_m=self.maximum_projection,
            sample_step_m=self.projection_step,
        )
        if collision.collision:
            # Stay stopped but allow the servo to follow a newly safe steering
            # request. Centering it here creates repeatable stop/retry drift.
            self.applied_steering = slew(
                self.applied_steering,
                desired.steering_command,
                self.maximum_steering_rate,
                dt_sec,
            )
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                desired.curvature,
                "lidar_swept_collision",
                live=self.drive_enabled,
            )
            return

        guarded_speed, guard_reason = self.direction_guard.filter(
            desired.speed_command,
            now_sec,
        )
        if guard_reason == "direction_change_dwell":
            # Pre-position steering while the car is physically stopped.
            self.applied_steering = slew(
                self.applied_steering,
                desired.steering_command,
                self.maximum_steering_rate,
                dt_sec,
            )
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                desired.curvature,
                guard_reason,
                live=self.drive_enabled,
            )
            return

        next_steering = slew(
            self.applied_steering,
            desired.steering_command,
            self.maximum_steering_rate,
            dt_sec,
        )
        if (
            abs(desired.steering_command - next_steering)
            > self.steering_settle_tolerance
        ):
            # MPPI assumes its requested Ackermann curvature is available now.
            # Stop traction until the measured-rate servo is close enough so
            # a tight arc is not geometrically shortened or extended.
            self.applied_steering = next_steering
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                desired.curvature,
                "steering_settle",
                live=self.drive_enabled,
            )
            return
        self.applied_steering = next_steering
        # Do not ramp through commands below 4: the real drivetrain cannot
        # move there. The VESC driver smooths the physical acceleration while
        # this adapter continuously publishes exactly +4 or -4.
        self.applied_speed = guarded_speed
        self._publish(
            self.applied_steering,
            self.applied_speed,
            desired.curvature,
            "ok",
            live=self.drive_enabled,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._stop("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
