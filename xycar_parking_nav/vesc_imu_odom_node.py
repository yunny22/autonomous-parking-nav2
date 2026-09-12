"""Fuse measured VESC travel with IMU yaw into continuous local odometry."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from xycar_msgs.msg import XycarVescState

from .vesc_imu_odom_core import (
    OdomState,
    adapt_gyro_bias,
    corrected_gyro_yaw_rate,
    integrate_distance,
    normalize_angle,
    signed_int32_delta,
    trapezoidal_distance,
    unwrap_yaw_delta,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _quaternion_yaw(orientation) -> float:
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


class VescImuOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("vesc_imu_odom")
        self._declare_parameters()
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_tachometer = bool(
            self.get_parameter("use_tachometer").value
        )
        self.meters_per_count = float(
            self.get_parameter("meters_per_tachometer_count").value
        )
        self.tachometer_sign = float(
            self.get_parameter("tachometer_sign").value
        )
        self.max_tachometer_step = float(
            self.get_parameter("maximum_tachometer_step_counts").value
        )
        self.fallback_to_speed = bool(
            self.get_parameter(
                "fallback_to_speed_on_tachometer_jump"
            ).value
        )
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.speed_offset = float(
            self.get_parameter("speed_offset_mps").value
        )
        self.speed_deadband = float(
            self.get_parameter("speed_deadband_mps").value
        )
        self.max_speed = abs(
            float(self.get_parameter("maximum_speed_mps").value)
        )
        self.maximum_dt = float(
            self.get_parameter("maximum_dt_sec").value
        )
        self.imu_yaw_sign = float(
            self.get_parameter("imu_yaw_sign").value
        )
        self.imu_yaw_source = str(
            self.get_parameter("imu_yaw_source").value
        ).strip().lower()
        if self.imu_yaw_source not in {"orientation", "gyro_z"}:
            raise ValueError(
                "imu_yaw_source must be 'orientation' or 'gyro_z'"
            )
        self.gyro_yaw_sign = float(
            self.get_parameter("gyro_yaw_sign").value
        )
        self.gyro_yaw_scale = float(
            self.get_parameter("gyro_yaw_scale").value
        )
        self.gyro_bias = float(
            self.get_parameter("gyro_z_bias_rad_s").value
        )
        self.gyro_deadband = abs(
            float(self.get_parameter("gyro_deadband_rad_s").value)
        )
        self.gyro_bias_adaptation_enabled = bool(
            self.get_parameter("gyro_bias_adaptation_enabled").value
        )
        self.gyro_bias_stationary_speed = abs(
            float(
                self.get_parameter(
                    "gyro_bias_stationary_speed_mps"
                ).value
            )
        )
        self.gyro_bias_stationary_hold = max(
            0.0,
            float(
                self.get_parameter(
                    "gyro_bias_stationary_hold_sec"
                ).value
            ),
        )
        self.gyro_bias_time_constant = max(
            1.0e-3,
            float(
                self.get_parameter(
                    "gyro_bias_time_constant_sec"
                ).value
            ),
        )
        self.gyro_bias_max_residual = abs(
            float(
                self.get_parameter(
                    "gyro_bias_max_residual_rad_s"
                ).value
            )
        )
        self.gyro_bias_vesc_timeout = max(
            0.0,
            float(
                self.get_parameter(
                    "gyro_bias_vesc_timeout_sec"
                ).value
            ),
        )
        self.imu_timeout = float(
            self.get_parameter("imu_timeout_sec").value
        )
        self.imu_hard_timeout = float(
            self.get_parameter("imu_hard_timeout_sec").value
        )
        self.max_imu_yaw_step = float(
            self.get_parameter("maximum_imu_yaw_step_rad").value
        )
        self.require_initial_imu = bool(
            self.get_parameter("require_initial_imu").value
        )

        self.state = OdomState()
        self.last_imu_wrapped = None
        self.last_imu_sec = None
        self.yaw_rate = 0.0
        self.last_integration_yaw = 0.0
        self.last_vesc_sec = None
        self.last_tachometer = None
        self.last_speed_mps = 0.0
        self.warned_no_imu = False
        self.warned_stale_imu = False
        self.warned_tachometer_jump = False
        self.gyro_stationary_since_sec = None
        self.last_gyro_bias_log_sec = None

        self.odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            50,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            XycarVescState,
            str(self.get_parameter("vesc_topic").value),
            self._on_vesc,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_service(Empty, "~/reset", self._on_reset)
        self.get_logger().info(
            "실측 오도메트리가 준비되었습니다: VESC 이동거리 + "
            f"IMU {self.imu_yaw_source} 방향각 -> "
            f"{self.get_parameter('odom_topic').value}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("vesc_topic", "/vehicle/vesc_state")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("odom_topic", "/slam/odom")
        self.declare_parameter("odom_frame", "slam_odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter(
            "meters_per_tachometer_count",
            1.0,
        )
        self.declare_parameter("tachometer_sign", 1.0)
        self.declare_parameter("use_tachometer", True)
        self.declare_parameter(
            "maximum_tachometer_step_counts",
            1000.0,
        )
        self.declare_parameter(
            "fallback_to_speed_on_tachometer_jump",
            True,
        )
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("speed_offset_mps", 0.0)
        self.declare_parameter("speed_deadband_mps", 0.01)
        self.declare_parameter("maximum_speed_mps", 2.0)
        self.declare_parameter("maximum_dt_sec", 0.20)
        self.declare_parameter("imu_yaw_source", "gyro_z")
        self.declare_parameter("imu_yaw_sign", 1.0)
        self.declare_parameter("gyro_yaw_sign", 1.0)
        self.declare_parameter("gyro_yaw_scale", 1.0)
        self.declare_parameter("gyro_z_bias_rad_s", 0.0)
        self.declare_parameter("gyro_deadband_rad_s", 0.01)
        self.declare_parameter("gyro_bias_adaptation_enabled", False)
        self.declare_parameter(
            "gyro_bias_stationary_speed_mps",
            0.03,
        )
        self.declare_parameter("gyro_bias_stationary_hold_sec", 2.0)
        self.declare_parameter("gyro_bias_time_constant_sec", 5.0)
        self.declare_parameter(
            "gyro_bias_max_residual_rad_s",
            0.08,
        )
        self.declare_parameter("gyro_bias_vesc_timeout_sec", 0.50)
        self.declare_parameter("imu_timeout_sec", 0.50)
        self.declare_parameter("imu_hard_timeout_sec", 1.0)
        self.declare_parameter("maximum_imu_yaw_step_rad", 0.50)
        self.declare_parameter("require_initial_imu", True)
        self.declare_parameter("pose_xy_variance", 0.10)
        self.declare_parameter("pose_yaw_variance", 0.10)
        self.declare_parameter("twist_linear_variance", 0.05)
        self.declare_parameter("twist_yaw_variance", 0.05)
        self.declare_parameter("stale_covariance_multiplier", 10.0)

    def _on_imu(self, message: Imu) -> None:
        stamp_sec = _stamp_seconds(message.header.stamp)
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1.0e-9

        if self.imu_yaw_source == "gyro_z":
            self._on_gyro_z(message, stamp_sec)
            return

        quaternion = message.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 0.5 or not math.isfinite(norm):
            return

        wrapped_yaw = normalize_angle(
            self.imu_yaw_sign * _quaternion_yaw(quaternion)
        )
        if self.last_imu_wrapped is None:
            self.last_imu_wrapped = wrapped_yaw
            self.last_imu_sec = stamp_sec
            return

        dt_sec = stamp_sec - self.last_imu_sec
        yaw_delta = unwrap_yaw_delta(
            self.last_imu_wrapped,
            wrapped_yaw,
        )
        self.last_imu_wrapped = wrapped_yaw
        self.last_imu_sec = stamp_sec
        if dt_sec <= 0.0 or dt_sec > self.maximum_dt:
            return
        if abs(yaw_delta) > self.max_imu_yaw_step:
            self.get_logger().warning(
                "갑자기 크게 변한 IMU 방향각 데이터를 거부했습니다"
            )
            return

        self.state.yaw += yaw_delta
        self.yaw_rate = yaw_delta / dt_sec
        self.warned_no_imu = False
        self.warned_stale_imu = False

    def _on_gyro_z(self, message: Imu, stamp_sec: float) -> None:
        raw_rate = float(message.angular_velocity.z)
        if not math.isfinite(raw_rate):
            return
        if self.last_imu_sec is None:
            self.last_imu_sec = stamp_sec
            self.yaw_rate = 0.0
            self.warned_no_imu = False
            self.warned_stale_imu = False
            return

        dt_sec = stamp_sec - self.last_imu_sec
        self.last_imu_sec = stamp_sec
        if dt_sec <= 0.0 or dt_sec > self.maximum_dt:
            return

        self._adapt_stationary_gyro_bias(
            raw_rate_rad_s=raw_rate,
            stamp_sec=stamp_sec,
            dt_sec=dt_sec,
        )
        yaw_rate = corrected_gyro_yaw_rate(
            raw_rate_rad_s=raw_rate,
            bias_rad_s=self.gyro_bias,
            sign=self.gyro_yaw_sign,
            scale=self.gyro_yaw_scale,
            deadband_rad_s=self.gyro_deadband,
        )
        yaw_delta = yaw_rate * dt_sec
        if abs(yaw_delta) > self.max_imu_yaw_step:
            self.get_logger().warning(
                "갑자기 크게 변한 IMU 자이로 방향각 데이터를 거부했습니다"
            )
            return

        self.state.yaw += yaw_delta
        self.yaw_rate = yaw_rate
        self.warned_no_imu = False
        self.warned_stale_imu = False

    def _adapt_stationary_gyro_bias(
        self,
        raw_rate_rad_s: float,
        stamp_sec: float,
        dt_sec: float,
    ) -> None:
        if not self.gyro_bias_adaptation_enabled:
            return

        vesc_is_recent = (
            self.last_vesc_sec is not None
            and 0.0
            <= stamp_sec - self.last_vesc_sec
            <= self.gyro_bias_vesc_timeout
        )
        is_stationary = (
            vesc_is_recent
            and abs(self.last_speed_mps)
            <= self.gyro_bias_stationary_speed
        )
        if not is_stationary:
            self.gyro_stationary_since_sec = None
            return

        if self.gyro_stationary_since_sec is None:
            self.gyro_stationary_since_sec = stamp_sec
            return
        if (
            stamp_sec - self.gyro_stationary_since_sec
            < self.gyro_bias_stationary_hold
        ):
            return

        self.gyro_bias = adapt_gyro_bias(
            current_bias_rad_s=self.gyro_bias,
            raw_rate_rad_s=raw_rate_rad_s,
            dt_sec=dt_sec,
            time_constant_sec=self.gyro_bias_time_constant,
            maximum_residual_rad_s=self.gyro_bias_max_residual,
        )
        if (
            self.last_gyro_bias_log_sec is None
            or stamp_sec - self.last_gyro_bias_log_sec >= 10.0
        ):
            self.get_logger().info(
                "정지 상태 자이로 편향을 "
                f"{self.gyro_bias:.6f} rad/s로 보정했습니다"
            )
            self.last_gyro_bias_log_sec = stamp_sec

    def _corrected_speed(self, message: XycarVescState) -> float:
        speed = (
            float(message.speed_mps) - self.speed_offset
        ) * self.speed_scale
        if not math.isfinite(speed):
            return 0.0
        speed = max(-self.max_speed, min(self.max_speed, speed))
        if abs(speed) < self.speed_deadband:
            return 0.0
        return speed

    def _on_vesc(self, message: XycarVescState) -> None:
        stamp_sec = _stamp_seconds(message.header.stamp)
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1.0e-9
        speed_mps = self._corrected_speed(message)
        tachometer = float(message.displacement)

        if self.last_vesc_sec is None:
            self.last_vesc_sec = stamp_sec
            self.last_tachometer = tachometer
            self.last_speed_mps = speed_mps
            self.last_integration_yaw = self.state.yaw
            if not self.require_initial_imu or self.last_imu_sec is not None:
                self._publish(message, speed_mps, False)
            return

        dt_sec = stamp_sec - self.last_vesc_sec
        self.last_vesc_sec = stamp_sec
        if dt_sec <= 0.0 or dt_sec > self.maximum_dt:
            self.last_tachometer = tachometer
            self.last_speed_mps = speed_mps
            self.last_integration_yaw = self.state.yaw
            return

        if self.last_imu_sec is None:
            self.last_tachometer = tachometer
            self.last_speed_mps = speed_mps
            if not self.warned_no_imu:
                self.get_logger().warning(
                    "유효한 첫 IMU 방향각 데이터를 기다리는 중입니다"
                )
                self.warned_no_imu = True
            if self.require_initial_imu:
                return

        imu_age = (
            stamp_sec - self.last_imu_sec
            if self.last_imu_sec is not None
            else math.inf
        )
        hard_stale = imu_age > self.imu_hard_timeout
        soft_stale = imu_age > self.imu_timeout
        if soft_stale and not self.warned_stale_imu:
            self.get_logger().warning(
                f"IMU 방향각 데이터가 {imu_age:.3f}초 동안 갱신되지 않았습니다"
            )
            self.warned_stale_imu = True

        distance_m = trapezoidal_distance(
            self.last_speed_mps,
            speed_mps,
            dt_sec,
        )
        if self.use_tachometer and self.last_tachometer is not None:
            count_delta = signed_int32_delta(
                self.last_tachometer,
                tachometer,
            )
            if abs(count_delta) <= self.max_tachometer_step:
                distance_m = (
                    self.tachometer_sign
                    * count_delta
                    * self.meters_per_count
                )
                self.warned_tachometer_jump = False
            elif not self.warned_tachometer_jump:
                self.get_logger().warning(
                    "갑자기 변한 VESC 타코미터 값을 거부하고 속도 적분값을 사용합니다"
                )
                self.warned_tachometer_jump = True
                if not self.fallback_to_speed:
                    distance_m = 0.0

        self.last_tachometer = tachometer
        self.last_speed_mps = speed_mps
        if not hard_stale:
            self.state = integrate_distance(
                self.state,
                distance_m,
                self.last_integration_yaw,
                self.state.yaw,
            )
        self.last_integration_yaw = self.state.yaw
        self._publish(message, speed_mps, soft_stale)

    def _publish(
        self,
        source_message: XycarVescState,
        speed_mps: float,
        stale_imu: bool,
    ) -> None:
        message = Odometry()
        message.header.stamp = source_message.header.stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.state.x
        message.pose.pose.position.y = self.state.y
        message.pose.pose.orientation.z = math.sin(self.state.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.state.yaw / 2.0)
        message.twist.twist.linear.x = speed_mps
        message.twist.twist.angular.z = self.yaw_rate

        multiplier = (
            float(
                self.get_parameter(
                    "stale_covariance_multiplier"
                ).value
            )
            if stale_imu
            else 1.0
        )
        message.pose.covariance[0] = (
            float(self.get_parameter("pose_xy_variance").value)
            * multiplier
        )
        message.pose.covariance[7] = message.pose.covariance[0]
        message.pose.covariance[35] = (
            float(self.get_parameter("pose_yaw_variance").value)
            * multiplier
        )
        message.twist.covariance[0] = float(
            self.get_parameter("twist_linear_variance").value
        )
        message.twist.covariance[35] = (
            float(self.get_parameter("twist_yaw_variance").value)
            * multiplier
        )
        self.odom_publisher.publish(message)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.state.x
            transform.transform.translation.y = self.state.y
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _on_reset(self, request, response):
        del request
        self.state = OdomState()
        self.last_integration_yaw = 0.0
        self.last_imu_wrapped = None
        self.last_imu_sec = None
        self.last_vesc_sec = None
        self.last_tachometer = None
        self.last_speed_mps = 0.0
        self.yaw_rate = 0.0
        self.gyro_stationary_since_sec = None
        self.last_gyro_bias_log_sec = None
        self.get_logger().info("VESC+IMU 오도메트리를 초기화했습니다")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VescImuOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
