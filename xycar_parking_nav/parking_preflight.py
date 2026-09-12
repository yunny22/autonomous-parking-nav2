"""One-shot terminal preflight for the complete real-car parking launch."""

from __future__ import annotations

import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from xycar_msgs.msg import XycarVescState

from .preflight_core import TopicProbe, inspect_device


class ParkingPreflight(Node):
    """Check serial devices, live topics, localization, and required TFs."""

    def __init__(self) -> None:
        super().__init__("parking_preflight")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("report_period_sec", 5.0)
        self.declare_parameter("drive_enabled", False)
        # Device paths are intentionally empty in the public template. Supply
        # local paths explicitly when running a hardware integration test.
        self.declare_parameter("lidar_device", "")
        self.declare_parameter("imu_device", "")
        self.declare_parameter("vesc_device", "")

        self.timeout_sec = max(1.0, float(self.get_parameter("timeout_sec").value))
        self.report_period_sec = max(
            1.0, float(self.get_parameter("report_period_sec").value)
        )
        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.started_sec = time.monotonic()
        self.next_report_sec = self.started_sec + self.report_period_sec
        self.finished = False
        self.succeeded = False
        self.device_failures: list[tuple[str, str, str]] = []
        self.ready_transforms: set[tuple[str, str]] = set()

        self.probes = {
            "scan": TopicProbe(
                "LiDAR 원본", "/scan", 3,
                "LiDAR 장치 경로와 드라이버 로그를 확인하세요.",
            ),
            "imu": TopicProbe(
                "IMU", "/imu", 3,
                "IMU 장치 연결과 드라이버 로그를 확인하세요.",
            ),
            "vesc": TopicProbe(
                "VESC", "/vehicle/vesc_state", 3,
                "VESC 장치 연결, 모터 배터리와 텔레메트리를 확인하세요.",
            ),
            "filtered_scan": TopicProbe(
                "LiDAR 필터", "/slam/scan_filtered", 3,
                "parking_scan_filter 노드와 /scan 입력을 확인하세요.",
            ),
            "odom": TopicProbe(
                "주행 오도메트리", "/slam/odom", 3,
                "VESC 타코미터와 IMU gyro_z 입력을 확인하세요.",
            ),
            "amcl": TopicProbe(
                "AMCL 위치추정", "/amcl_pose", 2,
                "초기 Pose, LiDAR-지도 정합, map->slam_odom TF를 확인하세요.",
            ),
        }
        self.transform_specs = (
            ("map", "slam_odom", "AMCL 지도 TF"),
            ("slam_odom", "base_footprint", "차량 오도메트리 TF"),
            ("base_footprint", "laser_frame", "LiDAR 장착 TF"),
        )

        tf_qos = QoSProfile(depth=100)
        tf_static_qos = QoSProfile(depth=1)
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscriptions = [
            self.create_subscription(
                LaserScan,
                self.probes["scan"].topic,
                self._callback_for("scan"),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Imu,
                self.probes["imu"].topic,
                self._callback_for("imu"),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                XycarVescState,
                self.probes["vesc"].topic,
                self._callback_for("vesc"),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LaserScan,
                self.probes["filtered_scan"].topic,
                self._callback_for("filtered_scan"),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                self.probes["odom"].topic,
                self._callback_for("odom"),
                20,
            ),
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.probes["amcl"].topic,
                self._callback_for("amcl"),
                10,
            ),
            self.create_subscription(
                TFMessage, "/tf", self._on_tf, tf_qos
            ),
            self.create_subscription(
                TFMessage, "/tf_static", self._on_tf, tf_static_qos
            ),
        ]
        self.timer = self.create_timer(0.25, self._on_timer)

        self._line("")
        self._line("========== 1단계: 주차 USB 연결 확인 ==========")
        self._check_device(
            "LiDAR USB",
            str(self.get_parameter("lidar_device").value),
            "LiDAR USB와 로컬 장치 경로를 확인하세요.",
        )
        self._check_device(
            "IMU USB",
            str(self.get_parameter("imu_device").value),
            "IMU USB와 로컬 장치 경로를 확인하세요.",
        )
        self._check_device(
            "VESC USB",
            str(self.get_parameter("vesc_device").value),
            "VESC USB, 모터 배터리와 로컬 장치 경로를 확인하세요.",
        )
        self._line("")
        self._line("========== 2단계: 주차 센서·위치추정 준비 확인 ==========")
        for probe in self.probes.values():
            self._line(f"  [대기] {probe.label:<16} {probe.topic}")
        for target, source, label in self.transform_specs:
            self._line(f"  [대기] {label:<16} {target} <- {source}")

    @staticmethod
    def _line(message: str) -> None:
        print(message, flush=True)

    def _check_device(self, label: str, path: str, hint: str) -> None:
        ready, detail = inspect_device(path)
        if ready:
            self._line(f"  [정상] {label:<16} {detail}")
            return
        self.device_failures.append((label, detail, hint))
        self._line(f"  [실패] {label:<16} {detail}")

    def _callback_for(self, key: str):
        def callback(_message) -> None:
            probe = self.probes[key]
            if probe.observe(time.monotonic()):
                self._line(
                    f"  [수신] {probe.label:<16} {probe.topic} 첫 메시지 확인"
                )

        return callback

    def _on_tf(self, message: TFMessage) -> None:
        labels = {
            (target, source): label
            for target, source, label in self.transform_specs
        }
        for transform in message.transforms:
            target = transform.header.frame_id.lstrip("/")
            source = transform.child_frame_id.lstrip("/")
            key = (target, source)
            if key not in labels or key in self.ready_transforms:
                continue
            self.ready_transforms.add(key)
            self._line(f"  [정상] {labels[key]:<16} {target} <- {source}")

    def _all_ready(self) -> bool:
        return (
            not self.device_failures
            and all(probe.ready for probe in self.probes.values())
            and len(self.ready_transforms) == len(self.transform_specs)
        )

    def _report_waiting(self, elapsed_sec: float) -> None:
        waiting = [probe.label for probe in self.probes.values() if not probe.ready]
        waiting.extend(
            label
            for target, source, label in self.transform_specs
            if (target, source) not in self.ready_transforms
        )
        if self.device_failures:
            waiting.extend(label for label, _detail, _hint in self.device_failures)
        self._line(
            f"  [대기] {elapsed_sec:.1f}/{self.timeout_sec:.1f}초 | "
            "아직 준비되지 않은 항목: " + ", ".join(waiting)
        )

    def _finish_success(self) -> None:
        self._line("")
        self._line("========== 모든 주차 센서·TF 정상 ==========")
        for probe in self.probes.values():
            rate = probe.rate_hz
            rate_text = f"약 {rate:.1f} Hz" if rate is not None else "메시지 정상"
            self._line(
                f"  [정상] {probe.label:<16} {probe.topic} | {rate_text}"
            )
        if self.drive_enabled:
            self._line("  [주의] drive_enabled=true: 모터 출력이 활성화된 상태입니다.")
        else:
            self._line(
                "  [안전] drive_enabled=false: 모터에는 보내지 않고 검증용 명령만 출력합니다."
            )
        self._line(
            "  [준비완료] RViz에서 LiDAR와 지도가 겹치는지 확인한 뒤 미션을 시작하세요."
        )
        self._line("  [시작명령] 아래 명령을 새 터미널에서 실행하세요.")
        self._line(
            "  ros2 service call /parking_mission_manager/start "
            "std_srvs/srv/Trigger '{}'"
        )
        self.succeeded = True
        self.finished = True

    def _finish_failure(self) -> None:
        self._line("")
        self._line("========== 주차 준비 확인 실패 | 차량을 출발시키지 마세요 ==========")
        for label, detail, hint in self.device_failures:
            self._line(f"  [실패] {label}: {detail}")
            self._line(f"         조치: {hint}")
        for probe in self.probes.values():
            if probe.ready:
                continue
            self._line(
                f"  [실패] {probe.label}: {probe.topic} "
                f"({probe.count}/{probe.minimum_samples}개 수신)"
            )
            self._line(f"         조치: {probe.hint}")
        for target, source, label in self.transform_specs:
            if (target, source) in self.ready_transforms:
                continue
            self._line(f"  [실패] {label}: {target} <- {source} 연결 없음")
            self._line("         조치: 연결 센서와 상위 실패 항목부터 해결하세요.")
        self._line("  [출발금지] 위 실패 항목을 해결한 뒤 통합 명령을 다시 실행하세요.")
        self.finished = True

    def _on_timer(self) -> None:
        if self.finished:
            return
        if self._all_ready():
            self._finish_success()
            return
        now_sec = time.monotonic()
        elapsed_sec = now_sec - self.started_sec
        if elapsed_sec >= self.timeout_sec:
            self._finish_failure()
            return
        if now_sec >= self.next_report_sec:
            self._report_waiting(elapsed_sec)
            self.next_report_sec = now_sec + self.report_period_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingPreflight()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        succeeded = node.succeeded
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not succeeded:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
