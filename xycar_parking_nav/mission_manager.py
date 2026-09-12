#!/usr/bin/env python3
"""Sequential Nav2 parking mission with localization and actuation gates."""

from __future__ import annotations

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

from .mission_core import (
    LocalizationGate,
    LocalizationGateConfig,
    MissionStep,
    Pose2D,
    assess_mission_time,
    mission_steps_from_dicts,
    pose_error,
    reference_pose_to_base,
)


STATE_KO = {
    "LOCALIZING": "위치추정 안정화 중",
    "READY": "시작 준비 완료",
    "RUNNING": "주차 미션 주행 중",
    "HOLDING": "목표 지점 정지 확인 중",
    "PAUSED_LOCALIZATION": "위치추정 이상으로 일시정지",
    "COMPLETED": "주차 미션 완료",
    "ABORTED": "주차 미션 중단",
}

REASON_KO = {
    "": "",
    "ok": "AMCL 연속 정상 표본 수가 아직 부족함",
    "no_pose": "AMCL 위치정보가 아직 없음",
    "bad_covariance": "AMCL 오차정보 형식이 올바르지 않음",
    "non_finite": "AMCL 위치정보에 유효하지 않은 값이 있음",
    "bad_stamp": "AMCL 시간정보가 올바르지 않음",
    "stale_pose": "AMCL 위치정보가 오래되어 갱신 필요",
    "xy_uncertain": "AMCL 위치 오차가 허용범위보다 큼",
    "yaw_uncertain": "AMCL 방향 오차가 허용범위보다 큼",
    "non_monotonic_stamp": "AMCL 시간정보 순서가 뒤바뀜",
    "position_jump": "AMCL 위치가 갑자기 크게 변함",
    "yaw_jump": "AMCL 방향이 갑자기 크게 변함",
    "operator_abort": "운전자 중단 요청",
    "operator_reset": "운전자 초기화 요청",
    "waiting_for_nav2_action_server": "Nav2 경로주행 서버 시작 대기 중",
    "waiting_for_nav2_activation": "Nav2 전체 활성화 대기 중(재시도 횟수 차감 안 함)",
    "nav2_activation_timeout": "Nav2가 제한시간 안에 활성화되지 않아 안전 중단",
    "goal_rejected": "Nav2가 목표를 거부함",
    "no_final_pose": "목표 도착 후 AMCL 위치정보가 없음",
    "goal_cancelled_for_localization": "위치추정 이상으로 현재 목표 취소",
    "returned_to_start": "출발지 복귀 완료",
    "mission_time_limit": "경기 제한시간을 초과하여 안전 중단",
    "amcl_stable": "AMCL 위치추정이 안정됨",
    "localization_recovered": "AMCL 위치추정이 다시 정상화됨",
}


def reason_to_korean(reason: str) -> str:
    if reason in REASON_KO:
        return REASON_KO[reason]
    if reason.startswith("sending_"):
        return f"{reason.removeprefix('sending_')} 목표 전송"
    if reason.startswith("reached_"):
        return f"{reason.removeprefix('reached_')} 도착"
    if reason.startswith("nav2_status_"):
        return f"Nav2 주행 실패(상태코드 {reason.removeprefix('nav2_status_')})"
    if reason.startswith("goal_error_xy_"):
        return "목표 도착 오차가 허용범위를 벗어남"
    if reason.endswith("_retries_exhausted"):
        return "단계별 재시도 횟수를 모두 사용하여 미션 중단"
    if "_retry_" in reason:
        retry = reason.rsplit("_retry_", 1)[-1]
        return f"목표 주행 실패로 {retry}번째 재시도 대기"
    return reason


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class ParkingMissionManager(Node):
    STATES = {
        "LOCALIZING",
        "READY",
        "RUNNING",
        "HOLDING",
        "PAUSED_LOCALIZATION",
        "COMPLETED",
        "ABORTED",
    }

    def __init__(self) -> None:
        super().__init__("parking_mission_manager")
        self._declare_parameters()
        share_dir = Path(get_package_share_directory("xycar_parking_nav"))
        mission_path = str(self.get_parameter("mission_config").value).strip()
        if not mission_path:
            mission_path = str(share_dir / "config" / "parking_mission.yaml")
        behavior_tree = str(self.get_parameter("behavior_tree").value).strip()
        if not behavior_tree:
            behavior_tree = str(
                share_dir / "behavior_trees" / "ackermann_navigate_to_pose.xml"
            )
        self.behavior_tree = behavior_tree
        self.frame_id, self.base_offset, self.initial_reference, self.steps = (
            self._load_mission(mission_path)
        )
        self.base_goals = [
            reference_pose_to_base(step.reference_pose, self.base_offset)
            for step in self.steps
        ]

        self.gate = LocalizationGate(
            LocalizationGateConfig(
                maximum_xy_variance=float(
                    self.get_parameter("maximum_xy_variance").value
                ),
                maximum_yaw_variance=float(
                    self.get_parameter("maximum_yaw_variance").value
                ),
                maximum_pose_age_sec=float(
                    self.get_parameter("maximum_pose_age_sec").value
                ),
                maximum_position_jump_m=float(
                    self.get_parameter("maximum_position_jump_m").value
                ),
                maximum_yaw_jump_rad=float(
                    self.get_parameter("maximum_yaw_jump_rad").value
                ),
                required_stable_samples=int(
                    self.get_parameter("required_stable_samples").value
                ),
            )
        )
        self.localization_loss_cancel_sec = float(
            self.get_parameter("localization_loss_cancel_sec").value
        )
        self.position_tolerance = float(
            self.get_parameter("goal_position_tolerance_m").value
        )
        self.yaw_tolerance = float(
            self.get_parameter("goal_yaw_tolerance_rad").value
        )
        self.action_server_timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self.nav2_activation_timeout = float(
            self.get_parameter("nav2_activation_timeout_sec").value
        )
        self.retry_delay = float(self.get_parameter("retry_delay_sec").value)
        self.mission_time_limit = float(
            self.get_parameter("mission_time_limit_sec").value
        )
        self.time_warning_remaining = float(
            self.get_parameter("time_warning_remaining_sec").value
        )
        self.time_log_period = float(
            self.get_parameter("time_log_period_sec").value
        )
        if self.mission_time_limit <= 0.0:
            raise ValueError("mission time limit must be positive")
        if self.nav2_activation_timeout <= 0.0:
            raise ValueError("Nav2 activation timeout must be positive")
        if not 0.0 <= self.time_warning_remaining < self.mission_time_limit:
            raise ValueError("time warning must be within the mission time limit")
        if self.time_log_period <= 0.0:
            raise ValueError("time log period must be positive")
        self.autostart_mission = bool(
            self.get_parameter("autostart_mission").value
        )

        self.state = "LOCALIZING"
        self.current_index = 0
        self.current_retry = 0
        self.current_pose: Pose2D | None = None
        self.localization_ready = False
        self.localization_reason = "no_pose"
        self.localization_lost_since: float | None = None
        self.hold_until: float | None = None
        self.retry_at: float | None = None
        self.mission_started_at: float | None = None
        self.mission_finished_at: float | None = None
        self.next_time_log_at: float | None = None
        self.time_warning_emitted = False
        self.nav2_first_goal_accepted = False
        self.nav2_activation_deadline: float | None = None
        self.start_requested = self.autostart_mission
        self.goal_handle = None
        self.goal_active = False
        self.goal_serial = 0
        self.cancel_for_localization = False
        self.initial_pose_remaining = int(
            self.get_parameter("initial_pose_publish_count").value
        )
        self.next_initial_pose_publish = 0.0
        self.initial_pose_period = float(
            self.get_parameter("initial_pose_publish_period_sec").value
        )

        self.authorization_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("authorization_topic").value),
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            10,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_array_publisher = self.create_publisher(
            PoseArray,
            str(self.get_parameter("goal_pose_array_topic").value),
            latched_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self._on_amcl_pose,
            20,
        )
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/abort", self._on_abort)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.create_timer(0.10, self._on_timer)
        self._publish_goal_array()
        self.get_logger().info(
            "%d단계 주차 미션을 불러왔습니다: 설정=%s, 차량 기준점 보정=%.3f m"
            % (len(self.steps), mission_path, self.base_offset)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("mission_config", "")
        self.declare_parameter("behavior_tree", "")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("authorization_topic", "/parking/drive_authorized")
        self.declare_parameter("state_topic", "/parking/mission_state")
        self.declare_parameter("goal_pose_array_topic", "/parking/mission_goals")
        self.declare_parameter("autostart_mission", False)
        self.declare_parameter("initial_pose_publish_count", 5)
        self.declare_parameter("initial_pose_publish_period_sec", 0.25)
        self.declare_parameter("maximum_xy_variance", 0.04)
        self.declare_parameter("maximum_yaw_variance", 0.08)
        self.declare_parameter("maximum_pose_age_sec", 0.50)
        self.declare_parameter("maximum_position_jump_m", 0.60)
        self.declare_parameter("maximum_yaw_jump_rad", 0.80)
        self.declare_parameter("required_stable_samples", 8)
        self.declare_parameter("localization_loss_cancel_sec", 0.50)
        self.declare_parameter("goal_position_tolerance_m", 0.09)
        self.declare_parameter("goal_yaw_tolerance_rad", 0.10)
        self.declare_parameter("action_server_timeout_sec", 1.0)
        self.declare_parameter("nav2_activation_timeout_sec", 10.0)
        self.declare_parameter("retry_delay_sec", 1.0)
        self.declare_parameter("mission_time_limit_sec", 180.0)
        self.declare_parameter("time_warning_remaining_sec", 30.0)
        self.declare_parameter("time_log_period_sec", 10.0)

    @staticmethod
    def _load_mission(path: str) -> tuple[str, float, Pose2D, list[MissionStep]]:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        mission = document["mission"]
        initial = mission["initial_pose"]
        return (
            str(mission.get("frame_id", "map")),
            float(mission.get("base_from_reference_x_m", 0.0)),
            Pose2D(float(initial["x"]), float(initial["y"]), float(initial["yaw"])),
            mission_steps_from_dicts(mission["steps"]),
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _set_state(self, new_state: str, reason: str = "") -> None:
        if new_state not in self.STATES:
            raise ValueError("unknown mission state: %s" % new_state)
        if new_state != self.state:
            detail = " | 사유: %s" % reason_to_korean(reason) if reason else ""
            self.get_logger().info(
                "미션 상태: %s -> %s%s"
                % (STATE_KO[self.state], STATE_KO[new_state], detail)
            )
            self.state = new_state
        self._publish_state(reason)

    def _mission_timing(self, now_sec: float | None = None):
        return assess_mission_time(
            started_at_sec=self.mission_started_at,
            now_sec=self._now_sec() if now_sec is None else float(now_sec),
            limit_sec=self.mission_time_limit,
            warning_remaining_sec=self.time_warning_remaining,
            finished_at_sec=self.mission_finished_at,
        )

    def _start_mission_clock(self, now_sec: float) -> None:
        if self.mission_started_at is not None:
            return
        self.mission_started_at = now_sec
        self.mission_finished_at = None
        self.next_time_log_at = now_sec + self.time_log_period
        self.time_warning_emitted = False
        self.get_logger().info(
            "경기 시간 측정을 시작합니다: 제한 %.0f초(3분), %.0f초 전 경고"
            % (self.mission_time_limit, self.time_warning_remaining)
        )

    def _publish_state(self, reason: str = "") -> None:
        step = self.steps[self.current_index].name if self.current_index < len(self.steps) else "done"
        timing = self._mission_timing()
        self.state_publisher.publish(
            String(
                data=(
                    "state=%s step=%s index=%d/%d retry=%d localization=%s "
                    "elapsed=%.1f remaining=%.1f limit=%.1f%s"
                    % (
                        self.state,
                        step,
                        self.current_index,
                        len(self.steps),
                        self.current_retry,
                        self.localization_reason,
                        timing.elapsed_sec,
                        timing.remaining_sec,
                        self.mission_time_limit,
                        " reason=" + reason if reason else "",
                    )
                )
            )
        )

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        current = Pose2D(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        stamp_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        assessment = self.gate.update(
            pose=current,
            covariance=message.pose.covariance,
            stamp_sec=stamp_sec,
            now_sec=self._now_sec(),
        )
        self.current_pose = current
        self.localization_ready = assessment.ready
        self.localization_reason = assessment.reason
        if assessment.ready:
            self.localization_lost_since = None

    def _publish_initial_pose(self) -> None:
        base_pose = reference_pose_to_base(self.initial_reference, self.base_offset)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = base_pose.x
        message.pose.pose.position.y = base_pose.y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            quaternion_from_yaw(base_pose.yaw)
        )
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.03
        self.initial_pose_publisher.publish(message)

    def _publish_goal_array(self) -> None:
        array = PoseArray()
        array.header.frame_id = self.frame_id
        array.header.stamp = self.get_clock().now().to_msg()
        for goal in self.base_goals:
            pose = PoseStamped().pose
            pose.position.x = goal.x
            pose.position.y = goal.y
            pose.orientation.z, pose.orientation.w = quaternion_from_yaw(goal.yaw)
            array.poses.append(pose)
        self.goal_array_publisher.publish(array)

    def _on_start(self, _request, response):
        if self.state in {"RUNNING", "HOLDING", "PAUSED_LOCALIZATION"}:
            response.success = False
            response.message = "주차 미션이 이미 진행 중입니다"
            return response
        if self.state == "COMPLETED":
            response.success = False
            response.message = "완료된 미션을 먼저 초기화하세요"
            return response
        if self.state == "ABORTED":
            response.success = False
            response.message = "중단된 미션을 먼저 초기화하세요"
            return response
        self.start_requested = True
        if not self.localization_ready:
            response.success = False
            response.message = (
                "시작 요청을 저장했습니다. AMCL 위치추정이 안정되면 자동으로 시작합니다. "
                "현재 부족 항목: %s" % reason_to_korean(self.localization_reason)
            )
            return response
        response.success = True
        response.message = "주차 미션 시작 요청을 승인했습니다"
        return response

    def _on_abort(self, _request, response):
        self.start_requested = False
        self._cancel_active_goal(localization_pause=False)
        if self.mission_started_at is not None:
            self.mission_finished_at = self._now_sec()
        self._set_state("ABORTED", "operator_abort")
        response.success = True
        response.message = "주차 미션을 중단했고 모터 주행 권한을 해제했습니다"
        return response

    def _on_reset(self, _request, response):
        self._cancel_active_goal(localization_pause=False)
        self.current_index = 0
        self.current_retry = 0
        self.hold_until = None
        self.retry_at = None
        self.mission_started_at = None
        self.mission_finished_at = None
        self.next_time_log_at = None
        self.time_warning_emitted = False
        self.nav2_first_goal_accepted = False
        self.nav2_activation_deadline = None
        self.start_requested = False
        self.initial_pose_remaining = int(
            self.get_parameter("initial_pose_publish_count").value
        )
        self.next_initial_pose_publish = 0.0
        self.gate.stable_samples = 0
        self.localization_ready = False
        self._set_state("LOCALIZING", "operator_reset")
        response.success = True
        response.message = "주차 미션과 초기 위치추정을 초기화했습니다"
        return response

    def _cancel_active_goal(self, *, localization_pause: bool) -> None:
        self.cancel_for_localization = localization_pause
        if not localization_pause:
            self.goal_serial += 1
        if self.goal_handle is not None and self.goal_active:
            self.goal_handle.cancel_goal_async()
        self.goal_active = False
        self.goal_handle = None

    def _send_current_goal(self) -> None:
        if self.current_index >= len(self.steps) or self.goal_active:
            return
        if not self.localization_ready:
            self._set_state("PAUSED_LOCALIZATION", self.localization_reason)
            return
        if not self.nav_client.wait_for_server(timeout_sec=self.action_server_timeout):
            self._defer_for_nav2_activation("waiting_for_nav2_action_server")
            return

        target = self.base_goals[self.current_index]
        message = NavigateToPose.Goal()
        message.pose.header.frame_id = self.frame_id
        message.pose.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = target.x
        message.pose.pose.position.y = target.y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            quaternion_from_yaw(target.yaw)
        )
        message.behavior_tree = self.behavior_tree
        self.cancel_for_localization = False
        self.goal_serial += 1
        serial = self.goal_serial
        future = self.nav_client.send_goal_async(
            message,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(
            lambda completed, goal_serial=serial: self._on_goal_response(
                completed, goal_serial
            )
        )
        self._set_state("RUNNING", "sending_" + self.steps[self.current_index].name)

    def _on_goal_response(self, future, serial: int) -> None:
        if serial != self.goal_serial:
            return
        handle = future.result()
        if not handle.accepted:
            if not self.nav2_first_goal_accepted:
                self._defer_for_nav2_activation("waiting_for_nav2_activation")
                return
            self._handle_goal_failure("goal_rejected")
            return
        self.nav2_first_goal_accepted = True
        self.nav2_activation_deadline = None
        self.goal_handle = handle
        self.goal_active = True
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, goal_serial=serial: self._on_goal_result(
                completed, goal_serial
            )
        )

    def _defer_for_nav2_activation(self, reason: str) -> None:
        """Wait for lifecycle activation without spending a driving retry."""

        now_sec = self._now_sec()
        if self.nav2_activation_deadline is None:
            self.nav2_activation_deadline = now_sec + self.nav2_activation_timeout
        if now_sec >= self.nav2_activation_deadline:
            self.start_requested = False
            if self.mission_started_at is not None:
                self.mission_finished_at = now_sec
            self._set_state("ABORTED", "nav2_activation_timeout")
            return
        self.retry_at = now_sec + self.retry_delay
        self._set_state("RUNNING", reason)

    def _on_feedback(self, _feedback) -> None:
        # Authorization is deliberately based on AMCL health, not action feedback.
        pass

    def _on_goal_result(self, future, serial: int) -> None:
        if serial != self.goal_serial:
            return
        wrapped = future.result()
        self.goal_active = False
        self.goal_handle = None
        if self.cancel_for_localization:
            self.cancel_for_localization = False
            self._set_state("PAUSED_LOCALIZATION", "goal_cancelled_for_localization")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._handle_goal_failure("nav2_status_%d" % wrapped.status)
            return
        if self.current_pose is None:
            self._handle_goal_failure("no_final_pose")
            return

        position_error, yaw_error = pose_error(
            self.current_pose,
            self.base_goals[self.current_index],
        )
        if position_error > self.position_tolerance or yaw_error > self.yaw_tolerance:
            self._handle_goal_failure(
                "goal_error_xy_%.3f_yaw_%.3f" % (position_error, yaw_error)
            )
            return

        step = self.steps[self.current_index]
        self.current_retry = 0
        self.hold_until = self._now_sec() + step.hold_sec
        self._set_state("HOLDING", "reached_" + step.name)

    def _handle_goal_failure(self, reason: str) -> None:
        step = self.steps[self.current_index]
        self.goal_active = False
        self.goal_handle = None
        if self.current_retry < step.maximum_retries:
            self.current_retry += 1
            self.retry_at = self._now_sec() + self.retry_delay
            self._set_state("RUNNING", "%s_retry_%d" % (reason, self.current_retry))
            return
        self.start_requested = False
        if self.mission_started_at is not None:
            self.mission_finished_at = self._now_sec()
        self._set_state("ABORTED", "%s_retries_exhausted" % reason)

    def _advance_after_hold(self) -> None:
        self.hold_until = None
        self.current_index += 1
        if self.current_index >= len(self.steps):
            self.start_requested = False
            self.mission_finished_at = self._now_sec()
            self._set_state("COMPLETED", "returned_to_start")
            timing = self._mission_timing()
            self.get_logger().info(
                "3분 제한 내 주차 미션 완주: %.1f/%.1f초, 남은 시간 %.1f초"
                % (
                    timing.elapsed_sec,
                    self.mission_time_limit,
                    timing.remaining_sec,
                )
            )
            return
        self._send_current_goal()

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        age = self.gate.age_assessment(now_sec)
        self.localization_ready = age.ready
        if not age.ready:
            self.localization_reason = age.reason

        if self.initial_pose_remaining > 0 and now_sec >= self.next_initial_pose_publish:
            self._publish_initial_pose()
            self.initial_pose_remaining -= 1
            self.next_initial_pose_publish = now_sec + self.initial_pose_period

        if self.state == "LOCALIZING" and self.localization_ready:
            self._set_state("READY", "amcl_stable")

        if self.state in {"RUNNING", "HOLDING"} and not self.localization_ready:
            if self.localization_lost_since is None:
                self.localization_lost_since = now_sec
            elif (
                self.goal_active
                and now_sec - self.localization_lost_since
                >= self.localization_loss_cancel_sec
            ):
                self._cancel_active_goal(localization_pause=True)
                self._set_state("PAUSED_LOCALIZATION", self.localization_reason)

        if self.state == "PAUSED_LOCALIZATION" and self.localization_ready:
            self.localization_lost_since = None
            self.retry_at = now_sec + self.retry_delay
            self._set_state("RUNNING", "localization_recovered")

        if self.start_requested and self.state == "READY":
            self._start_mission_clock(now_sec)
            self._send_current_goal()

        active_states = {"RUNNING", "HOLDING", "PAUSED_LOCALIZATION"}
        if self.mission_started_at is not None and self.state in active_states:
            timing = self._mission_timing(now_sec)
            if timing.expired:
                self.start_requested = False
                self._cancel_active_goal(localization_pause=False)
                self.mission_finished_at = now_sec
                self._set_state("ABORTED", "mission_time_limit")
                self.get_logger().error(
                    "경기 제한시간 초과: %.1f/%.1f초, 모터 권한을 해제했습니다"
                    % (timing.elapsed_sec, self.mission_time_limit)
                )
            else:
                if (
                    not self.time_warning_emitted
                    and timing.warning
                ):
                    self.time_warning_emitted = True
                    self.get_logger().warning(
                        "경기 종료까지 %.1f초 남았습니다: 현재 %d/%d단계"
                        % (
                            timing.remaining_sec,
                            self.current_index,
                            len(self.steps),
                        )
                    )
                if (
                    self.next_time_log_at is not None
                    and now_sec >= self.next_time_log_at
                ):
                    self.get_logger().info(
                        "경기 진행시간 %.1f초, 남은 시간 %.1f초, 진행 %d/%d단계"
                        % (
                            timing.elapsed_sec,
                            timing.remaining_sec,
                            self.current_index,
                            len(self.steps),
                        )
                    )
                    self.next_time_log_at = now_sec + self.time_log_period
        if (
            self.state == "RUNNING"
            and not self.goal_active
            and self.retry_at is not None
            and now_sec >= self.retry_at
        ):
            self.retry_at = None
            self._send_current_goal()
        if (
            self.state == "HOLDING"
            and self.hold_until is not None
            and now_sec >= self.hold_until
            and self.localization_ready
        ):
            self._advance_after_hold()

        authorized = (
            self.state == "RUNNING"
            and self.goal_active
            and self.localization_ready
        )
        self.authorization_publisher.publish(Bool(data=authorized))
        self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingMissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.authorization_publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
