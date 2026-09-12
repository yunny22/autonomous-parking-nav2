"""Pure mission and localization logic for deterministic parking tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("pose values must be finite")


@dataclass(frozen=True)
class MissionStep:
    name: str
    reference_pose: Pose2D
    hold_sec: float = 0.0
    parking_goal: bool = False
    maximum_retries: int = 2

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("mission step name must not be empty")
        if self.hold_sec < 0.0:
            raise ValueError("hold_sec must be non-negative")
        if self.maximum_retries < 0:
            raise ValueError("maximum_retries must be non-negative")


@dataclass(frozen=True)
class MissionTimeAssessment:
    elapsed_sec: float
    remaining_sec: float
    expired: bool
    warning: bool


def assess_mission_time(
    *,
    started_at_sec: float | None,
    now_sec: float,
    limit_sec: float,
    warning_remaining_sec: float,
    finished_at_sec: float | None = None,
) -> MissionTimeAssessment:
    """Assess the competition clock, including pauses and a frozen finish time."""

    values = (now_sec, limit_sec, warning_remaining_sec)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("mission timing values must be finite")
    if limit_sec <= 0.0:
        raise ValueError("mission time limit must be positive")
    if not 0.0 <= warning_remaining_sec < limit_sec:
        raise ValueError("mission warning must be within the time limit")
    if started_at_sec is None:
        return MissionTimeAssessment(0.0, float(limit_sec), False, False)
    if not math.isfinite(float(started_at_sec)):
        raise ValueError("mission start time must be finite")

    effective_now = float(now_sec)
    if finished_at_sec is not None:
        if not math.isfinite(float(finished_at_sec)):
            raise ValueError("mission finish time must be finite")
        effective_now = min(effective_now, float(finished_at_sec))
    elapsed = max(0.0, effective_now - float(started_at_sec))
    remaining = max(0.0, float(limit_sec) - elapsed)
    return MissionTimeAssessment(
        elapsed_sec=elapsed,
        remaining_sec=remaining,
        expired=elapsed >= float(limit_sec),
        warning=remaining <= float(warning_remaining_sec),
    )


def reference_pose_to_base(pose: Pose2D, base_from_reference_x_m: float) -> Pose2D:
    """Move a geometric-center target to the calibrated base-frame origin.

    The real vehicle currently defines ``base_footprint`` at the front wheel
    center. Competition targets are marked at the vehicle rectangle center.
    A positive offset therefore moves the target forward along target yaw.
    """

    offset = float(base_from_reference_x_m)
    if not math.isfinite(offset):
        raise ValueError("base reference offset must be finite")
    return Pose2D(
        x=pose.x + offset * math.cos(pose.yaw),
        y=pose.y + offset * math.sin(pose.yaw),
        yaw=normalize_angle(pose.yaw),
    )


def pose_error(current: Pose2D, target: Pose2D) -> tuple[float, float]:
    return (
        math.hypot(current.x - target.x, current.y - target.y),
        abs(normalize_angle(current.yaw - target.yaw)),
    )


@dataclass(frozen=True)
class LocalizationGateConfig:
    maximum_xy_variance: float = 0.04
    maximum_yaw_variance: float = 0.08
    maximum_pose_age_sec: float = 0.50
    maximum_position_jump_m: float = 0.60
    maximum_yaw_jump_rad: float = 0.80
    required_stable_samples: int = 8

    def __post_init__(self) -> None:
        positive = (
            self.maximum_xy_variance,
            self.maximum_yaw_variance,
            self.maximum_pose_age_sec,
            self.maximum_position_jump_m,
            self.maximum_yaw_jump_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("localization gate limits must be finite and positive")
        if self.required_stable_samples <= 0:
            raise ValueError("required_stable_samples must be positive")


@dataclass(frozen=True)
class LocalizationAssessment:
    ready: bool
    reason: str
    stable_samples: int


class LocalizationGate:
    """Reject stale, uncertain, or discontinuous AMCL estimates."""

    def __init__(self, config: LocalizationGateConfig) -> None:
        self.config = config
        self.stable_samples = 0
        self.last_pose: Pose2D | None = None
        self.last_stamp_sec: float | None = None
        self.last_reason = "no_pose"

    def update(
        self,
        *,
        pose: Pose2D,
        covariance: Sequence[float],
        stamp_sec: float,
        now_sec: float,
    ) -> LocalizationAssessment:
        reason = self._sample_reason(
            pose=pose,
            covariance=covariance,
            stamp_sec=stamp_sec,
            now_sec=now_sec,
        )
        if reason == "ok":
            self.stable_samples += 1
        else:
            self.stable_samples = 0

        self.last_pose = pose
        self.last_stamp_sec = stamp_sec
        self.last_reason = reason
        return LocalizationAssessment(
            ready=(
                reason == "ok"
                and self.stable_samples >= self.config.required_stable_samples
            ),
            reason=reason,
            stable_samples=self.stable_samples,
        )

    def age_assessment(self, now_sec: float) -> LocalizationAssessment:
        if self.last_stamp_sec is None:
            return LocalizationAssessment(False, "no_pose", self.stable_samples)
        if now_sec - self.last_stamp_sec > self.config.maximum_pose_age_sec:
            self.stable_samples = 0
            self.last_reason = "stale_pose"
            return LocalizationAssessment(False, self.last_reason, 0)
        return LocalizationAssessment(
            self.stable_samples >= self.config.required_stable_samples,
            self.last_reason,
            self.stable_samples,
        )

    def _sample_reason(
        self,
        *,
        pose: Pose2D,
        covariance: Sequence[float],
        stamp_sec: float,
        now_sec: float,
    ) -> str:
        if len(covariance) < 36:
            return "bad_covariance"
        checked = (covariance[0], covariance[7], covariance[35], stamp_sec, now_sec)
        if not all(math.isfinite(float(value)) for value in checked):
            return "non_finite"
        if stamp_sec <= 0.0 or now_sec < stamp_sec:
            return "bad_stamp"
        if now_sec - stamp_sec > self.config.maximum_pose_age_sec:
            return "stale_pose"
        if max(float(covariance[0]), float(covariance[7])) > self.config.maximum_xy_variance:
            return "xy_uncertain"
        if float(covariance[35]) > self.config.maximum_yaw_variance:
            return "yaw_uncertain"
        if self.last_pose is not None and self.last_stamp_sec is not None:
            if stamp_sec <= self.last_stamp_sec:
                return "non_monotonic_stamp"
            position_jump, yaw_jump = pose_error(pose, self.last_pose)
            if position_jump > self.config.maximum_position_jump_m:
                return "position_jump"
            if yaw_jump > self.config.maximum_yaw_jump_rad:
                return "yaw_jump"
        return "ok"


def mission_steps_from_dicts(items: Iterable[dict]) -> list[MissionStep]:
    steps = []
    for item in items:
        steps.append(
            MissionStep(
                name=str(item["name"]),
                reference_pose=Pose2D(
                    float(item["x"]),
                    float(item["y"]),
                    float(item["yaw"]),
                ),
                hold_sec=float(item.get("hold_sec", 0.0)),
                parking_goal=bool(item.get("parking_goal", False)),
                maximum_retries=int(item.get("maximum_retries", 2)),
            )
        )
    if not steps:
        raise ValueError("mission must contain at least one step")
    names = [step.name for step in steps]
    if len(names) != len(set(names)):
        raise ValueError("mission step names must be unique")
    return steps
