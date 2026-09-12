"""Ackermann command conversion and LiDAR swept-footprint safety checks."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def slew(current: float, target: float, maximum_rate: float, dt_sec: float) -> float:
    if dt_sec <= 0.0:
        return float(current)
    delta = abs(float(maximum_rate)) * dt_sec
    return clamp(float(target), float(current) - delta, float(current) + delta)


def interpolate_clamped(
    value: float,
    inputs: Sequence[float],
    outputs: Sequence[float],
) -> float:
    if len(inputs) != len(outputs) or len(inputs) < 2:
        raise ValueError("lookup inputs and outputs must have equal length >= 2")
    if any(right <= left for left, right in zip(inputs, inputs[1:])):
        raise ValueError("lookup inputs must be strictly increasing")
    if value <= inputs[0]:
        return float(outputs[0])
    if value >= inputs[-1]:
        return float(outputs[-1])
    upper = bisect_right(inputs, value)
    lower = upper - 1
    fraction = (value - inputs[lower]) / (inputs[upper] - inputs[lower])
    return float(outputs[lower] + fraction * (outputs[upper] - outputs[lower]))


@dataclass(frozen=True)
class MotorCalibration:
    speed_gain_mps_per_command: float
    minimum_moving_command: float
    maximum_forward_command: float
    maximum_reverse_command: float
    steering_commands: tuple[float, ...]
    steering_curvatures: tuple[float, ...]

    def __post_init__(self) -> None:
        if self.speed_gain_mps_per_command <= 0.0:
            raise ValueError("speed gain must be positive")
        if self.minimum_moving_command < 0.0:
            raise ValueError("minimum moving command must be non-negative")
        if self.maximum_forward_command <= 0.0 or self.maximum_reverse_command <= 0.0:
            raise ValueError("command limits must be positive magnitudes")
        if self.minimum_moving_command > min(
            self.maximum_forward_command,
            self.maximum_reverse_command,
        ):
            raise ValueError("minimum moving command must not exceed command limits")
        if len(self.steering_commands) != len(self.steering_curvatures):
            raise ValueError("steering calibration vectors must have equal lengths")
        if len(self.steering_commands) < 2:
            raise ValueError("at least two steering calibration points are required")
        if any(
            right <= left
            for left, right in zip(self.steering_commands, self.steering_commands[1:])
        ):
            raise ValueError("steering commands must be strictly increasing")
        diffs = [
            right - left
            for left, right in zip(self.steering_curvatures, self.steering_curvatures[1:])
        ]
        if not (all(diff > 0.0 for diff in diffs) or all(diff < 0.0 for diff in diffs)):
            raise ValueError("steering curvatures must be strictly monotonic")


@dataclass(frozen=True)
class MotorCommand:
    steering_command: float
    speed_command: float
    curvature: float
    reason: str = "ok"


def curvature_to_steering_command(
    curvature: float,
    calibration: MotorCalibration,
) -> float:
    curvatures = calibration.steering_curvatures
    commands = calibration.steering_commands
    if curvatures[0] > curvatures[-1]:
        curvatures = tuple(reversed(curvatures))
        commands = tuple(reversed(commands))
    return interpolate_clamped(curvature, curvatures, commands)


def motor_command_to_twist(
    steering_command: float,
    speed_command: float,
    calibration: MotorCalibration,
) -> tuple[float, float]:
    """Invert the calibrated motor interface for hardware-in-the-loop simulation."""

    if not all(
        math.isfinite(float(value)) for value in (steering_command, speed_command)
    ):
        return 0.0, 0.0
    curvature = interpolate_clamped(
        float(steering_command),
        calibration.steering_commands,
        calibration.steering_curvatures,
    )
    linear = float(speed_command) * calibration.speed_gain_mps_per_command
    return linear, linear * curvature


def twist_to_motor_command(
    linear_x_mps: float,
    angular_z_rad_s: float,
    calibration: MotorCalibration,
    *,
    stationary_speed_epsilon: float = 1.0e-3,
) -> MotorCommand:
    values = (linear_x_mps, angular_z_rad_s)
    if not all(math.isfinite(float(value)) for value in values):
        return MotorCommand(0.0, 0.0, 0.0, "non_finite_twist")
    speed = float(linear_x_mps)
    if abs(speed) <= abs(float(stationary_speed_epsilon)):
        # An Ackermann vehicle cannot execute a Nav2 rotate-in-place command.
        reason = "stopped" if abs(angular_z_rad_s) <= 1.0e-6 else "rotate_in_place_rejected"
        return MotorCommand(0.0, 0.0, 0.0, reason)

    requested_curvature = float(angular_z_rad_s) / speed
    steering = curvature_to_steering_command(requested_curvature, calibration)
    # The lookup clamps at the measured steering endpoints. Collision
    # prediction and simulation must use that physically achievable curvature,
    # not an impossible pre-saturation Twist request.
    curvature = interpolate_clamped(
        steering,
        calibration.steering_commands,
        calibration.steering_curvatures,
    )
    requested_speed_command = speed / calibration.speed_gain_mps_per_command
    limit = (
        calibration.maximum_forward_command
        if requested_speed_command > 0.0
        else calibration.maximum_reverse_command
    )
    requested_speed_command = clamp(requested_speed_command, -limit, limit)
    speed_command = requested_speed_command
    if 0.0 < abs(speed_command) < calibration.minimum_moving_command:
        speed_command = math.copysign(calibration.minimum_moving_command, speed_command)
    return MotorCommand(steering, speed_command, curvature, "ok")


@dataclass(frozen=True)
class Footprint:
    minimum_x: float
    maximum_x: float
    half_width: float

    def __post_init__(self) -> None:
        if self.minimum_x >= self.maximum_x:
            raise ValueError("footprint minimum_x must be less than maximum_x")
        if self.half_width <= 0.0:
            raise ValueError("footprint half_width must be positive")


@dataclass(frozen=True)
class CollisionAssessment:
    collision: bool
    obstacle_index: int | None
    travel_m: float
    predicted_distance_m: float


def scan_points_in_base(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
) -> list[tuple[float, float]]:
    points = []
    cosine = math.cos(laser_yaw)
    sine = math.sin(laser_yaw)
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        laser_point_x = distance * math.cos(angle)
        laser_point_y = distance * math.sin(angle)
        points.append(
            (
                laser_x + cosine * laser_point_x - sine * laser_point_y,
                laser_y + sine * laser_point_x + cosine * laser_point_y,
            )
        )
    return points


def predicted_stop_distance(
    speed_mps: float,
    *,
    reaction_time_sec: float,
    braking_deceleration_mps2: float,
    minimum_projection_m: float,
    maximum_projection_m: float,
) -> float:
    speed = abs(float(speed_mps))
    if braking_deceleration_mps2 <= 0.0:
        raise ValueError("braking deceleration must be positive")
    distance = (
        speed * max(0.0, reaction_time_sec)
        + speed * speed / (2.0 * braking_deceleration_mps2)
    )
    return clamp(max(distance, minimum_projection_m), 0.0, maximum_projection_m)


def _pose_after_signed_distance(distance: float, curvature: float) -> tuple[float, float, float]:
    heading = curvature * distance
    if abs(curvature) < 1.0e-9:
        return distance, 0.0, 0.0
    return (
        math.sin(heading) / curvature,
        (1.0 - math.cos(heading)) / curvature,
        heading,
    )


def swept_footprint_collision(
    points: Sequence[tuple[float, float]],
    *,
    speed_mps: float,
    curvature: float,
    footprint: Footprint,
    margin_m: float,
    reaction_time_sec: float,
    braking_deceleration_mps2: float,
    minimum_projection_m: float,
    maximum_projection_m: float,
    sample_step_m: float,
) -> CollisionAssessment:
    """Check a full rectangular body along the commanded bicycle arc."""

    projected = predicted_stop_distance(
        speed_mps,
        reaction_time_sec=reaction_time_sec,
        braking_deceleration_mps2=braking_deceleration_mps2,
        minimum_projection_m=minimum_projection_m,
        maximum_projection_m=maximum_projection_m,
    )
    if abs(speed_mps) < 1.0e-6:
        return CollisionAssessment(False, None, 0.0, projected)
    step = max(0.01, abs(sample_step_m))
    samples = max(1, int(math.ceil(projected / step)))
    direction = 1.0 if speed_mps > 0.0 else -1.0
    expanded_min_x = footprint.minimum_x - max(0.0, margin_m)
    expanded_max_x = footprint.maximum_x + max(0.0, margin_m)
    expanded_half_width = footprint.half_width + max(0.0, margin_m)

    for sample in range(samples + 1):
        travel = projected * sample / samples
        signed_distance = direction * travel
        pose_x, pose_y, heading = _pose_after_signed_distance(
            signed_distance,
            curvature,
        )
        cosine = math.cos(heading)
        sine = math.sin(heading)
        for obstacle_index, (point_x, point_y) in enumerate(points):
            delta_x = point_x - pose_x
            delta_y = point_y - pose_y
            local_x = cosine * delta_x + sine * delta_y
            local_y = -sine * delta_x + cosine * delta_y
            if (
                expanded_min_x <= local_x <= expanded_max_x
                and abs(local_y) <= expanded_half_width
            ):
                return CollisionAssessment(
                    True,
                    obstacle_index,
                    travel,
                    projected,
                )
    return CollisionAssessment(False, None, projected, projected)


class DirectionChangeGuard:
    """Insert a zero-speed dwell before changing drive direction."""

    def __init__(self, dwell_sec: float) -> None:
        self.dwell_sec = max(0.0, float(dwell_sec))
        self.active_direction = 0
        self.pending_direction = 0
        self.release_time_sec: float | None = None

    def filter(self, desired_speed_command: float, now_sec: float) -> tuple[float, str]:
        desired_direction = 0
        if desired_speed_command > 0.0:
            desired_direction = 1
        elif desired_speed_command < 0.0:
            desired_direction = -1

        if desired_direction == 0:
            return 0.0, "stopped"
        if self.active_direction == 0:
            self.active_direction = desired_direction
            return float(desired_speed_command), "ok"
        if desired_direction == self.active_direction and self.pending_direction == 0:
            return float(desired_speed_command), "ok"

        if self.pending_direction != desired_direction:
            self.pending_direction = desired_direction
            self.release_time_sec = now_sec + self.dwell_sec
            return 0.0, "direction_change_dwell"
        if self.release_time_sec is not None and now_sec < self.release_time_sec:
            return 0.0, "direction_change_dwell"

        self.active_direction = desired_direction
        self.pending_direction = 0
        self.release_time_sec = None
        return float(desired_speed_command), "ok"

    def reset(self) -> None:
        self.active_direction = 0
        self.pending_direction = 0
        self.release_time_sec = None
