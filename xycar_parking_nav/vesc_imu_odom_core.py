"""Planar odometry math for VESC distance and IMU heading."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_yaw_delta(previous_wrapped: float, current_wrapped: float) -> float:
    """Return the shortest signed change between wrapped yaw samples."""
    return normalize_angle(current_wrapped - previous_wrapped)


def corrected_gyro_yaw_rate(
    raw_rate_rad_s: float,
    bias_rad_s: float,
    sign: float,
    scale: float,
    deadband_rad_s: float,
) -> float:
    """Convert the measured IMU Z rate into ROS-positive vehicle yaw rate."""
    corrected = (
        float(sign)
        * (float(raw_rate_rad_s) - float(bias_rad_s))
        * float(scale)
    )
    if abs(corrected) < abs(float(deadband_rad_s)):
        return 0.0
    return corrected


def adapt_gyro_bias(
    current_bias_rad_s: float,
    raw_rate_rad_s: float,
    dt_sec: float,
    time_constant_sec: float,
    maximum_residual_rad_s: float,
) -> float:
    """Track stationary gyro bias with a bounded first-order filter."""
    values = (
        current_bias_rad_s,
        raw_rate_rad_s,
        dt_sec,
        time_constant_sec,
        maximum_residual_rad_s,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return float(current_bias_rad_s)
    if dt_sec <= 0.0 or time_constant_sec <= 0.0:
        return float(current_bias_rad_s)

    residual_limit = abs(float(maximum_residual_rad_s))
    residual = float(raw_rate_rad_s) - float(current_bias_rad_s)
    residual = max(-residual_limit, min(residual_limit, residual))
    alpha = 1.0 - math.exp(-float(dt_sec) / float(time_constant_sec))
    return float(current_bias_rad_s) + alpha * residual


def signed_int32_delta(previous: float, current: float) -> float:
    """Difference two VESC tachometer readings, including int32 rollover."""
    delta = float(current) - float(previous)
    modulus = float(2**32)
    half = float(2**31)
    if delta > half:
        delta -= modulus
    elif delta < -half:
        delta += modulus
    return delta


def trapezoidal_distance(
    previous_speed_mps: float,
    current_speed_mps: float,
    dt_sec: float,
) -> float:
    if dt_sec <= 0.0:
        return 0.0
    return 0.5 * (previous_speed_mps + current_speed_mps) * dt_sec


def integrate_distance(
    state: OdomState,
    distance_m: float,
    previous_yaw: float,
    current_yaw: float,
) -> OdomState:
    """Integrate signed distance using midpoint heading."""
    yaw_delta = normalize_angle(current_yaw - previous_yaw)
    midpoint_yaw = previous_yaw + 0.5 * yaw_delta
    return OdomState(
        x=state.x + distance_m * math.cos(midpoint_yaw),
        y=state.y + distance_m * math.sin(midpoint_yaw),
        yaw=current_yaw,
    )
