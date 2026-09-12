"""Deterministic kinematics and occupancy-grid ray casting for CI simulation."""

from __future__ import annotations

import math

from .map_core import OCCUPIED, OccupancyMap
from .mission_core import Pose2D, normalize_angle


def integrate_twist(pose: Pose2D, linear_x: float, angular_z: float, dt_sec: float) -> Pose2D:
    """Integrate planar body velocity with an exact constant-curvature step."""

    dt = max(0.0, float(dt_sec))
    velocity = float(linear_x)
    yaw_rate = float(angular_z)
    if not all(math.isfinite(value) for value in (velocity, yaw_rate, dt)):
        raise ValueError("kinematic inputs must be finite")
    yaw_delta = yaw_rate * dt
    if abs(yaw_rate) < 1.0e-9:
        delta_x_body = velocity * dt
        delta_y_body = 0.0
    else:
        radius = velocity / yaw_rate
        delta_x_body = radius * math.sin(yaw_delta)
        delta_y_body = radius * (1.0 - math.cos(yaw_delta))
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Pose2D(
        pose.x + cosine * delta_x_body - sine * delta_y_body,
        pose.y + sine * delta_x_body + cosine * delta_y_body,
        normalize_angle(pose.yaw + yaw_delta),
    )


def raycast_range(
    occupancy_map: OccupancyMap,
    *,
    origin_x: float,
    origin_y: float,
    heading: float,
    minimum_range_m: float,
    maximum_range_m: float,
    sample_step_m: float | None = None,
) -> float:
    """Return the first occupied-cell range; unknown/outside is no return."""

    minimum = max(0.0, float(minimum_range_m))
    maximum = float(maximum_range_m)
    if not math.isfinite(maximum) or maximum <= minimum:
        raise ValueError("maximum range must exceed minimum range")
    step = max(0.005, float(sample_step_m or occupancy_map.resolution * 0.5))
    cosine = math.cos(heading)
    sine = math.sin(heading)
    distance = minimum
    while distance <= maximum:
        cell = occupancy_map.world_cell(
            float(origin_x) + distance * cosine,
            float(origin_y) + distance * sine,
        )
        if cell == OCCUPIED:
            return distance
        distance += step
    return math.inf

