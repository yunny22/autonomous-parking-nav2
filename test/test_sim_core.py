import math

import pytest

from xycar_parking_nav.map_core import FREE, OCCUPIED, OccupancyMap
from xycar_parking_nav.mission_core import Pose2D
from xycar_parking_nav.sim_core import integrate_twist, raycast_range


def _wall_map() -> OccupancyMap:
    cells = []
    for _y in range(5):
        for x in range(5):
            cells.append(OCCUPIED if x == 3 else FREE)
    return OccupancyMap(5, 5, 1.0, 0.0, 0.0, 0.0, tuple(cells))


def test_integrate_straight_and_reverse():
    start = Pose2D(1.0, 2.0, math.pi / 2.0)
    forward = integrate_twist(start, 0.3, 0.0, 2.0)
    reverse = integrate_twist(forward, -0.3, 0.0, 2.0)

    assert forward.x == pytest.approx(1.0)
    assert forward.y == pytest.approx(2.6)
    assert reverse.x == pytest.approx(start.x)
    assert reverse.y == pytest.approx(start.y)


def test_integrate_constant_curvature_arc():
    result = integrate_twist(Pose2D(0.0, 0.0, 0.0), 1.0, 1.0, math.pi / 2.0)

    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(1.0)
    assert result.yaw == pytest.approx(math.pi / 2.0)


def test_raycast_returns_first_occupied_cell():
    distance = raycast_range(
        _wall_map(),
        origin_x=0.5,
        origin_y=2.5,
        heading=0.0,
        minimum_range_m=0.1,
        maximum_range_m=5.0,
        sample_step_m=0.05,
    )

    assert distance == pytest.approx(2.5, abs=0.051)


def test_raycast_without_hit_returns_infinity():
    distance = raycast_range(
        _wall_map(),
        origin_x=0.5,
        origin_y=2.5,
        heading=math.pi,
        minimum_range_m=0.1,
        maximum_range_m=2.0,
    )

    assert math.isinf(distance)

