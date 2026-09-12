import math

import pytest

from xycar_parking_nav.mission_core import (
    LocalizationGate,
    LocalizationGateConfig,
    Pose2D,
    assess_mission_time,
    mission_steps_from_dicts,
    pose_error,
    reference_pose_to_base,
)


def covariance(xy=0.01, yaw=0.02):
    result = [0.0] * 36
    result[0] = xy
    result[7] = xy
    result[35] = yaw
    return result


@pytest.mark.parametrize(
    "pose, expected",
    [
        (Pose2D(0.0, 0.0, 0.0), Pose2D(0.2, 0.0, 0.0)),
        (Pose2D(1.0, 1.0, -math.pi / 2.0), Pose2D(1.0, 0.8, -math.pi / 2.0)),
        (Pose2D(-1.0, 0.5, math.pi), Pose2D(-1.2, 0.5, math.pi)),
    ],
)
def test_reference_pose_is_shifted_along_heading(pose, expected):
    actual = reference_pose_to_base(pose, 0.2)
    assert actual.x == pytest.approx(expected.x)
    assert actual.y == pytest.approx(expected.y)
    assert pose_error(actual, expected) == pytest.approx((0.0, 0.0))


def test_localization_requires_consecutive_good_samples():
    gate = LocalizationGate(
        LocalizationGateConfig(required_stable_samples=3)
    )
    pose = Pose2D(1.0, 2.0, 0.2)
    assert not gate.update(
        pose=pose, covariance=covariance(), stamp_sec=1.0, now_sec=1.01
    ).ready
    assert not gate.update(
        pose=Pose2D(1.01, 2.0, 0.2),
        covariance=covariance(),
        stamp_sec=1.1,
        now_sec=1.11,
    ).ready
    assert gate.update(
        pose=Pose2D(1.02, 2.0, 0.2),
        covariance=covariance(),
        stamp_sec=1.2,
        now_sec=1.21,
    ).ready


def test_localization_jump_resets_stability():
    gate = LocalizationGate(
        LocalizationGateConfig(required_stable_samples=2)
    )
    gate.update(
        pose=Pose2D(0.0, 0.0, 0.0),
        covariance=covariance(),
        stamp_sec=1.0,
        now_sec=1.0,
    )
    assessment = gate.update(
        pose=Pose2D(1.0, 0.0, 0.0),
        covariance=covariance(),
        stamp_sec=1.1,
        now_sec=1.1,
    )
    assert assessment.reason == "position_jump"
    assert assessment.stable_samples == 0


def test_localization_age_fails_closed():
    gate = LocalizationGate(
        LocalizationGateConfig(required_stable_samples=1)
    )
    assert gate.update(
        pose=Pose2D(0.0, 0.0, 0.0),
        covariance=covariance(),
        stamp_sec=1.0,
        now_sec=1.0,
    ).ready
    assert gate.age_assessment(1.6).reason == "stale_pose"
    assert not gate.age_assessment(1.6).ready


def test_mission_parser_rejects_duplicate_names():
    with pytest.raises(ValueError, match="unique"):
        mission_steps_from_dicts(
            [
                {"name": "A", "x": 0, "y": 0, "yaw": 0},
                {"name": "A", "x": 1, "y": 0, "yaw": 0},
            ]
        )


def test_three_minute_clock_warns_expires_and_freezes_at_finish():
    normal = assess_mission_time(
        started_at_sec=100.0,
        now_sec=249.9,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
    )
    warning = assess_mission_time(
        started_at_sec=100.0,
        now_sec=250.0,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
    )
    expired = assess_mission_time(
        started_at_sec=100.0,
        now_sec=280.0,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
    )
    frozen = assess_mission_time(
        started_at_sec=100.0,
        now_sec=400.0,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
        finished_at_sec=212.9,
    )

    assert normal.elapsed_sec == pytest.approx(149.9)
    assert not normal.warning and not normal.expired
    assert warning.warning and not warning.expired
    assert expired.expired and expired.remaining_sec == 0.0
    assert frozen.elapsed_sec == pytest.approx(112.9)
    assert frozen.remaining_sec == pytest.approx(67.1)
    assert not frozen.expired
