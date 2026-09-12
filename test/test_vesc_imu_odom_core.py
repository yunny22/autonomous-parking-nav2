import math

import pytest

from xycar_parking_nav.vesc_imu_odom_core import (
    OdomState,
    adapt_gyro_bias,
    corrected_gyro_yaw_rate,
    integrate_distance,
    signed_int32_delta,
    trapezoidal_distance,
    unwrap_yaw_delta,
)


def test_stationary_gyro_bias_moves_toward_measurement():
    updated = adapt_gyro_bias(
        current_bias_rad_s=0.030,
        raw_rate_rad_s=0.040,
        dt_sec=1.0,
        time_constant_sec=5.0,
        maximum_residual_rad_s=0.08,
    )
    assert 0.030 < updated < 0.040


def test_stationary_gyro_bias_limits_outlier_influence():
    updated = adapt_gyro_bias(
        current_bias_rad_s=0.030,
        raw_rate_rad_s=5.0,
        dt_sec=1.0,
        time_constant_sec=5.0,
        maximum_residual_rad_s=0.08,
    )
    expected = 0.030 + (1.0 - math.exp(-0.2)) * 0.08
    assert updated == pytest.approx(expected)


@pytest.mark.parametrize("dt_sec", [0.0, -1.0, math.nan])
def test_stationary_gyro_bias_rejects_invalid_time(dt_sec):
    assert adapt_gyro_bias(0.030, 0.040, dt_sec, 5.0, 0.08) == 0.030


def test_gyro_yaw_rate_applies_bias_sign_and_scale():
    rate = corrected_gyro_yaw_rate(
        raw_rate_rad_s=-1.0,
        bias_rad_s=0.03,
        sign=-1.0,
        scale=0.9922,
        deadband_rad_s=0.015,
    )
    assert rate == pytest.approx(1.03 * 0.9922)


def test_gyro_yaw_rate_suppresses_stationary_quantization():
    rate = corrected_gyro_yaw_rate(
        raw_rate_rad_s=0.04,
        bias_rad_s=0.03,
        sign=-1.0,
        scale=0.9922,
        deadband_rad_s=0.015,
    )
    assert rate == 0.0


def test_yaw_unwrap_crosses_positive_pi_without_reversing():
    delta = unwrap_yaw_delta(
        math.radians(179.0),
        math.radians(-179.0),
    )
    assert delta == pytest.approx(math.radians(2.0))


def test_tachometer_delta_handles_int32_rollover():
    delta = signed_int32_delta(2**31 - 3, -(2**31) + 4)
    assert delta == pytest.approx(7.0)


def test_trapezoidal_speed_distance():
    assert trapezoidal_distance(0.0, 1.0, 2.0) == pytest.approx(1.0)


def test_straight_distance_integration():
    state = integrate_distance(OdomState(), 2.0, 0.0, 0.0)
    assert state.x == pytest.approx(2.0)
    assert state.y == pytest.approx(0.0)
    assert state.yaw == pytest.approx(0.0)


def test_turning_distance_uses_midpoint_heading():
    state = integrate_distance(
        OdomState(),
        1.0,
        0.0,
        math.pi / 2.0,
    )
    expected = math.sqrt(0.5)
    assert state.x == pytest.approx(expected)
    assert state.y == pytest.approx(expected)
    assert state.yaw == pytest.approx(math.pi / 2.0)
