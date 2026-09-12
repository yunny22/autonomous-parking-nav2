import pytest

from xycar_parking_nav.command_core import slew


def test_slew_limits_rate():
    assert slew(0.0, 10.0, 4.0, 0.5) == pytest.approx(2.0)
    assert slew(0.0, -10.0, 4.0, 0.5) == pytest.approx(-2.0)


def test_slew_reaches_near_target_without_overshoot():
    assert slew(1.0, 1.1, 4.0, 0.5) == pytest.approx(1.1)
