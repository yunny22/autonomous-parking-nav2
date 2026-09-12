import math

import pytest

from xycar_parking_nav.scan_filter_core import filter_ranges


def test_filter_ranges_rejects_invalid_near_and_far_returns():
    result = filter_ranges(
        [math.nan, math.inf, 0.05, 0.15, 2.0, 6.0, 6.01],
        minimum_range_m=0.15,
        maximum_range_m=6.0,
    )
    assert all(math.isnan(value) for value in result[:3])
    assert result[3:6] == [0.15, 2.0, 6.0]
    assert math.isnan(result[6])


def test_filter_ranges_rejects_invalid_limits():
    with pytest.raises(ValueError):
        filter_ranges(
            [1.0],
            minimum_range_m=2.0,
            maximum_range_m=1.0,
        )
