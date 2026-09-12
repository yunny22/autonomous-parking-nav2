"""Pure range filtering used by the SLAM LaserScan adapter."""

from __future__ import annotations

import math
from typing import Iterable


def filter_ranges(
    ranges: Iterable[float],
    *,
    minimum_range_m: float,
    maximum_range_m: float,
) -> list[float]:
    minimum = max(0.0, float(minimum_range_m))
    maximum = float(maximum_range_m)
    if not math.isfinite(maximum) or maximum <= minimum:
        raise ValueError("maximum range must be finite and greater than minimum")
    return [
        value
        if math.isfinite(value := float(raw))
        and minimum <= value <= maximum
        else math.nan
        for raw in ranges
    ]
