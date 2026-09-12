"""Pure readiness bookkeeping for the real-car parking preflight."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path


@dataclass
class TopicProbe:
    """Track whether a required ROS topic is alive and estimate its rate."""

    label: str
    topic: str
    minimum_samples: int
    hint: str
    count: int = 0
    first_sec: float | None = None
    last_sec: float | None = None

    def __post_init__(self) -> None:
        if self.minimum_samples < 1:
            raise ValueError("minimum_samples must be positive")

    def observe(self, stamp_sec: float) -> bool:
        """Record one sample and return True only for the first sample."""

        stamp = float(stamp_sec)
        if not math.isfinite(stamp):
            raise ValueError("sample time must be finite")
        first = self.count == 0
        if first:
            self.first_sec = stamp
        self.last_sec = stamp
        self.count += 1
        return first

    @property
    def ready(self) -> bool:
        return self.count >= self.minimum_samples

    @property
    def rate_hz(self) -> float | None:
        if (
            self.count < 2
            or self.first_sec is None
            or self.last_sec is None
            or self.last_sec <= self.first_sec
        ):
            return None
        return (self.count - 1) / (self.last_sec - self.first_sec)


def inspect_device(path_text: str) -> tuple[bool, str]:
    """Return serial-device availability and a user-facing description."""

    if not str(path_text).strip():
        return False, "장치 경로가 지정되지 않았습니다"
    path = Path(path_text)
    if not path.exists():
        return False, f"{path} 장치를 찾을 수 없습니다"
    resolved = path.resolve()
    if not os.access(path, os.R_OK | os.W_OK):
        return False, f"{path} -> {resolved} 읽기/쓰기 권한이 없습니다"
    return True, f"{path} -> {resolved}"
