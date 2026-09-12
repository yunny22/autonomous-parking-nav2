from pathlib import Path

import pytest

from xycar_parking_nav.preflight_core import TopicProbe, inspect_device


def test_topic_probe_requires_multiple_live_samples_and_reports_rate():
    probe = TopicProbe("LiDAR", "/scan", 3, "check lidar")

    assert probe.observe(10.0) is True
    assert probe.observe(10.1) is False
    assert probe.ready is False
    probe.observe(10.2)

    assert probe.ready is True
    assert probe.rate_hz == pytest.approx(10.0)


def test_topic_probe_rejects_invalid_sample_requirements_and_times():
    with pytest.raises(ValueError):
        TopicProbe("bad", "/bad", 0, "bad")

    probe = TopicProbe("IMU", "/imu", 1, "check imu")
    with pytest.raises(ValueError):
        probe.observe(float("nan"))


def test_device_inspection_distinguishes_existing_and_missing_paths(tmp_path: Path):
    device = tmp_path / "ttyTEST"
    device.touch()

    ready, detail = inspect_device(str(device))
    assert ready is True
    assert str(device) in detail

    ready, detail = inspect_device(str(tmp_path / "missing"))
    assert ready is False
    assert "찾을 수 없습니다" in detail
