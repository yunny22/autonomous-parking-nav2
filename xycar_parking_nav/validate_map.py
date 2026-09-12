#!/usr/bin/env python3
"""Validate the supplied map and every configured competition mission pose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml

from .command_core import Footprint
from .map_core import FREE, OCCUPIED, UNKNOWN, assess_pose_footprint, load_occupancy_map
from .mission_core import mission_steps_from_dicts, reference_pose_to_base


def validate(map_path: Path, mission_path: Path, margin_m: float) -> dict:
    occupancy_map = load_occupancy_map(map_path)
    with mission_path.open("r", encoding="utf-8") as stream:
        mission = yaml.safe_load(stream)["mission"]
    steps = mission_steps_from_dicts(mission["steps"])
    offset = float(mission.get("base_from_reference_x_m", 0.0))
    footprint = Footprint(-0.40, 0.40, 0.25)
    assessments = {}
    for step in steps:
        base_pose = reference_pose_to_base(step.reference_pose, offset)
        result = assess_pose_footprint(
            occupancy_map,
            base_pose,
            footprint,
            margin_m=margin_m,
        )
        assessments[step.name] = {
            "base_pose": [base_pose.x, base_pose.y, base_pose.yaw],
            "collision_free": result.collision_free,
            "occupied_cells": result.occupied_samples,
            "unknown_cells": result.unknown_samples,
            "outside_samples": result.outside_samples,
            "nearest_occupied_distance_m": result.nearest_occupied_distance_m,
        }
    counts = occupancy_map.counts()
    return {
        "map": {
            "width": occupancy_map.width,
            "height": occupancy_map.height,
            "resolution": occupancy_map.resolution,
            "origin": [
                occupancy_map.origin_x,
                occupancy_map.origin_y,
                occupancy_map.origin_yaw,
            ],
            "free_cells": counts[FREE],
            "occupied_cells": counts[OCCUPIED],
            "unknown_cells": counts[UNKNOWN],
        },
        "margin_m": margin_m,
        "poses": assessments,
        "valid": all(item["collision_free"] for item in assessments.values()),
    }


def main(argv=None) -> int:
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("xycar_parking_nav"))
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, default=share / "maps" / "example_map.yaml")
    parser.add_argument(
        "--mission",
        type=Path,
        default=share / "config" / "parking_mission.yaml",
    )
    parser.add_argument("--margin", type=float, default=0.095)
    args = parser.parse_args(argv)
    report = validate(args.map, args.mission, args.margin)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    sys.exit(main())
