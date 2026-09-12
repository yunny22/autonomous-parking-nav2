#!/usr/bin/env python3
"""Plan every configured leg with the independent Hybrid-A* verifier."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml

from xycar_parking_nav.command_core import Footprint
from xycar_parking_nav.hybrid_astar_core import HybridAStarPlanner
from xycar_parking_nav.map_core import load_occupancy_map
from xycar_parking_nav.mission_core import (
    Pose2D,
    mission_steps_from_dicts,
    reference_pose_to_base,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    package = args.package.resolve()
    occupancy_map = load_occupancy_map(package / "maps" / "example_map.yaml")
    with (package / "config" / "parking_mission.yaml").open(
        "r", encoding="utf-8"
    ) as stream:
        mission = yaml.safe_load(stream)["mission"]
    offset = float(mission["base_from_reference_x_m"])
    initial = mission["initial_pose"]
    start = reference_pose_to_base(
        Pose2D(float(initial["x"]), float(initial["y"]), float(initial["yaw"])),
        offset,
    )
    steps = mission_steps_from_dicts(mission["steps"])
    planner = HybridAStarPlanner(
        occupancy_map,
        Footprint(-0.40, 0.40, 0.25),
    )
    report = {"legs": [], "valid": True}
    for step in steps:
        goal = reference_pose_to_base(step.reference_pose, offset)
        result = planner.plan(start, goal)
        report["legs"].append(
            {
                "name": step.name,
                "success": result.success,
                "reason": result.reason,
                "cost": result.cost,
                "expansions": result.expansions,
                "path_points": len(result.points),
                "direction_changes": result.direction_changes,
                "start": [start.x, start.y, start.yaw],
                "goal": [goal.x, goal.y, goal.yaw],
                "path": [
                    [
                        point.pose.x,
                        point.pose.y,
                        point.pose.yaw,
                        point.direction,
                        point.curvature,
                    ]
                    for point in result.points
                ],
            }
        )
        if not result.success:
            report["valid"] = False
            break
        start = result.points[-1].pose
    payload = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    sys.exit(main())
