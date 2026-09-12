from pathlib import Path

from xycar_parking_nav.command_core import Footprint
from xycar_parking_nav.hybrid_astar_core import HybridAStarPlanner
from xycar_parking_nav.map_core import load_occupancy_map
from xycar_parking_nav.mission_core import Pose2D


PACKAGE = Path(__file__).resolve().parents[1]


def test_public_map_planner_handles_a_short_free_space_leg():
    planner = HybridAStarPlanner(
        load_occupancy_map(PACKAGE / "maps/example_map.yaml"),
        Footprint(-0.25, 0.25, 0.15),
    )
    result = planner.plan(Pose2D(-1.5, -1.5, 0.0), Pose2D(-0.5, -1.5, 0.0))
    assert result.success
    assert result.points
