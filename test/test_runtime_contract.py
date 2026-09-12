import ast
from ast import literal_eval
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _yaml(relative_path: str):
    with (PACKAGE / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_nav2_uses_ackermann_reeds_shepp_stack():
    config = _yaml("config/nav2_parking.yaml")
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert planner["plugin"] in {
        "nav2_smac_planner/SmacPlannerHybrid",
        "nav2_smac_planner::SmacPlannerHybrid",
    }
    assert planner["motion_model_for_search"] == "REEDS_SHEPP"
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "Ackermann"
    assert controller["vx_min"] < 0.0 < controller["vx_max"]


def test_public_mission_contains_31_goal_poses():
    mission = _yaml("config/parking_mission.yaml")["mission"]
    steps = mission["steps"]
    assert len(steps) == 31
    assert len({step["name"] for step in steps}) == len(steps)
    assert sum(bool(step.get("parking_goal")) for step in steps) == 2
    assert all({"name", "x", "y", "yaw"} <= set(step) for step in steps)


def test_public_defaults_are_safe_and_generic():
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    assert adapter["drive_enabled"] is False
    assert adapter["minimum_moving_command"] <= adapter["maximum_forward_command"]
    assert adapter["minimum_moving_command"] <= adapter["maximum_reverse_command"]
    assert len(adapter["steering_map_commands"]) == len(
        adapter["steering_map_curvatures"]
    )
    assert adapter["laser_x"] == 0.0
    assert adapter["laser_y"] == 0.0
    assert adapter["laser_yaw"] == 0.0


def test_costmap_footprints_match_adapter():
    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    expected = literal_eval(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )
    local = literal_eval(
        nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )
    assert expected == local
    assert min(point[0] for point in expected) == adapter["footprint_minimum_x"]
    assert max(point[0] for point in expected) == adapter["footprint_maximum_x"]
    assert max(abs(point[1]) for point in expected) == adapter["footprint_half_width"]


def test_behavior_tree_has_ackermann_path_actions():
    root = ET.parse(PACKAGE / "behavior_trees/ackermann_navigate_to_pose.xml").getroot()
    tags = {element.tag for element in root.iter()}
    assert "ComputePathToPose" in tags
    assert "FollowPath" in tags
    assert "Spin" not in tags


def test_launch_templates_have_no_machine_specific_paths():
    for path in (PACKAGE / "launch").glob("*.launch.py"):
        text = path.read_text(encoding="utf-8")
        assert "/dev/" not in text
        assert "/home/" not in text
        ast.parse(text)
    real = (PACKAGE / "launch/parking_real.launch.py").read_text(encoding="utf-8")
    assert 'default_value="false"' in real
