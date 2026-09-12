"""Small deterministic Hybrid-A* verifier matching the Nav2 parking model.

The runtime planner remains Nav2 SmacPlannerHybrid. This implementation is an
independent offline feasibility oracle so map and mission changes can be tested
without motor hardware or installed Nav2 binaries.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable

from .command_core import Footprint
from .map_core import FREE, OccupancyMap
from .mission_core import Pose2D, normalize_angle, pose_error


@dataclass(frozen=True)
class HybridAStarConfig:
    minimum_turning_radius_m: float = 0.50
    primitive_length_m: float = 0.20
    collision_sample_step_m: float = 0.05
    xy_quantization_m: float = 0.10
    yaw_bins: int = 72
    footprint_margin_m: float = 0.05
    goal_position_tolerance_m: float = 0.15
    goal_yaw_tolerance_rad: float = 0.20
    reverse_multiplier: float = 1.25
    direction_change_cost: float = 0.20
    non_straight_multiplier: float = 1.10
    steering_change_cost: float = 0.03
    maximum_expansions: int = 350000

    def __post_init__(self) -> None:
        if self.minimum_turning_radius_m <= 0.0:
            raise ValueError("minimum turning radius must be positive")
        if self.primitive_length_m <= 0.0 or self.collision_sample_step_m <= 0.0:
            raise ValueError("motion sample lengths must be positive")
        if self.xy_quantization_m <= 0.0 or self.yaw_bins < 8:
            raise ValueError("state quantization is invalid")


@dataclass(frozen=True)
class HybridPathPoint:
    pose: Pose2D
    direction: int
    curvature: float


@dataclass(frozen=True)
class HybridPlan:
    success: bool
    points: tuple[HybridPathPoint, ...]
    cost: float
    expansions: int
    reason: str

    @property
    def direction_changes(self) -> int:
        directions = [point.direction for point in self.points if point.direction]
        return sum(a != b for a, b in zip(directions, directions[1:]))


@dataclass
class _Node:
    pose: Pose2D
    direction: int
    curvature: float
    cost: float
    parent_key: tuple[int, int, int, int] | None


class HybridAStarPlanner:
    def __init__(
        self,
        occupancy_map: OccupancyMap,
        footprint: Footprint,
        config: HybridAStarConfig | None = None,
    ) -> None:
        self.map = occupancy_map
        self.footprint = footprint
        self.config = config or HybridAStarConfig()
        maximum_curvature = 1.0 / self.config.minimum_turning_radius_m
        self.curvatures = (
            -maximum_curvature,
            -0.5 * maximum_curvature,
            0.0,
            0.5 * maximum_curvature,
            maximum_curvature,
        )
        self.footprint_samples = self._make_footprint_samples()

    def _make_footprint_samples(self) -> tuple[tuple[float, float], ...]:
        margin = self.config.footprint_margin_m
        minimum_x = self.footprint.minimum_x - margin
        maximum_x = self.footprint.maximum_x + margin
        half_width = self.footprint.half_width + margin
        step = min(self.map.resolution * 0.75, 0.04)
        nx = max(1, math.ceil((maximum_x - minimum_x) / step))
        ny = max(1, math.ceil(2.0 * half_width / step))
        return tuple(
            (
                minimum_x + (maximum_x - minimum_x) * ix / nx,
                -half_width + 2.0 * half_width * iy / ny,
            )
            for ix in range(nx + 1)
            for iy in range(ny + 1)
        )

    def pose_is_free(self, pose: Pose2D) -> bool:
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        for local_x, local_y in self.footprint_samples:
            world_x = pose.x + cosine * local_x - sine * local_y
            world_y = pose.y + sine * local_x + cosine * local_y
            if self.map.world_cell(world_x, world_y) != FREE:
                return False
        return True

    def _key(self, pose: Pose2D, direction: int) -> tuple[int, int, int, int]:
        yaw_fraction = (normalize_angle(pose.yaw) + math.pi) / (2.0 * math.pi)
        yaw_index = int(round(yaw_fraction * self.config.yaw_bins)) % self.config.yaw_bins
        return (
            int(round(pose.x / self.config.xy_quantization_m)),
            int(round(pose.y / self.config.xy_quantization_m)),
            yaw_index,
            1 if direction >= 0 else -1,
        )

    @staticmethod
    def _integrate(pose: Pose2D, signed_distance: float, curvature: float) -> Pose2D:
        heading_delta = curvature * signed_distance
        if abs(curvature) < 1.0e-9:
            return Pose2D(
                pose.x + signed_distance * math.cos(pose.yaw),
                pose.y + signed_distance * math.sin(pose.yaw),
                pose.yaw,
            )
        final_yaw = pose.yaw + heading_delta
        return Pose2D(
            pose.x + (math.sin(final_yaw) - math.sin(pose.yaw)) / curvature,
            pose.y + (-math.cos(final_yaw) + math.cos(pose.yaw)) / curvature,
            normalize_angle(final_yaw),
        )

    def _rollout(self, pose: Pose2D, direction: int, curvature: float) -> Pose2D | None:
        samples = max(
            1,
            math.ceil(
                self.config.primitive_length_m
                / self.config.collision_sample_step_m
            ),
        )
        endpoint = pose
        for sample in range(1, samples + 1):
            signed_distance = (
                direction
                * self.config.primitive_length_m
                * sample
                / samples
            )
            endpoint = self._integrate(pose, signed_distance, curvature)
            if not self.pose_is_free(endpoint):
                return None
        return endpoint

    def _holonomic_heuristic(self, goal: Pose2D) -> list[float]:
        distances = [math.inf] * (self.map.width * self.map.height)
        goal_grid = self.map.world_to_grid(goal.x, goal.y)
        if goal_grid is None:
            return distances
        goal_index = goal_grid[1] * self.map.width + goal_grid[0]
        distances[goal_index] = 0.0
        queue = [(0.0, goal_grid[0], goal_grid[1])]
        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )
        while queue:
            cost, grid_x, grid_y = heapq.heappop(queue)
            index = grid_y * self.map.width + grid_x
            if cost > distances[index] + 1.0e-12:
                continue
            for delta_x, delta_y, multiplier in neighbors:
                next_x = grid_x + delta_x
                next_y = grid_y + delta_y
                if not (0 <= next_x < self.map.width and 0 <= next_y < self.map.height):
                    continue
                if self.map.cell(next_x, next_y) != FREE:
                    continue
                next_index = next_y * self.map.width + next_x
                next_cost = cost + multiplier * self.map.resolution
                if next_cost < distances[next_index]:
                    distances[next_index] = next_cost
                    heapq.heappush(queue, (next_cost, next_x, next_y))
        return distances

    def _heuristic(self, pose: Pose2D, goal: Pose2D, grid_distances: list[float]) -> float:
        grid = self.map.world_to_grid(pose.x, pose.y)
        grid_distance = math.inf
        if grid is not None:
            grid_distance = grid_distances[grid[1] * self.map.width + grid[0]]
        euclidean = math.hypot(goal.x - pose.x, goal.y - pose.y)
        translation = grid_distance if math.isfinite(grid_distance) else 3.0 * euclidean
        yaw = abs(normalize_angle(goal.yaw - pose.yaw))
        return max(euclidean, translation) + 0.20 * self.config.minimum_turning_radius_m * yaw

    def plan(self, start: Pose2D, goal: Pose2D) -> HybridPlan:
        if not self.pose_is_free(start):
            return HybridPlan(False, (), math.inf, 0, "start_collision")
        if not self.pose_is_free(goal):
            return HybridPlan(False, (), math.inf, 0, "goal_collision")

        grid_distances = self._holonomic_heuristic(goal)
        open_queue: list[tuple[float, int, tuple[int, int, int, int]]] = []
        nodes: dict[tuple[int, int, int, int], _Node] = {}
        best_cost: dict[tuple[int, int, int, int], float] = {}
        counter = 0
        # Search both initial direction labels; the first primitive determines motion.
        start_key = self._key(start, 1)
        start_node = _Node(start, 0, 0.0, 0.0, None)
        nodes[start_key] = start_node
        best_cost[start_key] = 0.0
        heapq.heappush(
            open_queue,
            (self._heuristic(start, goal, grid_distances), counter, start_key),
        )

        expansions = 0
        reached_key = None
        while open_queue and expansions < self.config.maximum_expansions:
            _, _, key = heapq.heappop(open_queue)
            node = nodes[key]
            if node.cost > best_cost.get(key, math.inf) + 1.0e-9:
                continue
            expansions += 1
            position_error, yaw_error = pose_error(node.pose, goal)
            if (
                position_error <= self.config.goal_position_tolerance_m
                and yaw_error <= self.config.goal_yaw_tolerance_rad
            ):
                reached_key = key
                break

            for direction in (1, -1):
                for curvature in self.curvatures:
                    endpoint = self._rollout(node.pose, direction, curvature)
                    if endpoint is None:
                        continue
                    next_key = self._key(endpoint, direction)
                    edge_cost = self.config.primitive_length_m
                    if direction < 0:
                        edge_cost *= self.config.reverse_multiplier
                    if abs(curvature) > 1.0e-9:
                        edge_cost *= self.config.non_straight_multiplier
                    if node.direction and node.direction != direction:
                        edge_cost += self.config.direction_change_cost
                    if node.direction and abs(node.curvature - curvature) > 1.0e-6:
                        edge_cost += self.config.steering_change_cost
                    candidate_cost = node.cost + edge_cost
                    if candidate_cost >= best_cost.get(next_key, math.inf) - 1.0e-9:
                        continue
                    best_cost[next_key] = candidate_cost
                    nodes[next_key] = _Node(
                        endpoint,
                        direction,
                        curvature,
                        candidate_cost,
                        key,
                    )
                    counter += 1
                    priority = candidate_cost + self._heuristic(
                        endpoint, goal, grid_distances
                    )
                    heapq.heappush(open_queue, (priority, counter, next_key))

        if reached_key is None:
            reason = "maximum_expansions" if expansions >= self.config.maximum_expansions else "no_path"
            return HybridPlan(False, (), math.inf, expansions, reason)

        path = []
        key = reached_key
        while key is not None:
            node = nodes[key]
            path.append(HybridPathPoint(node.pose, node.direction, node.curvature))
            key = node.parent_key
        path.reverse()
        return HybridPlan(
            True,
            tuple(path),
            nodes[reached_key].cost,
            expansions,
            "ok",
        )
