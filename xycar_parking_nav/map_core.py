"""Occupancy-map parsing and full-footprint parking pose validation."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

from PIL import Image, UnidentifiedImageError
import yaml

from .command_core import Footprint
from .mission_core import Pose2D


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


@dataclass(frozen=True)
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: tuple[int, ...]

    def __post_init__(self) -> None:
        if len(self.cells) != self.width * self.height:
            raise ValueError("occupancy cell count does not match dimensions")
        if self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        delta_x = float(x) - self.origin_x
        delta_y = float(y) - self.origin_y
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        grid_x = math.floor(local_x / self.resolution)
        grid_y = math.floor(local_y / self.resolution)
        if not (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return None
        return int(grid_x), int(grid_y)

    def cell(self, grid_x: int, grid_y: int) -> int:
        return self.cells[grid_y * self.width + grid_x]

    def world_cell(self, x: float, y: float) -> int | None:
        grid = self.world_to_grid(x, y)
        return None if grid is None else self.cell(*grid)

    def counts(self) -> Counter:
        return Counter(self.cells)


@dataclass(frozen=True)
class PoseMapAssessment:
    collision_free: bool
    occupied_samples: int
    unknown_samples: int
    outside_samples: int
    nearest_occupied_distance_m: float


def pixel_occupancy(
    pixel_value: int,
    *,
    negate: bool,
    free_threshold: float,
    occupied_threshold: float,
) -> int:
    shade = float(pixel_value) / 255.0
    probability = shade if negate else 1.0 - shade
    if occupied_threshold < probability:
        return OCCUPIED
    if probability < free_threshold:
        return FREE
    return UNKNOWN


def load_occupancy_map(yaml_path: str | Path) -> OccupancyMap:
    yaml_path = Path(yaml_path).resolve()
    with yaml_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = Path(str(metadata["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    try:
        image = Image.open(image_path).convert("L")
        width, height = image.size
        pixels = image.load()
        pixel_at = lambda x, y: pixels[x, y]
    except UnidentifiedImageError:
        # Pillow builds used in some minimal ROS images only enable binary PGM.
        # Keep the public synthetic map human-readable by accepting the P2 form.
        tokens = []
        for line in image_path.read_text(encoding="ascii").splitlines():
            tokens.extend(line.split("#", 1)[0].split())
        if len(tokens) < 4 or tokens[0] != "P2":
            raise
        width, height, max_value = map(int, tokens[1:4])
        if max_value <= 0:
            raise ValueError("PGM max value must be positive")
        values = [int(value) for value in tokens[4:]]
        if len(values) != width * height:
            raise ValueError("ASCII PGM pixel count does not match dimensions")
        pixel_at = lambda x, y: values[y * width + x]
    negate = bool(int(metadata.get("negate", 0)))
    free_threshold = float(metadata["free_thresh"])
    occupied_threshold = float(metadata["occupied_thresh"])
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise ValueError("map thresholds must satisfy 0 <= free < occupied <= 1")
    if str(metadata.get("mode", "trinary")).lower() != "trinary":
        raise ValueError("parking validator currently requires trinary map mode")

    # PGM rows are top-down; OccupancyGrid rows are bottom-up.
    cells = []
    for grid_y in range(height):
        image_y = height - 1 - grid_y
        for grid_x in range(width):
            cells.append(
                pixel_occupancy(
                    pixel_at(grid_x, image_y),
                    negate=negate,
                    free_threshold=free_threshold,
                    occupied_threshold=occupied_threshold,
                )
            )
    origin = metadata["origin"]
    return OccupancyMap(
        width=width,
        height=height,
        resolution=float(metadata["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        cells=tuple(cells),
    )


def assess_pose_footprint(
    occupancy_map: OccupancyMap,
    pose: Pose2D,
    footprint: Footprint,
    *,
    margin_m: float = 0.0,
    sample_step_m: float | None = None,
) -> PoseMapAssessment:
    step = sample_step_m or occupancy_map.resolution * 0.5
    step = max(0.005, float(step))
    minimum_x = footprint.minimum_x - margin_m
    maximum_x = footprint.maximum_x + margin_m
    half_width = footprint.half_width + margin_m
    x_samples = max(1, math.ceil((maximum_x - minimum_x) / step))
    y_samples = max(1, math.ceil((2.0 * half_width) / step))
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    occupied_cells: set[tuple[int, int]] = set()
    unknown_cells: set[tuple[int, int]] = set()
    outside = 0
    for x_index in range(x_samples + 1):
        local_x = minimum_x + (maximum_x - minimum_x) * x_index / x_samples
        for y_index in range(y_samples + 1):
            local_y = -half_width + 2.0 * half_width * y_index / y_samples
            world_x = pose.x + cosine * local_x - sine * local_y
            world_y = pose.y + sine * local_x + cosine * local_y
            grid = occupancy_map.world_to_grid(world_x, world_y)
            if grid is None:
                outside += 1
                continue
            value = occupancy_map.cell(*grid)
            if value == OCCUPIED:
                occupied_cells.add(grid)
            elif value == UNKNOWN:
                unknown_cells.add(grid)

    occupied_centers = []
    for grid_y in range(occupancy_map.height):
        for grid_x in range(occupancy_map.width):
            if occupancy_map.cell(grid_x, grid_y) == OCCUPIED:
                occupied_centers.append(
                    (
                        occupancy_map.origin_x + (grid_x + 0.5) * occupancy_map.resolution,
                        occupancy_map.origin_y + (grid_y + 0.5) * occupancy_map.resolution,
                    )
                )
    nearest = min(
        (math.hypot(pose.x - x, pose.y - y) for x, y in occupied_centers),
        default=math.inf,
    )
    return PoseMapAssessment(
        collision_free=not occupied_cells and not unknown_cells and outside == 0,
        occupied_samples=len(occupied_cells),
        unknown_samples=len(unknown_cells),
        outside_samples=outside,
        nearest_occupied_distance_m=nearest,
    )
