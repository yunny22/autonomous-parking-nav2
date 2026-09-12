from pathlib import Path

from xycar_parking_nav.map_core import FREE, OCCUPIED, load_occupancy_map


PACKAGE = Path(__file__).resolve().parents[1]


def test_synthetic_public_map_loads():
    occupancy_map = load_occupancy_map(PACKAGE / "maps/example_map.yaml")
    counts = occupancy_map.counts()
    assert (occupancy_map.width, occupancy_map.height) == (12, 12)
    assert occupancy_map.resolution == 0.5
    assert counts[FREE] > 0
    assert counts[OCCUPIED] > 0


def test_competition_map_is_not_part_of_public_staging():
    maps = {path.name for path in (PACKAGE / "maps").iterdir()}
    assert "parking_map.pgm" not in maps
    assert "parking_map.yaml" not in maps
    assert "parking_map_original.yaml" not in maps
