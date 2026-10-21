"""Occupancy map geometry, map files and the direction ranking."""
import math
import os

import numpy as np
import pytest

import direction_search as ds
from occupancy_map import OccupancyMap, line_sphere_entry, segment_point_distance

E = np.array([0.20, -0.095, 0.25])


def test_segment_point_distance():
    assert segment_point_distance([0, 0, 0], [0, 0, -1], [0.3, 0, -0.5]) == pytest.approx(0.3)
    assert segment_point_distance([0, 0, 0], [0, 0, -1], [0, 0, 1]) == pytest.approx(1.0)
    assert segment_point_distance([0, 0, 0], [0, 0, 0], [0, 3, 4]) == pytest.approx(5.0)


def test_line_sphere_entry():
    down = np.array([0, 0, -1.0])
    assert line_sphere_entry([0, 0, 0], down, 1.0, [0, 0, -0.5], 0.1) == pytest.approx(0.4)
    assert line_sphere_entry([0, 0, 0], down, 0.3, [0, 0, -0.5], 0.1) is None
    assert line_sphere_entry([0, 0, 0], down, 1.0, [0.2, 0, -0.5], 0.1) is None
    assert line_sphere_entry([0, 0, -0.5], down, 1.0, [0, 0, -0.5], 0.1) == 0.0


def test_map_roundtrip_and_fresh(tmp_path):
    m = OccupancyMap.open(str(tmp_path), "block A", "right", 3.0)
    assert m.name == "block_A" and not m.hits
    m.set_entry_point(E)
    m.add_hit(E + [0, 0, -0.021], attempt=1)
    m.add_line(E, E + [0, 0, -0.02], outcome="bone")
    m.save()
    again = OccupancyMap.open(str(tmp_path), "block A")
    assert len(again.hits) == 1 and np.allclose(again.entry_point, E)
    assert again.first_entry_mm(E, [0, 0, -1], 0.05) == pytest.approx(18.0, abs=1e-6)
    fresh = OccupancyMap.open(str(tmp_path), "block A", fresh=True)
    assert not fresh.hits
    assert any(f.startswith("block_A.") and f != "block_A.json" for f in os.listdir(tmp_path))


def test_rank_empty_map_prefers_straight_down():
    m = OccupancyMap.new("/tmp", "x")
    usable, blocked = ds.rank(E, m, tilt_steps_deg=[5, 10], rot_step_deg=90, max_tilt_deg=20,
                              h_safe_mm=15, insert_mm=30, line_margin_mm=1)
    assert not blocked and usable[0].key == (0.0, 0.0)
    assert len(usable) == 1 + 2 * 4


def test_rank_hit_below_entry_needs_enough_tilt():
    # The plan's example: a hit 18 mm straight below E with a 3 + 1 mm margin
    # needs tan(tilt) >= 4/18 -> tilt >= 12.5 deg, so 5 and 10 are blocked.
    m = OccupancyMap.new("/tmp", "x", hit_radius_mm=3.0)
    m.add_hit(E + [0, 0, -0.018])
    usable, blocked = ds.rank(E, m, tilt_steps_deg=[5, 10, 15, 20], rot_step_deg=45, max_tilt_deg=20,
                              h_safe_mm=15, insert_mm=30, line_margin_mm=1, exclude={(0.0, 0.0)})
    assert {c.tilt_deg for c in blocked} == {5.0, 10.0}
    assert usable[0].tilt_deg == 15.0
    assert usable[0].clearance_mm == pytest.approx(18 * math.sin(math.radians(15)), abs=0.05)


def test_rank_moves_away_from_offset_hits():
    m = OccupancyMap.new("/tmp", "x", hit_radius_mm=3.0)
    m.add_hit(E + [0, 0, -0.020])
    m.add_hit(E + [0.005, 0, -0.020])          # second hit off to +x
    usable, _ = ds.rank(E, m, tilt_steps_deg=[15], rot_step_deg=45, max_tilt_deg=20,
                        h_safe_mm=15, insert_mm=30, line_margin_mm=1, exclude={(0.0, 0.0)})
    assert usable[0].rot_deg == 180.0          # straight away from the +x hit


def test_line_through_geometry():
    s, t, d = ds.line_through(E, 15, 90, h_safe_mm=15, insert_mm=30)
    assert (s - E)[2] == pytest.approx(0.015)           # 15 mm above the surface, vertically
    assert np.linalg.norm(t - E) == pytest.approx(0.030)
    assert d[1] > 0 and d[2] < 0
