"""
Choosing the next insertion direction after a bone hit -- every line passes
through the same entry point E on the surface, tilted a little from straight
down. Plan: docs/09_loadcell_force_feedback_plan.md §4.3-4.4.

Candidates are (tilt, rot) on rings of growing tilt (tilt_steps_deg, every
rot_step_deg about the vertical), plus straight down. A candidate is blocked
if its needle segment passes within hit_radius_mm + line_margin_mm of any hit
in the occupancy map. The unblocked ones are ranked smallest tilt first, then
largest clearance to the nearest hit (which also points them away from the
hits), then furthest from all the hits together (away from the ones off to
the side), then rot.

Direction convention as line mode's --angles: (sin t cos r, sin t sin r, -cos t),
so (0, 0) is straight down. Pure; no kinematics.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from occupancy_map import OccupancyMap, segment_point_distance


def direction(tilt_deg: float, rot_deg: float) -> np.ndarray:
    t, r = math.radians(tilt_deg), math.radians(rot_deg)
    return np.array([math.sin(t) * math.cos(r), math.sin(t) * math.sin(r), -math.cos(t)])


@dataclass
class Candidate:
    tilt_deg: float
    rot_deg: float
    d: np.ndarray          # unit direction, robot_base
    start: np.ndarray      # S: on the line, h_safe above the surface, m
    end: np.ndarray        # T: insert_mm past E along the line, m
    clearance_mm: float    # to the nearest hit (inf with none)

    @property
    def key(self) -> tuple[float, float]:
        return (round(self.tilt_deg, 3), round(self.rot_deg % 360.0, 3) if self.tilt_deg else 0.0)

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.end - self.start)) * 1000

    def __str__(self) -> str:
        c = "no hits" if math.isinf(self.clearance_mm) else f"clearance {self.clearance_mm:.1f} mm"
        return f"tilt {self.tilt_deg:g} deg, rot {self.rot_deg:g} deg ({c})"


def candidate_angles(tilt_steps_deg, rot_step_deg: float, max_tilt_deg: float) -> list[tuple[float, float]]:
    out = [(0.0, 0.0)]
    n_rot = max(1, int(round(360.0 / rot_step_deg)))
    for tilt in sorted(float(t) for t in tilt_steps_deg):
        if 0.0 < tilt <= max_tilt_deg + 1e-9:
            out += [(tilt, k * 360.0 / n_rot) for k in range(n_rot)]
    return out


def line_through(entry, tilt_deg: float, rot_deg: float, h_safe_mm: float,
                 insert_mm: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(S, T, d): the line through the entry point E along (tilt, rot), from
    h_safe_mm *above the surface* (measured vertically) to insert_mm along the
    needle past E."""
    d = direction(tilt_deg, rot_deg)
    e = np.asarray(entry, dtype=float)
    above = h_safe_mm / 1000 / max(-d[2], 1e-6)   # along-line distance from S to E
    return e - d * above, e + d * insert_mm / 1000, d


def rank(entry, omap: OccupancyMap, *, tilt_steps_deg, rot_step_deg: float, max_tilt_deg: float,
         h_safe_mm: float, insert_mm: float, line_margin_mm: float,
         exclude: set | None = None) -> tuple[list[Candidate], list[Candidate]]:
    """(usable, blocked): usable candidates best first, and the blocked ones
    (for reporting / the map view). `exclude` holds Candidate.key values
    already tried or found unreachable."""
    exclude = exclude or set()
    need = omap.hit_radius_mm + line_margin_mm
    usable, blocked = [], []
    for tilt, rot in candidate_angles(tilt_steps_deg, rot_step_deg, max_tilt_deg):
        s, t, d = line_through(entry, tilt, rot, h_safe_mm, insert_mm)
        c = Candidate(tilt, rot, d, s, t, omap.clearance_mm(s, t))
        if c.key in exclude:
            continue
        (blocked if c.clearance_mm < need else usable).append(c)
    # Ties on clearance (the nearest hit is often the one straight below E,
    # equally far from every direction on a ring) go to the direction furthest
    # from the hits overall -- away from the ones off to the side.
    hits = omap.hit_points()

    def spread(c: Candidate) -> float:
        if not len(hits):
            return 0.0
        return sum(segment_point_distance(c.start, c.end, h) for h in hits) * 1000

    usable.sort(key=lambda c: (c.tilt_deg, -round(min(c.clearance_mm, 1e6), 1), -round(spread(c), 1),
                               c.rot_deg))
    return usable, blocked
