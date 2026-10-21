"""
Occupancy map of a phantom (the silicone block): where the needle has hit bone,
so no later insertion drives it into a known bone region again. Plan:
docs/09_loadcell_force_feedback_plan.md §4.4.

A map is one JSON file per phantom setup, recordings/maps/<name>.json, in the
robot_base frame -- so it's only valid while the block stays where it was.
Clear it (start a new one) whenever the block moves.

    {"version": 1, "name": "block_A", "arm": "right", "created": "...",
     "hit_radius_mm": 3.0,
     "entry_point_m": [x, y, z] | null,        # where the needle first touched the surface
     "hits": [{"xyz_m": [...], "attempt": 1, "dir_deg": [0, 0], "force": 1.2e6,
               "session": "20261012_153000", "time": "..."}],
     "lines": [{"from_m": [...], "to_m": [...], "dir_deg": [...], "outcome": "depth"}]}

Each hit is the needle-tip position when bone was decided, treated as a sphere
of hit_radius_mm. `lines` records every attempt for the map view; only hits
block anything.

Pure apart from load/save; no kinematics.
"""
from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np

_VERSION = 1


def safe_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip()).strip("._") or "map"


def segment_point_distance(p0, p1, x) -> float:
    """Distance (same units as the inputs) from point x to the segment p0-p1."""
    p0, p1, x = (np.asarray(v, dtype=float) for v in (p0, p1, x))
    d = p1 - p0
    dd = float(np.dot(d, d))
    u = 0.0 if dd == 0.0 else min(max(float(np.dot(x - p0, d)) / dd, 0.0), 1.0)
    return float(np.linalg.norm(p0 + u * d - x))


def line_sphere_entry(p0, direction, length, center, radius) -> float | None:
    """Distance along the ray p0 + s*direction (unit), 0 <= s <= length, at
    which it first is inside the sphere -- 0 if it starts inside -- or None."""
    p0, d, c = (np.asarray(v, dtype=float) for v in (p0, direction, center))
    m = p0 - c
    if float(np.dot(m, m)) <= radius * radius:
        return 0.0
    b = float(np.dot(m, d))
    disc = b * b - (float(np.dot(m, m)) - radius * radius)
    if disc < 0.0:
        return None
    s = -b - math.sqrt(disc)
    return s if 0.0 <= s <= length else None


@dataclass
class OccupancyMap:
    name: str
    path: str
    arm: str = ""
    hit_radius_mm: float = 3.0
    created: str = ""
    entry_point: np.ndarray | None = None
    hits: list[dict] = field(default_factory=list)
    lines: list[dict] = field(default_factory=list)

    # ── files ──────────────────────────────────────────────────────────────

    @staticmethod
    def path_for(directory: str, name: str) -> str:
        return os.path.join(directory, f"{safe_name(name)}.json")

    @classmethod
    def new(cls, directory: str, name: str, arm: str = "", hit_radius_mm: float = 3.0) -> "OccupancyMap":
        return cls(name=safe_name(name), path=cls.path_for(directory, name), arm=arm,
                   hit_radius_mm=float(hit_radius_mm),
                   created=datetime.now().isoformat(timespec="seconds"))

    @classmethod
    def load(cls, path: str) -> "OccupancyMap":
        with open(path) as f:
            d = json.load(f)
        if d.get("version") != _VERSION:
            raise ValueError(f"{path}: unsupported map version {d.get('version')!r}")
        ep = d.get("entry_point_m")
        return cls(name=d.get("name", os.path.splitext(os.path.basename(path))[0]), path=path,
                   arm=d.get("arm", ""), hit_radius_mm=float(d.get("hit_radius_mm", 3.0)),
                   created=d.get("created", ""),
                   entry_point=None if ep is None else np.array(ep, dtype=float),
                   hits=list(d.get("hits", [])), lines=list(d.get("lines", [])))

    @classmethod
    def open(cls, directory: str, name: str, arm: str = "", hit_radius_mm: float = 3.0,
             fresh: bool = False) -> "OccupancyMap":
        """The map called `name`, or a new empty one if it doesn't exist (or
        fresh=True, which archives an existing file as <name>.<stamp>.json)."""
        path = cls.path_for(directory, name)
        if os.path.exists(path):
            if not fresh:
                return cls.load(path)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            os.replace(path, path[:-len(".json")] + f".{stamp}.json")
        return cls.new(directory, name, arm, hit_radius_mm)

    def to_dict(self) -> dict:
        return {
            "version": _VERSION, "name": self.name, "arm": self.arm, "created": self.created,
            "hit_radius_mm": self.hit_radius_mm,
            "entry_point_m": None if self.entry_point is None else [round(float(v), 6) for v in self.entry_point],
            "hits": self.hits, "lines": self.lines,
        }

    def save(self, path: str | None = None) -> str:
        path = path or self.path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.to_dict(), f, indent=1)
        os.replace(tmp, path)
        return path

    # ── contents ───────────────────────────────────────────────────────────

    def hit_points(self) -> np.ndarray:
        """(N, 3) hit positions, metres."""
        return np.array([h["xyz_m"] for h in self.hits], dtype=float).reshape(-1, 3)

    def add_hit(self, xyz, **info) -> dict:
        hit = {"xyz_m": [round(float(v), 6) for v in xyz],
               "time": datetime.now().isoformat(timespec="seconds"), **info}
        self.hits.append(hit)
        return hit

    def add_line(self, p0, p1, **info) -> None:
        self.lines.append({"from_m": [round(float(v), 6) for v in p0],
                           "to_m": [round(float(v), 6) for v in p1], **info})

    def set_entry_point(self, xyz) -> None:
        self.entry_point = np.asarray(xyz, dtype=float)

    # ── queries ────────────────────────────────────────────────────────────

    def clearance_mm(self, p0, p1) -> float:
        """Smallest distance (mm) from any hit to the segment p0-p1 (metres);
        inf with no hits."""
        if not self.hits:
            return math.inf
        return min(segment_point_distance(p0, p1, h) for h in self.hit_points()) * 1000

    def first_entry_mm(self, p0, direction, length_m: float, radius_mm: float | None = None) -> float | None:
        """Distance along the line (mm) from p0 at which it first enters a hit
        sphere of radius_mm (default hit_radius_mm), or None if it never does
        within length_m."""
        r = (self.hit_radius_mm if radius_mm is None else radius_mm) / 1000
        best = None
        for h in self.hit_points():
            s = line_sphere_entry(p0, direction, length_m, h, r)
            if s is not None and (best is None or s < best):
                best = s
        return None if best is None else best * 1000
