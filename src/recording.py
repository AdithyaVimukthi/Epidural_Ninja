"""
Recorded joint trajectories -- the file `record` writes and `replay` reads, and
the checks both sides of it need.

A recording is a CSV with one row per sample: seconds since the first sample,
then each recorded arm's seven joint angles in degrees, as measured. The four
`#` lines at the top carry metadata; outside this module, numpy reads one with
`np.genfromtxt(path, delimiter=",", names=True, skip_header=4)`:

    # openarm-recording v1
    # created: 2026-09-15T10:32:05
    # rate_hz: 50
    # trimmed_s: 2.300 3.100
    t_s,right_joint1_deg,right_joint2_deg,right_joint3_deg,right_joint4_deg,...
    0.0000,-1.2345,12.0000,0.5000,45.1000,...

A two-arm recording has left_joint1_deg..left_joint7_deg columns ahead of the
right arm's. Full reference: docs/07_main_script.md.
"""
import math
import os
from dataclasses import dataclass

import numpy as np

from config import RECORDINGS_DIR

MAGIC = "# openarm-recording v1"
SIDES = ("left", "right")

# Trimming: "still" means every joint within STILL_TOL_DEG of where the take
# starts (or ends); STILL_MARGIN_S of that is kept either side of the motion.
STILL_TOL_DEG = 1.0
STILL_MARGIN_S = 0.2

# Fastest any joint may move in a real replay unless --max-speed says otherwise.
# Hand-guided motion stays under it; an arm dropping after being let go, or a
# --speed typo, doesn't.
REPLAY_MAX_SPEED_DEG_S = 90.0

# Past a limit by less than this is rounding, not worth a warning.
_LIMIT_SLACK_RAD = math.radians(0.01)


class RecordingError(Exception):
    """A recording that can't be used: missing, unreadable, or not this format."""


@dataclass
class Recording:
    t: np.ndarray                   # (N,) seconds since the first sample, strictly increasing
    q: dict[str, np.ndarray]        # side -> (N, 7) joint positions, radians
    rate_hz: float
    created: str
    trimmed_s: tuple[float, float] = (0.0, 0.0)   # stillness cut from the start, end

    @property
    def arms(self) -> list[str]:
        return [side for side in SIDES if side in self.q]

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    def __len__(self) -> int:
        return len(self.t)


def recording_path(name: str) -> str:
    """A bare name lives in RECORDINGS_DIR; a name with a directory in it is a path."""
    path = os.path.expanduser(name)
    if not os.path.dirname(path):
        path = os.path.join(RECORDINGS_DIR, path)
    if not path.endswith(".csv"):
        path += ".csv"
    return os.path.abspath(path)


def list_recordings() -> list[str]:
    if not os.path.isdir(RECORDINGS_DIR):
        return []
    return sorted(os.path.join(RECORDINGS_DIR, f)
                  for f in os.listdir(RECORDINGS_DIR) if f.endswith(".csv"))


def _columns(arms: list[str]) -> list[str]:
    return ["t_s"] + [f"{side}_joint{i}_deg" for side in arms for i in range(1, 8)]


def save(rec: Recording, path: str) -> None:
    """Written to a temp file first, so an interrupted save never leaves half a recording."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    deg = np.hstack([np.degrees(rec.q[side]) for side in rec.arms])
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(f"{MAGIC}\n")
        f.write(f"# created: {rec.created}\n")
        f.write(f"# rate_hz: {rec.rate_hz:g}\n")
        f.write(f"# trimmed_s: {rec.trimmed_s[0]:.3f} {rec.trimmed_s[1]:.3f}\n")
        f.write(",".join(_columns(rec.arms)) + "\n")
        for t, row in zip(rec.t, deg):
            f.write(f"{t:.4f}," + ",".join(f"{v:.4f}" for v in row) + "\n")
    os.replace(tmp, path)


def load(path: str) -> Recording:
    try:
        with open(path) as f:
            lines = f.read().splitlines()
    except FileNotFoundError:
        raise RecordingError(f"no recording at {path}") from None
    except (OSError, UnicodeDecodeError) as e:
        raise RecordingError(f"can't read {path}: {e}") from None
    if not lines or lines[0].strip() != MAGIC:
        raise RecordingError(f"{path} is not a recording -- its first line should be '{MAGIC}'")

    meta, header, rows = {}, None, []
    for lineno, line in enumerate(lines[1:], start=2):
        if not line.strip():
            continue
        if line.startswith("#"):
            key, _, value = line[1:].partition(":")
            meta[key.strip()] = value.strip()
        elif header is None:
            header = [cell.strip() for cell in line.split(",")]
        else:
            cells = line.split(",")
            if len(cells) != len(header):
                raise RecordingError(f"{path} line {lineno}: {len(cells)} values, expected {len(header)}")
            try:
                rows.append([float(cell) for cell in cells])
            except ValueError:
                raise RecordingError(f"{path} line {lineno}: not a number") from None

    arms = [side for side in SIDES if header and f"{side}_joint1_deg" in header]
    if not arms or header != _columns(arms):
        raise RecordingError(f"{path}: expected columns t_s, then <side>_joint1_deg .. "
                             f"<side>_joint7_deg for left and/or right")
    if not rows:
        raise RecordingError(f"{path} has no samples")
    data = np.array(rows)
    if not np.isfinite(data).all():
        raise RecordingError(f"{path} contains nan or inf")
    t = data[:, 0]
    backwards = np.nonzero(np.diff(t) <= 0)[0]
    if len(backwards):
        raise RecordingError(f"{path}: t_s must increase row to row "
                             f"(it doesn't after {t[backwards[0]]:.4f})")

    try:
        rate_hz = float(meta.get("rate_hz", "nan"))
    except ValueError:
        rate_hz = float("nan")
    try:
        start_s, end_s = (float(v) for v in meta.get("trimmed_s", "").split())
    except ValueError:
        start_s, end_s = 0.0, 0.0
    return Recording(
        t=t - t[0],
        q={side: np.radians(data[:, 1 + 7 * k:8 + 7 * k]) for k, side in enumerate(arms)},
        rate_hz=rate_hz,
        created=meta.get("created", "unknown"),
        trimmed_s=(start_s, end_s),
    )


def trim_stillness(rec: Recording, tol_deg: float = STILL_TOL_DEG,
                   margin_s: float = STILL_MARGIN_S) -> Recording | None:
    """Cut the still stretch before the arms first move and after they last
    move, keeping margin_s of each. Returns None if nothing ever moved more
    than tol_deg from the starting pose."""
    if len(rec) < 2:
        return None
    pose = np.hstack([rec.q[side] for side in rec.arms])
    tol = math.radians(tol_deg)
    off_start = np.nonzero(np.max(np.abs(pose - pose[0]), axis=1) > tol)[0]
    if len(off_start) == 0:
        return None
    off_end = np.nonzero(np.max(np.abs(pose - pose[-1]), axis=1) > tol)[0]
    first = int(off_start[0])                                   # >= 1
    last = int(off_end[-1]) if len(off_end) else len(rec) - 2   # <= N-2

    t = rec.t
    # Always keep one still sample either side, so playback starts and ends at rest.
    lo = min(int(np.searchsorted(t, t[first] - margin_s, side="left")), first - 1)
    hi = max(int(np.searchsorted(t, t[last] + margin_s, side="right")) - 1, last + 1, lo + 1)
    return Recording(
        t=t[lo:hi + 1] - t[lo],
        q={side: rec.q[side][lo:hi + 1] for side in rec.arms},
        rate_hz=rec.rate_hz,
        created=rec.created,
        trimmed_s=(float(t[lo] - t[0]), float(t[-1] - t[hi])),
    )


def peak_speed(t: np.ndarray, q: np.ndarray, window_s: float = 0.1) -> tuple[float, int, float]:
    """Fastest any joint moves in an (N, 7) trajectory: (deg/s, joint index, time
    it happens). Measured across window_s rather than sample to sample, so one
    noisy reading doesn't count as a burst of speed."""
    if len(t) < 2:
        return 0.0, 0, 0.0
    i = np.arange(len(t))
    j = np.minimum(np.searchsorted(t, t + window_s, side="left"), len(t) - 1)
    keep = j > i
    i, j = i[keep], j[keep]
    speed = np.degrees(np.abs(q[j] - q[i]) / (t[j] - t[i])[:, None])
    peak = float(speed.max())
    # A steady move ties many windows. Name the earliest within 0.05 deg/s of the
    # peak, so a take straight off the arm and the same take read back from its
    # (rounded) file point at the same moment.
    row, joint = np.argwhere(speed >= peak - 0.05)[0]
    return peak, int(joint), float(t[i[row]])


def speed_factor_within(limit_deg_s: float, peak_deg_s: float) -> float:
    """Largest --speed, rounded down to 0.01, that keeps a peak of peak_deg_s
    (at --speed 1) under limit_deg_s."""
    return math.floor(limit_deg_s / peak_deg_s * 100) / 100


def out_of_range(q: np.ndarray, limits: list[tuple[float, float]]) -> list[tuple[int, float, int]]:
    """For each joint that leaves its limits anywhere in an (N, 7) trajectory:
    (joint index, furthest past the limit in degrees, samples outside)."""
    lo = np.array([low for low, _ in limits])
    hi = np.array([high for _, high in limits])
    excess = np.maximum(lo - q, q - hi)
    report = []
    for joint in range(7):
        outside = excess[:, joint] > _LIMIT_SLACK_RAD
        if outside.any():
            report.append((joint, math.degrees(float(excess[outside, joint].max())), int(outside.sum())))
    return report
