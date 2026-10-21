"""
ForceGuard -- what the needle does next, decided every control tick from the
load cell, for one insertion along one straight line (plunger point 4, or one
attempt of the bone-avoidance search). Plan: docs/09_loadcell_force_feedback_plan.md §4.1.

Pure logic: no I/O, no kinematics, no clock of its own. The caller (the
insertion executor in modes/insertion.py, or an offline replay) feeds it, per
tick, the time, the latest load-cell reading and where the needle is along the
line -- commanded and measured, in mm from the line's start -- and gets back the
path *rate*: how fast the path's own time should advance, as a multiple of the
speed the line was planned at. 1 = planned speed, 0 = hold, negative = run the
same planned knots backwards (retract along exactly the same line).

States, in the order an insertion normally passes through them:

  approach    free air at the approach speed, waiting for contact
  insert      in the material at the insertion speed, slowed as the force
              rises towards f_soft; watching for bone and the occupancy map
  release     the arm is a soft spring (kp 20-40 Nm/rad): its command runs
              ahead of the measured needle, and that lag is what pushes the
              needle. Before backing out, the command is ramped back to where
              the needle measurably is (plus the free-air sag), which takes the
              stored push off the needle instead of leaving it there.
  retract     back along the line to its start (above the surface)
  out_check   at the start: the needle counts as out once |force| stays under
              f_out for out_hold_s
  hold        finished, holding where it is

Outcomes (`outcome`): "depth" (planned depth reached), "bone", "map_stop"
(about to enter a known bone region from the occupancy map), "abort" (a safety
trip -- see `abort_reason`). `out_confirmed` says whether a retract ended with
the needle confirmed out of the material.

Modes:
  on   everything above.
  log  data collection: the needle keeps the planned speed whatever the force
       does; contact / bone / map events are only recorded (as would_*), but
       the safety trips still abort, release and retract.

Force values are in the guard's units -- load-cell counts (tared, signed so
that pushing the needle in is positive) until the cell is calibrated, newtons
after; GuardConfig.units says which, and the caller passes the matching value.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

APPROACH, INSERT, RELEASE, RETRACT, OUT_CHECK, HOLD = (
    "approach", "insert", "release", "retract", "out_check", "hold")
STATES = (APPROACH, INSERT, RELEASE, RETRACT, OUT_CHECK, HOLD)

# A depth this close to the end (or the start) of the line counts as there, mm.
_EPS_MM = 1e-3


@dataclass
class ForceSample:
    """The load-cell reading the guard sees on one tick."""
    value: float          # tared force in the guard's units
    age_s: float          # how old the newest sample is
    seq: int              # changes with every new sample
    dropped: int = 0      # samples lost since the load cell was opened
    saturated: bool = False   # the ADC is at its limit: the real force is unknown (and large)


@dataclass
class GuardConfig:
    units: str = "counts"
    # contact: force above f_contact on contact_samples consecutive samples
    f_contact: float = 100_000.0
    contact_samples: int = 3
    # The surface is placed back where the force was still below this fraction
    # of f_contact: contact fires only once the force passes f_contact, which on
    # 40k counts/mm silicone is already ~2.5 mm in.
    contact_back_frac: float = 0.25
    # insert slows linearly from f_contact to f_soft, down to s_min of its speed
    f_soft: float | None = 2_500_000.0
    s_min: float = 0.2
    # bone: force above f_bone_min AND (stiffness above k_bone OR a stall),
    # continuously for t_persist_s. None switches a criterion off.
    f_bone_min: float | None = 1_000_000.0
    k_bone: float | None = 1_000_000.0      # force units per mm of measured depth
    stiff_window_mm: float = 0.5            # stiffness measured over this much travel
    stall_ratio: float | None = 0.25        # measured advance < this x commanded = stall
    stall_window_s: float = 0.3
    t_persist_s: float = 0.12
    # safety trips (any state that moves the needle in)
    f_abort: float = 3_000_000.0
    f_pull_abort: float = 1_000_000.0
    lag_max_mm: float = 20.0
    stale_s: float = 0.1
    max_dropped: int = 5
    # release / retract / out check
    release_s: float = 0.2
    # "out of the material": |force| under f_out (None = f_contact) for
    # out_hold_s. Looser than f_contact because the zero can shift after a
    # hard insertion (+95k counts after overload presses on 2026-10-12), while
    # withdrawing through silicone reads -1.6 M .. -2.3 M.
    f_out: float | None = None
    out_hold_s: float = 0.3
    out_timeout_s: float = 5.0
    # free-air lag (gravity sag) baseline: median over this long before contact
    lag_baseline_s: float = 0.3

    @classmethod
    def from_dict(cls, d: dict | None) -> "GuardConfig":
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in (d or {}).items() if k in known})


@dataclass
class Event:
    t: float
    kind: str               # contact, bone, map_stop, abort, depth, out, not_out, would_* ...
    cmd_depth_mm: float
    meas_depth_mm: float
    force: float
    detail: str = ""

    def __str__(self) -> str:
        extra = f" ({self.detail})" if self.detail else ""
        return (f"{self.kind} at t={self.t:.3f}s, depth cmd {self.cmd_depth_mm:.2f} / meas "
                f"{self.meas_depth_mm:.2f} mm, force {self.force:.6g}{extra}")


@dataclass
class _Point:
    t: float
    cmd: float
    meas: float
    force: float


class ForceGuard:
    def __init__(self, cfg: GuardConfig, *, length_mm: float, plan_speed_mm_s: float,
                 approach_speed_mm_s: float | None = None, insert_speed_mm_s: float | None = None,
                 retract_speed_mm_s: float | None = None, mode: str = "on",
                 after_bone: str = "retract", after_depth: str = "hold",
                 map_stop_at_mm: float | None = None):
        """length_mm: the line's length. plan_speed_mm_s: the speed its path
        time was planned at (rate 1). after_bone/after_depth: "retract" (back
        out of the material and confirm it) or "hold". map_stop_at_mm: stop
        before the commanded needle passes this depth along the line (the
        occupancy map's first known bone region, less its stop margin)."""
        if mode not in ("on", "log"):
            raise ValueError(f"mode must be 'on' or 'log', got {mode!r}")
        self.cfg = cfg
        self.mode = mode
        self.length_mm = float(length_mm)
        self.plan_speed = float(plan_speed_mm_s)
        self.v_approach = float(approach_speed_mm_s or plan_speed_mm_s)
        self.v_insert = float(insert_speed_mm_s or plan_speed_mm_s)
        self.v_retract = float(retract_speed_mm_s or plan_speed_mm_s)
        self.after_bone = after_bone
        self.after_depth = after_depth
        self.map_stop_at_mm = map_stop_at_mm

        self.state = APPROACH
        self.rate = self.v_approach / self.plan_speed if mode == "on" else 1.0
        self.outcome: str | None = None
        self.abort_reason = ""
        self.out_confirmed = False
        self.finished = False
        self.events: list[Event] = []
        self.contact_depth_mm: float | None = None   # measured, at contact onset
        self.hit_depth_mm: float | None = None       # measured, when bone was decided
        self.lag0_mm = 0.0
        self.max_force = -math.inf
        self.force = math.nan

        self._last_seq: int | None = None
        self._dropped0: int | None = None
        self._contact_run = 0
        self._contact_onset: _Point | None = None
        self._lag_hist: deque[tuple[float, float]] = deque()
        self._approach_hist: deque[_Point] = deque(maxlen=400)
        self._hist: deque[_Point] = deque()
        self._bone_since: float | None = None
        self._would_bone = False
        self._would_map = False
        self._release: tuple[float, float, float] | None = None   # (t0, from cmd, to)
        self._then_retract = True
        self._quiet_since: float | None = None
        self._out_t0: float | None = None

    # ── public ─────────────────────────────────────────────────────────────

    def update(self, t: float, sample: ForceSample | None, cmd_depth_mm: float,
               meas_depth_mm: float) -> float:
        """One tick. Returns the path rate to use until the next tick."""
        if self.finished:
            return 0.0
        new = sample is not None and sample.seq != self._last_seq
        if sample is not None:
            if self._dropped0 is None:
                self._dropped0 = sample.dropped
            self._last_seq = sample.seq
            self.force = sample.value
            if new:
                self.max_force = max(self.max_force, sample.value)
        f = self.force
        p = _Point(t, cmd_depth_mm, meas_depth_mm, f)

        if self.state in (APPROACH, INSERT):
            trip = self._safety(sample, p)
            if trip:
                self._abort(p, trip)
            elif self.state == APPROACH:
                self._approach(p, new)
            else:
                self._insert(p, new)
        elif self.state == RELEASE:
            self._do_release(p)
        elif self.state == RETRACT:
            self._retract(p)
        elif self.state == OUT_CHECK:
            self._out_check(p, sample)
        return 0.0 if self.finished else self.rate

    def status(self) -> dict:
        return {"state": self.state, "rate": self.rate, "force": self.force,
                "outcome": self.outcome, "units": self.cfg.units}

    # ── states ─────────────────────────────────────────────────────────────

    def _safety(self, s: ForceSample | None, p: _Point) -> str:
        c = self.cfg
        if s is None:
            return "no load-cell reading"
        if s.age_s > c.stale_s:
            return f"load cell stale ({s.age_s * 1000:.0f} ms since the last sample)"
        if s.saturated:
            return "load cell saturated (HX711 at the end of its range) -- the force is beyond what it can read"
        if self._dropped0 is not None and s.dropped - self._dropped0 > c.max_dropped:
            return f"load cell dropped {s.dropped - self._dropped0} samples"
        if p.force > c.f_abort:
            return f"force {p.force:.6g} above f_abort {c.f_abort:g}"
        if p.force < -c.f_pull_abort:
            return f"force {p.force:.6g} below -f_pull_abort {-c.f_pull_abort:g} (needle pulled)"
        lag = (p.cmd - p.meas) - self.lag0_mm
        if self.state == INSERT and lag > c.lag_max_mm:
            return f"arm {lag:.1f} mm behind its command (lag_max {c.lag_max_mm:g})"
        return ""

    def _approach(self, p: _Point, new: bool) -> None:
        c = self.cfg
        self._lag_hist.append((p.t, p.cmd - p.meas))
        while self._lag_hist and p.t - self._lag_hist[0][0] > c.lag_baseline_s:
            self._lag_hist.popleft()
        if new:
            if p.force > c.f_contact:
                if self._contact_run == 0:
                    self._contact_onset = p
                self._contact_run += 1
            else:
                self._contact_run, self._contact_onset = 0, None
                self._approach_hist.append(p)
        if self._contact_run >= c.contact_samples:
            onset = self._contact_onset or p
            # Where the force started rising: the last sample still under
            # contact_back_frac x f_contact before it crossed.
            surface = onset
            for q in reversed(self._approach_hist):
                if q.force <= c.contact_back_frac * c.f_contact:
                    surface = q
                    break
            self.contact_depth_mm = surface.meas
            lags = sorted(lag for _, lag in self._lag_hist)
            self.lag0_mm = lags[len(lags) // 2] if lags else 0.0
            self._event(onset, "contact", f"surface at {surface.meas:.2f} mm, lag0 {self.lag0_mm:.2f} mm")
            self.state = INSERT
            self._hist.clear()
            self._insert(p, new=False)
            return
        if self._at_end(p):
            self._reached_depth(p)
            return
        if self._map_check(p):
            return
        self.rate = self.v_approach / self.plan_speed if self.mode == "on" else 1.0

    def _insert(self, p: _Point, new: bool) -> None:
        c = self.cfg
        if new:
            self._hist.append(p)
            keep = max(c.stall_window_s, c.t_persist_s) * 4 + 1.0
            while self._hist and p.t - self._hist[0].t > keep:
                self._hist.popleft()
            if self._bone_check(p):
                return
        if self._at_end(p):
            self._reached_depth(p)
            return
        if self._map_check(p):
            return
        if self.mode == "log":
            self.rate = 1.0
            return
        scale = 1.0
        if c.f_soft is not None and c.f_soft > c.f_contact:
            scale = (c.f_soft - p.force) / (c.f_soft - c.f_contact)
            scale = min(max(scale, c.s_min), 1.0)
        self.rate = self.v_insert / self.plan_speed * scale

    def _bone_check(self, p: _Point) -> bool:
        """True if bone was decided (state changed)."""
        c = self.cfg
        if c.f_bone_min is None or p.force <= c.f_bone_min:
            self._bone_since, self._would_bone = None, False
            return False
        stiff = self._stiffness(p)
        stall = self._stalled(p)
        hit = ((c.k_bone is not None and stiff is not None and stiff > c.k_bone)
               or (c.stall_ratio is not None and stall))
        if not hit:
            self._bone_since, self._would_bone = None, False
            return False
        if self._bone_since is None:
            self._bone_since = p.t
        if p.t - self._bone_since < c.t_persist_s:
            return False
        why = (f"stiffness {stiff:.6g}/mm" if stiff is not None else "stiffness n/a") + \
              (", stalled" if stall else "")
        if self.mode == "log":
            if not self._would_bone:
                self._event(p, "would_bone", why)
                self._would_bone = True
            return False
        self.hit_depth_mm = p.meas
        self.outcome = "bone"
        self._event(p, "bone", why)
        self._begin_release(p, then_retract=self.after_bone == "retract")
        return True

    def _stiffness(self, p: _Point) -> float | None:
        """Force change per mm of measured travel over the last stiff_window_mm."""
        w = self.cfg.stiff_window_mm
        for old in reversed(self._hist):
            if p.meas - old.meas >= w:
                return (p.force - old.force) / (p.meas - old.meas)
        return None

    def _stalled(self, p: _Point) -> bool:
        """The needle stopped advancing although its command didn't, while the
        force rose: what pushing against something hard looks like on a
        compliant arm."""
        c = self.cfg
        if c.stall_ratio is None:
            return False
        old = None
        for q in self._hist:
            if p.t - q.t <= c.stall_window_s:
                old = q
                break
        if old is None or p.t - old.t < 0.5 * c.stall_window_s:
            return False
        cmd_adv, meas_adv = p.cmd - old.cmd, p.meas - old.meas
        return cmd_adv > 0.05 and meas_adv < c.stall_ratio * cmd_adv and p.force > old.force

    def _map_check(self, p: _Point) -> bool:
        if self.map_stop_at_mm is None or p.cmd < self.map_stop_at_mm:
            self._would_map = False
            return False
        if self.mode == "log":
            if not self._would_map:
                self._event(p, "would_map_stop", f"known bone region ahead (stop at "
                                                 f"{self.map_stop_at_mm:.2f} mm)")
                self._would_map = True
            return False
        self.outcome = "map_stop"
        self._event(p, "map_stop", f"known bone region ahead (stop at {self.map_stop_at_mm:.2f} mm)")
        self._begin_release(p, then_retract=self.after_bone == "retract")
        return True

    def _reached_depth(self, p: _Point) -> None:
        self.outcome = "depth"
        self._event(p, "depth", "" if self.contact_depth_mm is not None else "no contact")
        if self.after_depth == "retract":
            self._begin_release(p, then_retract=True)
        else:
            self._hold()

    def _abort(self, p: _Point, reason: str) -> None:
        self.outcome, self.abort_reason = "abort", reason
        self._event(p, "abort", reason)
        self._begin_release(p, then_retract=True)

    def _begin_release(self, p: _Point, then_retract: bool) -> None:
        """Ramp the command back to where the needle measurably is (plus the
        free-air sag lag0), so the arm's stored push comes off the needle."""
        target = min(max(p.meas + self.lag0_mm, 0.0), p.cmd)
        self._release = (p.t, p.cmd, target)
        self._then_retract = then_retract
        self.state = RELEASE
        self._do_release(p)

    def _do_release(self, p: _Point) -> None:
        assert self._release is not None
        t0, _, target = self._release
        c = self.cfg
        if p.t - t0 >= c.release_s or p.cmd <= target + _EPS_MM:
            if self._then_retract:
                self.state = RETRACT
                self._retract(p)
            else:
                self._hold()
            return
        remaining = max(c.release_s - (p.t - t0), 1e-3)
        self.rate = (target - p.cmd) / (remaining * self.plan_speed)

    def _retract(self, p: _Point) -> None:
        if p.cmd <= _EPS_MM:
            self.state = OUT_CHECK
            self._out_t0 = p.t
            self._quiet_since = None
            self.rate = 0.0
            return
        self.rate = -self.v_retract / self.plan_speed

    def _out_check(self, p: _Point, s: ForceSample | None) -> None:
        c = self.cfg
        self.rate = 0.0
        healthy = s is not None and s.age_s <= c.stale_s
        f_out = c.f_out if c.f_out is not None else c.f_contact
        if healthy and abs(p.force) < f_out:
            if self._quiet_since is None:
                self._quiet_since = p.t
            if p.t - self._quiet_since >= c.out_hold_s:
                self.out_confirmed = True
                self._event(p, "out")
                self._hold()
                return
        else:
            self._quiet_since = None
        if p.t - (self._out_t0 if self._out_t0 is not None else p.t) > c.out_timeout_s:
            self._event(p, "not_out", "load cell stale" if not healthy else
                        f"|force| {abs(p.force):.6g} still above f_out")
            self._hold()

    def _hold(self) -> None:
        self.state = HOLD
        self.rate = 0.0
        self.finished = True

    # ── helpers ────────────────────────────────────────────────────────────

    def _at_end(self, p: _Point) -> bool:
        return p.cmd >= self.length_mm - _EPS_MM

    def _event(self, p: _Point, kind: str, detail: str = "") -> None:
        self.events.append(Event(p.t, kind, p.cmd, p.meas, p.force, detail))
