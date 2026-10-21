"""
Guarded needle insertion: plunger point 4 with the load cell in the loop
(--guard log|on), and the bone-avoidance search (--search). Plan:
docs/09_loadcell_force_feedback_plan.md §4.1-4.4, Phases 3-6.

One *attempt* is one straight line, planned like plunger point 4 (IK knots,
tool held on the line) and streamed at the control rate -- but the path's own
time tau is advanced every tick by `dt x rate`, with the rate coming from the
ForceGuard (motion/force_guard.py): slow down as the force builds, stop,
release the arm's stored push, run the same knots backwards to retract.

With --search, after a bone hit (or a stop in front of a known one) and a
confirmed retract out of the material, the next direction is chosen from the
occupancy map (motion/direction_search.py) -- always through the same entry
point E on the surface -- the arm re-aims in a guarded joint-space move at a
safe height, re-tares, and inserts again. It ends at the target depth, after
max_attempts, or when no reachable unblocked direction is left. A safety trip
(force limit, stale load cell, ...) always ends it.

Logs, one row per control tick (v3; read with pandas `read_csv(path,
comment="#")` or load_log() below):

    single run:  recordings/plunger/<YYYYmmdd>_<HHMMSS>_<side>[_<name>].csv
    search:      recordings/search/<YYYYmmdd>_<HHMMSS>_<side>[_<name>]/
                     attempt_01.csv ...   one per attempt
                     summary.csv          one row per attempt
                     map.json             the occupancy map after the run

    t_s,mark,state,rate,cmd_depth_mm,meas_depth_mm,lag_mm,force_raw,force,
    force_age_ms,force_seq,joint1_deg..joint7_deg,joint1_nm..joint7_nm

cmd/meas depth are mm along the attempt's line from its start (point 3 for the
first). `force` is in the guard's units (header `force_units`); torques are
empty in sim.
"""
from __future__ import annotations

import contextlib
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np

import axis_hold
import direction_search as ds
import loadcell as lcmod
from config import AXIS_HOLD_CFG, CTRL_HZ, GUARD_CFG, JOINT_NAMES, MAP_CFG, MAPS_DIR, RECORDINGS_DIR, SEARCH_CFG
from force_guard import Event, ForceGuard, ForceSample, GuardConfig
from kinematics import TOOL_AXIS, ArmKinematics
from mode_line import Section, _plan_oriented, _plan_section, _q_at
from motion import ramp_start, warn_if_start_out_of_range
from occupancy_map import OccupancyMap
from session import Session
from streaming import MIRROR_RATE_HZ, SIM_RATE_HZ, SimMirror, real_sender, sim_sender

DOWN = np.array([0.0, 0.0, -1.0])
PLUNGER_LOG_DIR = os.path.join(RECORDINGS_DIR, "plunger")
SEARCH_DIR = os.path.join(RECORDINGS_DIR, "search")
LOG_MAGIC = "# openarm-plunger v3"
# As plunger point 4: refuse if the arm moved this much while planning.
_MAX_DRIFT_DEG = 2.0
# The path time advances at most this many tick periods per tick (see run_path).
MAX_STEP_PERIODS = 3
# The re-aim move's tip height is checked at this many points.
_TRANSIT_SAMPLES = 40

# ── live state for the GUI (replaced, never mutated, so reads need no lock) ───
LIVE: dict = {}
# side -> (the line's planned Section, path time it ended at) while a needle is
# left in the material (held at depth, or held on bone): point 5 backs it out
# along exactly that line, and point 3 refuses to move until it has.
NEEDLE_IN: dict = {}
# Guard thresholds changed while the program runs (the GUI's threshold boxes):
# GuardConfig field -> value (None = criterion off). Applied on top of
# config.yaml at the start of every run, and to a run already going on its next
# tick -- each change lands in that attempt's events.
GUARD_OVERRIDES: dict = {}
LIVE_KEYS = ("f_contact", "f_soft", "f_bone_min", "f_abort", "f_out")


def set_guard_override(key: str, value: float | None) -> None:
    if key not in GuardConfig.__dataclass_fields__:
        raise KeyError(f"no guard setting {key!r}")
    GUARD_OVERRIDES[key] = None if value is None else float(value)


def guard_value(key: str):
    """The value a run would use now: the live override, else config.yaml."""
    return GUARD_OVERRIDES[key] if key in GUARD_OVERRIDES else GUARD_CFG.get(key)


def _apply_overrides(cfg: GuardConfig) -> list[str]:
    """Copy changed overrides into cfg; returns 'key old -> new' for each change."""
    changed = []
    for k, v in list(GUARD_OVERRIDES.items()):
        if getattr(cfg, k) != v:
            changed.append(f"{k} {getattr(cfg, k)} -> {v}")
            setattr(cfg, k, v)
    return changed
MAP_VIEW: dict = {}
_active_log = None


def active_log():
    return _active_log


def _publish(**kw) -> None:
    global LIVE
    LIVE = {**LIVE, **kw, "t": time.monotonic()}


def _publish_map(omap: OccupancyMap | None, entry, attempts: list) -> None:
    global MAP_VIEW
    MAP_VIEW = {
        "entry": None if entry is None else np.array(entry),
        "hits": np.zeros((0, 3)) if omap is None else omap.hit_points(),
        "hit_radius_mm": None if omap is None else omap.hit_radius_mm,
        "lines": [(a.start.copy(), a.reached_xyz(), a.outcome or "running") for a in attempts],
        "t": time.monotonic(),
    }


# ── settings: config.yaml merged with the command's flags ────────────────────

@dataclass
class Settings:
    guard: GuardConfig
    mode: str                      # "on" / "log"
    plan_speed_mm_s: float
    approach_speed_mm_s: float
    insert_speed_mm_s: float
    retract_speed_mm_s: float
    after_bone: str
    after_depth: str
    search: bool
    tilt_steps: list
    rot_step: float
    max_tilt: float
    max_attempts: int
    h_safe_mm: float
    h_safe_max_mm: float
    h_min_mm: float
    entry_tol_mm: float
    transit_speed_mm_s: float
    map_name: str
    map_new: bool
    map_skip_blocked: bool
    hit_radius_mm: float
    line_margin_mm: float
    map_stop_mm: float
    tare_s: float
    tare_noise_max: float
    tare_jump_max: float

    @classmethod
    def from_args(cls, args) -> "Settings":
        g, s, m = GUARD_CFG, SEARCH_CFG, MAP_CFG
        guard = GuardConfig.from_dict(g)
        _apply_overrides(guard)
        if getattr(args, "f_contact", None) is not None:
            guard.f_contact = args.f_contact
        if getattr(args, "f_abort", None) is not None:
            guard.f_abort = args.f_abort
        plan = args.speed * 1000
        insert = args.insert_speed * 1000 if getattr(args, "insert_speed", None) else \
            float(g.get("insert_speed_mm_s") or plan)
        return cls(
            guard=guard, mode=args.guard, plan_speed_mm_s=plan,
            approach_speed_mm_s=float(g.get("approach_speed_mm_s") or plan),
            insert_speed_mm_s=insert,
            retract_speed_mm_s=float(g.get("retract_speed_mm_s") or plan),
            after_bone="hold" if getattr(args, "on_bone", "retract") == "hold" else "retract",
            after_depth="retract" if getattr(args, "on_success", "hold") == "retract" else "hold",
            search=bool(getattr(args, "search", False)),
            tilt_steps=list(s.get("tilt_steps_deg", [5, 10, 15, 20])),
            rot_step=float(s.get("rot_step_deg", 45)),
            max_tilt=float(args.max_tilt if getattr(args, "max_tilt", None) is not None
                           else s.get("max_tilt_deg", 20)),
            max_attempts=int(args.max_attempts if getattr(args, "max_attempts", None) is not None
                             else s.get("max_attempts", 6)),
            h_safe_mm=float(s.get("h_safe_mm", 15)), h_safe_max_mm=float(s.get("h_safe_max_mm", 45)),
            h_min_mm=float(s.get("h_min_mm", 5)), entry_tol_mm=float(s.get("entry_tol_mm", 3)),
            transit_speed_mm_s=float(s.get("transit_speed_mm_s", 10)),
            map_name=getattr(args, "map", None) or m.get("name", "phantom"),
            map_new=bool(getattr(args, "map_new", False)),
            map_skip_blocked=bool(getattr(args, "map_skip_blocked", False)),
            hit_radius_mm=float(m.get("hit_radius_mm", 3)),
            line_margin_mm=float(m.get("line_margin_mm", 1)),
            map_stop_mm=float(m.get("map_stop_mm", 2)),
            tare_s=float(g.get("tare_s", 0.5)), tare_noise_max=float(g.get("tare_noise_max", 40000)),
            tare_jump_max=float(g.get("tare_jump_max", 300000)),
        )


# ── the arm we drive: real (CAN) or sim ───────────────────────────────────────

@dataclass
class Rig:
    side: str
    km: ArmKinematics
    rate_hz: float
    send: object
    measure: object                  # q_cmd -> (q_meas rad array, torques list | None)
    real: bool
    lc: object
    mirror: SimMirror | None = None
    # Keeps the needle on its line under load (motion/axis_hold.py); real arm only.
    hold: axis_hold.AxisHold | None = None

    def tip(self, q) -> np.ndarray:
        return np.array(self.km.robot.fkine(np.asarray(q, dtype=float)).t)


def _make_rig(session: Session, side: str, args, km: ArmKinematics, lc) -> Rig:
    want_real = args.mode in ("real", "both")
    rate = args.rate if args.rate is not None else (CTRL_HZ if want_real else SIM_RATE_HZ)
    if want_real:
        real_ctrl = session.real(side)
        mirror = None
        if args.mode == "both":
            print(f"[sim] connecting mirror to CoppeliaSim at {args.sim_host}:{args.sim_port} for {side} arm...")
            mirror = SimMirror(side, args.sim_host, args.sim_port)

        def measure(q_cmd):
            states = real_ctrl.read_joint_states()[:7]
            return np.array([s.position_rad for s in states]), [s.torque for s in states]

        return Rig(side, km, rate, real_sender(real_ctrl, args.kp, args.kd, mirror), measure, True, lc, mirror)
    sim_ctrl = session.sim(side, args.sim_host, args.sim_port)
    # Sim is set exactly to each command, so the command is the measurement --
    # no ZMQ round trip per tick.
    return Rig(side, km, rate, sim_sender(sim_ctrl), lambda q_cmd: (np.asarray(q_cmd, dtype=float), None),
               False, lc)


def _open_loadcell(session: Session, args, point3) -> object:
    kind = args.loadcell or ("real" if args.mode in ("real", "both") else "fake")
    if kind == "fake":
        lc = lcmod.from_config("fake", point3=point3, phantom=args.fake_phantom).open()
        print(f"[loadcell] {lc.describe()}")
        return lc
    return session.loadcell()


# ── per-tick log ──────────────────────────────────────────────────────────────

class AttemptLog:
    """One row per control tick, kept in memory; save() writes the CSV."""

    def __init__(self, path: str, header: dict):
        self.path = path
        self.header = header
        self.rows: list[tuple] = []
        self.marks: list[float] = []
        self.stamp = datetime.now()
        self._t0: float | None = None
        self._depth = 0.0

    def add(self, t, state, rate, cmd, meas, reading, force, q, tau, tilt_deg=0.0, hold_deg=0.0) -> None:
        if self._t0 is None:
            self._t0 = time.monotonic() - t
        self._depth = cmd
        self.rows.append((t, state, rate, cmd, meas, reading, force, q, tau, tilt_deg, hold_deg))

    def mark(self) -> tuple[int, float, float] | None:
        if self._t0 is None:
            return None
        t = time.monotonic() - self._t0
        self.marks.append(t)
        return len(self.marks), t, self._depth

    def save(self, events=()) -> str | None:
        if not self.rows:
            return None
        names = JOINT_NAMES[:7]
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        marks = sorted(self.marks)
        with open(tmp, "w") as f:
            f.write(f"{LOG_MAGIC}\n")
            f.write(f"# created: {self.stamp.isoformat(timespec='seconds')}\n")
            for k, v in self.header.items():
                f.write(f"# {k}: {v}\n")
            f.write(f"# marks_s: {' '.join(f'{m:.4f}' for m in marks) or 'none'}\n")
            f.write(f"# events_s: {'; '.join(f'{e.kind} {e.t:.4f}' for e in events) or 'none'}\n")
            f.write(",".join(["t_s", "mark", "state", "rate", "cmd_depth_mm", "meas_depth_mm", "lag_mm",
                              "force_raw", "force", "force_age_ms", "force_seq"]
                             + [f"{n}_deg" for n in names] + [f"{n}_nm" for n in names]
                             + ["tilt_deg", "axis_hold_deg"]) + "\n")
            pending = 0
            for t, state, rate, cmd, meas, r, force, q, tau, tilt, hold_deg in self.rows:
                number = 0
                while pending < len(marks) and marks[pending] <= t:
                    pending += 1
                    number = pending
                rf = (f"{r.raw},{force:.1f},{r.age_s * 1000:.1f},{r.seq}" if r is not None else ",,,")
                tq = ",".join(f"{v:.4f}" for v in tau) if tau is not None else "," * 6
                f.write(f"{t:.4f},{number},{state},{rate:.4f},{cmd:.3f},{meas:.3f},{cmd - meas:.3f},{rf},"
                        + ",".join(f"{math.degrees(v):.4f}" for v in q) + f",{tq},{tilt:.3f},{hold_deg:.3f}\n")
        os.replace(tmp, self.path)
        return self.path


def load_log(path: str) -> tuple[dict, dict]:
    """(header, columns) of a v3 log: header key -> text, column -> numpy array
    (state as strings)."""
    header, lines = {}, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                if ":" in line:
                    k, _, v = line[1:].partition(":")
                    header[k.strip()] = v.strip()
            else:
                lines.append(line.rstrip("\n"))
    names = lines[0].split(",")
    cols = {n: [] for n in names}
    for line in lines[1:]:
        for n, v in zip(names, line.split(",")):
            cols[n].append(v)
    out = {}
    for n, vals in cols.items():
        if n == "state":
            out[n] = np.array(vals)
        else:
            out[n] = np.array([float(v) if v != "" else np.nan for v in vals])
    return header, out


# ── the governed executor ─────────────────────────────────────────────────────

def run_path(rig: Rig, section: Section, governor, tau0: float = 0.0,
             hold_frozen: bool = False) -> tuple[float, float, np.ndarray]:
    """Stream a planned section with its path time tau advanced by
    `dt x rate`, rate from governor(t, tau, q_cmd, q_meas, torques) -> (rate,
    done), called right after each tick's command.

    Unlike line mode, tau never catches up on lost time: dt is capped at
    max_step_periods ticks, so after a stall (a ZMQ hiccup in sim) the needle
    carries on from where it was instead of jumping millimetres in one tick.

    With rig.hold, each tick's measurement is compared with the plan that
    produced it and the command is the plan plus the hold's correction;
    hold_frozen keeps the correction as it is (moves that aren't along a line).
    Returns (longest gap between commands, final tau, final q_cmd)."""
    period = 1.0 / rig.rate_hz
    max_dt = MAX_STEP_PERIODS * period
    total = section.duration
    tau, rate = min(max(tau0, 0.0), total), 0.0
    t0 = last = time.monotonic()
    tick, max_gap, last_sent = 0, 0.0, None
    hold = rig.hold
    prev_plan = prev_meas = None
    while True:
        now = time.monotonic()
        tau = min(max(tau + min(now - last, max_dt) * rate, 0.0), total)
        dt_tick = min(now - last, max_dt)
        last = now
        q_plan = _q_at(section, tau)
        if hold is not None and not hold_frozen and prev_meas is not None:
            hold.observe(prev_plan, prev_meas, dt_tick)
        q_cmd = hold.command(q_plan) if hold is not None else q_plan
        rig.send(q_cmd)
        sent = time.monotonic()
        if last_sent is not None:
            max_gap = max(max_gap, sent - last_sent)
        last_sent = sent
        q_meas, torques = rig.measure(q_cmd)
        prev_plan, prev_meas = q_plan, q_meas
        rate, done = governor(now - t0, tau, q_cmd, q_meas, torques)
        if done:
            return max_gap, tau, q_cmd
        tick += 1
        sleep = t0 + tick * period - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        elif sleep < -5 * period:
            tick = int((time.monotonic() - t0) / period)   # fell far behind: don't sprint


def _sample(rig: Rig, units: str) -> tuple[object, ForceSample | None, float]:
    r = rig.lc.latest()
    if r is None:
        return None, None, math.nan
    v = r.value(units)
    return r, ForceSample(v, r.age_s, r.seq, r.dropped, r.saturated), v


# ── one attempt ───────────────────────────────────────────────────────────────

@dataclass
class Attempt:
    n: int
    tilt: float
    rot: float
    d: np.ndarray
    start: np.ndarray
    end: np.ndarray
    section: Section | None = None
    outcome: str | None = None
    detail: str = ""
    out_confirmed: bool = False
    contact_mm: float | None = None       # measured, along the line from start
    hit_mm: float | None = None
    reached_mm: float = 0.0
    max_force: float = math.nan
    log_path: str | None = None
    events: list = field(default_factory=list)
    tau_end: float = 0.0

    def point(self, mm: float) -> np.ndarray:
        return self.start + self.d * mm / 1000

    def reached_xyz(self) -> np.ndarray:
        return self.point(self.reached_mm)

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.end - self.start)) * 1000


def _settle_and_tare(rig: Rig, st: Settings, q_hold, prev_offset: float | None) -> str | None:
    """Hold still for tare_s (+ a little), then tare on the median. Returns a
    reason to refuse, or None."""
    tip = rig.tip(q_hold)
    t_end = time.monotonic() + st.tare_s + 0.15
    while time.monotonic() < t_end:
        q_meas, _ = rig.measure(q_hold)
        rig.lc.observe(rig.tip(q_meas) if rig.real else tip)
        time.sleep(0.01)
    try:
        tr = rig.lc.tare(st.tare_s)
    except lcmod.LoadCellError as e:
        return str(e)
    note = f"tare {tr.offset:.0f} counts (sd {tr.sd:.0f}, n {tr.n})"
    if tr.sd > st.tare_noise_max:
        return f"{note}: too noisy (tare_noise_max {st.tare_noise_max:g}) -- is the needle touching something?"
    if prev_offset is not None and abs(tr.offset - prev_offset) > st.tare_jump_max:
        return (f"{note}: {tr.offset - prev_offset:+.0f} from the previous tare (tare_jump_max "
                f"{st.tare_jump_max:g}) -- the needle may still be touching something")
    print(f"    [loadcell] {note}")
    return None


def _run_attempt(rig: Rig, st: Settings, att: Attempt, log: AttemptLog, map_stop_at: float | None,
                 label: str) -> float:
    """Insert along att's planned line under a ForceGuard. Fills the attempt's
    outcome (and att.tau_end); returns the longest gap between commands."""
    global _active_log
    sec = att.section
    length = att.length_mm
    guard = ForceGuard(st.guard, length_mm=length, plan_speed_mm_s=st.plan_speed_mm_s,
                       approach_speed_mm_s=st.approach_speed_mm_s, insert_speed_mm_s=st.insert_speed_mm_s,
                       retract_speed_mm_s=st.retract_speed_mm_s, mode=st.mode,
                       after_bone=st.after_bone, after_depth=st.after_depth, map_stop_at_mm=map_stop_at)
    units = st.guard.units
    total = sec.duration
    printed = [0]
    state_seen = [None]

    def governor(t, tau, q_cmd, q_meas, torques):
        cmd = tau / total * length if total > 0 else 0.0
        tip = rig.tip(q_meas)
        meas = float(np.dot(tip - att.start, att.d)) * 1000
        for change in _apply_overrides(guard.cfg):
            guard.events.append(Event(t, "setting", cmd, meas, guard.force, f"{change} (changed live)"))
        rig.lc.observe(tip)
        reading, sample, value = _sample(rig, units)
        rate = guard.update(t, sample, cmd, meas)
        axis = rig.km.robot.fkine(np.asarray(q_meas, dtype=float)).R @ TOOL_AXIS
        tilt = math.degrees(math.acos(float(np.clip(np.dot(axis, att.d), -1.0, 1.0))))
        log.add(t, guard.state, rate, cmd, meas, reading, value, q_meas, torques, tilt,
                rig.hold.correction_deg if rig.hold is not None else 0.0)
        att.reached_mm = max(att.reached_mm, meas)
        while printed[0] < len(guard.events):
            print(f"[{label}] {rig.side} attempt {att.n}: {guard.events[printed[0]]}", flush=True)
            printed[0] += 1
        if guard.state != state_seen[0]:
            state_seen[0] = guard.state
        _publish(state=guard.state, force=value, units=units, cmd_depth_mm=cmd, meas_depth_mm=meas,
                 rate=rate, attempt=att.n, tilt=att.tilt, rot=att.rot)
        return rate, guard.finished

    _active_log = log
    try:
        max_gap, att.tau_end, _ = run_path(rig, sec, governor)
    finally:
        _active_log = None
        att.outcome = att.outcome or guard.outcome or "stopped"
        att.detail = guard.abort_reason
        att.out_confirmed = guard.out_confirmed
        att.contact_mm = guard.contact_depth_mm
        att.hit_mm = guard.hit_depth_mm
        att.max_force = guard.max_force
        att.events = list(guard.events)
        att.log_path = log.save(guard.events)
    return max_gap


# ── re-aiming between attempts ────────────────────────────────────────────────

def _transit_clear(rig: Rig, q_from, q_to, floor_z: float) -> float:
    """Lowest tip height (m) over the joint-space move, relative to floor_z."""
    lowest = math.inf
    for u in np.linspace(0.0, 1.0, _TRANSIT_SAMPLES):
        lowest = min(lowest, rig.tip(q_from + (q_to - q_from) * u)[2] - floor_z)
    return lowest


def _run_transit(rig: Rig, st: Settings, section: Section, label: str) -> str | None:
    """The guarded re-aim move. Any contact (force over f_contact on 3 samples)
    or a stale load cell reverses it back to where it started. Returns the
    reason it was reversed, or None."""
    g = st.guard
    units = g.units
    total = section.duration
    state = {"bad": "", "run": 0, "seq": None}

    def governor(t, tau, q_cmd, q_meas, torques):
        for change in _apply_overrides(g):
            print(f"[{label}] {rig.side} re-aim: {change} (changed live)", flush=True)
        rig.lc.observe(rig.tip(q_meas))
        r, s, v = _sample(rig, units)
        if not state["bad"]:
            if s is None or s.age_s > g.stale_s:
                state["bad"] = "load cell stale"
            elif s.saturated:
                state["bad"] = "load cell saturated"
            elif s.seq != state["seq"]:
                state["seq"] = s.seq
                state["run"] = state["run"] + 1 if abs(v) > g.f_contact else 0
                if state["run"] >= g.contact_samples:
                    state["bad"] = f"force {v:.6g} during the re-aim move -- it touched something"
            if state["bad"]:
                print(f"[{label}] {rig.side} re-aim: {state['bad']}; going back", flush=True)
        _publish(state="re-aim", force=v, units=units, rate=-1.0 if state["bad"] else 1.0)
        if state["bad"]:
            return -1.0, tau <= 0.0
        return 1.0, tau >= total

    run_path(rig, section, governor, hold_frozen=True)
    return state["bad"] or None


def _plan_next(rig: Rig, st: Settings, omap: OccupancyMap, entry: np.ndarray, insert_mm: float,
               q_cur: np.ndarray, tried: set, plan_step: float, plan_speed: float):
    """The best unblocked, reachable direction through `entry`, planned with a
    transit that keeps the tip h_min above the surface. Returns
    (attempt-to-be, transit section) or (None, why)."""
    km = rig.km
    floor_z = entry[2] + st.h_min_mm / 1000
    usable, blocked = ds.rank(entry, omap, tilt_steps_deg=st.tilt_steps, rot_step_deg=st.rot_step,
                              max_tilt_deg=st.max_tilt, h_safe_mm=st.h_safe_mm, insert_mm=insert_mm,
                              line_margin_mm=st.line_margin_mm, exclude=tried)
    print(f"    [search] {len(usable)} direction(s) clear of the {len(omap.hits)} known hit(s), "
          f"{len(blocked)} blocked, {len(tried)} already tried", flush=True)
    for cand in usable:
        tried.add(cand.key)
        h = st.h_safe_mm
        while h <= st.h_safe_max_mm + 1e-9:
            s, t, d = ds.line_through(entry, cand.tilt_deg, cand.rot_deg, h, insert_mm)
            cur_tip = rig.tip(q_cur)
            sections = [Section("re-aim", cur_tip, s, st.transit_speed_mm_s / 1000, joint_space=True),
                        Section("insert", s, t, plan_speed, d)]
            print(f"    [search] trying {cand} from {h:.0f} mm above the surface...", flush=True)
            failure = _plan_oriented(km, sections, q_cur, plan_step)
            if failure:
                print(f"    [search]   unreachable: {failure}", flush=True)
                break
            transit, line = sections
            low = _transit_clear(rig, q_cur, transit.knots[0], entry[2])
            if low * 1000 >= st.h_min_mm:
                att = Attempt(0, cand.tilt_deg, cand.rot_deg, d, s, t, section=line)
                print(f"    [search]   planned: re-aim {transit.duration:.1f}s (tip stays "
                      f"{low * 1000:.1f} mm above the surface), line {att.length_mm:.1f} mm", flush=True)
                return att, transit
            print(f"    [search]   the re-aim move takes the tip to {low * 1000:.1f} mm above the "
                  f"surface (h_min {st.h_min_mm:g}) -- starting higher", flush=True)
            h += 10.0
    return None, "no reachable direction clear of the known hits"


# ── the whole run ─────────────────────────────────────────────────────────────

def run(session: Session, side: str, args, km: ArmKinematics, q_now: np.ndarray, label: str) -> bool:
    """Plunger point 4 with the guard (and maybe the search). The EE is at
    point 3 with the tool straight down (checked by the caller)."""
    st = Settings.from_args(args)
    start = np.array(km.fkine(np.degrees(q_now)).t)
    lc = _open_loadcell(session, args, start)
    if st.guard.units == "N" and not lc.scale:
        print("Error: guard.units is N but the load cell isn't calibrated "
              "(loadcell.scale_counts_per_n) -- calibrate it or use units: counts")
        return False
    rig = _make_rig(session, side, args, km, lc)
    if rig.real and not args.no_axis_hold:
        rig.hold = axis_hold.from_config(km, DOWN, AXIS_HOLD_CFG)
    _publish(side=side, state="planning", attempt=1, force=math.nan, units=st.guard.units,
             loadcell=lc, events=[])

    omap = None
    if st.mode == "on":
        omap = OccupancyMap.open(MAPS_DIR, st.map_name, side, st.hit_radius_mm, fresh=st.map_new)
        print(f"[map] '{omap.name}': {len(omap.hits)} known hit(s)"
              + (f", entry point {np.array2string(omap.entry_point, precision=4)}"
                 if omap.entry_point is not None else "") + f" ({omap.path})")

    stamp = datetime.now()
    suffix = f"_{args.name}" if args.name else ""
    folder = os.path.join(SEARCH_DIR, f"{stamp:%Y%m%d_%H%M%S}_{side}{suffix}") if st.search else None

    def log_path(n):
        if folder:
            return os.path.join(folder, f"attempt_{n:02d}.csv")
        return os.path.join(PLUNGER_LOG_DIR, f"{stamp:%Y%m%d_%H%M%S}_{side}{suffix}.csv")

    # Attempt 1: straight down from point 3, as plain point 4.
    depth_mm = args.depth * 1000
    first = Attempt(1, 0.0, 0.0, DOWN.copy(), start, start + DOWN * args.depth)
    first.section = Section("descent", first.start, first.end, args.speed, DOWN)
    print(f"    planning the descent: {first.section.knot_count(args.plan_step)} knots over "
          f"{depth_mm:.1f}mm...", flush=True)
    _, failure = _plan_section(km, first.section, q_now, args.plan_step)
    if failure:
        print(f"    [{side}] descent not reachable -- {failure}\n    [{side}] nothing moved.")
        return False

    attempts: list[Attempt] = []
    entry = None
    insert_mm = None
    tried: set = set()
    stop_reason = ""
    prev_tare = None
    max_gap = 0.0

    with contextlib.ExitStack() as stack:
        if rig.mirror is not None:
            stack.enter_context(rig.mirror)
        if rig.real:
            # q_now is the measured position (the descent starts where the
            # needle really is); the motors hold a setpoint a few mm away from
            # it. That difference is the starting correction, so the first
            # command is what they already hold -- no jump.
            real_ctrl = session.real(side)
            q_check = np.array([s.position_rad for s in real_ctrl.read_joint_states()[:7]])
            drift = float(np.degrees(np.max(np.abs(q_check - q_now))))
            if drift > _MAX_DRIFT_DEG:
                print(f"    [{side}] arm moved {drift:.2f} deg while planning (max {_MAX_DRIFT_DEG}) "
                      f"-- nothing moved; re-run to plan from where it is now.")
                return False
            q_held = np.array(ramp_start(real_ctrl)[0])
            warn_if_start_out_of_range(list(q_held), real_ctrl.arm_limits_rad)
            first.section.q_begin = q_now
            if rig.hold is None:
                rig.hold = axis_hold.AxisHold(km, DOWN, adaptive=False)
            cut = rig.hold.preload(q_held - q_now)
            if cut > 0.05:
                print(f"    [{side}] WARNING setpoint and measured position differ by more than the "
                      f"correction limit ({cut:.1f} deg over) -- expect a small jump at the start")

        if (omap is not None and st.search and st.map_skip_blocked and omap.entry_point is not None
                and omap.clearance_mm(first.start, first.end) < st.hit_radius_mm + st.line_margin_mm):
            print("[search] straight down is blocked by the map -- skipping it (--map-skip-blocked)")
            entry = omap.entry_point.copy()
            insert_mm = depth_mm - float(np.dot(entry - start, DOWN)) * 1000
            first.outcome, first.out_confirmed = "map_stop", True
            tried.add((0.0, 0.0))
            attempts.append(first)
            att = None
        else:
            att = first

        mode_note = ("guard ON" + (", search" if st.search else "")) if st.mode == "on" else \
            "guard LOG (constant speed; safety trips active)"
        print(f"[{label}] {side}: streaming at {rig.rate_hz:.0f} Hz"
              + (f", sim mirrored at {MIRROR_RATE_HZ:.0f} Hz" if rig.mirror else "")
              + f"; {mode_note}; axis hold {'on' if rig.hold is not None and rig.hold.adaptive else 'off'}"
              + f"; force in {st.guard.units}; {lc.describe()}", flush=True)
        try:
            while True:
                if att is None:
                    # ── choose and fly to the next direction ──
                    prev = attempts[-1]
                    if len(attempts) >= st.max_attempts:
                        stop_reason = f"gave up after {len(attempts)} attempts (max_attempts)"
                        break
                    q_cur = prev.section.q_begin if prev.section is not None else first.section.q_begin
                    nxt, transit = _plan_next(rig, st, omap, entry, insert_mm, np.asarray(q_cur),
                                              tried, args.plan_step, args.speed)
                    if nxt is None:
                        stop_reason = transit
                        break
                    nxt.n = len(attempts) + 1
                    transit.q_begin = np.asarray(q_cur)
                    _publish(state="re-aim", attempt=nxt.n, tilt=nxt.tilt, rot=nxt.rot)
                    print(f"[{label}] {side}: re-aiming to tilt {nxt.tilt:g} deg, rot {nxt.rot:g} deg "
                          f"({transit.duration:.1f}s)...", flush=True)
                    why = _run_transit(rig, st, transit, label)
                    if why:
                        stop_reason = f"re-aim move reversed: {why}"
                        break
                    att = nxt

                # ── one attempt ──
                reason = _settle_and_tare(rig, st, att.section.q_begin, prev_tare)
                if reason:
                    stop_reason = f"attempt {att.n} not started: {reason}"
                    break
                prev_tare = lc.offset
                map_stop_at = None
                if omap is not None and omap.hits:
                    entry_mm = omap.first_entry_mm(att.start, att.d, att.length_mm / 1000)
                    if entry_mm is not None:
                        map_stop_at = max(entry_mm - st.map_stop_mm, 0.0)
                        print(f"    [map] a known hit lies ahead at {entry_mm:.1f} mm -- will stop at "
                              f"{map_stop_at:.1f} mm", flush=True)
                if rig.hold is not None:
                    rig.hold.set_direction(att.d)
                header = _header(args, st, rig, att)
                log = AttemptLog(log_path(att.n), header)
                print(f"[{label}] {side} attempt {att.n}: tilt {att.tilt:g} deg, rot {att.rot:g} deg, "
                      f"{att.length_mm:.1f} mm from {np.array2string(att.start, precision=4)}", flush=True)
                attempts.append(att)
                _publish_map(omap, entry, attempts)
                try:
                    max_gap = max(max_gap, _run_attempt(rig, st, att, log, map_stop_at, label))
                finally:
                    entry, insert_mm = _absorb(att, entry, insert_mm, depth_mm, omap, st, folder, stamp)
                    _publish_map(omap, entry, attempts)
                print(f"    attempt {att.n}: {att.outcome}" + (f" ({att.detail})" if att.detail else "")
                      + (", needle confirmed out" if att.out_confirmed else "")
                      + f"; log -> {att.log_path}", flush=True)

                if att.outcome == "depth":
                    stop_reason = "reached the target depth"
                    break
                if att.outcome == "abort":
                    stop_reason = f"safety trip: {att.detail}"
                    break
                if att.outcome not in ("bone", "map_stop"):
                    stop_reason = att.outcome
                    break
                if not st.search:
                    stop_reason = f"{att.outcome}" + ("; needle out" if att.out_confirmed else "")
                    break
                if not att.out_confirmed:
                    stop_reason = "the needle wasn't confirmed out of the material -- not re-aiming"
                    break
                if entry is None and omap is not None and omap.entry_point is not None:
                    # Stopped by the map before touching the surface: the map
                    # remembers where the surface was entered before.
                    entry = omap.entry_point.copy()
                    insert_mm = depth_mm - float(np.dot(entry - first.start, DOWN)) * 1000
                    print(f"    entry point E = {np.array2string(entry, precision=4)} (from the map)")
                if entry is None:
                    stop_reason = "no entry point known (no contact yet) -- can't re-aim"
                    break
                tried.add((round(att.tilt, 3), round(att.rot % 360.0, 3) if att.tilt else 0.0))
                att = None
        finally:
            if folder and attempts:
                _write_summary(os.path.join(folder, "summary.csv"), attempts)
            last = attempts[-1] if attempts else None
            if last is not None and last.section is not None and last.contact_mm is not None \
                    and not last.out_confirmed:
                NEEDLE_IN[side] = (last.section, last.tau_end,
                                   rig.hold.c.copy() if rig.hold is not None else None)
                print(f"[{label}] {side}: the needle is still in the material -- plunger --point 5 "
                      f"retracts it along the same line")
            else:
                NEEDLE_IN.pop(side, None)
            _publish(state="done", stop_reason=stop_reason)

    _report(side, attempts, stop_reason, max_gap, rig, folder)
    return True


# ── point 5: back out ─────────────────────────────────────────────────────────

def retract(session: Session, side: str, args, label: str) -> None:
    """Plunger point 5: take the needle out of the material. Along the line
    the last insertion left it on (NEEDLE_IN), backwards to that line's start;
    otherwise straight back along the tool axis by --depth."""
    km = ArmKinematics(side, limit_source=args.limits)
    want_real = args.mode in ("real", "both")
    if want_real:
        q_now = np.array(ramp_start(session.real(side))[0])     # the held setpoint
        q_meas = np.array([s.position_rad for s in session.real(side).read_joint_states()[:7]])
    else:
        q_now = np.array(session.sim(side, args.sim_host, args.sim_port).read_joint_positions())
        q_meas = q_now
    speed_mm_s = float(GUARD_CFG.get("retract_speed_mm_s") or args.speed * 1000)
    last = NEEDLE_IN.get(side)
    tau0, section = 0.0, None
    c0 = None
    if last is not None:
        sec, tau_end, c0 = last
        expect = _q_at(sec, tau_end) + (c0 if c0 is not None else 0.0)
        off = float(np.degrees(np.max(np.abs(expect - q_now))))
        if off <= _MAX_DRIFT_DEG:
            section, tau0 = sec, tau_end
            back_mm = tau_end / sec.duration * sec.length * 1000 if sec.duration > 0 else 0.0
            print(f"[{label}] {side}: backing out {back_mm:.1f} mm along the last insertion line at "
                  f"{speed_mm_s:g} mm/s")
        else:
            print(f"[{label}] {side}: the arm is {off:.1f} deg from where the last insertion left it -- "
                  f"backing straight out along the tool axis instead")
    if section is None:
        if not args.depth or args.depth <= 0:
            print("Error: no insertion line on record -- point 5 needs --depth (how far to back out)")
            return
        # From the measured position, as points 3-4; (held - measured) is the
        # starting correction, so it starts without a jump.
        T = km.fkine(np.degrees(q_meas))
        tip, axis = np.array(T.t), T.R @ np.array([-1.0, 0.0, 0.0])
        section = Section("retract", tip, tip - axis * args.depth, speed_mm_s / 1000, axis)
        print(f"    planning {args.depth * 1000:.1f} mm straight back along the tool axis...", flush=True)
        c0, tau0 = q_now - q_meas, 0.0
        _, failure = _plan_section(km, section, q_meas, args.plan_step)
        if failure:
            print(f"    [{side}] not reachable -- {failure}\n    [{side}] nothing moved.")
            return
        rate_dir = 1.0
    else:
        rate_dir = -1.0
    if args.dry_run:
        print(f"    [{side}] --dry-run, not moving.")
        return
    rig = _make_rig(session, side, args, km, None)
    if rig.real and section.axis is not None:
        rig.hold = None if args.no_axis_hold else axis_hold.from_config(km, section.axis, AXIS_HOLD_CFG)
        if c0 is not None:
            # carry on from the correction in force now -- no jump at the start
            if rig.hold is None:
                rig.hold = axis_hold.AxisHold(km, section.axis, adaptive=False)
            rig.hold.preload(c0)
    plan_speed = section.speed * 1000
    rate = rate_dir * speed_mm_s / plan_speed if rate_dir < 0 else 1.0
    total = section.duration

    def governor(t, tau, q_cmd, q_meas, torques):
        _publish(state="retract", rate=rate)
        return rate, (tau <= 0.0 if rate < 0 else tau >= total)

    with rig.mirror if rig.mirror is not None else contextlib.nullcontext():
        max_gap, _, q_end = run_path(rig, section, governor, tau0=tau0)
    NEEDLE_IN.pop(side, None)
    _publish(state="done", stop_reason="retracted")
    print(f"[{label}] {side}: out; EE {np.array2string(rig.tip(q_end), precision=4)}; longest gap "
          f"between commands {max_gap * 1000:.1f}ms")


def _absorb(att: Attempt, entry, insert_mm, depth_mm: float, omap: OccupancyMap | None, st: Settings,
            folder: str | None, stamp: datetime):
    """What one attempt tells us: the entry point (first contact), bone hits
    into the occupancy map, the line for the map view. Runs even when the
    attempt was cut short. Returns (entry, insert_mm)."""
    if att.contact_mm is not None:
        contact = att.point(att.contact_mm)
        if entry is None:
            entry = contact
            if att.n == 1:
                insert_mm = depth_mm - att.contact_mm
            print(f"    entry point E = {np.array2string(entry, precision=4)} "
                  f"(insertion {insert_mm:.1f} mm along the needle)", flush=True)
            if omap is not None:
                if omap.entry_point is not None and \
                        np.linalg.norm(omap.entry_point - entry) * 1000 > st.entry_tol_mm:
                    print(f"    WARNING the map's entry point is "
                          f"{np.linalg.norm(omap.entry_point - entry) * 1000:.1f} mm from this one -- "
                          f"has the block moved? (--map-new starts a fresh map)")
                omap.set_entry_point(entry)
        elif np.linalg.norm(contact - entry) * 1000 > st.entry_tol_mm:
            print(f"    WARNING contact {np.linalg.norm(contact - entry) * 1000:.1f} mm from the entry "
                  f"point (entry_tol {st.entry_tol_mm:g} mm) -- has the block moved?")
    if omap is not None:
        if att.outcome == "bone" and att.hit_mm is not None:
            hit = att.point(att.hit_mm)
            omap.add_hit(hit, attempt=att.n, dir_deg=[att.tilt, att.rot],
                         force=round(float(att.max_force), 1), session=f"{stamp:%Y%m%d_%H%M%S}")
            print(f"    [map] hit {len(omap.hits)} at {np.array2string(hit, precision=4)}")
        omap.add_line(att.start, att.reached_xyz(), dir_deg=[att.tilt, att.rot],
                      outcome=att.outcome, session=f"{stamp:%Y%m%d_%H%M%S}")
        omap.save()
        if folder:
            omap.save(os.path.join(folder, "map.json"))
    return entry, insert_mm


def _header(args, st: Settings, rig: Rig, att: Attempt) -> dict:
    tr = rig.lc.tare_result
    g = st.guard
    return {
        "arm": rig.side,
        "mode": f"{args.mode}  guard {st.mode}" + ("  search" if st.search else ""),
        "attempt": f"{att.n}  tilt_deg {att.tilt:g}  rot_deg {att.rot:g}",
        "line_m": f"{' '.join(f'{v:.5f}' for v in att.start)} -> {' '.join(f'{v:.5f}' for v in att.end)}",
        "speeds_mm_s": f"plan {st.plan_speed_mm_s:g}  approach {st.approach_speed_mm_s:g}  "
                       f"insert {st.insert_speed_mm_s:g}  retract {st.retract_speed_mm_s:g}",
        "rate_hz": f"{rig.rate_hz:g}",
        "gains": (f"kp {' '.join(f'{v:g}' for v in args.kp)}  kd {' '.join(f'{v:g}' for v in args.kd)}"
                  if rig.real else "sim"),
        "loadcell": f"{rig.lc.describe()}; tare_raw {tr.offset:.0f} sd {tr.sd:.0f}" if tr else rig.lc.describe(),
        "force_units": g.units,
        "guard": (f"f_contact {g.f_contact:g}  f_soft {g.f_soft}  f_bone_min {g.f_bone_min}  "
                  f"k_bone {g.k_bone}  stall_ratio {g.stall_ratio}  t_persist_s {g.t_persist_s:g}  "
                  f"f_abort {g.f_abort:g}  f_pull_abort {g.f_pull_abort:g}  lag_max_mm {g.lag_max_mm:g}"),
    }


def _write_summary(path: str, attempts: list[Attempt]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("attempt,tilt_deg,rot_deg,outcome,detail,out_confirmed,contact_mm,hit_mm,reached_mm,"
                "max_force,start_x,start_y,start_z,end_x,end_y,end_z,log\n")
        for a in attempts:
            def num(v):
                return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.3f}"
            f.write(f"{a.n},{a.tilt:g},{a.rot:g},{a.outcome},\"{a.detail}\",{int(a.out_confirmed)},"
                    f"{num(a.contact_mm)},{num(a.hit_mm)},{a.reached_mm:.3f},{num(a.max_force)},"
                    + ",".join(f"{v:.5f}" for v in a.start) + "," + ",".join(f"{v:.5f}" for v in a.end)
                    + f",{os.path.basename(a.log_path) if a.log_path else ''}\n")


def _report(side: str, attempts: list[Attempt], stop_reason: str, max_gap: float, rig: Rig,
            folder: str | None) -> None:
    print(f"[{side}] insertion result: {stop_reason}")
    print(f"      {'#':>2}  {'tilt/rot deg':>13}  {'outcome':<9} {'contact':>9} {'hit':>8} "
          f"{'reached':>8} {'max force':>11}  out")
    for a in attempts:
        def mm(v):
            return f"{v:7.2f}mm" if v is not None else f"{'-':>9}"
        mf = f"{a.max_force:11.6g}" if not math.isnan(a.max_force) else f"{'-':>11}"
        print(f"      {a.n:>2}  {a.tilt:>5g}/{a.rot:<7g}  {a.outcome or '-':<9} {mm(a.contact_mm)} "
              f"{mm(a.hit_mm)} {a.reached_mm:7.2f}mm {mf}  {'yes' if a.out_confirmed else '-'}")
    print(f"      longest gap between commands {max_gap * 1000:.1f}ms (tick period "
          f"{1000 / rig.rate_hz:.1f}ms)")
    if rig.hold is not None:
        print(f"      {rig.hold.summary()}")
    if folder:
        print(f"      search folder -> {folder}")
    if rig.mirror is not None:
        print(f"      sim mirror: {rig.mirror.updates} updates, {rig.mirror.skipped} skipped")
