"""
Line mode -- drive the EE along a straight line in three sections, each at its
own speed: current pose -> --start -> --split of --depth -> full --depth.

Direction (--angles TILT ROT, degrees) is (sin(tilt)cos(rot), sin(tilt)sin(rot),
-cos(tilt)), so `0 0` is straight down.

With --orient path (the default) the tool axis (kinematics.TOOL_AXIS, the TCP's
-X: wrist -> fingertips) is held parallel to the line, pointing along the
direction of travel, from --start to full depth; roll about it is left free.
Sections 2..3 are planned first, from whichever aligned configuration at --start
carries the tool continuously along the line; the approach is then a joint-space
move to that configuration, so it arrives at --start already aligned (its EE path
is not a straight line). --orient free is position-only IK throughout, with a
straight-line approach.

On the real arm, the measured joint torques from the moment the EE reaches
--start until the path ends are saved to recordings/line_torque/, one CSV per
arm per run, named by the time --start was reached:

    # openarm-line-torque v1
    # created: 2026-09-22T14:30:05
    # arm: right
    # line: --start 0.3 -0.35 0.3 --angles 0 0 --depth 0.06 --speeds 0.02 0.005 0.002 --split 0.6 --orient path
    # rate_hz: 100
    # gains: kp 40 40 40 40 40 40 40  kd 0.5 0.5 0.5 0.5 0.5 0.5 0.5
    t_s,section,cmd_depth_mm,joint1_deg,...,joint7_deg,joint1_nm,...,joint7_nm
    0.0042,2,0.021,-1.2345,...

t_s counts from when the schedule reached --start; cmd_depth_mm is how far past
--start the path had the EE at that moment. numpy reads one with
`np.genfromtxt(path, delimiter=",", names=True, skip_header=6)`.
Full reference: docs/07_main_script.md.
"""
import bisect
import contextlib
import functools
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np

from config import CTRL_HZ, JOINT_NAMES, RECORDINGS_DIR
from kinematics import ArmKinematics, TOOL_AXIS
from motion import ramp_start, warn_if_start_out_of_range
from session import Session, resolve_sides
from streaming import MIRROR_RATE_HZ, SIM_RATE_HZ, SimMirror, real_sender, sim_sender

# Seeds sit millimetres from the answer; more restarts only slow down ruling out
# a genuinely unreachable point.
_PLAN_RESTARTS = 2
# Planning takes seconds. If the arm's starting point moved more than this in the
# meantime (bumped while unheld, or its motors let go), refuse rather than
# stream from a stale plan.
_MAX_DRIFT_DEG = 2.0

# --orient path: the approach is a joint-space move whose fastest joint turns
# at most this fast (and whose EE covers the straight-line distance no faster
# than S1).
APPROACH_JOINT_SPEED_DEG = 15.0
# --orient path: IK seeds tried for the aligned pose at --start (the current
# configuration, zero and mid-range, then random ones). Distinct solutions are
# tried nearest-first until one carries the tool continuously along the line.
_START_RANDOM_SEEDS = 12
_START_DISTINCT_DEG = 1.0
# With the tool axis held, the solver has 2 free DOF and can hop to a different
# arm configuration between knots; joint-space interpolation across such a hop
# leaves the line. That start configuration is rejected instead.
_MAX_KNOT_JUMP_DEG = 10.0

# Kept out of RECORDINGS_DIR itself, which replay lists as its recordings.
TORQUE_LOG_DIR = os.path.join(RECORDINGS_DIR, "line_torque")
_TORQUE_LOG_MAGIC = "# openarm-line-torque v1"


def direction_from_angles(tilt_deg: float, rot_deg: float) -> np.ndarray:
    t, r = math.radians(tilt_deg), math.radians(rot_deg)
    return np.array([
        math.sin(t) * math.cos(r),
        math.sin(t) * math.sin(r),
        -math.cos(t),
    ])


def axis_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Angle between two unit vectors, radians."""
    return math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0)))


@dataclass
class Section:
    name: str
    begin: np.ndarray
    end: np.ndarray
    speed: float                                  # m/s
    # Tool axis (unit, robot_base frame) held over the section; None = free
    # (position-only IK).
    axis: np.ndarray | None = None
    # A single joint-space move from q_begin to knots[0] instead of a Cartesian
    # line (the --orient path approach); begin/end are then just its EE ends.
    joint_space: bool = False
    q_begin: np.ndarray | None = None             # joint config at `begin`
    knots: list = field(default_factory=list)     # solved q at each knot after `begin`

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end - self.begin))

    @property
    def duration(self) -> float:
        if self.joint_space and self.knots:
            turn = float(np.degrees(np.max(np.abs(self.knots[0] - self.q_begin))))
            return max(self.length / self.speed, turn / APPROACH_JOINT_SPEED_DEG)
        return self.length / self.speed

    def knot_count(self, step_m: float) -> int:
        return max(1, int(math.ceil(self.length / step_m)))


def _read_real(real_ctrl) -> np.ndarray:
    return np.array([s.position_rad for s in real_ctrl.read_joint_states()[:7]])


def _plan_section(km: ArmKinematics, section: Section, q_seed: np.ndarray,
                  step_m: float) -> tuple[np.ndarray, str | None]:
    """IK-solve evenly spaced knots, seeding each solve with the previous
    solution. Fills section.q_begin/knots; returns (last_q, failure_or_None)."""
    section.q_begin = q_seed
    n = section.knot_count(step_m)
    for i in range(1, n + 1):
        point = section.begin + (section.end - section.begin) * (i / n)
        if section.axis is None:
            sol = km.ikine_position(point, q0_deg=np.degrees(q_seed), restarts=_PLAN_RESTARTS)
        else:
            # No restarts: a random seed that converges lands on another branch,
            # which the jump check below would reject anyway.
            sol = km.ikine_axis(point, section.axis, q0_deg=np.degrees(q_seed), restarts=0)
        if not sol.success:
            return q_seed, (
                f"{section.name}: knot {i}/{n} at "
                f"{np.array2string(point, precision=4)} unreachable "
                f"(residual {sol.residual:.3g})"
            )
        jump = float(np.degrees(np.max(np.abs(sol.q - q_seed))))
        if section.axis is not None and jump > _MAX_KNOT_JUMP_DEG:
            return q_seed, (
                f"{section.name}: knot {i}/{n} at {np.array2string(point, precision=4)} "
                f"needs a {jump:.1f}deg joint jump from the previous knot (max "
                f"{_MAX_KNOT_JUMP_DEG:g}) -- the tool can't stay on the line continuously"
            )
        section.knots.append(sol.q)
        q_seed = sol.q
    return q_seed, None


def _plan_oriented(km: ArmKinematics, sections: list[Section], q_now: np.ndarray,
                   step_m: float) -> str | None:
    """--orient path: find a configuration at --start with the tool on the line
    from which sections 2..3 plan continuously, then make section 1 (approach)
    a joint-space move to it. Planning the line first is what makes this work:
    turning the tool while also tracking a Cartesian approach line runs the
    narrow real limits into a wall on most branches. Returns failure or None."""
    approach, line = sections[0], sections[1:]
    start, axis = line[0].begin, line[0].axis
    lo, hi = km.qlim
    rng = np.random.default_rng(0)
    seeds = [q_now, np.zeros(km.n), (lo + hi) / 2]
    seeds += [rng.uniform(lo, hi) for _ in range(_START_RANDOM_SEEDS)]
    candidates = []
    for seed in seeds:
        sol = km.ikine_axis(start, axis, q0_deg=np.degrees(seed), restarts=0)
        if sol.success and all(np.degrees(np.max(np.abs(sol.q - c))) > _START_DISTINCT_DEG
                               for c in candidates):
            candidates.append(sol.q)
    if not candidates:
        return (f"no joint configuration puts the TCP at --start "
                f"{np.array2string(start, precision=4)} with the tool along the line "
                f"within the {km.limit_source} limits")
    candidates.sort(key=lambda c: float(np.max(np.abs(c - q_now))))
    print(f"    {len(candidates)} start configuration(s) with the tool on the line; "
          f"trying nearest first", flush=True)

    failure = None
    for n, q_start in enumerate(candidates, start=1):
        q_seed, failure = q_start, None
        for sec in line:
            sec.knots = []
            q_seed, failure = _plan_section(km, sec, q_seed, step_m)
            if failure:
                break
        if not failure:
            approach.q_begin, approach.knots = q_now, [q_start]
            turn = np.degrees(np.max(np.abs(q_start - q_now)))
            print(f"    start configuration {n}/{len(candidates)}: line planned; "
                  f"approach turns its furthest joint {turn:.1f}deg", flush=True)
            return None
        print(f"    start configuration {n}/{len(candidates)}: {failure}", flush=True)
    return f"none of {len(candidates)} start configurations works (last: {failure})"


def _timeline(sections: list[Section]) -> tuple[list[Section], list[float], float]:
    """Sections that have knots, each one's start time, and the total duration."""
    active = [s for s in sections if s.knots]
    starts, total = [], 0.0
    for sec in active:
        starts.append(total)
        total += sec.duration
    return active, starts, total


def _q_at(section: Section, t: float) -> np.ndarray:
    """Joint config `t` seconds into a section. knots[i] is reached at (i+1)/n
    of the section's duration, joint-space linear in between."""
    n = len(section.knots)
    u = min(max(t / section.duration, 0.0), 1.0) * n
    i = min(int(u), n - 1)
    q0 = section.q_begin if i == 0 else section.knots[i - 1]
    return q0 + (section.knots[i] - q0) * (u - i)


def _execute(sections: list[Section], rate_hz: float, send, measure, label: str, side: str,
             log=None):
    """Stream the path, evaluated against wall-clock time rather than one
    waypoint per tick: a late tick lands where the path should be now, instead
    of a burst of catch-up waypoints briefly driving faster than commanded.

    Returns (marks, longest gap between commands). marks holds
    (section index, or None for the end; schedule time; measured q), each taken
    right after that tick's command so it can be compared with the schedule.
    log, if given, is called with the schedule time right after every command."""
    active, starts, total = _timeline(sections)
    period = 1.0 / rate_hz
    marks = []
    current, tick, max_gap, last_sent = -1, 0, 0.0, None
    t0 = time.time()
    while True:
        t = time.time() - t0
        done = t >= total
        k = len(active) - 1 if done else bisect.bisect_right(starts, t) - 1
        send(active[-1].knots[-1] if done else _q_at(active[k], t - starts[k]))
        sent = time.time()
        if last_sent is not None:
            max_gap = max(max_gap, sent - last_sent)
        last_sent = sent
        if log is not None:
            log(t)

        if k != current:
            q_meas = measure()
            for j in range(current + 1, k + 1):
                marks.append((j, t, q_meas))
                sec = active[j]
                how = ("joint-space move" if sec.joint_space
                       else f"{sec.length * 1000:.1f}mm at {sec.speed * 1000:.1f}mm/s")
                print(f"[{label}] {side} {sec.name}: {how} ({sec.duration:.2f}s)...")
            current = k
        if done:
            marks.append((None, t, measure()))
            break

        tick += 1
        sleep = (t0 + tick * period) - time.time()
        if sleep > 0:
            time.sleep(sleep)
    return marks, max_gap


def _report(km: ArmKinematics, sections: list[Section], marks, max_gap: float,
            rate_hz: float, end: np.ndarray, side: str) -> None:
    """Tracking error at each checkpoint: measured EE vs where the schedule had
    the EE at that same moment. Being on schedule at both ends of a section is
    what shows the section ran at its commanded speed. With --orient path, axis
    error is the angle between the measured and the scheduled tool axis."""
    active, starts, _ = _timeline(sections)
    line_axis = sections[-1].axis
    print(f"[{side}] result:")
    print(f"      {'checkpoint':<22}{'tracking error':>16}"
          + (f"{'axis error':>13}" if line_axis is not None else ""))
    for j, t, q in marks:
        T = km.fkine(np.degrees(q))
        xyz, tool = np.array(T.t), T.R @ TOOL_AXIS
        if j is None:
            axis_err = (f"{math.degrees(axis_angle(tool, line_axis)):>11.2f}deg"
                        if line_axis is not None else "")
            print(f"      {'end':<22}{np.linalg.norm(xyz - end) * 1000:>14.2f}mm{axis_err}"
                  f"   final EE {np.array2string(xyz, precision=4)}")
            continue
        k = bisect.bisect_right(starts, t) - 1
        sec = active[k]
        T_plan = km.fkine(np.degrees(_q_at(sec, t - starts[k])))
        if sec.joint_space:
            want = np.array(T_plan.t)
        else:
            want = sec.begin + (sec.end - sec.begin) * min(max((t - starts[k]) / sec.duration, 0.0), 1.0)
        axis_err = (f"{math.degrees(axis_angle(tool, T_plan.R @ TOOL_AXIS)):>11.2f}deg"
                    if line_axis is not None else "")
        print(f"      {'start ' + active[j].name:<22}{np.linalg.norm(xyz - want) * 1000:>14.2f}mm{axis_err}")
    print(f"      longest gap between commands {max_gap * 1000:.1f}ms "
          f"(tick period {1000 / rate_hz:.1f}ms)")


class _TorqueLog:
    """The real arm's measured joint torques, one sample per tick, from when the
    schedule reaches --start (the end of the approach) until the path ends.
    Held in memory so the tick loop does no file I/O; save() writes the file."""

    def __init__(self, real_ctrl, sections: list[Section], side: str, args, rate_hz: float):
        self.real_ctrl = real_ctrl
        self.side = side
        self.args = args
        self.rate_hz = rate_hz
        self.approach = sections[0]
        self.number = {id(sec): i for i, sec in enumerate(sections, start=1)}
        self.active, self.starts, _ = _timeline(sections)
        # Schedule time the EE reaches --start (0 if the approach was skipped as
        # zero length), and where --start is.
        self.t_start = self.approach.duration if self.approach.knots else 0.0
        self.origin = sections[1].begin
        self.stamp: datetime | None = None
        self.rows = []

    def sample(self, t: float) -> None:
        k = bisect.bisect_right(self.starts, t) - 1
        sec = self.active[k]
        if sec is self.approach:
            return
        if self.stamp is None:
            self.stamp = datetime.now()
        u = min(max((t - self.starts[k]) / sec.duration, 0.0), 1.0)
        depth = float(np.linalg.norm(sec.begin + (sec.end - sec.begin) * u - self.origin))
        states = self.real_ctrl.read_joint_states()[:7]
        self.rows.append((t - self.t_start, self.number[id(sec)], depth * 1000,
                          [s.position_deg for s in states] + [s.torque for s in states]))

    def save(self) -> None:
        if not self.rows:
            print(f"[line] {self.side} torque log: --start was never reached -- nothing saved")
            return
        a = self.args
        names = JOINT_NAMES[:7]
        path = os.path.join(TORQUE_LOG_DIR, f"{self.stamp:%Y%m%d_%H%M%S}_{self.side}.csv")
        os.makedirs(TORQUE_LOG_DIR, exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(f"{_TORQUE_LOG_MAGIC}\n")
            f.write(f"# created: {self.stamp.isoformat(timespec='seconds')}\n")
            f.write(f"# arm: {self.side}\n")
            f.write(f"# line: --start {' '.join(f'{v:g}' for v in a.start)} "
                    f"--angles {a.angles[0]:g} {a.angles[1]:g} --depth {a.depth:g} "
                    f"--speeds {' '.join(f'{v:g}' for v in a.speeds)} --split {a.split:g} "
                    f"--orient {a.orient}\n")
            f.write(f"# rate_hz: {self.rate_hz:g}\n")
            f.write(f"# gains: kp {' '.join(f'{v:g}' for v in a.kp)}  "
                    f"kd {' '.join(f'{v:g}' for v in a.kd)}\n")
            f.write(",".join(["t_s", "section", "cmd_depth_mm"]
                             + [f"{n}_deg" for n in names] + [f"{n}_nm" for n in names]) + "\n")
            for t, section, depth_mm, values in self.rows:
                f.write(f"{t:.4f},{section},{depth_mm:.3f},"
                        + ",".join(f"{v:.4f}" for v in values) + "\n")
        os.replace(tmp, path)
        print(f"[line] {self.side} torque log: {len(self.rows)} samples from --start "
              f"({self.rows[-1][0]:.2f}s) -> {path}")


def cmd_line(session: Session, args) -> None:
    if args.depth <= 0:
        print("Error: --depth must be positive")
        return
    if not 0.0 < args.split < 1.0:
        print("Error: --split must be between 0 and 1 (exclusive)")
        return
    if any(s <= 0 for s in args.speeds):
        print("Error: --speeds must all be positive")
        return
    if args.plan_step <= 0:
        print("Error: --plan-step must be positive")
        return
    if args.rate is not None and args.rate <= 0:
        print("Error: --rate must be positive")
        return

    want_real = args.mode in ("real", "both")
    if want_real and args.limits != "real":
        print("Error: --limits sim can't be combined with --mode real/both -- the solve must be "
              "constrained to the real arm's limits, or it can return angles the arm can't reach")
        return

    s1, s2, s3 = args.speeds
    if not s1 > s2 > s3:
        print(f"Note: speeds {s1} {s2} {s3} are not strictly decreasing -- "
              f"section 1 > 2 > 3 is the usual staging.")

    # Real paths stream at the configured control rate like every other real move;
    # sim only needs to look smooth.
    rate = args.rate if args.rate is not None else (CTRL_HZ if want_real else SIM_RATE_HZ)
    label = {"sim": "sim", "real": "real", "both": "real+sim"}[args.mode]

    direction = direction_from_angles(*args.angles)
    start = np.array(args.start, dtype=float)
    mid = start + direction * args.depth * args.split
    end = start + direction * args.depth

    print(f"[line] tilt={args.angles[0]} deg  rot={args.angles[1]} deg  "
          f"-> direction {np.array2string(direction, precision=4)}")
    print(f"       start        {np.array2string(start, precision=4)}")
    print(f"       {args.split:>4.0%} of depth {np.array2string(mid, precision=4)}")
    print(f"       full depth   {np.array2string(end, precision=4)}")
    oriented = args.orient == "path"
    print("       tool axis    " + ("held parallel to the line (roll free)" if oriented
                                    else "free (position-only IK)"))

    for side in resolve_sides(args.arm):
        km = ArmKinematics(side, limit_source=args.limits)

        # Plan from the configuration the real move will start from (ramp_start);
        # planning the approach from the sim's pose would aim the first real
        # command wherever sim happens to be.
        if want_real:
            real_ctrl = session.real(side)
            q_now, source = ramp_start(real_ctrl)
            q_now, source = np.array(q_now), f"real arm {source}"
        else:
            sim_ctrl = session.sim(side, args.sim_host, args.sim_port)
            q_now, source = np.array(sim_ctrl.read_joint_positions()), "sim position"
        current_xyz = np.array(km.fkine(np.degrees(q_now)).t)
        print(f"[{side}] current EE {np.array2string(current_xyz, precision=4)} (from {source})")

        line_axis = direction if oriented else None
        sections = [
            Section("1 approach", current_xyz, start, s1, joint_space=oriented),
            Section(f"2 to {args.split:.0%}", start, mid, s2, line_axis),
            Section("3 to depth", mid, end, s3, line_axis),
        ]

        # Solve the entire path before anything moves.
        if oriented:
            tool_now = km.tool_axis(np.degrees(q_now))
            print(f"[{side}] tool axis now {np.array2string(tool_now, precision=4)}, "
                  f"{math.degrees(axis_angle(tool_now, direction)):.1f} deg off the line")
            print(f"    planning 2..3 with the tool on the line: "
                  f"{sum(sec.knot_count(args.plan_step) for sec in sections[1:])} knots over "
                  f"{args.depth * 1000:.1f}mm; approach = joint-space move to its start "
                  f"(furthest joint at most {APPROACH_JOINT_SPEED_DEG:g} deg/s)", flush=True)
            failure = _plan_oriented(km, sections, q_now, args.plan_step)
            approach = sections[0]
            if not failure and approach.length < 1e-6 and approach.duration < 1e-3:
                print("    approach: already there, skipped")
                approach.knots = []
        else:
            failure = None
            q_seed = q_now
            for sec in sections:
                if sec.length < 1e-6:
                    print(f"    planning {sec.name}: zero length, skipped")
                    sec.q_begin = q_seed
                    continue
                print(f"    planning {sec.name}: {sec.knot_count(args.plan_step)} knots "
                      f"over {sec.length * 1000:.1f}mm...", flush=True)
                q_seed, failure = _plan_section(km, sec, q_seed, args.plan_step)
                if failure:
                    break

        print(f"[{side}] plan:")
        print(f"      {'section':<14}{'length':>11}{'speed':>12}{'time':>9}{'knots':>7}")
        for sec in sections:
            print(f"      {sec.name:<14}{sec.length * 1000:>9.1f}mm"
                  f"{sec.speed * 1000:>9.1f}mm/s{sec.duration:>8.2f}s{len(sec.knots):>7}")

        if failure:
            print(f"    [{side}] path not reachable -- {failure}")
            if oriented:
                print(f"    [{side}] (--orient free drops the tool-orientation constraint)")
            print(f"    [{side}] nothing moved.")
            continue

        total_time = sum(sec.duration for sec in sections)
        total_knots = sum(len(sec.knots) for sec in sections)
        print(f"      {'total':<14}{'':>11}{'':>12}{total_time:>8.2f}s{total_knots:>7}")

        if args.dry_run:
            print(f"    [{side}] --dry-run, not moving.")
            continue

        mirror, torque_log = None, None
        if want_real:
            q_start, _ = ramp_start(real_ctrl)
            q_start = np.array(q_start)
            drift = float(np.degrees(np.max(np.abs(q_start - q_now))))
            if drift > _MAX_DRIFT_DEG:
                print(f"    [{side}] arm moved {drift:.2f} deg while planning (max {_MAX_DRIFT_DEG}) "
                      f"-- nothing moved; re-run to plan from where it is now.")
                continue
            print(f"    [{side}] arm drift since planning: {drift:.2f} deg")
            warn_if_start_out_of_range(list(q_start), real_ctrl.arm_limits_rad)
            # Stream from the real starting point, so any drift is absorbed into
            # the first segment instead of arriving as a step.
            next(sec for sec in sections if sec.knots).q_begin = q_start

            if args.mode == "both":
                print(f"[sim] connecting mirror to CoppeliaSim at {args.sim_host}:{args.sim_port} "
                      f"for {side} arm...")
                mirror = SimMirror(side, args.sim_host, args.sim_port)
            send = real_sender(real_ctrl, args.kp, args.kd, mirror)
            measure = functools.partial(_read_real, real_ctrl)
            torque_log = _TorqueLog(real_ctrl, sections, side, args, rate)
            mirror_note = (f", sim mirrored from its own thread at {MIRROR_RATE_HZ:.0f} Hz"
                           if mirror is not None else "")
            print(f"[{label}] {side}: streaming at {rate:.0f} Hz{mirror_note}; "
                  f"joint torques logged from --start on")
        else:
            send = sim_sender(sim_ctrl)
            measure = functools.partial(lambda c: np.array(c.read_joint_positions()), sim_ctrl)

        try:
            with mirror if mirror is not None else contextlib.nullcontext():
                marks, max_gap = _execute(sections, rate, send, measure, label, side,
                                          torque_log.sample if torque_log is not None else None)
        finally:
            # Saved even when Ctrl+C or an error cuts the line short.
            if torque_log is not None:
                torque_log.save()

        _report(km, sections, marks, max_gap, rate, end, side)
        if mirror is not None:
            print(f"      sim mirror: {mirror.updates} updates, {mirror.skipped} skipped")
            if mirror.error is not None:
                print(f"      WARNING sim mirror stopped early ({mirror.error}) -- "
                      f"the real arm was unaffected")

    print("Done.")
