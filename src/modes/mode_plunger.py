"""
Plunger mode -- a plunging experiment in four points, run one at a time
(--point N), each from wherever the previous one left the arm. The arm is
expected to start at all zeros.

  1  safe point     --deg D1..D7. Joints move one at a time, joint1 first and
                    joint7 last, each finishing before the next starts.
  2  joint3 -> 0    only joint3 moves, back to 0.
  3  plunge start   --xyz X Y Z with the tool pointing straight down (the tool
                    axis, kinematics.TOOL_AXIS = TCP -X, along robot_base -Z;
                    roll about it free). One joint-space move. --depth is
                    needed too: the configuration chosen at --xyz is one from
                    which the whole descent to point 4 plans continuously.
  4  plunge         straight down --depth from point 3, at --speed, tool held
                    straight down. From any pose: if the EE isn't at point 3
                    (--xyz) pointing down, it goes there first (point 3's
                    move), then plunges. Without --xyz it plunges from where
                    the EE is, if the tool points down.
  5  retract out    back out of the material: along the line the last guarded
                    point 4 left the needle on, else straight back along the
                    tool axis by --depth. Point 3 refuses to move while a
                    guarded point 4 has left the needle in.

Points 1-3 move the joints at --joint-speed (deg/s on average; minimum-jerk
ramps on the real arm). Point 4 is planned like a line-mode section and
streamed at the control rate.

On the real arm, point 4 records the measured joint angles and torques, every
tick from the start of the descent to full depth, to recordings/plunger/,
one CSV per arm per run, named by the time the descent started:

    recordings/plunger/<YYYYmmdd>_<HHMMSS>_<side>[_<name>].csv

    # openarm-plunger v2
    # created: 2026-10-06T14:30:05
    # arm: right
    # point3_m: 0.3 -0.35 0.25
    # depth_m: 0.03  speed_m_s: 0.002
    # rate_hz: 100
    # gains: kp 40 40 40 40 40 40 40  kd 0.5 0.5 0.5 0.5 0.5 0.5 0.5
    # marks_s: 6.2135 9.0410
    t_s,mark,cmd_depth_mm,meas_depth_mm,joint1_deg,...,joint7_deg,joint1_nm,...,joint7_nm

t_s counts from the start of the descent; cmd_depth_mm is how far below
point 3 the path had the EE then, meas_depth_mm how far the measured joint
angles put it (forward kinematics). numpy reads one with
`np.genfromtxt(path, delimiter=",", names=True, skip_header=8)`.

Marks: mark() -- the GUI's Mark button, e.g. pressed the moment the needle
touches the sample -- records the time since the descent started. marks_s
lists them exactly (or "none"); the mark column is the mark's number on the
first sample at or after it, 0 elsewhere.

With --guard log|on, point 4 runs with the needle load cell in the loop
instead (modes/insertion.py): tare, contact, bone detection, release and
retract, the occupancy map, and with --search the bone-avoidance search. Its
logs are v3 (force columns); see insertion.py. --guard off (the default) is
exactly the descent above.
"""
import contextlib
import functools
import math
import os
import time
from datetime import datetime

import numpy as np

import axis_hold
import insertion
from config import AXIS_HOLD_CFG, CTRL_HZ, JOINT_NAMES, REAL_ARM_LIMITS_RAD, RECORDINGS_DIR
from kinematics import ArmKinematics, TOOL_AXIS
from mode_line import Section, _execute, _plan_oriented, _plan_section, _read_real, _report, axis_angle
from motion import (
    clamp_to_limits, drive_real_and_mirror_sim, ramp_real, ramp_start, validate_target,
    warn_if_start_out_of_range,
)
from session import Session, resolve_sides
from streaming import MIRROR_RATE_HZ, SIM_RATE_HZ, SimMirror, real_sender, sim_sender

DOWN = np.array([0.0, 0.0, -1.0])

# A joint-space move never takes less than this, however small.
MIN_MOVE_S = 1.0
# A joint already this close to its target isn't moved.
_SKIP_DEG = 0.05
# Point 4 refuses to start unless the EE is this close to point 3 and the tool
# this close to straight down.
_AT_POINT3_MM = 5.0
_AT_POINT3_DEG = 3.0
# As in line mode: refuse if the arm moved this much while point 4 was planned.
_MAX_DRIFT_DEG = 2.0

LOG_DIR = os.path.join(RECORDINGS_DIR, "plunger")
_LOG_MAGIC = "# openarm-plunger v2"

POINT_NAMES = {1: "safe point", 2: "joint3 to 0", 3: "plunge start", 4: "plunge", 5: "retract out"}


def _current_q(session: Session, side: str, args) -> tuple[np.ndarray, str]:
    """Where the next move starts: the real arm's (ramp_start) under real/both,
    else the sim's."""
    if args.mode in ("real", "both"):
        q, source = ramp_start(session.real(side))
        return np.array(q), f"real arm {source}"
    return np.array(session.sim(side, args.sim_host, args.sim_port).read_joint_positions()), "sim position"


def _measured_q(session: Session, side: str, args) -> tuple[np.ndarray, str]:
    """Where the arm really is: its encoders under real/both, else the sim.
    Points 3 and 4 plan from this (not from the setpoint the motors hold)."""
    if args.mode in ("real", "both"):
        return _read_real(session.real(side)), "real arm measured position"
    return np.array(session.sim(side, args.sim_host, args.sim_port).read_joint_positions()), "sim position"


def _move(session: Session, side: str, args, target_rad, duration: float) -> None:
    """One joint-space move of every joint at once, as joint mode drives it."""
    target_rad = [float(v) for v in target_rad]
    if args.mode == "both":
        drive_real_and_mirror_sim(session.real(side), session.sim(side, args.sim_host, args.sim_port),
                                  target_rad, duration, kp=args.kp, kd=args.kd)
    elif args.mode == "real":
        ramp_real(session.real(side), target_rad, duration, kp=args.kp, kd=args.kd)
    else:
        session.sim(side, args.sim_host, args.sim_port).move_joints_over_time(target_rad, duration)


def _duration(turn_rad: float, joint_speed_deg: float) -> float:
    return max(MIN_MOVE_S, math.degrees(turn_rad) / joint_speed_deg)


def _joint_by_joint(session: Session, side: str, args, label: str, target_rad) -> None:
    """Points 1 and 2: joint1..joint7 in turn, each to its target while the
    others hold."""
    limits = REAL_ARM_LIMITS_RAD[side]
    validate_target(list(target_rad), limits)
    q_now, source = _current_q(session, side, args)
    print(f"[{side}] starting from the {source} (deg): {np.array2string(np.degrees(q_now), precision=2)}")
    # Every intermediate target is validated in full, so a joint that starts a
    # hair outside its range is aimed at the boundary until its own turn.
    q = clamp_to_limits(list(q_now), limits)
    for j in range(7):
        delta = target_rad[j] - q[j]
        if abs(math.degrees(delta)) < _SKIP_DEG:
            continue
        duration = _duration(abs(delta), args.joint_speed)
        print(f"[{label}] {side} {JOINT_NAMES[j]}: {math.degrees(q[j]):.2f} -> "
              f"{math.degrees(target_rad[j]):.2f} deg over {duration:.1f}s")
        q[j] = target_rad[j]
        _move(session, side, args, q, duration)


def _point3(session: Session, side: str, args, label: str) -> np.ndarray | None:
    """Go to point 3 from wherever the arm is. Returns the configuration there
    (with --dry-run: the planned one, nothing moved), or None if it can't."""
    if side in insertion.NEEDLE_IN and not args.dry_run:
        print(f"    [{side}] the needle is still in the material from the last point 4 -- run point 5 "
              f"(retract out) first. Nothing moved.")
        return None
    km = ArmKinematics(side, limit_source=args.limits)
    q_now, source = _measured_q(session, side, args)
    start = np.array(args.xyz, dtype=float)
    end = start + DOWN * args.depth
    current_xyz = np.array(km.fkine(np.degrees(q_now)).t)
    print(f"[{side}] current EE {np.array2string(current_xyz, precision=4)} (from {source}), "
          f"tool {math.degrees(axis_angle(km.tool_axis(np.degrees(q_now)), DOWN)):.1f} deg off "
          f"straight down")
    print(f"    planning point 3 {np.array2string(start, precision=4)} with the tool straight down, "
          f"from which the {args.depth * 1000:.1f}mm descent plans continuously...", flush=True)
    # The approach section only carries the chosen configuration (knots[0]); it
    # is driven below as a minimum-jerk ramp, not streamed.
    sections = [Section("approach", current_xyz, start, 1.0, joint_space=True),
                Section("descent", start, end, args.speed, DOWN)]
    failure = _plan_oriented(km, sections, q_now, args.plan_step)
    if failure:
        print(f"    [{side}] point 3 not reachable -- {failure}")
        print(f"    [{side}] nothing moved.")
        return None
    q_target = sections[0].knots[0]
    turn = np.abs(q_target - q_now)
    duration = _duration(float(np.max(turn)), args.joint_speed)
    print(f"    q at point 3 (deg) = {np.array2string(np.degrees(q_target), precision=2)}")
    print(f"    max joint change {np.degrees(np.max(turn)):.2f} deg "
          f"({JOINT_NAMES[int(np.argmax(turn))]}) -> {duration:.1f}s")
    if args.dry_run:
        print(f"    [{side}] --dry-run, not moving.")
        return q_target
    print(f"[{label}] moving {side} to point 3 over {duration:.1f}s...")
    _move(session, side, args, q_target, duration)
    return q_target


# The log of the descent running now, for mark(); set and cleared by _point4.
_active_log = None


def mark() -> tuple[int, float, float] | None:
    """Mark this moment in the running descent's log. Safe to call from any
    thread (the GUI's). Returns (mark number, s since the descent started,
    commanded depth mm), or None if no descent is being recorded."""
    log = _active_log or insertion.active_log()
    return log.mark() if log is not None else None


class _PlungeLog:
    """The real arm's measured joint angles and torques, one sample per tick
    of the descent. Held in memory so the tick loop does no file I/O; save()
    writes the file."""

    def __init__(self, real_ctrl, km: ArmKinematics, section: Section, side: str, args,
                 rate_hz: float, hold: axis_hold.AxisHold | None = None):
        self.real_ctrl = real_ctrl
        self.hold = hold
        self.km = km
        self.section = section
        self.side = side
        self.args = args
        self.rate_hz = rate_hz
        self.stamp: datetime | None = None
        self.rows = []
        self.marks: list[float] = []
        # time.monotonic() at schedule time 0, so a mark from another thread
        # lands on the same clock as the samples.
        self._t0: float | None = None

    def _depth_mm(self, t: float) -> float:
        return min(max(t / self.section.duration, 0.0), 1.0) * self.section.length * 1000

    def sample(self, t: float) -> None:
        if self.stamp is None:
            self.stamp = datetime.now()
            self._t0 = time.monotonic() - t
        states = self.real_ctrl.read_joint_states()[:7]
        self.rows.append((t, self._depth_mm(t),
                          [s.position_deg for s in states], [s.torque for s in states],
                          self.hold.correction_deg if self.hold is not None else 0.0))

    def mark(self) -> tuple[int, float, float] | None:
        if self._t0 is None:
            return None
        t = time.monotonic() - self._t0
        self.marks.append(t)
        return len(self.marks), t, self._depth_mm(t)

    def save(self) -> None:
        if not self.rows:
            print(f"[plunger] {self.side} log: the descent never started -- nothing saved")
            return
        a = self.args
        names = JOINT_NAMES[:7]
        origin = self.section.begin
        suffix = f"_{a.name}" if a.name else ""
        path = os.path.join(LOG_DIR, f"{self.stamp:%Y%m%d_%H%M%S}_{self.side}{suffix}.csv")
        os.makedirs(LOG_DIR, exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(f"{_LOG_MAGIC}\n")
            f.write(f"# created: {self.stamp.isoformat(timespec='seconds')}\n")
            f.write(f"# arm: {self.side}\n")
            f.write(f"# point3_m: {' '.join(f'{v:.5f}' for v in origin)}\n")
            f.write(f"# depth_m: {a.depth:g}  speed_m_s: {a.speed:g}\n")
            f.write(f"# rate_hz: {self.rate_hz:g}  axis_hold: "
                    f"{'on' if self.hold is not None and self.hold.adaptive else 'off'}  start: measured\n")
            f.write(f"# gains: kp {' '.join(f'{v:g}' for v in a.kp)}  "
                    f"kd {' '.join(f'{v:g}' for v in a.kd)}\n")
            marks = sorted(self.marks)
            f.write(f"# marks_s: {' '.join(f'{m:.4f}' for m in marks) or 'none'}\n")
            f.write(",".join(["t_s", "mark", "cmd_depth_mm", "meas_depth_mm"]
                             + [f"{n}_deg" for n in names] + [f"{n}_nm" for n in names]
                             + ["tilt_deg", "axis_hold_deg"]) + "\n")
            pending = 0   # marks[pending] is the next one still to be placed on a row
            for t, cmd_depth_mm, deg, tau, hold_deg in self.rows:
                number = 0
                while pending < len(marks) and marks[pending] <= t:
                    pending += 1
                    number = pending
                T = self.km.fkine(deg)
                meas = float(np.dot(np.array(T.t) - origin, DOWN)) * 1000
                tilt = math.degrees(axis_angle(T.R @ TOOL_AXIS, DOWN))
                f.write(f"{t:.4f},{number},{cmd_depth_mm:.3f},{meas:.3f},"
                        + ",".join(f"{v:.4f}" for v in deg + tau) + f",{tilt:.3f},{hold_deg:.3f}\n")
        os.replace(tmp, path)
        if len(marks) > pending:
            print(f"[plunger] {self.side} log: {len(marks) - pending} mark(s) after the last "
                  f"sample are in marks_s only")
        print(f"[plunger] {self.side} log: {len(self.rows)} samples over "
              f"{self.rows[-1][0]:.2f}s saved -> {path}")


def _point4(session: Session, side: str, args, label: str) -> bool:
    """Point 4. True when the plunge ran (or, with --dry-run, planned), False
    when it was refused -- so a sequence (the GUI's Run 1 -> 4) can stop."""
    want_real = args.mode in ("real", "both")
    rate = args.rate if args.rate is not None else (CTRL_HZ if want_real else SIM_RATE_HZ)
    km = ArmKinematics(side, limit_source=args.limits)

    def where_now():
        """(measured q and its source, and the commanded pose: tip, tilt)
        -- "at point 3?" is judged on what point 3 commanded; the arm sits a
        few mm off that under gravity, and the descent is planned from there."""
        qm, src = _measured_q(session, side, args)
        Tc = km.fkine(np.degrees(_current_q(session, side, args)[0]))
        return qm, src, np.array(Tc.t), math.degrees(axis_angle(Tc.R @ TOOL_AXIS, DOWN))

    q_now, source, cmd_tip, tilt = where_now()
    off_mm = float(np.linalg.norm(cmd_tip - np.array(args.xyz))) * 1000 if args.xyz else 0.0
    print(f"[{side}] commanded EE {np.array2string(cmd_tip, precision=4)}, tool {tilt:.2f} deg off "
          f"straight down")
    if off_mm > _AT_POINT3_MM or tilt > _AT_POINT3_DEG:
        where = (f"EE {off_mm:.1f}mm from point 3 (max {_AT_POINT3_MM:g}), tool {tilt:.1f} deg off "
                 f"straight down (max {_AT_POINT3_DEG:g})")
        if args.xyz is None:
            print(f"    [{side}] {where} -- give --xyz (point 3) to go there first. Nothing moved.")
            return False
        # From anywhere: point 3 first (its joint-space move, tool turned
        # straight down), then the plunge from there.
        print(f"    [{side}] {where} -- going to point 3 first")
        q3 = _point3(session, side, args, label)
        if q3 is None:
            return False
        if args.dry_run:
            q_now, source = q3, "planned point 3"
            T3 = km.fkine(np.degrees(q3))
            cmd_tip, tilt = np.array(T3.t), math.degrees(axis_angle(T3.R @ TOOL_AXIS, DOWN))
        else:
            q_now, source, cmd_tip, tilt = where_now()
        off_mm = float(np.linalg.norm(cmd_tip - np.array(args.xyz))) * 1000
        print(f"[{side}] at point 3: commanded EE {np.array2string(cmd_tip, precision=4)}, "
              f"tool {tilt:.2f} deg off straight down")
        if off_mm > _AT_POINT3_MM or tilt > _AT_POINT3_DEG:
            print(f"    [{side}] didn't arrive at point 3 ({off_mm:.1f}mm, {tilt:.1f} deg off) -- "
                  f"not plunging.")
            return False

    # The descent starts where the needle really is: the measured position.
    T = km.fkine(np.degrees(q_now))
    start = np.array(T.t)
    print(f"[{side}] measured EE {np.array2string(start, precision=4)} (from {source}), tool "
          f"{math.degrees(axis_angle(T.R @ TOOL_AXIS, DOWN)):.2f} deg off straight down"
          + (f", {np.linalg.norm(start - np.array(args.xyz)) * 1000:.1f} mm from point 3 -- the line starts here"
             if args.xyz else ""))

    if args.guard != "off" and not args.dry_run:
        return insertion.run(session, side, args, km, q_now, label)

    end = start + DOWN * args.depth
    section = Section("descent", start, end, args.speed, DOWN)
    print(f"    planning the descent: {section.knot_count(args.plan_step)} knots over "
          f"{args.depth * 1000:.1f}mm at {args.speed * 1000:.2f}mm/s ({section.duration:.1f}s)...",
          flush=True)
    _, failure = _plan_section(km, section, q_now, args.plan_step)
    if failure:
        print(f"    [{side}] descent not reachable -- {failure}")
        print(f"    [{side}] nothing moved.")
        return False
    print(f"    point 4 {np.array2string(end, precision=4)}, "
          f"q (deg) = {np.array2string(np.degrees(section.knots[-1]), precision=2)}")
    if args.dry_run:
        print(f"    [{side}] --dry-run, not moving.")
        return True

    mirror, log, hold = None, None, None
    if want_real:
        real_ctrl = session.real(side)
        drift = float(np.degrees(np.max(np.abs(_read_real(real_ctrl) - q_now))))
        if drift > _MAX_DRIFT_DEG:
            print(f"    [{side}] arm moved {drift:.2f} deg while planning (max {_MAX_DRIFT_DEG}) "
                  f"-- nothing moved; re-run to plan from where it is now.")
            return False
        q_held = np.array(ramp_start(real_ctrl)[0])
        warn_if_start_out_of_range(list(q_held), real_ctrl.arm_limits_rad)
        section.q_begin = q_now
        if args.mode == "both":
            print(f"[sim] connecting mirror to CoppeliaSim at {args.sim_host}:{args.sim_port} "
                  f"for {side} arm...")
            mirror = SimMirror(side, args.sim_host, args.sim_port)
        send = real_sender(real_ctrl, args.kp, args.kd, mirror)
        measure = functools.partial(_read_real, real_ctrl)
        # Keep the needle on its line under load (motion/axis_hold.py). Off,
        # it still carries the start offset: the plan starts at the measured
        # position, the motors hold the setpoint a few mm away from it, and
        # (held - measured) as the starting correction makes the first command
        # what they already hold -- no jump at the start.
        hold = None if args.no_axis_hold else axis_hold.from_config(km, DOWN, AXIS_HOLD_CFG)
        if hold is None:
            hold = axis_hold.AxisHold(km, DOWN, adaptive=False)
        cut = hold.preload(q_held - q_now)
        if cut > 0.05:
            print(f"    [{side}] WARNING setpoint and measured position differ by more than the correction "
                  f"limit ({cut:.1f} deg over) -- expect a small jump at the start")
        send = axis_hold.hold_sender(measure, send, hold, 1.0 / rate)
        log = _PlungeLog(real_ctrl, km, section, side, args, rate, hold)
        mirror_note = (f", sim mirrored from its own thread at {MIRROR_RATE_HZ:.0f} Hz"
                       if mirror is not None else "")
        print(f"[{label}] {side}: streaming at {rate:.0f} Hz{mirror_note}; "
              f"axis hold {'on' if hold.adaptive else 'off'}; joint angles and torques recorded")
    else:
        sim_ctrl = session.sim(side, args.sim_host, args.sim_port)
        send = sim_sender(sim_ctrl)
        measure = functools.partial(lambda c: np.array(c.read_joint_positions()), sim_ctrl)
        print(f"[{label}] {side}: streaming at {rate:.0f} Hz (sim has no torques -- nothing recorded)")

    # Remember the last planned q sent, so a needle left in the material
    # (the end of the descent, or wherever Stop cut it) is on record: points 3
    # and 4 then won't drag it sideways, and point 5 backs it out along the line.
    last_plan: dict = {"q": None}
    inner_send = send

    def send(q):
        last_plan["q"] = np.asarray(q, dtype=float)
        inner_send(q)

    global _active_log
    _active_log = log
    try:
        with mirror if mirror is not None else contextlib.nullcontext():
            marks, max_gap = _execute([section], rate, send, measure, label, side,
                                      log.sample if log is not None else None)
    finally:
        _active_log = None
        # Saved even when Stop or an error cuts the descent short.
        if log is not None:
            log.save()
        if last_plan["q"] is not None:
            reached = float(np.dot(np.array(km.robot.fkine(last_plan["q"]).t) - section.begin, DOWN))
            if reached > 0.0005:
                tau_end = min(max(reached / section.length, 0.0), 1.0) * section.duration
                insertion.NEEDLE_IN[side] = (section, tau_end, hold.c.copy() if hold is not None else None)
                print(f"[{label}] {side}: the needle is {reached * 1000:.1f} mm down the line -- point 5 "
                      f"backs it out along it (points 3 and 4 won't move until then)")

    _report(km, [section], marks, max_gap, rate, end, side)
    if hold is not None:
        print(f"      {hold.summary()}")
    if mirror is not None:
        print(f"      sim mirror: {mirror.updates} updates, {mirror.skipped} skipped")
        if mirror.error is not None:
            print(f"      WARNING sim mirror stopped early ({mirror.error}) -- "
                  f"the real arm was unaffected")
    return True


def cmd_plunger(session: Session, args) -> bool:
    """True when the point completed on every arm, False when it was refused."""
    p = args.point
    if p == 1 and args.deg is None:
        print("Error: point 1 needs --deg (7 joint angles)")
        return False
    if p == 3 and args.xyz is None:
        print("Error: point 3 needs --xyz")
        return False
    if p in (3, 4) and (args.depth is None or args.depth <= 0):
        print(f"Error: point {p} needs a positive --depth")
        return False
    if args.speed <= 0 or args.joint_speed <= 0 or args.plan_step <= 0:
        print("Error: --speed, --joint-speed and --plan-step must be positive")
        return False
    if args.rate is not None and args.rate <= 0:
        print("Error: --rate must be positive")
        return False
    if args.mode in ("real", "both") and args.limits != "real":
        print("Error: --limits sim can't be combined with --mode real/both -- the solve must be "
              "constrained to the real arm's limits, or it can return angles the arm can't reach")
        return False
    if args.dry_run and p in (1, 2):
        print("Error: --dry-run applies to points 3 and 4 only")
        return False
    if args.search and args.guard != "on":
        print("Error: --search needs --guard on")
        return False
    if args.search and args.on_bone == "hold":
        print("Error: --search retracts after every bone hit; it can't be combined with --on-bone hold")
        return False
    if args.insert_speed is not None and args.insert_speed <= 0:
        print("Error: --insert-speed must be positive")
        return False

    label = {"sim": "sim", "real": "real", "both": "real+sim"}[args.mode]
    print(f"[plunger] point {p}: {POINT_NAMES[p]}")
    for side in resolve_sides(args.arm):
        if p in (1, 2):
            if p == 1:
                target = [math.radians(d) for d in args.deg]
            else:
                target = clamp_to_limits(list(_current_q(session, side, args)[0]),
                                         REAL_ARM_LIMITS_RAD[side])
                target[2] = 0.0
            try:
                _joint_by_joint(session, side, args, label, target)
            except ValueError as e:
                print(f"Error: {e} -- {side} arm not moved")
                return False
        elif p == 3:
            if _point3(session, side, args, label) is None:
                return False
        elif p == 4:
            if not _point4(session, side, args, label):
                return False
        else:
            insertion.retract(session, side, args, label)
    print("Done.")
    return True
