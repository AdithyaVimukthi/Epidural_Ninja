"""
Replay mode -- play back a recording made by `record`, on sim, the real arm, or
both.

Each arm first ramps in joint space from where it is to the recording's first
pose (--approach), then follows the recorded samples against wall-clock time,
--speed times as fast. Arms recorded together play together, in the same
ticks. Full reference: docs/07_main_script.md.
"""
import contextlib
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from config import CTRL_HZ, JOINT_NAMES, REAL_ARM_LIMITS_RAD
from motion import ramp_start, warn_if_start_out_of_range
from recording import (
    RECORDINGS_DIR, RecordingError, list_recordings, load, out_of_range, peak_speed,
    recording_path, speed_factor_within,
)
from session import Session, resolve_sides
from sim_bridge import SimArmController
from streaming import MIRROR_RATE_HZ, SIM_RATE_HZ, SimMirror, real_sender, sim_sender

_STATUS_EVERY_S = 0.25


@dataclass
class _Track:
    """One arm's playback: its samples (clamped into range), where it starts
    from, and how to command and read it."""
    side: str
    t: np.ndarray                     # recording time of each sample, from 0
    q: np.ndarray                     # (N, 7) radians, inside the limits
    lo: np.ndarray
    hi: np.ndarray
    q_start: np.ndarray | None = None
    source: str = ""
    send: Callable | None = None
    measure: Callable | None = None
    sim_ctrl: SimArmController | None = None
    mirror: SimMirror | None = None
    q_end: np.ndarray | None = None
    max_gap: float = 0.0

    def q_at(self, t: float, approach_s: float, speed: float) -> np.ndarray:
        """Where the arm should be `t` seconds after starting: a straight
        joint-space ramp to the first sample, then the samples, interpolated."""
        if t < approach_s:
            return self.q_start + (self.q[0] - self.q_start) * (t / approach_s)
        tr = (t - approach_s) * speed
        i = int(np.searchsorted(self.t, tr, side="right"))
        if i >= len(self.t):
            return self.q[-1]
        a = (tr - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return self.q[i - 1] + (self.q[i] - self.q[i - 1]) * a


def _print_list() -> None:
    paths = list_recordings()
    if not paths:
        print(f"No recordings in {RECORDINGS_DIR} yet -- make one with `record`.")
        return
    print(f"Recordings in {RECORDINGS_DIR}:")
    print(f"  {'name':<28}{'arms':<12}{'length':>9}{'samples':>9}  created")
    for path in paths:
        name = os.path.splitext(os.path.basename(path))[0]
        try:
            rec = load(path)
        except RecordingError as e:
            print(f"  {name:<28}unreadable: {e}")
            continue
        print(f"  {name:<28}{'+'.join(rec.arms):<12}{rec.duration:>8.2f}s{len(rec):>9}  {rec.created}")


def _play(tracks: list[_Track], approach_s: float, playback_s: float, speed: float,
          rate_hz: float) -> None:
    """Stream every arm against wall-clock time -- a late tick sends where the
    path should be by now, never a burst of catch-up commands -- then read
    where each one ended up."""
    total = approach_s + playback_s
    period = 1.0 / rate_hz
    status_line = sys.stdout.isatty()
    last_sent: dict[str, float] = {}
    t0 = time.monotonic()
    tick, next_status = 0, 0.0
    try:
        while True:
            t = time.monotonic() - t0
            done = t >= total
            for tr in tracks:
                q = tr.q[-1] if done else tr.q_at(t, approach_s, speed)
                tr.send(np.clip(q, tr.lo, tr.hi))
                sent = time.monotonic()
                if tr.side in last_sent:
                    tr.max_gap = max(tr.max_gap, sent - last_sent[tr.side])
                last_sent[tr.side] = sent
            if status_line and (done or t >= next_status):
                phase = (f"approach {t:5.1f} / {approach_s:.1f} s" if t < approach_s else
                         f"playback {min(t, total) - approach_s:5.1f} / {playback_s:.1f} s")
                sys.stdout.write(f"\r[replay] {phase}\033[K")
                sys.stdout.flush()
                next_status = t + _STATUS_EVERY_S
            if done:
                break
            tick = max(tick + 1, int((time.monotonic() - t0) / period) + 1)
            sleep = t0 + tick * period - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
    finally:
        if status_line:
            print()
    for tr in tracks:
        tr.q_end = tr.measure()


def cmd_replay(session: Session, args) -> None:
    if args.list:
        _print_list()
        return
    if not args.name:
        print("Error: replay needs --name (--list shows the saved recordings)")
        return
    for flag, value in (("--speed", args.speed), ("--approach", args.approach),
                        ("--max-speed", args.max_speed)):
        if value <= 0:
            print(f"Error: {flag} must be positive")
            return
    if args.rate is not None and args.rate <= 0:
        print("Error: --rate must be positive")
        return

    path = recording_path(args.name)
    try:
        rec = load(path)
    except RecordingError as e:
        print(f"Error: {e}")
        return

    sides = rec.arms if args.arm is None else resolve_sides(args.arm)
    missing = [side for side in sides if side not in rec.arms]
    if missing:
        print(f"Error: {path} has no {' or '.join(missing)} arm -- it recorded {' and '.join(rec.arms)}")
        return

    want_real = args.mode in ("real", "both")
    rate = args.rate if args.rate is not None else (CTRL_HZ if want_real else SIM_RATE_HZ)
    label = {"sim": "sim", "real": "real", "both": "real+sim"}[args.mode]
    playback_s = rec.duration / args.speed
    only = f" -- playing {'+'.join(sides)} only" if set(sides) != set(rec.arms) else ""
    print(f"[replay] {path}")
    print(f"         {'+'.join(rec.arms)}, recorded {rec.created}, {rec.duration:.2f} s, "
          f"{len(rec)} samples{only}")

    # Checks on the recording itself, before connecting to anything.
    tracks, too_fast = [], False
    for side in sides:
        limits = REAL_ARM_LIMITS_RAD[side]
        lo = np.array([low for low, _ in limits])
        hi = np.array([high for _, high in limits])
        for joint, excess, count in out_of_range(rec.q[side], limits):
            print(f"[{side}] {JOINT_NAMES[joint]} goes up to {excess:.2f} deg past its limit "
                  f"[{math.degrees(lo[joint]):.1f}, {math.degrees(hi[joint]):.1f}] in {count} samples "
                  f"-- clamped to the limit")
        q = np.clip(rec.q[side], lo, hi)
        peak, joint, at = peak_speed(rec.t, q)
        print(f"[{side}] peak joint speed {peak * args.speed:.1f} deg/s at --speed {args.speed:g} "
              f"({JOINT_NAMES[joint]}, {at:.2f} s into the recording)")
        if peak * args.speed > args.max_speed:
            too_fast = True
            print(f"[{side}] that's over --max-speed {args.max_speed:g} deg/s; --speed "
                  f"{speed_factor_within(args.max_speed, peak):g} or lower stays under it")
        tracks.append(_Track(side, rec.t, q, lo, hi))
    if too_fast:
        if want_real:
            print("[replay] nothing moved. Slow it down with --speed, or raise --max-speed if that "
                  "motion is intended.")
            return
        print("[replay] sim only, so playing anyway -- the real arm would refuse this.")

    # Where each arm starts from: the real arm's held setpoint (ramp_start), or sim's pose.
    for tr in tracks:
        if want_real:
            ctrl = session.real(tr.side)
            q_start, source = ramp_start(ctrl)
            tr.source = f"real arm {source}"
            warn_if_start_out_of_range(q_start, ctrl.arm_limits_rad)
            tr.measure = lambda c=ctrl: np.array([s.position_rad for s in c.read_joint_states()[:7]])
        else:
            tr.sim_ctrl = session.sim(tr.side, args.sim_host, args.sim_port)
            q_start, tr.source = tr.sim_ctrl.read_joint_positions(), "sim position"
            tr.measure = lambda c=tr.sim_ctrl: np.array(c.read_joint_positions())
        tr.q_start = np.array(q_start, dtype=float)

        change = np.degrees(np.abs(tr.q[0] - tr.q_start))
        joint = int(np.argmax(change))
        approach_speed = change[joint] / args.approach
        print(f"[{tr.side}] approach from {tr.source}: up to {change[joint]:.1f} deg "
              f"({JOINT_NAMES[joint]}) in {args.approach:g} s, {approach_speed:.1f} deg/s")
        if approach_speed > args.max_speed:
            longer = math.ceil(change[joint] / args.max_speed * 10) / 10
            if want_real:
                print(f"[{tr.side}] that's over --max-speed {args.max_speed:g} deg/s -- nothing moved; "
                      f"use --approach {longer:g} or longer")
                return
            print(f"[{tr.side}] over --max-speed {args.max_speed:g} deg/s, which only the real arm enforces")

    with contextlib.ExitStack() as stack:
        for tr in tracks:
            if want_real:
                if args.mode == "both":
                    print(f"[sim] connecting mirror to CoppeliaSim at {args.sim_host}:{args.sim_port} "
                          f"for {tr.side} arm...")
                    tr.mirror = stack.enter_context(SimMirror(tr.side, args.sim_host, args.sim_port))
                tr.send = real_sender(session.real(tr.side), args.kp, args.kd, tr.mirror)
            else:
                tr.send = sim_sender(tr.sim_ctrl)
        mirror_note = (f", sim mirrored from its own thread at up to {MIRROR_RATE_HZ:.0f} Hz"
                       if args.mode == "both" else "")
        print(f"[{label}] {'+'.join(sides)}: {args.approach:g} s approach + {playback_s:.2f} s "
              f"playback at {rate:g} Hz{mirror_note}")
        _play(tracks, args.approach, playback_s, args.speed, rate)

    for tr in tracks:
        err = np.degrees(np.abs(tr.q_end - tr.q[-1]))
        joint = int(np.argmax(err))
        sag = " -- measured as playback ended, so it includes lag and gravity sag" if want_real else ""
        print(f"[{tr.side}] result:")
        print(f"      end pose error {err[joint]:.2f} deg ({JOINT_NAMES[joint]}){sag}")
        print(f"      longest gap between commands {tr.max_gap * 1000:.1f}ms "
              f"(tick period {1000 / rate:.1f}ms)")
        if tr.mirror is not None:
            print(f"      sim mirror: {tr.mirror.updates} updates, {tr.mirror.skipped} skipped")
            if tr.mirror.error is not None:
                print(f"      WARNING sim mirror stopped early ({tr.mirror.error}) -- "
                      f"the real arm was unaffected")
        if tr.sim_ctrl is not None:
            for line in tr.sim_ctrl.unreached([float(v) for v in tr.q[-1]]):
                print(f"[sim] WARNING {tr.side} {line}")
    print("Done.")
