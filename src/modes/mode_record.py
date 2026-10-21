"""
Record mode -- move the real arm by hand and save the motion, for `replay`.

After a countdown the seven arm joints go limp (kp=0; the gripper is left
alone) and are sampled at --rate until Enter, Ctrl+C or --duration. The arm is
then held where it stopped, and the take -- minus the still stretches at either
end -- is saved as a recording (recording.py). Full reference:
docs/07_main_script.md.
"""
import contextlib
import math
import os
import select
import shlex
import sys
import termios
import time
from datetime import datetime

import numpy as np

from config import JOINT_NAMES
from motion import clamp_to_limits, warn_if_start_out_of_range
from recording import (
    REPLAY_MAX_SPEED_DEG_S, STILL_TOL_DEG, Recording, out_of_range, peak_speed,
    recording_path, save, speed_factor_within, trim_stillness,
)
from session import Session, resolve_sides
from streaming import MIRROR_RATE_HZ, SimMirror

_STATUS_EVERY_S = 0.25


def _stdin_is_terminal() -> bool:
    try:
        return sys.stdin.isatty()
    except (AttributeError, ValueError):
        return False


def _enter_pressed() -> bool:
    """Non-blocking. The terminal is line-buffered while a command runs, so
    stdin only turns readable once Enter is pressed."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def _discard_typed_ahead() -> None:
    termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)


def _countdown(seconds: float) -> None:
    remaining = seconds
    while remaining > 1e-6:
        whole = math.ceil(remaining)
        print(f" {whole}...", end="", flush=True)
        step = remaining - (whole - 1)
        time.sleep(step)
        remaining -= step
    print()


def _sample(real: dict, mirrors: dict, rate_hz: float, duration_s: float | None,
            watch_enter: bool):
    """Read every arm at rate_hz until something stops the take. Returns
    (times, {side: positions}, what stopped it, the exception if one did)."""
    sides = list(real)
    times, positions = [], {side: [] for side in sides}
    stopped_by, error = None, None
    hint = ("Enter" if watch_enter else "Ctrl+C") + " to stop"
    if duration_s is not None:
        hint = f"stops at {duration_s:g} s" + (", or Enter" if watch_enter else "")
    status_line = sys.stdout.isatty()
    shown = False
    period = 1.0 / rate_hz

    if watch_enter:
        _discard_typed_ahead()   # an Enter pressed during the countdown must not end the take at once
    t0 = time.monotonic()
    tick, next_status = 0, 0.0
    try:
        while True:
            t = time.monotonic() - t0
            if duration_s is not None and t >= duration_s:
                stopped_by = f"--duration {duration_s:g}"
                break
            sample = {side: [s.position_rad for s in real[side].read_joint_states()[:7]]
                      for side in sides}
            times.append(t)
            for side in sides:
                positions[side].append(sample[side])
                if side in mirrors:
                    # Sim refuses poses outside the configured limits, and a hand-moved
                    # arm can go past them: show it pinned at the limit, not frozen.
                    mirrors[side].latest = clamp_to_limits(sample[side], real[side].arm_limits_rad)
            if watch_enter and _enter_pressed():
                stopped_by = "Enter"
                break
            if status_line and t >= next_status:
                sys.stdout.write(f"\r[record] {t:6.1f} s  {len(times):5d} samples  ({hint})\033[K")
                sys.stdout.flush()
                shown, next_status = True, t + _STATUS_EVERY_S
            # A late tick moves the schedule on rather than taking catch-up samples.
            tick = max(tick + 1, int((time.monotonic() - t0) / period) + 1)
            sleep = t0 + tick * period - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        stopped_by = "Ctrl+C"
    except Exception as e:
        stopped_by, error = "an error", e
    if shown and stopped_by != "Enter":   # Enter's echo already moved to a new line
        print()
    if watch_enter:
        _discard_typed_ahead()
    n = min(len(times), *(len(p) for p in positions.values()))
    return times[:n], {side: p[:n] for side, p in positions.items()}, stopped_by, error


def _hold(real: dict, kp, kd) -> None:
    for side, ctrl in real.items():
        try:
            q = [s.position_rad for s in ctrl.read_joint_states()[:7]]
            warn_if_start_out_of_range(q, ctrl.arm_limits_rad)
            ctrl.move_joints(clamp_to_limits(q, ctrl.arm_limits_rad), kp=kp, kd=kd)
            print(f"[real] holding {side} where it stopped (kp {kp}, kd {kd})")
        except Exception as e:
            print(f"[real] WARNING couldn't hold {side} ({e}) -- it is still limp, keep supporting it")


def _summary(rec: Recording, limits: dict) -> None:
    names = JOINT_NAMES[:7]
    for side in rec.arms:
        head = f"[{side}] "
        pad = " " * len(head)
        deg = np.degrees(rec.q[side])
        print(f"{head}{'':<8}" + "".join(f"{n:>8}" for n in names))
        print(f"{pad}{'min deg':<8}" + "".join(f"{v:>8.1f}" for v in deg.min(axis=0)))
        print(f"{pad}{'max deg':<8}" + "".join(f"{v:>8.1f}" for v in deg.max(axis=0)))
        peak, joint, at = peak_speed(rec.t, rec.q[side])
        print(f"{head}peak joint speed {peak:.1f} deg/s ({names[joint]}, {at:.2f} s in)")
        if peak > REPLAY_MAX_SPEED_DEG_S:
            print(f"{head}NOTE that's over replay's default --max-speed of {REPLAY_MAX_SPEED_DEG_S:g} deg/s: "
                  f"on the real arm, replay it with --speed "
                  f"{speed_factor_within(REPLAY_MAX_SPEED_DEG_S, peak):g} or lower, or re-record")
        for joint, excess, count in out_of_range(rec.q[side], limits[side]):
            lo, hi = (math.degrees(v) for v in limits[side][joint])
            print(f"{head}NOTE {names[joint]} went up to {excess:.2f} deg past its limit "
                  f"[{lo:.1f}, {hi:.1f}] ({count} samples) -- replay clamps it to the limit")


def cmd_record(session: Session, args) -> None:
    if args.rate <= 0:
        print("Error: --rate must be positive")
        return
    if args.duration is not None and args.duration <= 0:
        print("Error: --duration must be positive")
        return
    if args.countdown < 0:
        print("Error: --countdown can't be negative")
        return
    if not 0.0 <= args.damping <= 5.0:
        print("Error: --damping must be between 0 and 5 Nm·s/rad (the DM4310's kd range)")
        return

    name = args.name or datetime.now().strftime("rec_%Y%m%d_%H%M%S")
    path = recording_path(name)
    if os.path.exists(path) and not args.overwrite:
        print(f"Error: {path} already exists -- choose another --name, or add --overwrite to replace it")
        return

    sides = resolve_sides(args.arm)
    real = {side: session.real(side) for side in sides}
    watch_enter = _stdin_is_terminal()
    stops = ((["Enter"] if watch_enter else [])
             + ([f"--duration {args.duration:g} s"] if args.duration is not None else [])
             + ["Ctrl+C"])

    with contextlib.ExitStack() as stack:
        mirrors = {}
        if args.mode == "both":
            for side in sides:
                print(f"[sim] connecting mirror to CoppeliaSim at {args.sim_host}:{args.sim_port} "
                      f"for {side} arm...")
                mirrors[side] = stack.enter_context(SimMirror(side, args.sim_host, args.sim_port))

        print(f"[record] {'+'.join(sides)} at {args.rate:g} Hz -> {path}")
        print(f"[record] stops on {' / '.join(stops)} -- the take is saved either way")
        if args.countdown > 0:
            print(f"[record] support the arm{'s' if len(sides) > 1 else ''} now; joints go limp in",
                  end="", flush=True)
            try:
                _countdown(args.countdown)
            except KeyboardInterrupt:
                print("\n[record] cancelled -- nothing released, nothing recorded")
                return

        created = datetime.now().isoformat(timespec="seconds")
        times, positions, stopped_by, error = [], {}, None, None
        try:
            for side in sides:
                real[side].release_arm(kd=args.damping)
            damping = f" with damping {args.damping:g}" if args.damping > 0 else ""
            follow = f", sim follows live (up to {MIRROR_RATE_HZ:.0f} Hz)" if mirrors else ""
            print(f"[real] {'/'.join(sides)} limp{damping} -- move by hand{follow}")
            times, positions, stopped_by, error = _sample(real, mirrors, args.rate, args.duration,
                                                          watch_enter)
            ended_at = times[-1] if times else 0.0
            print(f"[record] stopped by {stopped_by} at {ended_at:.2f} s, {len(times)} samples")
        finally:
            _hold(real, args.kp, args.kd)

    for side, mirror in mirrors.items():
        if mirror.error is not None:
            print(f"[sim] WARNING {side} mirror stopped early ({mirror.error}) -- the recording is unaffected")

    if not times:
        if error is not None:
            raise error
        print("[record] nothing recorded -- no file written")
        return

    t = np.array(times)
    rec = Recording(t=t - t[0], q={side: np.array(positions[side]) for side in sides},
                    rate_hz=args.rate, created=created)
    if not args.no_trim:
        trimmed = trim_stillness(rec)
        if trimmed is None:
            print(f"[record] NOTE no joint moved more than {STILL_TOL_DEG:g} deg -- nothing trimmed")
        else:
            rec = trimmed
            print(f"[record] trimmed {rec.trimmed_s[0]:.2f} s of stillness from the start and "
                  f"{rec.trimmed_s[1]:.2f} s from the end, {rec.duration:.2f} s kept")
    save(rec, path)
    _summary(rec, {side: real[side].arm_limits_rad for side in sides})
    print(f"[record] saved {path}")
    if error is not None:
        print("[record] (the take ended early because of the error below; what was recorded is saved)")
        raise error
    print(f"[record] play it back with: replay --name {shlex.quote(name)}")
    print("Done.")
