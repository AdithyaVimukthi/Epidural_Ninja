"""
Axis hold: keep the needle on its line while it's pushed into the material.

The motors are position servos with kp of ~40 Nm/rad, i.e. springs: under the
insertion load each joint settles away from its command by load / kp. With
the needle ~93 mm to the side of the wrist axis, the push along it is a
moment on the wrist, so the tool tilts -- measured on the real arm on
2026-10-13: joint6 4.8 deg behind its command, the needle tilted 2-4 deg and
its tip 16-27 mm off the line by 25-40 mm deep, although the IK plan held it
exactly straight (0.000 deg). The plan is right; the arm doesn't follow it.

The fix is an outer integral loop on the measured pose. Every tick:

    e_lat  = sideways offset of the measured tip from the planned tip (the part
             perpendicular to the needle line)
    e_tilt = rotation taking the measured tool axis onto the planned one (no
             roll: rotation about the needle is free)
    dq     = damped least squares of [e_lat; e_tilt] through the Jacobian,
             each row projected the same way
    c     += ki * dt * dq        (rate- and size-limited, kept within joint limits)
    command = planned q + c

Motion *along* the needle is deliberately left alone. The arm stays a soft
spring in that direction, and the load-cell guard relies on that (contact,
bone stall, the release ramp). Only the tilt and the sideways drift are
corrected.

Real arm only: in sim the joints are exactly where they're told, so the
errors are zero and it does nothing.
"""
from __future__ import annotations

import math

import numpy as np

from kinematics import ArmKinematics, TOOL_AXIS


class AxisHold:
    def __init__(self, km: ArmKinematics, direction, ki_per_s: float = 2.0, max_deg: float = 8.0,
                 max_rate_deg_s: float = 5.0, damping: float = 0.02, adaptive: bool = True):
        """direction: the needle line's unit direction (robot_base). ki_per_s:
        how fast an error is taken out (1/s; 2 = ~0.5 s). max_deg: largest
        correction per joint. max_rate_deg_s: fastest it may change.
        adaptive=False: only carry the correction it was preloaded with (the
        axis hold switched off, the start offset still kept -- see preload)."""
        self.km = km
        self.adaptive = adaptive
        self.d = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
        self.ki = float(ki_per_s)
        self.max = math.radians(max_deg)
        self.max_rate = math.radians(max_rate_deg_s)
        self.lam2 = damping ** 2
        self.c = np.zeros(km.n)                # joint correction, rad
        self.tilt_deg = 0.0                    # latest measured tilt from the planned axis
        self.lateral_mm = 0.0                  # latest sideways offset of the tip from the plan
        self.peak_tilt_deg = 0.0
        self.peak_lateral_mm = 0.0
        self.peak_c_deg = 0.0
        self.saturated: list[int] = []         # joints (1-based) at a limit right now
        self.ever_saturated: set[int] = set()
        lo, hi = km.qlim
        self._lo, self._hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)

    def errors(self, q_plan, q_meas):
        """(sideways tip error, tool-axis rotation error) of the measured pose
        against the planned one, both perpendicular to what's left free."""
        robot = self.km.robot
        Tp, Tm = robot.fkine(q_plan), robot.fkine(q_meas)
        a_plan, a_meas = Tp.R @ TOOL_AXIS, Tm.R @ TOOL_AXIS
        e_p = np.asarray(Tp.t) - np.asarray(Tm.t)
        e_lat = e_p - self.d * float(np.dot(self.d, e_p))
        e_rot = np.cross(a_meas, a_plan)       # rotates the measured axis onto the planned one
        return e_lat, e_rot, a_meas

    def preload(self, c0) -> float:
        """Start from correction c0 (rad per joint): with the plan starting at
        the arm's MEASURED position, c0 = (setpoint the motors hold) - (measured)
        makes the first command exactly what they already hold -- no jump --
        and keeps the static sag compensated from there on. Returns the largest
        part of c0 that didn't fit under max_deg, in degrees (0 normally)."""
        c0 = np.asarray(c0, dtype=float)
        self.c = np.clip(c0, -self.max, self.max)
        return math.degrees(float(np.max(np.abs(c0 - self.c))))

    def set_direction(self, direction) -> None:
        """A new needle line (the next search attempt); the correction carries over."""
        self.d = np.asarray(direction, dtype=float) / np.linalg.norm(direction)

    def command(self, q_plan) -> np.ndarray:
        """What to send for this planned q: q_plan + the correction, within the joint limits."""
        q_plan = np.asarray(q_plan, dtype=float)
        return np.clip(q_plan + self.c, self._lo, self._hi)

    def update(self, q_plan, q_meas, dt: float) -> np.ndarray:
        """observe() then command() for the same q_plan."""
        self.observe(q_plan, q_meas, dt)
        return self.command(q_plan)

    def observe(self, q_plan, q_meas, dt: float) -> None:
        """Update the correction from a measurement: q_meas is where the arm is
        while it was commanded command(q_plan)."""
        q_plan = np.asarray(q_plan, dtype=float)
        q_meas = np.asarray(q_meas, dtype=float)
        e_lat, e_rot, a_meas = self.errors(q_plan, q_meas)
        self.lateral_mm = float(np.linalg.norm(e_lat)) * 1000
        self.tilt_deg = math.degrees(math.asin(min(1.0, float(np.linalg.norm(e_rot)))))
        self.peak_tilt_deg = max(self.peak_tilt_deg, self.tilt_deg)
        self.peak_lateral_mm = max(self.peak_lateral_mm, self.lateral_mm)
        if not self.adaptive:
            return
        if dt > 0:
            J = self.km.robot.jacob0(q_meas)
            P_lat = np.eye(3) - np.outer(self.d, self.d)       # free: along the needle line
            P_rot = np.eye(3) - np.outer(a_meas, a_meas)       # free: roll about the needle
            Jt = np.vstack([P_lat @ J[:3], P_rot @ J[3:]])
            e = np.concatenate([P_lat @ e_lat, P_rot @ e_rot])
            # A joint at its correction or joint limit can't help any more:
            # solve again without it, so the others don't count on it (which
            # would make the needle worse, not just less better).
            free = np.ones(self.km.n, dtype=bool)
            for _ in range(self.km.n):
                Jf = Jt[:, free]
                dq = np.zeros(self.km.n)
                dq[free] = Jf.T @ np.linalg.solve(Jf @ Jf.T + self.lam2 * np.eye(6), e)
                step = np.clip(self.ki * dt * dq, -self.max_rate * dt, self.max_rate * dt)
                new = self.c + step
                q_new = q_plan + new
                stuck = free & (((np.abs(new) > self.max) & (np.sign(step) == np.sign(new)))
                                | ((q_new < self._lo) & (step < 0)) | ((q_new > self._hi) & (step > 0)))
                if not stuck.any() or stuck.sum() == free.sum():
                    break
                free &= ~stuck
            self.saturated = [i + 1 for i in np.flatnonzero(~free)]
            c = np.clip(new, -self.max, self.max)
            # keep the command inside the joint limits -- a correction that a
            # limit would cut off only winds up
            self.c = np.clip(q_plan + c, self._lo, self._hi) - q_plan
            self.peak_c_deg = max(self.peak_c_deg, math.degrees(float(np.max(np.abs(self.c)))))
            if self.saturated:
                self.ever_saturated.update(self.saturated)

    @property
    def correction_deg(self) -> float:
        return math.degrees(float(np.max(np.abs(self.c))))

    def summary(self) -> str:
        if not self.adaptive:
            return (f"axis hold off (start offset kept, {self.correction_deg:.2f} deg max): needle tilt reached "
                    f"{self.peak_tilt_deg:.2f} deg, sideways {self.peak_lateral_mm:.1f} mm")
        out = (f"axis hold: peak tilt {self.peak_tilt_deg:.2f} deg, peak sideways {self.peak_lateral_mm:.1f} mm, "
               f"largest joint correction {self.peak_c_deg:.2f} deg; now tilt {self.tilt_deg:.2f} deg, "
               f"sideways {self.lateral_mm:.1f} mm")
        if self.ever_saturated:
            out += (f" -- WARNING joint(s) {sorted(self.ever_saturated)} reached the correction limit: the load "
                    f"was more than it can take out (slower insertion, a lower force limit, or higher kp)")
        return out


def from_config(km: ArmKinematics, direction, cfg: dict | None) -> AxisHold | None:
    """An AxisHold from config.yaml `axis_hold:`, or None when it's switched off."""
    cfg = cfg or {}
    if not cfg.get("enabled", True):
        return None
    return AxisHold(km, direction, ki_per_s=float(cfg.get("ki_per_s", 2.0)),
                    max_deg=float(cfg.get("max_correction_deg", 8.0)),
                    max_rate_deg_s=float(cfg.get("max_rate_deg_s", 5.0)),
                    damping=float(cfg.get("damping", 0.02)))


def hold_sender(read_q, send, hold: AxisHold, period_s: float):
    """Wrap a per-tick send(q) (streaming.real_sender) for the line executor:
    each tick, read the arm (read_q() -> rad), let the hold observe it against
    the previous tick's plan, then send this tick's plan + correction."""
    import time
    state = {"t": None, "plan": None}

    def wrapped(q_plan):
        now = time.monotonic()
        if state["plan"] is not None:
            dt = min(now - state["t"], 3 * period_s)
            hold.observe(state["plan"], np.asarray(read_q(), dtype=float), dt)
        state["t"], state["plan"] = now, np.asarray(q_plan, dtype=float)
        send(hold.command(q_plan))
    return wrapped
