"""Axis hold on a compliant arm: each joint a spring of kp, the needle pushed
back along its axis by the material -- the model that reproduces the real
runs of 2026-10-13 (4.2 deg tilt, ~30 mm sideways at 40 mm)."""
import math

import numpy as np
import pytest

from axis_hold import AxisHold
from insertion import Rig, run_path
from kinematics import TOOL_AXIS, ArmKinematics
from mode_line import Section, _plan_section, _q_at

DOWN = np.array([0.0, 0.0, -1.0])
P3 = np.array([0.20, -0.08, 0.19])


@pytest.fixture(scope="module")
def km():
    return ArmKinematics("right")


@pytest.fixture(scope="module")
def descent(km):
    sol = km.ikine_axis(P3, DOWN, q0_deg=[0, 0, 0, 60, 0, 0, -50], restarts=8)
    assert sol.success
    sec = Section("descent", P3, P3 + DOWN * 0.02, 0.01, DOWN)       # 20 mm in 2 s
    _, fail = _plan_section(km, sec, sol.q, 0.002)
    assert fail is None
    return sec


class SpringArm:
    """Quasi-static: q = q_cmd + J(q)^T F / kp, F = k x depth below the surface,
    pushing back up the needle."""

    def __init__(self, km, kp=40.0, k_n_per_mm=1.0, surface_mm=2.0):
        self.km, self.kp, self.k, self.surface = km, kp, k_n_per_mm, surface_mm
        self.q_cmd = None

    def send(self, q):
        self.q_cmd = np.asarray(q, dtype=float)

    def settled(self, q_cmd, q):
        T = self.km.robot.fkine(q)
        a = T.R @ TOOL_AXIS
        depth_mm = float(np.dot(np.asarray(T.t) - P3, DOWN)) * 1000
        F = -a * self.k * max(0.0, depth_mm - self.surface)
        return q_cmd + self.km.robot.jacob0(q)[:3].T @ F / self.kp

    def measure(self, _q_cmd):
        # Relaxed fixed point (a plain one oscillates under big loads), from
        # the last answer; must converge or the model -- not the arm -- is wrong.
        q = self.q_cmd.copy() if getattr(self, "q_last", None) is None else self.q_last
        for _ in range(400):
            q_next = q + 0.3 * (self.settled(self.q_cmd, q) - q)
            if np.max(np.abs(q_next - q)) < 1e-7:
                break
            q = q_next
        else:
            raise AssertionError("spring-arm model didn't converge")
        self.q_last = q
        return q, None


def tilt_and_drift(km, q):
    T = km.robot.fkine(q)
    a = T.R @ TOOL_AXIS
    return math.degrees(math.acos(float(np.clip(-a[2], -1, 1)))), float(np.linalg.norm((np.asarray(T.t) - P3)[:2])) * 1000


def insert(km, sec, hold):
    arm = SpringArm(km)
    rig = Rig("right", km, 100.0, arm.send, arm.measure, True, None, hold=hold)
    last = {}

    def governor(t, tau, q_cmd, q_meas, torques):
        last["q"] = q_meas
        return 1.0, tau >= sec.duration
    run_path(rig, sec, governor)
    return tilt_and_drift(km, last["q"])


def stepped(km, sec, hold, k):
    """The descent at its own speed, stepped at 100 Hz without waiting."""
    arm = SpringArm(km, k_n_per_mm=k)
    prev, dt = None, 0.01
    for i in range(int(sec.duration / dt) + 1):
        qp = _q_at(sec, i * dt)
        if hold is not None and prev is not None:
            hold.observe(prev[0], prev[1], dt)
        arm.send(hold.command(qp) if hold is not None else qp)
        prev = (qp, arm.measure(None)[0])
    return tilt_and_drift(km, prev[1])


@pytest.mark.parametrize("k", [0.5, 1.0])
def test_hold_keeps_the_needle_straight_at_2mm_s(km, descent, k):
    sec = Section("descent", P3, P3 + DOWN * 0.02, 0.002, DOWN)    # 2 mm/s, as on the arm
    sec.q_begin, sec.knots = descent.q_begin, descent.knots
    tilt0, drift0 = stepped(km, sec, None, k)
    hold = AxisHold(km, DOWN)
    tilt1, drift1 = stepped(km, sec, hold, k)
    assert tilt0 > 2.0 and drift0 > 15.0              # the problem, as on the real arm
    assert tilt1 < 0.6 and drift1 < 4.0               # held on the line
    assert not hold.ever_saturated


def test_run_path_applies_the_hold(km, descent):
    tilt0, drift0 = insert(km, descent, None)          # real-time 2 s: 10 mm/s
    tilt1, drift1 = insert(km, descent, AxisHold(km, DOWN))
    assert tilt1 < 0.6 * tilt0 and drift1 < 0.6 * drift0


def test_hold_does_nothing_when_the_arm_follows(km, descent):
    hold = AxisHold(km, DOWN)

    def exact(q):
        return np.asarray(q, dtype=float), None
    sent = {}
    rig = Rig("right", km, 100.0, lambda q: sent.update(q=q), lambda q: exact(sent["q"]), True, None, hold=hold)
    run_path(rig, descent, lambda t, tau, qc, qm, tq: (1.0, tau >= descent.duration))
    assert hold.correction_deg < 0.05


def test_correction_is_limited(km, descent):
    hold = AxisHold(km, DOWN, ki_per_s=1000.0, max_deg=2.0, max_rate_deg_s=1000.0)
    q = descent.knots[-1]
    q_bent = q.copy()
    q_bent[5] += math.radians(20)                    # a huge wrist error
    for _ in range(200):
        hold.observe(q, q_bent, 0.01)
    assert hold.correction_deg <= 2.0 + 1e-9
    lo, hi = km.qlim
    cmd = hold.command(q)
    assert np.all(cmd >= lo - 1e-12) and np.all(cmd <= hi + 1e-12)


def test_preload_makes_the_first_command_what_the_motors_hold(km, descent):
    q_meas = descent.q_begin                          # the plan starts at the measured position
    q_held = q_meas + np.radians([0.3, -1.2, 0.4, 2.0, -0.5, 1.1, 0.0])   # gravity sag between them
    for adaptive in (True, False):
        hold = AxisHold(km, DOWN, adaptive=adaptive)
        assert hold.preload(q_held - q_meas) == 0.0
        assert np.allclose(hold.command(q_meas), q_held)       # no jump at the start


def test_offset_only_when_the_hold_is_off(km, descent):
    hold = AxisHold(km, DOWN, adaptive=False)
    hold.preload(np.radians([0, 1.0, 0, 0, 0, 0, 0]))
    c0 = hold.c.copy()
    bent = descent.knots[-1].copy()
    bent[5] += math.radians(5)
    for _ in range(50):
        hold.observe(descent.knots[-1], bent, 0.01)
    assert np.allclose(hold.c, c0) and hold.peak_tilt_deg > 2     # watched, not corrected
