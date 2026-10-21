"""Plunger point 4 on a stand-in real arm that sags: the descent is planned
from the MEASURED position, starts without a jump, and runs straight down
from where the needle measurably was."""
import math
from dataclasses import dataclass

import numpy as np

import insertion
import main
import mode_plunger
from config import REAL_ARM_LIMITS_RAD
from kinematics import TOOL_AXIS, ArmKinematics

DOWN = np.array([0.0, 0.0, -1.0])
P3 = np.array([0.20, -0.08, 0.19])
# joint offsets between the setpoint the motors hold and where the arm sits
SAG = np.radians([0.4, 1.5, 0.3, 2.0, -0.6, 1.2, 0.0])   # (joint 2 kept clear of its 0 deg limit)


@dataclass
class State:
    position_rad: float
    velocity: float = 0.0
    torque: float = 0.0

    @property
    def position_deg(self):
        return math.degrees(self.position_rad)


class SaggingArm:
    """measured = command - SAG, as a P-servo under a constant load settles."""

    def __init__(self, q_held):
        self.held_q = list(q_held)
        self.held_kp = [40.0] * 7
        self.arm_limits_rad = REAL_ARM_LIMITS_RAD["right"]
        self.sent = []

    def read_joint_states(self):
        q = np.array(self.held_q) - SAG
        return [State(v) for v in q] + [State(0.0)]

    def move_joints(self, q, kp=None, kd=None, dq=None):
        self.held_q = [float(v) for v in q]
        self.sent.append(np.array(q))


class FakeSession:
    def __init__(self, arm):
        self.arm = arm

    def real(self, side):
        return self.arm


def test_point4_plans_from_the_measured_position_without_a_jump():
    km = ArmKinematics("right")
    sol = km.ikine_axis(P3, DOWN, q0_deg=[0, 0, 0, 60, 0, 0, -50], restarts=8)
    assert sol.success
    arm = SaggingArm(sol.q)                          # point 3 commanded and held
    q_meas0 = np.array(sol.q) - SAG
    tip0 = np.array(km.robot.fkine(q_meas0).t)
    args = main.build_parser().parse_args(
        ["plunger", "--point", "4", "--mode", "real", "--xyz", *map(str, P3), "--depth", "0.02",
         "--speed", "0.01"])
    insertion.NEEDLE_IN.clear()
    mode_plunger._point4(FakeSession(arm), "right", args, "real")

    # no jump: the first command is the setpoint the motors were holding
    assert np.max(np.abs(arm.sent[0] - sol.q)) < math.radians(0.05)
    # straight down from where the needle measurably was
    q_end = np.array(arm.held_q) - SAG
    T = km.robot.fkine(q_end)
    tip, axis = np.array(T.t), T.R @ TOOL_AXIS
    assert np.linalg.norm((tip - tip0)[:2]) * 1000 < 1.0
    assert abs((tip0 - tip)[2] * 1000 - 20.0) < 1.0
    assert math.degrees(math.acos(np.clip(-axis[2], -1, 1))) < 0.5
    assert "right" in insertion.NEEDLE_IN                # the needle is on record as in
    insertion.NEEDLE_IN.clear()
