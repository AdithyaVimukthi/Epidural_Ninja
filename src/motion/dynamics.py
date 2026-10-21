"""
Dynamics for a single OpenArm 7-DOF arm (left or right), via Pinocchio.

Companion to kinematics.py: that script covers FK/IK (roboticstoolbox); this
one covers what roboticstoolbox doesn't -- gravity torque and the
Jacobian-transpose end-effector force mapping used for force control. Both
load the same dual-arm URDF and share ARM_LINKS (the TCP frame name per
side), so "the end effector" means the same point in both scripts.

Usage:
    from dynamics import ArmDynamics

    dyn = ArmDynamics("left")
    tau_g = dyn.gravity_torque([0, -30, 0, 45, 0, 20, 0])      # Nm, one per joint
    J = dyn.jacobian_world([0, -30, 0, 45, 0, 20, 0])          # 6x7, world-aligned at TCP
    tau = dyn.ee_force_to_torque(q_deg, [0, 0, 5, 0, 0, 0])    # 5N in world +z

CLI:
    python3 motion/dynamics.py --arm left --gravity 0 -30 0 45 0 20 0
    python3 motion/dynamics.py --arm left --force 0 0 5 --at 0 -30 0 45 0 20 0
    python3 motion/dynamics.py --arm left --force 0 0 5 --at 0 -30 0 45 0 20 0 --no-gravity-comp
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path

import numpy as np
import pinocchio as pin

from config import URDF_PATH
from kinematics import ARM_LINKS, JOINT_NAMES


class ArmDynamics:
    """Gravity torque and EE-force-to-joint-torque mapping for one arm, via Pinocchio.

    Loads the full dual-arm URDF (Pinocchio has no notion of "subchain" the
    way kinematics.py builds one with roboticstoolbox) and, per call, reads
    out only the rows/columns belonging to this arm's 7 joints -- looked up
    by name (openarm_{side}_joint1..7) rather than assumed to sit at a fixed
    offset, since the two arms land at different positions in the combined
    model's joint order.
    """

    def __init__(self, arm: str):
        if arm not in ARM_LINKS:
            raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
        self.arm = arm
        self.model = pin.buildModelFromUrdf(URDF_PATH)
        self.data = self.model.createData()

        joint_names = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
        missing = [n for n in joint_names if not self.model.existJointName(n)]
        if missing:
            raise RuntimeError(f"joint(s) {missing} not found in {URDF_PATH}")
        joint_ids = [self.model.getJointId(n) for n in joint_names]
        self._idx_q = [self.model.joints[j].idx_q for j in joint_ids]
        self._idx_v = [self.model.joints[j].idx_v for j in joint_ids]

        _, tcp_link = ARM_LINKS[arm]
        if not self.model.existFrame(tcp_link):
            raise RuntimeError(f"frame {tcp_link!r} not found in {URDF_PATH}")
        self._tcp_frame = self.model.getFrameId(tcp_link)

    def _full_q(self, q_deg) -> np.ndarray:
        """Embed this arm's 7 angles (degrees) into a full-model q, other
        joints at neutral. The other arm's pose doesn't affect this arm's own
        gravity torque or Jacobian -- the two arms share no joints, only the
        fixed body they're both mounted to."""
        q = pin.neutral(self.model)
        q[self._idx_q] = np.radians(np.asarray(q_deg, dtype=float))
        return q

    def gravity_torque(self, q_deg) -> np.ndarray:
        """Generalized gravity torque at this arm's 7 joints, Nm -- the
        `tau_ff` needed to hold q_deg against gravity alone (kp=kd=0).
        Same per-joint sign convention as arm_can.py's MITParam.tau_ff."""
        q = self._full_q(q_deg)
        tau_g_full = pin.computeGeneralizedGravity(self.model, self.data, q)
        return tau_g_full[self._idx_v]

    def jacobian_world(self, q_deg) -> np.ndarray:
        """6x7 geometric Jacobian at the TCP frame, world-aligned: linear
        rows are world x/y/z, angular rows are world-frame angular velocity.
        This is the frame convention an EE wrench given 'in world frame'
        should be multiplied against (see ee_force_to_torque)."""
        q = self._full_q(q_deg)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.getFrameJacobian(
            self.model, self.data, self._tcp_frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return J_full[:, self._idx_v]

    def ee_force_to_torque(self, q_deg, F_world, gravity_comp: bool = True) -> np.ndarray:
        """Joint torques (Nm) to realize wrench F_world = [Fx,Fy,Fz,Mx,My,Mz]
        (N, Nm) at the TCP, expressed along world-frame axes.

            tau = J(q)^T @ F_world              gravity_comp=False
                                                 (e.g. the zero-gravity
                                                 CoppeliaSim test)
            tau = tau_g(q) + J(q)^T @ F_world    gravity_comp=True
                                                 (real arm, or sim with
                                                 gravity enabled)
        """
        F = np.asarray(F_world, dtype=float)
        if F.shape != (6,):
            raise ValueError(f"F_world must be 6 values [Fx,Fy,Fz,Mx,My,Mz], got shape {F.shape}")
        J = self.jacobian_world(q_deg)
        tau = J.T @ F
        if gravity_comp:
            tau = tau + self.gravity_torque(q_deg)
        return tau


def _parse_args():
    p = argparse.ArgumentParser(
        description="OpenArm single-arm dynamics (Pinocchio): gravity torque + EE force -> joint torque"
    )
    p.add_argument("--arm", choices=["left", "right"], default="left")
    p.add_argument(
        "--gravity", nargs=7, type=float, metavar=tuple(f"q{i}" for i in range(1, 8)),
        help="7 joint angles (deg) -> print gravity torque at each joint (Nm)",
    )
    p.add_argument(
        "--force", nargs="+", type=float, metavar="V",
        help="EE wrench in world frame: 3 values 'Fx Fy Fz' (N) or 6 values "
             "'Fx Fy Fz Mx My Mz' (N, Nm) -- requires --at",
    )
    p.add_argument(
        "--at", nargs=7, type=float, metavar=tuple(f"q{i}" for i in range(1, 8)),
        help="joint angles (deg) to evaluate --force at",
    )
    p.add_argument(
        "--no-gravity-comp", action="store_true",
        help="with --force, compute tau = J^T F only, no gravity term "
             "(the zero-gravity CoppeliaSim test)",
    )
    return p.parse_args()


def main():
    args = _parse_args()
    dyn = ArmDynamics(args.arm)

    if args.gravity is not None:
        tau_g = dyn.gravity_torque(args.gravity)
        print(f"gravity torque at q(deg)={args.gravity}:")
        for name, t in zip(JOINT_NAMES, tau_g):
            print(f"  {name}: {t:8.4f} Nm")
        return

    if args.force is not None:
        if args.at is None:
            raise SystemExit("--force requires --at q1..q7 (joint angles to evaluate at)")
        F = args.force
        if len(F) == 3:
            F = F + [0.0, 0.0, 0.0]
        elif len(F) != 6:
            raise SystemExit("--force expects 3 values (Fx Fy Fz) or 6 (Fx Fy Fz Mx My Mz)")
        gravity_comp = not args.no_gravity_comp
        tau = dyn.ee_force_to_torque(args.at, F, gravity_comp=gravity_comp)
        print(f"F_world = {F}  (gravity_comp={gravity_comp})")
        print(f"tau (Nm) at q(deg)={args.at}:")
        for name, t in zip(JOINT_NAMES, tau):
            print(f"  {name}: {t:8.4f} Nm")
        return

    print("Nothing to do -- pass --gravity q1..q7, or --force Fx Fy Fz [Mx My Mz] --at q1..q7")


if __name__ == "__main__":
    main()
