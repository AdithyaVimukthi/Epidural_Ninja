"""
Kinematics for a single OpenArm 7-DOF arm (left or right).

The kinematic chain (link geometry, joint axes) comes from the dual-arm
URDF (config.yaml paths.urdf). Two joint-limit sources are available
(--limits / limit_source), default "real":

  real  config.yaml R_left_arm.limits_deg / R_right_arm.limits_deg:
        explicit per-arm hardware limits, used as-is (no mirroring).
  sim   the URDF's own per-arm joint limits (what the simulation enforces).

Usage:
    from kinematics import ArmKinematics

    km = ArmKinematics("left")               # real-arm limits by default
    T = km.fkine([0, -30, 0, 45, 0, 20, 0])          # degrees -> SE3 pose
    sol = km.ikine(T)                                 # SE3 pose -> IKSolution

CLI:
    python3 motion/kinematics.py --arm left
    python3 motion/kinematics.py --arm right --limits real --fk 0 -30 0 45 0 20 0
    python3 motion/kinematics.py --arm left  --ik 0.3 0.1 0.2 0 0 0
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path
from dataclasses import dataclass

import numpy as np
import roboticstoolbox as rtb
from scipy.optimize import least_squares
from spatialmath import SE3

from config import REAL_ARM_LIMITS_RAD, REAL_GRIPPER_RAD, SIM_ARM_LIMITS_RAD, TOOL_TIP_OFFSET_M, URDF_PATH

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]

# (base link, tcp link) per arm, as named in openarm_dual.urdf.
# Base is the shared "robot_base" (world) frame, NOT each arm's own mount
# link -- ets(start=X, ...) includes the transform *into* X as its first
# hop, so starting at e.g. "openarm_left_link0_respondable" would still
# leave the chain rooted at that link's parent, offset from world origin
# by the body-to-arm-mount transform. Rooting both arms at "robot_base"
# keeps left/right poses expressed in the same, world-aligned frame.
#
# The TCP is the needle tip, openarm_{side}_needle_tip, on an arm whose URDF
# has one (openarm_dual_needle.urdf, made by tools/make_needle_urdf.py), else
# the gripper's openarm_{side}_hand_tcp_visual -- so switching paths.urdf is
# all it takes.
def _tcp_link(side: str) -> str:
    names = {link.get("name") for link in ET.parse(URDF_PATH).getroot().iter("link")}
    tip = f"openarm_{side}_needle_tip"
    return tip if tip in names else f"openarm_{side}_hand_tcp_visual"


ARM_LINKS = {side: ("robot_base", _tcp_link(side)) for side in ("left", "right")}

# Direction the tool points, in the TCP frame: -X. The gripper TCP sits 0.08 m
# along -X of its hand link (wrist -> between the fingertips); the needle tip
# frames have the tool link's axes, with -X down the needle.
TOOL_AXIS = np.array([-1.0, 0.0, 0.0])

# config.yaml key holding the real-arm block (gripper + limits_deg) per arm
REAL_ARM_KEY = {"left": "R_left_arm", "right": "R_right_arm"}

LIMIT_SOURCES = ("sim", "real")


def _load_limits_rad(arm: str, source: str = "sim") -> np.ndarray:
    """Joint limits in radians, shape (2, 7) = [lower; upper].

    source="sim":  the URDF's openarm_{arm}_joint1..7 limits.
    source="real": config.yaml R_left_arm.limits_deg / R_right_arm.limits_deg,
                   used directly since each arm has its own explicit values.
    """
    if source not in LIMIT_SOURCES:
        raise ValueError(f"limit_source must be one of {LIMIT_SOURCES}, got {source!r}")
    limits = SIM_ARM_LIMITS_RAD[arm] if source == "sim" else REAL_ARM_LIMITS_RAD[arm]
    return np.array(limits).T


def _load_gripper_deg(arm: str) -> tuple[float, float]:
    """Real-arm gripper (open_deg, close_deg) for the given arm, from config.yaml."""
    g = REAL_GRIPPER_RAD[arm]
    return math.degrees(g["open"]), math.degrees(g["close"])


@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    residual: float   # norm of the (masked) pose error at `q`, in m / rad
    iterations: int   # total solver function evaluations across all restarts
    reason: str


class ArmKinematics:
    """Kinematic model of one OpenArm 7-DOF arm, limits sourced from config.yaml."""

    def __init__(self, arm: str = "left", limit_source: str = "real"):
        if arm not in ARM_LINKS:
            raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
        self.arm = arm
        self.limit_source = limit_source

        full = rtb.Robot.URDF(URDF_PATH)
        start, end = ARM_LINKS[arm]
        ets = full.ets(start=start, end=end)

        # The subchain keeps the joint indices (jindex) it had in the full
        # dual-arm tree (e.g. 9..15 for the right arm); renumber to 0..6 so
        # rtb.Robot accepts it as a standalone 7-DOF chain.
        offset = min(e.jindex for e in ets if e.isjoint)
        for e in ets:
            if e.isjoint:
                e.jindex -= offset

        # The needle tip (config.yaml tool.tip_offset_m, in the TCP frame) --
        # appended to the chain so fkine, jacob0 and every IK solve use it.
        # All zeros leaves the chain exactly as the URDF has it.
        for axis, v in zip((rtb.ET.tx, rtb.ET.ty, rtb.ET.tz), TOOL_TIP_OFFSET_M):
            if v:
                ets = ets * axis(float(v))

        self.robot = rtb.Robot(ets)
        self.robot.qlim = _load_limits_rad(arm, limit_source)

    @property
    def n(self) -> int:
        return self.robot.n

    @property
    def qlim(self) -> np.ndarray:
        """shape (2, 7): row 0 = lower limits, row 1 = upper limits (radians)."""
        return self.robot.qlim

    def in_limits(self, q) -> bool:
        lo, hi = self.qlim
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= lo) and np.all(q <= hi))

    def clip(self, q) -> np.ndarray:
        lo, hi = self.qlim
        return np.clip(np.asarray(q, dtype=float), lo, hi)

    def fkine(self, q_deg) -> SE3:
        """Forward kinematics. q_deg: 7 joint angles in degrees."""
        q = np.radians(np.asarray(q_deg, dtype=float))
        return self.robot.fkine(q)

    def jacobian(self, q_deg) -> np.ndarray:
        """Geometric Jacobian (world frame) at the given joint configuration."""
        q = np.radians(np.asarray(q_deg, dtype=float))
        return self.robot.jacob0(q)

    def ikine(
        self, T: SE3, q0_deg=None, mask=None, tol: float = 1e-4,
        restarts: int = 8, seed: int = 0,
    ) -> IKResult:
        """Inverse kinematics: bounded least-squares, respecting config.yaml limits.

        mask: optional 6-vector (x,y,z,rx,ry,rz), 1 = must match, 0 = free.
        Defaults to None, i.e. all 6 (full pose). Orientation of `T` is
        ignored wherever its mask entry is 0, so it can be anything.

        Uses scipy.optimize.least_squares with box bounds = self.qlim (a
        trust-region-reflective solver, which -- unlike roboticstoolbox's
        own ikine_LM -- reliably finds solutions that sit exactly on a
        joint-limit boundary; several reachable OpenArm targets, given the
        narrow real-arm limits, only have solutions right at that boundary).
        Tries the given q0 (or zero) first, then `restarts` random seeds
        within the joint limits, stopping early once a seed converges
        within `tol` (m for position error, combined with rad for
        orientation error via the SE3 log/twist norm).
        """
        mask_arr = np.ones(6, dtype=bool) if mask is None else np.asarray(mask, dtype=bool)

        def residual(q):
            err = (T.inv() * self.robot.fkine(q)).log(twist=True)
            return err[mask_arr]

        return self._solve(residual, q0_deg, tol, restarts, seed)

    def ikine_position(self, xyz, q0_deg=None, **kwargs) -> IKResult:
        """Position-only IK: end-effector must reach xyz, orientation is left free."""
        x, y, z = xyz
        return self.ikine(SE3(x, y, z), q0_deg=q0_deg, mask=[1, 1, 1, 0, 0, 0], **kwargs)

    def tool_axis(self, q_deg) -> np.ndarray:
        """Unit vector the tool points along (TOOL_AXIS), in robot_base frame."""
        return self.fkine(q_deg).R @ TOOL_AXIS

    def ikine_axis(
        self, xyz, axis, q0_deg=None, tol: float = 1e-4, restarts: int = 8, seed: int = 0,
        stay: float = 0.05,
    ) -> IKResult:
        """IK with the TCP at xyz and the tool pointing along `axis` (a direction
        in robot_base frame; normalised here). Roll about the tool axis is left
        free, so this is a 5-constraint solve with 2 redundant DOF on a 7-DOF arm.

        The orientation residual is (tool axis - target axis), which is zero only
        when they point the same way -- not when they are anti-parallel, as a
        cross product would allow. It is unitless (~rad for small errors) and is
        combined with the position error in metres against `tol`, as in ikine().

        stay: weight (per rad) pulling each solve toward its seed. The 2 free DOF
        let an unregularised solve slide tens of degrees along the null space
        between two targets millimetres apart; this keeps it on the nearest
        solution. Each regularised solve is then polished without it, so the
        result is exactly on target -- `stay` only chooses which solution.
        """
        target = np.asarray(xyz, dtype=float)
        a = np.asarray(axis, dtype=float)
        a = a / np.linalg.norm(a)

        def residual(q):
            T = self.robot.fkine(q)
            return np.concatenate([T.t - target, T.R @ TOOL_AXIS - a])

        def jacobian(q):
            # d(R a_tool)/dq_i = w_i x (R a_tool) = -[R a_tool]x w_i
            v = self.robot.fkine(q).R @ TOOL_AXIS
            J = self.robot.jacob0(q)
            skew = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
            return np.vstack([J[:3], -skew @ J[3:]])

        return self._solve(residual, q0_deg, tol, restarts, seed, stay, jacobian)

    def _solve(self, residual, q0_deg, tol: float, restarts: int, seed: int,
               stay: float = 0.0, jacobian=None) -> IKResult:
        """Bounded least-squares on `residual(q)` over the joint limits, from q0
        (if given), zero, then `restarts` random seeds -- see ikine(). With
        stay > 0, each seed is first solved with `stay * (q - seed)` appended to
        the residual, then polished from there on `residual` alone. jacobian(q),
        if given, is d residual / dq (else finite differences)."""
        lo, hi = self.qlim
        opts = dict(bounds=(lo, hi), xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=300)
        jac = jacobian if jacobian is not None else "2-point"
        stay_jac = (lambda q: np.vstack([jacobian(q), stay * np.eye(self.n)])) if jacobian else "2-point"
        seeds = []
        if q0_deg is not None:
            seeds.append(np.radians(q0_deg))
        seeds.append(np.zeros(self.n))
        rng = np.random.default_rng(seed)
        seeds.extend(rng.uniform(lo, hi) for _ in range(restarts))

        best_dist, best_q, total_nfev = None, None, 0
        for q0 in seeds:
            x0 = np.clip(q0, lo, hi)
            if stay > 0:
                near = least_squares(lambda q: np.concatenate([residual(q), stay * (q - x0)]),
                                     x0, jac=stay_jac, **opts)
                total_nfev += near.nfev
                x0 = near.x
            res = least_squares(residual, x0, jac=jac, **opts)
            total_nfev += res.nfev
            dist = float(np.linalg.norm(res.fun))
            if best_dist is None or dist < best_dist:
                best_dist, best_q = dist, res.x
            if best_dist < tol:
                break

        success = best_dist < tol
        reason = "Success" if success else (
            f"best residual {best_dist:.4g} exceeds tol {tol:.4g} "
            "after all restarts -- target likely unreachable within these limits"
        )
        return IKResult(success=success, q=best_q, residual=best_dist,
                         iterations=total_nfev, reason=reason)

    def limits_report(self) -> str:
        lo, hi = self.qlim
        src = "the URDF" if self.limit_source == "sim" else f"config.yaml {REAL_ARM_KEY[self.arm]}.limits_deg"
        lines = [f"Joint limits ({self.arm} arm, from {src}):"]
        for name, l, h in zip(JOINT_NAMES, lo, hi):
            lines.append(f"  {name}: [{math.degrees(l):8.2f}, {math.degrees(h):8.2f}] deg")
        if self.limit_source == "real":
            open_deg, close_deg = _load_gripper_deg(self.arm)
            lines.append(f"  gripper: open={open_deg:.1f} deg, close={close_deg:.1f} deg")
        return "\n".join(lines)


def _parse_args():
    p = argparse.ArgumentParser(description="OpenArm single-arm kinematics (roboticstoolbox)")
    p.add_argument("--arm", choices=["left", "right"], default="left")
    p.add_argument(
        "--limits", choices=list(LIMIT_SOURCES), default="real",
        help="limit set to enforce: real = config.yaml, sim = URDF (default: real)",
    )
    p.add_argument(
        "--fk", nargs=7, type=float, metavar=tuple(f"q{i}" for i in range(1, 8)),
        help="7 joint angles in degrees -> print end-effector pose",
    )
    p.add_argument(
        "--ik", nargs="+", type=float, metavar="V",
        help="target pose -> solve joint angles. Either 3 values 'x y z' "
             "(position-only, orientation left free) or 6 values "
             "'x y z roll pitch yaw' (rpy in degrees, full pose)",
    )
    return p.parse_args()


def main():
    args = _parse_args()
    km = ArmKinematics(args.arm, limit_source=args.limits)
    print(km.limits_report())
    print()

    if args.fk is not None:
        q = args.fk
        if not km.in_limits(np.radians(q)):
            print(f"WARNING: requested joint angles are outside the [{args.limits}] limits")
        T = km.fkine(q)
        xyz = T.t
        rpy = T.rpy(unit="deg")
        print(f"FK  q(deg)  = {q}")
        print(f"    xyz (m) = {xyz}")
        print(f"    rpy(deg)= {rpy}")
        return

    if args.ik is not None:
        if len(args.ik) == 3:
            x, y, z = args.ik
            T = SE3(x, y, z)
            sol = km.ikine(T, mask=[1, 1, 1, 0, 0, 0])
            print(f"IK  target xyz={[x, y, z]}  (position-only, orientation free)")
        elif len(args.ik) == 6:
            x, y, z, r, p, yaw = args.ik
            T = SE3(x, y, z) * SE3.RPY([r, p, yaw], unit="deg", order="xyz")
            sol = km.ikine(T)
            print(f"IK  target xyz={[x, y, z]}  rpy(deg)={[r, p, yaw]}")
        else:
            raise SystemExit("--ik expects 3 values (x y z) or 6 values (x y z roll pitch yaw)")

        print(f"    success  = {sol.success}  residual = {sol.residual:.5g}  nfev = {sol.iterations}")
        if sol.success:
            q_deg = np.degrees(sol.q)
            print(f"    q (deg)  = {q_deg}")
            print(f"    in limits= {km.in_limits(sol.q)}")
        else:
            print(f"    {sol.reason}")
            print(f"    closest q (deg) = {np.degrees(sol.q)}")
        return

    # No --fk/--ik given: run a small demo (zero pose + a round-trip IK check).
    q_zero = [0.0] * 7
    T0 = km.fkine(q_zero)
    print(f"Demo: FK at q=0 -> xyz={T0.t}, rpy(deg)={T0.rpy(unit='deg')}")

    q_sample = [10, -20, 0, 45, 0, 15, 0]
    T_sample = km.fkine(q_sample)
    sol = km.ikine(T_sample, q0_deg=q_zero)
    print(f"Demo: FK(q={q_sample}) -> IK round-trip success={sol.success}")
    if sol.success:
        print(f"      recovered q (deg) = {np.degrees(sol.q)}")


if __name__ == "__main__":
    main()
