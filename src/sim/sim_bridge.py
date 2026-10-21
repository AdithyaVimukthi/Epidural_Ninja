"""
CoppeliaSim bridge -- send/read joint angles for the OpenArm dual-arm scene
(config.yaml paths.sim_scenes: dual_openarm_org.ttt or dual_openarm_switched.ttt) over the ZMQ
Remote API.

Joint objects are located by alias (the URDF joint name CoppeliaSim's
importer preserves as the object's leaf name) via a scene-wide search, not
by tree path -- so it works regardless of where the importer nested them.
Discover the live names in a running scene with:

    python3 sim/sim_bridge.py list

Usage as a library:

    from sim_bridge import SimArmController

    sim_arm = SimArmController("left")
    sim_arm.connect()
    q = sim_arm.read_joint_positions()   # radians, 7 values
    sim_arm.set_joint_positions(q)       # kinematic write (teleport)
    sim_arm.move_joints(q)               # dynamic write (setJointTargetPosition)

Usage as a CLI:

    python3 sim/sim_bridge.py list
    python3 sim/sim_bridge.py status --arm left
    python3 sim/sim_bridge.py read   --arm left --rate 20
    python3 sim/sim_bridge.py move   --arm left --deg 0 -30 0 60 0 0 0
    python3 sim/sim_bridge.py move   --arm left --deg 0 -30 0 60 0 0 0 --dynamics
    python3 sim/sim_bridge.py gripper --arm left --open
    python3 sim/sim_bridge.py gripper --arm left --close

Requires CoppeliaSim running with the ZMQ Remote API server enabled
(on by default since CoppeliaSim 4.x) and the OpenArm scene loaded.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path
import time
from dataclasses import dataclass

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from config import (
    JOINT_NAMES, REAL_ARM_LIMITS_RAD, SIM_HOST, SIM_PORT, SIM_GRIPPER_FINGER_RANGE_M,
)

# CoppeliaSim object aliases, as preserved from openarm_dual.urdf joint names
ARM_JOINT_ALIAS = {
    side: [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    for side in ("left", "right")
}
FINGER_JOINT_ALIAS = {
    side: [f"openarm_{side}_finger_joint1", f"openarm_{side}_finger_joint2"]
    for side in ("left", "right")
}


@dataclass
class SimJointState:
    name: str
    position: float  # radians for arm joints, metres for gripper fingers


class SimArmController:
    """Read/write interface to one OpenArm side in a live CoppeliaSim scene."""

    def __init__(self, side: str, host: str = SIM_HOST, port: int = SIM_PORT):
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        self.side = side
        self.host = host
        self.port = port
        self.limits_rad = REAL_ARM_LIMITS_RAD[side]
        self.client = None
        self.sim = None
        self.arm_handles: list[int] = []
        self.finger_handles: list[int] = []

    def connect(self) -> None:
        self.client = RemoteAPIClient(host=self.host, port=self.port)
        self.sim = self.client.require('sim')
        self._resolve_handles()

    def _resolve_handles(self) -> None:
        by_alias = self.list_scene_joints()
        missing = [n for n in ARM_JOINT_ALIAS[self.side] if n not in by_alias]
        if missing:
            raise RuntimeError(
                f"joint(s) {missing} not found in scene -- "
                f"run `sim_bridge.py list` to see what's actually loaded"
            )
        self.arm_handles = [by_alias[n] for n in ARM_JOINT_ALIAS[self.side]]
        self.finger_handles = [by_alias[n] for n in FINGER_JOINT_ALIAS[self.side] if n in by_alias]

    def list_scene_joints(self) -> dict[str, int]:
        """Every joint object in the scene, by alias (leaf name) -> handle."""
        all_joints = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.object_joint_type, 0)
        return {self.sim.getObjectAlias(h, 0): h for h in all_joints}

    # ── read ──────────────────────────────────────────────────────────────

    def read_joint_positions(self) -> list[float]:
        """7 arm-joint positions, radians."""
        return [self.sim.getJointPosition(h) for h in self.arm_handles]

    def read_joint_states(self) -> list[SimJointState]:
        states = [
            SimJointState(name, self.sim.getJointPosition(h))
            for name, h in zip(ARM_JOINT_ALIAS[self.side], self.arm_handles)
        ]
        for name, h in zip(FINGER_JOINT_ALIAS[self.side], self.finger_handles):
            states.append(SimJointState(name, self.sim.getJointPosition(h)))
        return states

    def read_gripper_position(self) -> float | None:
        """Mean of the two finger positions (metres), or None if not found in scene."""
        if not self.finger_handles:
            return None
        vals = [self.sim.getJointPosition(h) for h in self.finger_handles]
        return sum(vals) / len(vals)

    # ── write ─────────────────────────────────────────────────────────────

    def _check_limits(self, positions_rad: list[float]) -> None:
        for q, (lo, hi), name in zip(positions_rad, self.limits_rad, JOINT_NAMES[:7]):
            if not lo <= q <= hi:
                raise ValueError(
                    f"{name} = {math.degrees(q):.2f} deg out of R_{self.side}_arm.limits_deg range "
                    f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]"
                )

    def set_joint_positions(self, positions_rad: list[float]) -> None:
        """Kinematic write (instant teleport, ignores dynamics) -- use this
        for a plain visualization scene with no motion control enabled."""
        if len(positions_rad) != 7:
            raise ValueError(f"need exactly 7 joint positions, got {len(positions_rad)}")
        self._check_limits(positions_rad)
        for h, q in zip(self.arm_handles, positions_rad):
            self.sim.setJointPosition(h, q)

    def move_joints(self, positions_rad: list[float]) -> None:
        """Dynamic write via setJointTargetPosition -- needs the simulation
        running and each joint's motion control loop enabled in the scene."""
        if len(positions_rad) != 7:
            raise ValueError(f"need exactly 7 joint positions, got {len(positions_rad)}")
        self._check_limits(positions_rad)
        for h, q in zip(self.arm_handles, positions_rad):
            self.sim.setJointTargetPosition(h, q)

    def unreached(self, target_rad: list[float], tol_deg: float = 0.5) -> list[str]:
        """Joints that did not end up where they were told to go.

        CoppeliaSim clamps setJointPosition to each joint's *own* interval
        configured in the scene, silently and with no error -- and that
        interval is not necessarily the one in config.yaml (e.g. the scene's
        openarm_left_joint4 is [40, 180] where config says [0, 140]). Without
        this check a move reports success while the arm never moved.
        """
        report = []
        for q_want, q_got, name, h in zip(target_rad, self.read_joint_positions(),
                                          JOINT_NAMES[:7], self.arm_handles):
            if abs(q_want - q_got) <= math.radians(tol_deg):
                continue
            _, (lo, rng) = self.sim.getJointInterval(h)
            report.append(
                f"{name}: asked {math.degrees(q_want):.2f} deg, got "
                f"{math.degrees(q_got):.2f} deg (scene limit "
                f"[{math.degrees(lo):.1f}, {math.degrees(lo + rng):.1f}])"
            )
        return report

    def move_joints_over_time(self, target_rad: list[float], duration_s: float, rate_hz: float = 50.0) -> None:
        """Linearly ramp from the current position to target_rad over
        duration_s, via repeated kinematic writes (set_joint_positions)."""
        if len(target_rad) != 7:
            raise ValueError(f"need exactly 7 joint positions, got {len(target_rad)}")
        self._check_limits(target_rad)
        start = self.read_joint_positions()
        period = 1.0 / rate_hz
        steps = max(1, int(duration_s * rate_hz))
        for step in range(1, steps + 1):
            t0 = time.time()
            frac = step / steps
            waypoint = [s + (q - s) * frac for s, q in zip(start, target_rad)]
            for h, q in zip(self.arm_handles, waypoint):
                self.sim.setJointPosition(h, q)
            sleep = period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

        for line in self.unreached(target_rad):
            print(f"[sim] WARNING {self.side} {line}")

    def set_gripper(self, position_m: float) -> None:
        """Drive both finger joints to the same value (no mimic constraint
        in the URDF, so they're independent). Range comes from the URDF."""
        lo, hi = SIM_GRIPPER_FINGER_RANGE_M
        if not lo <= position_m <= hi:
            raise ValueError(f"gripper = {position_m:.4f} m out of range [{lo}, {hi}]")
        if not self.finger_handles:
            raise RuntimeError(f"no finger joints found for {self.side} arm in scene")
        for h in self.finger_handles:
            self.sim.setJointTargetPosition(h, position_m)

    def open_gripper(self) -> None:
        self.set_gripper(SIM_GRIPPER_FINGER_RANGE_M[1])

    def close_gripper(self) -> None:
        self.set_gripper(SIM_GRIPPER_FINGER_RANGE_M[0])


def _print_states(side: str, states: list[SimJointState]) -> None:
    print(f"-- sim:{side} --")
    print(f"  {'joint':<28}{'value':>10}  unit")
    for s in states:
        is_finger = "finger" in s.name
        unit = "m" if is_finger else "rad"
        extra = "" if is_finger else f"  ({math.degrees(s.position):.2f} deg)"
        print(f"  {s.name:<28}{s.position:>10.4f}  {unit}{extra}")


def main() -> None:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--arm", choices=["left", "right"], default="left")
    common.add_argument("--host", default=SIM_HOST)
    common.add_argument("--port", type=int, default=SIM_PORT)

    parser = argparse.ArgumentParser(description="Send/read joint angles to/from a live CoppeliaSim OpenArm scene")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", parents=[common], help="print every joint alias found in the scene")
    sub.add_parser("status", parents=[common], help="print one joint-state reading")

    p_read = sub.add_parser("read", parents=[common], help="continuously print joint states")
    p_read.add_argument("--rate", type=float, default=10.0, help="polling rate in Hz (default 10)")

    p_move = sub.add_parser("move", parents=[common], help="move all 7 joints to target angles (degrees)")
    p_move.add_argument("--deg", type=float, nargs=7, required=True, metavar="D",
                         help="7 target angles in degrees: joint1..joint7")
    p_move.add_argument("--duration", type=float, default=2.0,
                         help="ramp time in seconds for the kinematic write (default 2, ignored with --dynamics)")
    p_move.add_argument("--dynamics", action="store_true",
                         help="drive via setJointTargetPosition (needs simulation running + "
                              "motion control enabled on the joints) instead of the default "
                              "kinematic teleport-and-ramp")

    p_grip = sub.add_parser("gripper", parents=[common], help="open or close the gripper (linear, metres)")
    grip_group = p_grip.add_mutually_exclusive_group(required=True)
    grip_group.add_argument("--open", action="store_true")
    grip_group.add_argument("--close", action="store_true")

    args = parser.parse_args()

    print(f"Connecting to CoppeliaSim at {args.host}:{args.port}...")
    client = RemoteAPIClient(host=args.host, port=args.port)
    sim = client.require('sim')

    if args.command == "list":
        all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type, 0)
        for h in all_joints:
            print(f"  {sim.getObjectAlias(h, 0)}")
        return

    ctrl = SimArmController(args.arm, host=args.host, port=args.port)
    ctrl.client, ctrl.sim = client, sim
    ctrl._resolve_handles()

    if args.command == "status":
        _print_states(args.arm, ctrl.read_joint_states())

    elif args.command == "read":
        print(f"Reading sim:{args.arm} at {args.rate} Hz. Ctrl+C to stop.")
        period = 1.0 / args.rate
        try:
            while True:
                t0 = time.time()
                _print_states(args.arm, ctrl.read_joint_states())
                sleep = period - (time.time() - t0)
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            print("\nInterrupted.")

    elif args.command == "move":
        target = [math.radians(d) for d in args.deg]
        if args.dynamics:
            print(f"Driving sim:{args.arm} to {args.deg} deg (dynamics)...")
            ctrl.move_joints(target)
        else:
            print(f"Setting sim:{args.arm} to {args.deg} deg over {args.duration}s (kinematic)...")
            ctrl.move_joints_over_time(target, args.duration)

    elif args.command == "gripper":
        if args.open:
            ctrl.open_gripper()
        else:
            ctrl.close_gripper()

    print("Done.")


if __name__ == "__main__":
    main()
