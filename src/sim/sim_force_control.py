"""
CoppeliaSim end-effector force-control test rig.

Two things live here:
  1. One-time scripted scene setup -- gravity toggle and per-joint torque
     dynamic-control mode. See the `gravity` and `mode` CLI commands.
  2. The force-control loop: read q from the sim, compute
     tau = J(q)^T @ F_world (+ gravity comp) via dynamics.ArmDynamics, stream
     it to the joints with sim.setJointTargetForce, and optionally read back
     a force sensor for comparison. See the `test` CLI command.

The cube (to push against) and the force sensor + probe (to measure what
actually lands at the tool tip) are NOT created by this script -- add them
by hand in the CoppeliaSim GUI once, per
docs/06_ee_force_control.md, so their placement can be seen and adjusted
visually. This script only needs the force sensor's object alias afterwards
(`--sensor` on `test`); `list` below will show it once added.

Requires the 7 arm joints already in **Force/torque** dynamic control mode
(`mode --arm left --torque` does this) -- in the default **Position** mode
(what sim_bridge.py's `move --dynamics` relies on), setJointTargetForce only
sets the max-force clamp for the position PID, it doesn't command an actual
torque.

Usage:
    python3 sim/sim_force_control.py list                             # every shape/joint/force-sensor/dummy alias in the scene
    python3 sim/sim_force_control.py sim --start                      # press Play, from the CLI
    python3 sim/sim_force_control.py sim --stop
    python3 sim/sim_force_control.py gravity --off                    # zero gravity -- the ideal-case test
    python3 sim/sim_force_control.py gravity --on                     # back to -9.81 m/s^2 in z
    python3 sim/sim_force_control.py mode --arm left --torque         # 7 joints -> force/torque control
    python3 sim/sim_force_control.py mode --arm left --position       # 7 joints -> back to position control (undo)
    python3 sim/sim_force_control.py test --arm left --force 0 0 5 --duration 5
    python3 sim/sim_force_control.py test --arm left --force 0 0 5 --gravity-comp --sensor openarm_force_test_sensor

Run everything from OpenArm_v3/src/, same as sim_bridge.py.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from config import SIM_HOST, SIM_PORT
from dynamics import ArmDynamics
from sim_bridge import SimArmController

# ── gravity ──────────────────────────────────────────────────────────────

EARTH_GRAVITY = [0.0, 0.0, -9.81]
ZERO_GRAVITY = [0.0, 0.0, 0.0]


def set_gravity(sim, enabled: bool) -> None:
    sim.setArrayParameter(sim.arrayparam_gravity, EARTH_GRAVITY if enabled else ZERO_GRAVITY)


# ── joint dynamic control mode ───────────────────────────────────────────

def set_arm_torque_mode(sim, arm_handles: list[int], torque: bool) -> None:
    """Switch all given joints between Force/torque and Position dynamic
    control mode. Torque mode is what setJointTargetForce needs to command
    an actual torque instead of a position-PID force clamp."""
    mode = sim.jointdynctrl_force if torque else sim.jointdynctrl_position
    for h in arm_handles:
        sim.setObjectInt32Param(h, sim.jointintparam_dynctrlmode, mode)


# ── listing (every relevant object, not just joints) ─────────────────────

def list_all_objects(sim) -> None:
    type_names = {
        sim.object_shape_type: "shape",
        sim.object_joint_type: "joint",
        sim.object_forcesensor_type: "forcesensor",
        sim.object_dummy_type: "dummy",
    }
    for obj_type, label in type_names.items():
        for h in sim.getObjectsInTree(sim.handle_scene, obj_type, 0):
            print(f"  [{label:<11}] {sim.getObjectAlias(h, 0)}")


# ── force sensor read ─────────────────────────────────────────────────────

def read_force_sensor_world(sim, sensor_alias: str) -> np.ndarray:
    """[Fx,Fy,Fz,Mx,My,Mz] measured by the named force sensor, rotated from
    its own local frame into world-frame axes so it's directly comparable to
    the F_world you commanded."""
    handle = sim.getObject(f"/{sensor_alias}")
    result, force, torque = sim.readForceSensor(handle)
    if result <= 0:
        return np.zeros(6)
    m = np.array(sim.getObjectMatrix(handle, -1)).reshape(3, 4)
    R = m[:, :3]
    return np.concatenate([R @ np.array(force), R @ np.array(torque)])


# ── control loop ──────────────────────────────────────────────────────────

def run_force_test(
    arm: str, F_world, duration_s: float, gravity_comp: bool,
    sensor_alias: str | None, rate_hz: float = 50.0,
    host: str = SIM_HOST, port: int = SIM_PORT, max_torque: float = 15.0,
) -> None:
    ctrl = SimArmController(arm, host=host, port=port)
    ctrl.connect()
    dyn = ArmDynamics(arm)
    F = np.array(F_world, dtype=float)

    print(f"Streaming tau = {'tau_g(q) + ' if gravity_comp else ''}J(q)^T @ F_world to "
          f"sim:{arm} for {duration_s}s at {rate_hz} Hz.  F_world = {F.tolist()}"
          f"  (clamped to +/-{max_torque} Nm per joint)")
    if sensor_alias is None:
        print("(no --sensor given -- add the force sensor from "
              "docs/06_ee_force_control.md and pass its alias to compare)")
    print("Ctrl+C to stop early.")

    period = 1.0 / rate_hz
    t_end = time.time() + duration_s
    try:
        while time.time() < t_end:
            t0 = time.time()
            q_deg = np.degrees(ctrl.read_joint_positions())
            tau = dyn.ee_force_to_torque(q_deg, F, gravity_comp=gravity_comp)
            clamped = np.clip(tau, -max_torque, max_torque)
            if not np.allclose(clamped, tau):
                print(f"\n  [warning] tau clamped: raw={np.round(tau, 2)} -> {np.round(clamped, 2)}")
            for h, t in zip(ctrl.arm_handles, clamped):
                ctrl.sim.setJointTargetForce(h, float(t))
            if sensor_alias:
                measured = read_force_sensor_world(ctrl.sim, sensor_alias)
                print(f"\r  measured F_world (sensor) = {np.round(measured, 3)}", end="", flush=True)
            sleep = period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        print("Stopping: setting tau_ff = 0 on all joints.")
        for h in ctrl.arm_handles:
            ctrl.sim.setJointTargetForce(h, 0.0)


def main() -> None:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--host", default=SIM_HOST)
    common.add_argument("--port", type=int, default=SIM_PORT)

    parser = argparse.ArgumentParser(description="CoppeliaSim end-effector force-control test rig")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", parents=[common], help="print every shape/joint/force-sensor/dummy alias in the scene")

    p_sim = sub.add_parser("sim", parents=[common], help="start/stop the CoppeliaSim simulation")
    sim_group = p_sim.add_mutually_exclusive_group(required=True)
    sim_group.add_argument("--start", action="store_true")
    sim_group.add_argument("--stop", action="store_true")

    p_grav = sub.add_parser("gravity", parents=[common], help="toggle scene gravity")
    grav_group = p_grav.add_mutually_exclusive_group(required=True)
    grav_group.add_argument("--off", action="store_true", help="zero gravity -- the ideal-case test")
    grav_group.add_argument("--on", action="store_true", help="restore -9.81 m/s^2 in z")

    p_mode = sub.add_parser("mode", parents=[common], help="switch a side's 7 arm joints between torque and position control")
    p_mode.add_argument("--arm", choices=["left", "right"], required=True)
    mode_group = p_mode.add_mutually_exclusive_group(required=True)
    mode_group.add_argument("--torque", action="store_true", help="force/torque control -- required before `test`")
    mode_group.add_argument("--position", action="store_true", help="back to position control (undoes --torque)")

    p_test = sub.add_parser("test", parents=[common], help="stream tau = J^T F_world (+ gravity comp) to one arm")
    p_test.add_argument("--arm", choices=["left", "right"], default="right")
    p_test.add_argument("--force", nargs="+", type=float, required=True, metavar="V",
                         help="EE wrench in world frame: 3 values 'Fx Fy Fz' (N) or "
                              "6 values 'Fx Fy Fz Mx My Mz' (N, Nm)")
    p_test.add_argument("--duration", type=float, default=5.0, help="seconds to stream (default 5)")
    p_test.add_argument("--rate", type=float, default=50.0, help="control loop rate in Hz (default 50)")
    p_test.add_argument("--gravity-comp", action="store_true",
                         help="add tau_g(q) -- use once the sim's gravity is back on (`gravity --on`); "
                              "leave off for the zero-gravity ideal-case test")
    p_test.add_argument("--sensor", default=None,
                         help="alias of a force sensor object to read back and compare against --force")
    p_test.add_argument("--max-torque", type=float, default=15.0,
                         help="safety clamp: max |tau| per joint in Nm (default 15) -- "
                              "guards against a bad/wild q blowing up J(q)^T F")

    args = parser.parse_args()

    if args.command in ("list", "sim", "gravity", "mode"):
        client = RemoteAPIClient(host=args.host, port=args.port)
        sim = client.require('sim')

    if args.command == "list":
        list_all_objects(sim)

    elif args.command == "sim":
        if args.start:
            sim.startSimulation()
            print("Simulation started.")
        else:
            sim.stopSimulation()
            print("Simulation stopped.")

    elif args.command == "gravity":
        set_gravity(sim, enabled=args.on)
        print(f"Gravity set to {EARTH_GRAVITY if args.on else ZERO_GRAVITY}")

    elif args.command == "mode":
        ctrl = SimArmController(args.arm, host=args.host, port=args.port)
        ctrl.connect()
        set_arm_torque_mode(sim, ctrl.arm_handles, torque=args.torque)
        print(f"sim:{args.arm} 7 arm joints -> {'force/torque' if args.torque else 'position'} control")

    elif args.command == "test":
        F = args.force
        if len(F) == 3:
            F = F + [0.0, 0.0, 0.0]
        elif len(F) != 6:
            raise SystemExit("--force expects 3 values (Fx Fy Fz) or 6 (Fx Fy Fz Mx My Mz)")
        run_force_test(
            args.arm, F, args.duration, args.gravity_comp, args.sensor,
            rate_hz=args.rate, host=args.host, port=args.port, max_torque=args.max_torque,
        )


if __name__ == "__main__":
    main()
