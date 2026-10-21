"""
Kinematic mode -- give an end-effector pose, solve IK, drive the result.

Once IK has turned the pose into joint angles, driving them is exactly joint
mode: the same MIT position ramp, the same strict target validation, the same
closed-loop real->sim mirroring under --mode both. The difference is that you
didn't pick those angles yourself, so before sending an unfamiliar pose to the
real arm, run it with --mode sim first and check the `max joint change` line.
"""
import math

import numpy as np
from spatialmath import SE3

from kinematics import ArmKinematics
from motion import ramp_real, ramp_start, drive_real_and_mirror_sim
from session import Session, resolve_sides


def cmd_kinematic(session: Session, args) -> None:
    pose = args.pose
    if len(pose) not in (3, 6):
        print("Error: --pose expects 3 values (x y z) or 6 values (x y z roll pitch yaw)")
        return

    want_real = args.mode in ("real", "both")
    want_sim = args.mode in ("sim", "both")
    if want_real and args.limits != "real":
        print("Error: --limits sim can't be combined with --mode real/both -- the solve must be "
              "constrained to the real arm's limits, or it can return angles the arm can't reach")
        return

    for side in resolve_sides(args.arm):
        km = ArmKinematics(side, limit_source=args.limits)
        print(km.limits_report())

        real_ctrl = session.real(side) if want_real else None
        sim_ctrl = session.sim(side, args.sim_host, args.sim_port) if want_sim else None

        # Seed the solver with the configuration the move will start from, so of
        # a 7-DOF arm's many solutions for one pose it returns the nearest -- the
        # smallest move. On the real arm that's what ramp_start() picks; seeding
        # from sim could land on a solution far from where the real arm is.
        if real_ctrl is not None:
            q_now_rad, source = ramp_start(real_ctrl)
            seed_from = f"real arm {source}"
        else:
            q_now_rad, seed_from = sim_ctrl.read_joint_positions(), "sim position"
        q_now_deg = [math.degrees(q) for q in q_now_rad]
        print(f"[{side}] seeding IK from {seed_from} (deg): "
              f"{np.array2string(np.array(q_now_deg), precision=2)}")

        if len(pose) == 3:
            sol = km.ikine_position(pose, q0_deg=q_now_deg)
            print(f"[{side}] IK target xyz={pose} (position-only, orientation free)")
        else:
            x, y, z, r, p, yaw = pose
            T = SE3(x, y, z) * SE3.RPY([r, p, yaw], unit="deg", order="xyz")
            sol = km.ikine(T, q0_deg=q_now_deg)
            print(f"[{side}] IK target xyz={pose[:3]} rpy(deg)={pose[3:]}")

        print(f"    success = {sol.success}  residual = {sol.residual:.5g}  nfev = {sol.iterations}")
        if not sol.success:
            print(f"    [{side}] IK failed: {sol.reason} -- skipping this arm, nothing moved")
            continue

        q_deg = np.degrees(sol.q)
        print(f"    q (deg) = {q_deg}")
        print(f"    max joint change from current: "
              f"{np.max(np.abs(q_deg - np.array(q_now_deg))):.2f} deg")

        target_rad = [float(q) for q in sol.q]
        if args.mode == "both":
            print(f"[real+sim] driving {side} to solved pose over {args.duration}s "
                  f"-- sim mirrors the real arm's measured position live...")
            drive_real_and_mirror_sim(real_ctrl, sim_ctrl, target_rad, args.duration,
                                      kp=args.kp, kd=args.kd)
        elif real_ctrl is not None:
            print(f"[real] moving {side} to solved pose over {args.duration}s...")
            ramp_real(real_ctrl, target_rad, args.duration, kp=args.kp, kd=args.kd)
        else:
            print(f"[sim] moving {side} to solved pose over {args.duration}s...")
            sim_ctrl.move_joints_over_time(target_rad, args.duration)

    print("Done.")
