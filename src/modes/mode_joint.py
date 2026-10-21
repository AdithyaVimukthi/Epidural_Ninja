"""
Joint mode -- direct joint-angle control, in degrees.

--mode sim / real drives just that one backend to the target. --mode both
drives the real arm and mirrors its actual measured position into sim every
tick, so sim shows what the arm is really doing rather than an idealized
copy ramped independently to the same target.

`home` is this mode with --deg fixed at all zeros.
"""
import math

from motion import ramp_real, drive_real_and_mirror_sim
from session import Session, resolve_sides


def cmd_joint(session: Session, args) -> None:
    if args.deg is None and args.gripper is None:
        print("Error: joint mode needs --deg and/or --gripper")
        return

    want_real = args.mode in ("real", "both")
    want_sim = args.mode in ("sim", "both")
    target_rad = [math.radians(d) for d in args.deg] if args.deg is not None else None

    for side in resolve_sides(args.arm):
        real_ctrl = session.real(side) if want_real else None
        sim_ctrl = session.sim(side, args.sim_host, args.sim_port) if want_sim else None

        if target_rad is not None:
            if args.mode == "both":
                print(f"[real+sim] driving {side} to {args.deg} deg over {args.duration}s "
                      f"-- sim mirrors the real arm's measured position live...")
                drive_real_and_mirror_sim(real_ctrl, sim_ctrl, target_rad, args.duration,
                                          kp=args.kp, kd=args.kd)
            elif real_ctrl is not None:
                print(f"[real] moving {side} to {args.deg} deg over {args.duration}s...")
                ramp_real(real_ctrl, target_rad, args.duration, kp=args.kp, kd=args.kd)
            elif sim_ctrl is not None:
                print(f"[sim] moving {side} to {args.deg} deg over {args.duration}s...")
                sim_ctrl.move_joints_over_time(target_rad, args.duration)

        if args.gripper is not None:
            action = "open_gripper" if args.gripper == "open" else "close_gripper"
            if real_ctrl is not None:
                print(f"[real] {side} {args.gripper} gripper...")
                getattr(real_ctrl, action)()
            if sim_ctrl is not None:
                print(f"[sim] {side} {args.gripper} gripper...")
                getattr(sim_ctrl, action)()

    print("Done.")
