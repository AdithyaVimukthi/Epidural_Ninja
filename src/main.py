"""
Unified entry point for OpenArm control -- ties together arm_can.py (real
hardware), sim_bridge.py (CoppeliaSim) and kinematics.py (FK/IK) behind one
CLI and one interactive shell.

This file is only the CLI, the shell, and dispatch. Each drive mode lives in
its own module, and the pieces they share sit alongside:

  modes/mode_joint.py      joint      direct joint-angle control (also backs `home`)
  modes/mode_kinematic.py  kinematic  EE pose -> IK -> drive the solved angles
  modes/mode_imitate.py    imitate    real arm -> sim shadow, backdriven by hand
  modes/mode_line.py       line       EE along a straight line, in 3 speed-staged sections
  modes/mode_plunger.py    plunger    4-point plunging experiment, one point per command
  modes/mode_record.py     record     move the real arm by hand, save the motion
  modes/mode_replay.py     replay     play a saved recording on sim / real / both
  session.py                          per-side real/sim connection cache
  motion/motion.py                    real-arm ramps + joint-limit handling
  streaming.py                        per-tick senders + the threaded sim mirror
  recording.py                        the recording file format
  config/config.py                    loads config/config.yaml (settings + file paths)
  _bootstrap.py                       puts the sub-directories above on sys.path

--arm accepts left, right, or both (default: right). --mode accepts sim,
real, or both (default: sim, so nothing touches real hardware unless asked;
record, which needs the real arm, takes real or both).

Run from OpenArm_v3/src/ (python3 main.py ...); the sub-directory scripts
run as e.g. python3 comm/arm_can.py, and work from any directory.

Two ways to run it:

1. One-shot -- connects, runs one command, disconnects:

    python3 main.py joint --deg 0 -30 0 60 0 0 0                     # sim only (default), right arm
    python3 main.py joint --arm left --mode real --deg 0 -30 0 60 0 0 0
    python3 main.py joint --arm both --mode both --deg 0 0 0 0 0 0 0 --duration 3
    python3 main.py kinematic --pose 0.3 0.1 0.2                     # position-only IK -> sim
    python3 main.py imitate                                         # move it by hand, sim follows
    python3 main.py home --arm both --mode both
    python3 main.py line --start 0.30 -0.35 0.30 --angles 0 0 --depth 0.06
    python3 main.py plunger --point 1 --deg 0 30 0 60 0 0 0          # then --point 2, 3, 4
    python3 main.py replay --name demo                               # play a recording in sim

2. Interactive shell -- run with no arguments. Connections stay open across
   commands, so you can drive several moves back-to-back without
   reconnecting or re-enabling motors each time. Record here rather than
   one-shot: a one-shot run disables the motors as it exits, and the arm drops.

    $ python3 main.py
    openarm> joint --arm right --deg 10 -20 0 45 0 0 0 --mode both
    openarm> line --start 0.30 -0.35 0.30 --angles 15 0 --depth 0.06 --dry-run
    openarm> record --name demo --mode both
    openarm> replay --name demo --mode both
    openarm> home --arm both --mode both
    openarm> exit

   Ctrl+C cancels the current line or command without exiting the shell;
   `exit`/`quit` or Ctrl+D disconnects everything and leaves.

`--mode both` on joint/home is closed-loop: the real arm is commanded, and
after every tick its actual measured position (not the nominal waypoint) is
read back and written into sim. `--arm both` handles left then right in
sequence (imitate, record and replay are the exceptions -- they handle both
sides within the same tick loop, so left and right move in lockstep).

Real motors are always disabled on the way out -- end of the one-shot
command, `exit`/Ctrl+D from the shell, or SIGTERM either way.
"""
import argparse
import os
import readline  # noqa: F401 -- importing this enables arrow-key line editing
                  # and history for input() in run_repl(), even unused directly
import shlex
import signal

import _bootstrap  # noqa: F401 -- puts every src/ sub-directory on sys.path
from config import KP, KD, SIM_HOST, SIM_PORT
from session import Session
from mode_joint import cmd_joint
from mode_kinematic import cmd_kinematic
from mode_imitate import cmd_imitate
from mode_line import cmd_line
from mode_plunger import cmd_plunger
from mode_record import cmd_record
from mode_replay import cmd_replay
from recording import REPLAY_MAX_SPEED_DEG_S

HISTORY_FILE = os.path.expanduser("~/.openarm_shell_history")


class _ShutdownRequested(Exception):
    """Raised from the REPL's SIGTERM handler -- distinct from KeyboardInterrupt
    so a plain Ctrl+C at the prompt cancels a line instead of tearing the
    whole session down, while an actual termination signal still does."""


def dispatch(session: Session, args) -> None:
    if args.drive_mode == "joint":
        cmd_joint(session, args)
    elif args.drive_mode == "kinematic":
        cmd_kinematic(session, args)
    elif args.drive_mode == "imitate":
        cmd_imitate(session, args)
    elif args.drive_mode == "line":
        cmd_line(session, args)
    elif args.drive_mode == "plunger":
        cmd_plunger(session, args)
    elif args.drive_mode == "record":
        cmd_record(session, args)
    elif args.drive_mode == "replay":
        cmd_replay(session, args)
    elif args.drive_mode == "home":
        args.deg = [0.0] * 7
        args.gripper = None
        cmd_joint(session, args)


def build_parser() -> argparse.ArgumentParser:
    # --arm goes on each subcommand's own parser (not the top-level one) --
    # see docs/02_arm_can_script.md#why---arm-goes-after-the-subcommand for
    # the argparse gotcha this sidesteps.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--arm", choices=["left", "right", "both"], default="right",
                         help="which arm to drive (default: right)")
    # Separate from `common` because replay needs these but its own --arm default:
    # a subparser can't redefine an option it inherited, and set_defaults() on one
    # subparser would change the shared --arm for all of them.
    sim_opts = argparse.ArgumentParser(add_help=False)
    sim_opts.add_argument("--sim-host", default=SIM_HOST, help="CoppeliaSim ZMQ host")
    sim_opts.add_argument("--sim-port", type=int, default=SIM_PORT, help="CoppeliaSim ZMQ port")

    mode_help = ("sim/real drive just that backend; both drives the real arm and "
                 "mirrors its live measured position into sim (default: sim)")

    parser = argparse.ArgumentParser(description="OpenArm unified control entry point")
    # required=False: no subcommand at all means "start the interactive shell"
    sub = parser.add_subparsers(dest="drive_mode", required=False)

    p_joint = sub.add_parser("joint", parents=[common, sim_opts], help="direct joint-angle control")
    p_joint.add_argument("--mode", choices=["sim", "real", "both"], default="sim", help=mode_help)
    p_joint.add_argument("--deg", type=float, nargs=7, metavar="D",
                          help="7 target angles in degrees: joint1..joint7")
    p_joint.add_argument("--gripper", choices=["open", "close"], help="open or close the gripper")
    p_joint.add_argument("--duration", type=float, default=2.0, help="ramp time in seconds (default 2)")
    p_joint.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    p_joint.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")

    p_kin = sub.add_parser("kinematic", parents=[common, sim_opts], help="drive via IK-solved joint targets")
    p_kin.add_argument("--mode", choices=["sim", "real", "both"], default="sim", help=mode_help)
    p_kin.add_argument("--pose", type=float, nargs="+", required=True, metavar="V",
                        help="target EE pose: 'x y z' (position-only) or "
                             "'x y z roll pitch yaw' (rpy in degrees, full pose)")
    p_kin.add_argument("--limits", choices=["real", "sim"], default="real",
                        help="limit set constraining the IK solve: real = config.yaml, sim = URDF "
                             "(default: real; must be real with --mode real/both)")
    p_kin.add_argument("--duration", type=float, default=2.0, help="ramp time in seconds (default 2)")
    p_kin.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    p_kin.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")

    p_imi = sub.add_parser("imitate", parents=[common, sim_opts],
                            help="real arm -> sim shadow (backdrivable real arm mirrored into sim)")
    p_imi.add_argument("--rate", type=float, default=20.0, help="mirror rate in Hz (default 20)")
    p_imi.add_argument("--duration", type=float, default=None,
                        help="stop after N seconds (default: run until Ctrl+C)")

    p_line = sub.add_parser("line", parents=[common, sim_opts],
                             help="drive the EE along a straight line in 3 speed-staged sections")
    p_line.add_argument("--mode", choices=["sim", "real", "both"], default="sim", help=mode_help)
    p_line.add_argument("--start", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                         help="line start point in metres, robot_base frame")
    p_line.add_argument("--angles", type=float, nargs=2, required=True, metavar=("TILT", "ROT"),
                         help="direction in degrees: TILT away from straight-down, ROT about the "
                              "vertical axis setting which way the tilt points (0 0 = straight down)")
    p_line.add_argument("--depth", type=float, required=True,
                         help="distance to travel along the line, metres")
    p_line.add_argument("--speeds", type=float, nargs=3, default=[0.02, 0.005, 0.002],
                         metavar=("S1", "S2", "S3"),
                         help="m/s per section: approach, start->split, split->depth "
                              "(default: 0.02 0.005 0.002)")
    p_line.add_argument("--split", type=float, default=0.6,
                         help="fraction of depth where section 2 ends and 3 begins (default 0.6)")
    p_line.add_argument("--orient", choices=["path", "free"], default="path",
                         help="tool orientation: path = tool axis (TCP -X, wrist -> fingertips) held "
                              "parallel to the line, pointing along it, roll free; the approach turns "
                              "it onto the line. free = position-only IK (default: path)")
    p_line.add_argument("--limits", choices=["real", "sim"], default="real",
                         help="limit set constraining the IK solve: real = config.yaml, sim = URDF "
                              "(default: real; must be real with --mode real/both)")
    p_line.add_argument("--rate", type=float, default=None,
                         help="execution tick rate in Hz (default: the config.yaml control rate "
                              "when driving the real arm, 50 for sim only)")
    p_line.add_argument("--plan-step", type=float, default=0.002,
                         help="Cartesian spacing between IK-solved knots, metres (default 0.002). "
                              "Execution interpolates in joint space between them.")
    p_line.add_argument("--dry-run", action="store_true",
                         help="plan and report the path without moving anything (with --mode "
                              "real/both it still connects to read the arm's pose)")
    p_line.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    p_line.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")

    p_plg = sub.add_parser("plunger", parents=[common, sim_opts],
                            help="plunging experiment in 4 points, one per command: 1 safe pose "
                                 "(joint by joint), 2 joint3 to 0, 3 above the plunge (tool down), "
                                 "4 straight down (joint angles + torques saved to CSV)")
    p_plg.add_argument("--point", type=int, choices=[1, 2, 3, 4, 5], required=True,
                        help="which point to go to: 1 safe point, 2 joint3 to 0, 3 plunge start, "
                             "4 plunge to depth, 5 retract out of the material")
    p_plg.add_argument("--mode", choices=["sim", "real", "both"], default="sim", help=mode_help)
    p_plg.add_argument("--deg", type=float, nargs=7, metavar="D",
                        help="point 1: the safe pose, 7 angles in degrees (joint1..joint7), "
                             "reached one joint at a time from joint1 to joint7")
    p_plg.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="point 3: where the plunge starts, metres, robot_base frame, tool "
                             "pointing straight down. Point 4: goes there first if the EE isn't there")
    p_plg.add_argument("--depth", type=float,
                        help="points 3 and 4: plunge depth below point 3, metres")
    p_plg.add_argument("--speed", type=float, default=0.002,
                        help="point 4: descent speed, m/s (default 0.002)")
    p_plg.add_argument("--joint-speed", type=float, default=10.0,
                        help="points 1-3: average joint speed of each move, deg/s (default 10)")
    p_plg.add_argument("--name",
                        help="point 4: appended to the log's file name, "
                             "<date>_<time>_<side>_NAME.csv")
    p_plg.add_argument("--limits", choices=["real", "sim"], default="real",
                        help="limit set constraining the IK solve: real = config.yaml, sim = URDF "
                             "(default: real; must be real with --mode real/both)")
    p_plg.add_argument("--rate", type=float, default=None,
                        help="point 4: tick rate in Hz (default: the config.yaml control rate "
                             "when driving the real arm, 50 for sim only)")
    p_plg.add_argument("--plan-step", type=float, default=0.002,
                        help="points 3-4: Cartesian spacing between IK-solved knots of the "
                             "descent, metres (default 0.002)")
    p_plg.add_argument("--dry-run", action="store_true",
                        help="points 3-4: plan and report without moving anything")
    p_plg.add_argument("--no-axis-hold", action="store_true",
                        help="points 4-5, real arm: don't correct the needle's tilt / sideways drift under "
                             "load (config.yaml axis_hold)")
    p_plg.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    p_plg.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    # Needle load cell in the loop (point 4): docs/09_loadcell_force_feedback_plan.md
    g = p_plg.add_argument_group("load-cell guard, point 4 (modes/insertion.py)")
    g.add_argument("--guard", choices=["off", "log", "on"], default="off",
                   help="off: plain descent (default). log: constant speed, force logged, safety trips "
                        "abort+retract, bone only noted. on: contact / slow-down / bone stop / retract")
    g.add_argument("--loadcell", choices=["real", "fake"], default=None,
                   help="real board, or a fake one fed from a phantom model (default: real with the "
                        "real arm, fake in sim)")
    g.add_argument("--fake-phantom", default=None,
                   help="fake load cell: a config.yaml loadcell.fake_phantoms entry (sphere, slab, none)")
    g.add_argument("--insert-speed", type=float, default=None,
                   help="m/s after contact (default: config guard.insert_speed_mm_s)")
    g.add_argument("--on-bone", choices=["retract", "hold"], default="retract",
                   help="after bone: retract fully out of the material (default) or hold there")
    g.add_argument("--on-success", choices=["hold", "retract"], default="hold",
                   help="at the target depth: hold there (default) or retract out")
    g.add_argument("--f-contact", type=float, default=None, help="override guard.f_contact (guard units)")
    g.add_argument("--f-abort", type=float, default=None, help="override guard.f_abort (guard units)")
    g.add_argument("--search", action="store_true",
                   help="after a bone hit: retract out, re-aim through the same entry point (map-guided), "
                        "try again -- needs --guard on")
    g.add_argument("--map", default=None, help="occupancy map name (default: config map.name)")
    g.add_argument("--map-new", action="store_true",
                   help="start the map afresh (the old file is kept with a time stamp) -- do this "
                        "whenever the block has moved")
    g.add_argument("--map-skip-blocked", action="store_true",
                   help="--search: don't run a first straight-down line the map already blocks")
    g.add_argument("--max-attempts", type=int, default=None, help="--search: default config search.max_attempts")
    g.add_argument("--max-tilt", type=float, default=None, help="--search: degrees, default config search.max_tilt_deg")

    p_home = sub.add_parser("home", parents=[common, sim_opts],
                             help="ramp to the zero pose (shortcut for `joint --deg 0 0 0 0 0 0 0`)")
    p_home.add_argument("--mode", choices=["sim", "real", "both"], default="sim", help=mode_help)
    p_home.add_argument("--duration", type=float, default=2.0, help="ramp time in seconds (default 2)")
    p_home.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    p_home.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")

    p_rec = sub.add_parser("record", parents=[common, sim_opts],
                            help="move the real arm by hand and save the motion for replay")
    p_rec.add_argument("--mode", choices=["real", "both"], default="real",
                        help="real just records; both also mirrors the arm into sim live (default: real)")
    p_rec.add_argument("--name",
                        help="save as recordings/NAME.csv, or at NAME if it contains a directory "
                             "(default: rec_<date>_<time>)")
    p_rec.add_argument("--overwrite", action="store_true",
                        help="replace an existing recording with the same name")
    p_rec.add_argument("--rate", type=float, default=50.0, help="samples per second (default 50)")
    p_rec.add_argument("--duration", type=float, default=None,
                        help="stop after N seconds (default: run until Enter or Ctrl+C)")
    p_rec.add_argument("--countdown", type=float, default=3.0,
                        help="seconds to take hold of the arm before its joints go limp (default 3)")
    p_rec.add_argument("--damping", type=float, default=0.0,
                        help="kd while limp, Nm·s/rad (0-5): resists fast motion, so an arm "
                             "that's let go drops slower (default 0)")
    p_rec.add_argument("--no-trim", action="store_true",
                        help="keep the still stretches before the arm first moves and after it "
                             "last moves")
    p_rec.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="gain for holding the arm where it stopped (default from config.yaml)")
    p_rec.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="damping for holding the arm where it stopped (default from config.yaml)")

    p_rep = sub.add_parser("replay", parents=[sim_opts],
                            help="play back a recording on sim, the real arm, or both")
    p_rep.add_argument("--name", help="recording to play: a name from --list, or a path")
    p_rep.add_argument("--list", action="store_true", help="list the saved recordings and stop")
    p_rep.add_argument("--arm", choices=["left", "right", "both"], default=None,
                        help="which recorded arm(s) to play (default: every arm in the recording)")
    p_rep.add_argument("--mode", choices=["sim", "real", "both"], default="sim", help=mode_help)
    p_rep.add_argument("--speed", type=float, default=1.0,
                        help="playback speed: 0.5 = half speed, 2 = double (default 1)")
    p_rep.add_argument("--approach", type=float, default=3.0,
                        help="seconds to ramp from the current pose to the recording's first pose "
                             "(default 3)")
    p_rep.add_argument("--max-speed", type=float, default=REPLAY_MAX_SPEED_DEG_S,
                        help=f"refuse to move the real arm if any joint would move faster than this, "
                             f"deg/s, approach included (default {REPLAY_MAX_SPEED_DEG_S:g})")
    p_rep.add_argument("--rate", type=float, default=None,
                        help="tick rate in Hz (default: the config.yaml control rate when driving "
                             "the real arm, 50 for sim only)")
    p_rep.add_argument("--kp", type=float, nargs="+", default=KP, metavar="KP",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")
    p_rep.add_argument("--kd", type=float, nargs="+", default=KD, metavar="KD",
                        help="real arm only: 1 value for every joint or 7 values joint1..joint7 (default from config.yaml)")

    return parser


def run_repl(parser: argparse.ArgumentParser) -> None:
    def _on_sigterm(signum, frame):
        raise _ShutdownRequested

    signal.signal(signal.SIGTERM, _on_sigterm)

    readline.set_history_length(1000)
    try:
        readline.read_history_file(HISTORY_FILE)
    except FileNotFoundError:
        pass

    print("OpenArm interactive shell -- connections stay open between commands.")
    print("Commands: joint / kinematic / imitate / line / plunger / home / record / replay "
          "(same flags as the one-shot CLI).")
    print("Type 'help' for full usage, 'exit'/'quit' (or Ctrl+D) to disconnect and leave.\n")

    session = Session()
    try:
        while True:
            try:
                line = input("openarm> ").strip()
            except KeyboardInterrupt:
                print("\n(Ctrl+C -- type 'exit' or 'quit' to leave)")
                continue
            except EOFError:
                print()
                break

            if not line:
                continue
            if line in ("exit", "quit"):
                break
            if line in ("help", "-h", "--help"):
                parser.print_help()
                continue

            try:
                cmd_args = parser.parse_args(shlex.split(line))
            except SystemExit:
                continue  # argparse already printed usage/error -- back to the prompt
            if cmd_args.drive_mode is None:
                continue

            try:
                dispatch(session, cmd_args)
            except KeyboardInterrupt:
                print("Interrupted current command.")
            except Exception as e:
                print(f"Error: {e}")
    except _ShutdownRequested:
        print("\nReceived termination signal, shutting down...")
    finally:
        session.close()
        readline.write_history_file(HISTORY_FILE)


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.drive_mode is None:
        run_repl(parser)
        return

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    session = Session()
    try:
        dispatch(session, args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        session.close()


if __name__ == "__main__":
    main()
