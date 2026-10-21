"""
Low-level CAN read/write interface to one OpenArm v2.0 arm (7 joints + gripper)
over a SAVVYCANFD channel, using the openarm_can library.

Library / init sequence per docs/01_hardware_setup.md: motors must be enabled
with CallbackMode.IGNORE, drained with recv_all(), THEN switched to STATE mode
-- otherwise the enable-response frame gets misread as a state update.

Usage as a library:

    from arm_can import ArmController

    arm = ArmController("can0")     # or ArmController.for_side("right")
    arm.enable()
    states = arm.read_joint_states()
    arm.move_joints([0.0] * 7)      # radians, MIT position hold
    arm.disable()

Usage as a CLI:

    python3 comm/arm_can.py status --arm right
    python3 comm/arm_can.py params --arm right
    python3 comm/arm_can.py read --arm right --rate 20
    python3 comm/arm_can.py zero --arm right --duration 2
    python3 comm/arm_can.py move --arm right --deg 0 -30 0 60 0 0 0 --duration 2
    python3 comm/arm_can.py gripper --arm right --open
    python3 comm/arm_can.py gripper --arm right --close

Press Ctrl+C at any time -- motors are always disabled before the process exits.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path
import signal
import time
from dataclasses import dataclass

from trajectory import run_trajectory

from config import (
    SEND_IDS, RECV_IDS, GRIPPER_SND, GRIPPER_RCV, JOINT_NAMES,
    REAL_ARM_LIMITS_RAD, REAL_GRIPPER_RAD, IFACE_BY_ARM, SIDE_BY_IFACE, KP, KD, CTRL_HZ,
    GRIPPER_KP, GRIPPER_KD, TAU_G_ENABLE, OPENARM_CAN_DIR, per_joint,
    ARM_MOTOR_TYPES, GRIPPER_TYPE, ARM_TAU_MAX_NM, GRIPPER_ENABLED,
)

# openarm_can is a system apt package living outside the venv. Put its
# directory on sys.path only for this one import, then take it back out --
# leaving it there breaks any script that later imports spatialmath, because
# mpl_toolkits is a namespace package and so merges across every sys.path
# entry regardless of order, pulling in the system matplotlib (3.5.1)
# alongside the venv's (3.10.9). Location: config.yaml paths.openarm_can.
_DIST_PACKAGES = OPENARM_CAN_DIR
_added_dist_packages = _DIST_PACKAGES not in sys.path
if _added_dist_packages:
    sys.path.append(_DIST_PACKAGES)
# Without it (a computer with no arm: sim only) everything else still works;
# connecting to the real arm then says why it can't.
OPENARM_CAN_ERROR: str | None = None
try:
    import openarm_can as oa
except ImportError as e:
    oa = None
    OPENARM_CAN_ERROR = (f"openarm_can isn't installed ({e}) -- the real arm needs it: "
                         f"sudo add-apt-repository ppa:openarm/main && sudo apt install python3-openarm-can")
finally:
    if _added_dist_packages:
        sys.path.remove(_DIST_PACKAGES)

GAIN_HELP = "1 value for every joint, or 7 values joint1..joint7 (default from config.yaml)"

# MIT torque range per arm joint (config.yaml motors.arm.types) -- the most
# tau_ff that can be encoded for each motor.
_TAU_MAX_NM: list[float] = ARM_TAU_MAX_NM


@dataclass
class JointState:
    name: str
    position_rad: float
    velocity: float
    torque: float

    @property
    def position_deg(self) -> float:
        return math.degrees(self.position_rad)


def _clamp(value: float, lo: float, hi: float, label: str) -> float:
    if not lo <= value <= hi:
        raise ValueError(
            f"{label} = {value:.4f} rad ({math.degrees(value):.2f} deg) "
            f"out of range [{lo:.4f}, {hi:.4f}] rad "
            f"([{math.degrees(lo):.1f}, {math.degrees(hi):.1f}] deg)"
        )
    return value


class ArmController:
    """Read/write CAN interface to the 7 arm joints + gripper on one channel.

    Real-hardware joint and gripper limits differ per side (right and left
    are NOT mirror images of each other -- see config.yaml R_right_arm /
    R_left_arm), so every ArmController needs to know which side it's
    driving. If `side` isn't given explicitly, it's inferred from
    `can_interface` via the configured can0/can1 <-> right/left mapping;
    that only fails for a non-standard --iface, where `side` must be passed.
    """

    def __init__(self, can_interface: str, side: str | None = None, can_fd: bool = True):
        if oa is None:
            raise RuntimeError(OPENARM_CAN_ERROR)
        self.iface = can_interface
        self.side = side or SIDE_BY_IFACE.get(can_interface)
        if self.side is None:
            raise ValueError(
                f"can't infer side (left/right) for iface {can_interface!r} -- "
                f"pass side='left' or side='right' explicitly"
            )
        self.arm_limits_rad = REAL_ARM_LIMITS_RAD[self.side]
        self.gripper_rad = REAL_GRIPPER_RAD[self.side]

        self.arm = oa.OpenArm(can_interface, can_fd)
        # The declared types set the ranges openarm_can encodes MIT commands and
        # decodes state with -- they must match the motors (arm_can.py params).
        self.arm.init_arm_motors([getattr(oa.MotorType, t) for t in ARM_MOTOR_TYPES],
                                 SEND_IDS, RECV_IDS)
        # motors.gripper.enabled: false once the gripper motor is gone -- then
        # nothing here touches it (reads report it as 0).
        self.has_gripper = GRIPPER_ENABLED[self.side]
        if self.has_gripper:
            self.arm.init_gripper_motor(getattr(oa.MotorType, GRIPPER_TYPE), GRIPPER_SND, GRIPPER_RCV)
        self.arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
        self._enabled = False
        # The last position command sent by move_joints() and its kp -- what the
        # motors keep servoing to until told otherwise. None when nothing is held.
        self.held_q: list[float] | None = None
        self.held_kp = [0.0] * 7
        # kd of the current limp (kp=0) command, or None when not limp. While limp,
        # read_joint_states() re-sends the command so the gravity feedforward
        # follows the arm as it's moved by hand.
        self._limp_kd: float | None = None

        # Gravity feedforward, masked per joint by control.joint_tauG_enable.
        # Pinocchio is only loaded when at least one joint uses it.
        self._tau_g_mask = list(TAU_G_ENABLE)
        self._dyn = None
        if any(self._tau_g_mask):
            from dynamics import ArmDynamics
            self._dyn = ArmDynamics(self.side)

    @classmethod
    def for_side(cls, side: str, can_fd: bool = True) -> "ArmController":
        if side not in IFACE_BY_ARM:
            raise ValueError(f"side must be one of {list(IFACE_BY_ARM)}, got {side!r}")
        return cls(IFACE_BY_ARM[side], side=side, can_fd=can_fd)

    def enable(self) -> None:
        """Required init sequence: enable -> drain responses -> switch to STATE."""
        if self._enabled:
            return
        self.arm.enable_all()
        self.arm.recv_all()
        self.arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self._enabled = True
        self.held_q = None
        self._limp_kd = None

    def disable(self) -> None:
        if not self._enabled:
            return
        self.arm.disable_all()
        self.arm.recv_all()
        self._enabled = False
        self.held_q = None
        self._limp_kd = None

    # ── gravity feedforward ───────────────────────────────────────────────────

    def gravity_ff(self, positions_rad: list[float]) -> list[float]:
        """Gravity torque (Nm) at positions_rad for the joints enabled in
        joint_tauG_enable, 0 for the rest, clamped to each motor's torque range."""
        if self._dyn is None:
            return [0.0] * 7
        tau_g = self._dyn.gravity_torque([math.degrees(q) for q in positions_rad[:7]])
        return [
            min(max(float(t) * m, -lim), lim)
            for t, m, lim in zip(tau_g, self._tau_g_mask, _TAU_MAX_NM)
        ]

    def _send_limp(self, positions_rad: list[float]) -> None:
        tau = self.gravity_ff(positions_rad)
        self.arm.get_arm().mit_control_all([oa.MITParam(0, self._limp_kd, 0, 0, t) for t in tau])
        self.arm.recv_all()

    # ── motor registers ───────────────────────────────────────────────────────

    def query_params(self, rids=None) -> dict[str, dict[int, float]]:
        """Read registers (default PMAX/VMAX/TMAX -- the ranges the motor itself
        uses to decode MIT commands) from all 7 joints + gripper. Needs the motors
        disabled; sends no motion command. Returns {joint name: {rid: value}},
        -1 where a motor didn't answer."""
        if self._enabled:
            raise RuntimeError("query_params() needs the motors disabled")
        if rids is None:
            rids = (oa.PMAX, oa.VMAX, oa.TMAX)
        # get_motors() returns copies, so it's re-read after every receive.
        def motors():
            grip = self.arm.get_gripper().get_motors() if self.has_gripper else []
            return self.arm.get_arm().get_motors() + grip

        self.arm.set_callback_mode_all(oa.CallbackMode.PARAM)
        try:
            for rid in rids:
                for _ in range(3):   # re-ask: replies can get lost on a busy bus
                    self.arm.query_param_all(rid.value)
                    for _ in range(10):
                        self.arm.recv_all(2000)
                        if all(m.get_param(rid.value) != -1 for m in motors()):
                            break
                    if all(m.get_param(rid.value) != -1 for m in motors()):
                        break
        finally:
            self.arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
        return {name: {rid.value: m.get_param(rid.value) for rid in rids}
                for name, m in zip(JOINT_NAMES, motors())}

    # ── read ──────────────────────────────────────────────────────────────────

    def read_joint_states(self) -> list[JointState]:
        """Poll fresh state frames and return all 7 joints + gripper, in order."""
        self.arm.refresh_all()
        self.arm.recv_all()
        states = []
        for name, m in zip(JOINT_NAMES[:7], self.arm.get_arm().get_motors()):
            states.append(JointState(name, m.get_position(), m.get_velocity(), m.get_torque()))
        if self.has_gripper:
            g = self.arm.get_gripper().get_motors()[0]
            states.append(JointState(JOINT_NAMES[7], g.get_position(), g.get_velocity(), g.get_torque()))
        else:
            states.append(JointState(JOINT_NAMES[7], 0.0, 0.0, 0.0))
        if self._limp_kd is not None and self._dyn is not None:
            self._send_limp([s.position_rad for s in states[:7]])
        return states

    # ── write ─────────────────────────────────────────────────────────────────

    def zero_torque(self) -> None:
        """MIT command with kp=kd=0 -- arm goes limp / backdrivable by hand. The
        enabled joints still get the gravity feedforward, updated on every
        read_joint_states(), so they float instead of dropping."""
        if self.has_gripper:
            self.arm.get_gripper().mit_control_all([oa.MITParam(0, 0, 0, 0, 0)])
            self.arm.recv_all()
        self.release_arm(kd=0.0)

    def release_arm(self, kd: float = 0.0) -> None:
        """MIT command with kp=0 on the 7 arm joints only: limp and backdrivable by
        hand, with kd resisting speed (0 = none), plus the gravity feedforward on the
        enabled joints (kept current by read_joint_states()). The gripper keeps its
        last command."""
        self._limp_kd = kd
        self.held_q = None
        if self._dyn is None:
            self._send_limp([0.0] * 7)
        else:
            self.read_joint_states()   # reads the pose and sends the limp command

    def move_joints(self, positions_rad: list[float], kp=KP, kd=KD, dq=None) -> None:
        """Send one MIT position command to all 7 arm joints (radians), with the
        gravity torque at the commanded pose as feedforward on the joints enabled in
        joint_tauG_enable. The motors keep servoing to it until the next command.
        kp/kd: one number for every joint, or 7 per-joint values.
        dq: 7 target velocities (rad/s) when streaming a path -- kd then damps
        the error from the path's speed instead of braking toward 0. Default 0."""
        if len(positions_rad) != 7:
            raise ValueError(f"need exactly 7 joint positions, got {len(positions_rad)}")
        kp, kd = per_joint(kp, "kp"), per_joint(kd, "kd")
        dq = [0.0] * 7 if dq is None else [float(v) for v in dq]
        if len(dq) != 7:
            raise ValueError(f"need exactly 7 joint velocities, got {len(dq)}")
        q_cmd = [
            _clamp(q, lo, hi, name)
            for q, (lo, hi), name in zip(positions_rad, self.arm_limits_rad, JOINT_NAMES[:7])
        ]
        tau = self.gravity_ff(q_cmd)
        self.arm.get_arm().mit_control_all(
            [oa.MITParam(p, d, q, v, t) for p, d, q, v, t in zip(kp, kd, q_cmd, dq, tau)])
        self.arm.recv_all()
        self._limp_kd = None
        self.held_q, self.held_kp = [float(q) for q in q_cmd], kp

    def set_gripper(self, position_rad: float, kp: float = GRIPPER_KP, kd: float = GRIPPER_KD) -> None:
        """Move the gripper to an arbitrary target, clamped to this side's open/close span."""
        if not self.has_gripper:
            raise RuntimeError("no gripper (config.yaml motors.gripper.enabled is false)")
        lo, hi = sorted((self.gripper_rad["open"], self.gripper_rad["close"]))
        q = _clamp(position_rad, lo, hi, "gripper")
        self.arm.get_gripper().mit_control_all([oa.MITParam(kp, kd, q, 0.0, 0.0)])
        self.arm.recv_all()

    def open_gripper(self, kp: float = GRIPPER_KP, kd: float = GRIPPER_KD) -> None:
        self.set_gripper(self.gripper_rad["open"], kp=kp, kd=kd)

    def close_gripper(self, kp: float = GRIPPER_KP, kd: float = GRIPPER_KD) -> None:
        self.set_gripper(self.gripper_rad["close"], kp=kp, kd=kd)

    def move_joints_over_time(
        self, target_rad: list[float], duration_s: float,
        kp=KP, kd=KD, rate_hz: float = CTRL_HZ,
    ) -> None:
        """Move from the current position to target_rad over duration_s along a
        minimum-jerk path (trajectory.py), sending MIT commands at rate_hz. Avoids
        slamming a big position error (and the torque spike that comes with it)
        into the motors in one step."""
        if len(target_rad) != 7:
            raise ValueError(f"need exactly 7 joint positions, got {len(target_rad)}")
        for q, (lo, hi), name in zip(target_rad, self.arm_limits_rad, JOINT_NAMES[:7]):
            _clamp(q, lo, hi, name)

        start = [s.position_rad for s in self.read_joint_states()[:7]]
        run_trajectory(start, target_rad, duration_s, rate_hz,
                       lambda q, dq: self.move_joints(q, kp=kp, kd=kd, dq=dq)).report(rate_hz)


def _print_states(iface: str, states: list[JointState]) -> None:
    print(f"-- {iface} --")
    print(f"  {'joint':<10}{'deg':>9}{'rad':>9}{'vel rad/s':>12}{'torque Nm':>12}")
    for s in states:
        print(f"  {s.name:<10}{s.position_deg:>9.2f}{s.position_rad:>9.4f}"
              f"{s.velocity:>12.3f}{s.torque:>12.3f}")


def _print_params(iface: str, ctrl: ArmController) -> None:
    """Print each motor's PMAX/VMAX/TMAX next to the range openarm_can encodes
    MIT commands with for the declared type. A TMAX mismatch scales tau_ff."""
    params = ctrl.query_params()
    motors = ctrl.arm.get_arm().get_motors() + (ctrl.arm.get_gripper().get_motors() if ctrl.has_gripper else [])
    print(f"-- {iface} motor registers (motor | library, declared type) --")
    print(f"  {'joint':<10}{'PMAX':>14}{'VMAX':>14}{'TMAX':>14}   tau_ff scale")
    for (name, p), m in zip(params.items(), motors):
        lim = oa.Motor.get_limit_param(m.get_motor_type())
        cells = [f"{p[r.value]:6.1f}|{v:<6.1f}" for r, v in
                 ((oa.PMAX, lim.pMax), (oa.VMAX, lim.vMax), (oa.TMAX, lim.tMax))]
        t = p[oa.TMAX.value]
        scale = f"x{t / lim.tMax:.2f}" if t > 0 else "no reply"
        print(f"  {name:<10}" + "".join(f"{c:>14}" for c in cells) + f"   {scale}")


def main() -> None:
    # shared by every subcommand so --arm/--iface go after the subcommand,
    # e.g. `arm_can.py status --arm right` (defining these on the top-level
    # parser too would let a subparser's unset default silently clobber them)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--arm", choices=["left", "right"], help="use the configured left/right CAN channel")
    common.add_argument("--iface", help="override with an explicit SocketCAN interface, e.g. can0")

    parser = argparse.ArgumentParser(description="Read/write CAN interface to an OpenArm arm")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", parents=[common], help="enable, print one state reading, disable")
    sub.add_parser("params", parents=[common],
                   help="read each motor's PMAX/VMAX/TMAX registers (motors stay disabled)")

    p_read = sub.add_parser("read", parents=[common], help="continuously print joint states (zero-torque, backdrivable)")
    p_read.add_argument("--rate", type=float, default=10.0, help="polling rate in Hz (default 10)")

    p_zero = sub.add_parser("zero", parents=[common], help="move all 7 joints to 0 rad")
    p_zero.add_argument("--duration", type=float, default=2.0, help="ramp time in seconds")
    p_zero.add_argument("--kp", type=float, nargs="+", default=KP, help=GAIN_HELP)
    p_zero.add_argument("--kd", type=float, nargs="+", default=KD, help=GAIN_HELP)

    p_move = sub.add_parser("move", parents=[common], help="move all 7 joints to target angles (degrees)")
    p_move.add_argument("--deg", type=float, nargs=7, required=True, metavar="D",
                         help="7 target angles in degrees: joint1..joint7")
    p_move.add_argument("--duration", type=float, default=2.0, help="ramp time in seconds")
    p_move.add_argument("--kp", type=float, nargs="+", default=KP, help=GAIN_HELP)
    p_move.add_argument("--kd", type=float, nargs="+", default=KD, help=GAIN_HELP)

    p_grip = sub.add_parser("gripper", parents=[common], help="open or close the gripper")
    grip_group = p_grip.add_mutually_exclusive_group(required=True)
    grip_group.add_argument("--open", action="store_true")
    grip_group.add_argument("--close", action="store_true")

    args = parser.parse_args()

    if args.iface:
        iface = args.iface
    elif args.arm:
        iface = IFACE_BY_ARM[args.arm]
    else:
        parser.error("specify --arm {left,right} or --iface canN")

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    print(f"Connecting to {iface}...")
    ctrl = ArmController(iface, side=args.arm)
    if args.command == "params":
        _print_params(iface, ctrl)
        return
    ctrl.enable()
    try:
        if args.command == "status":
            _print_states(iface, ctrl.read_joint_states())

        elif args.command == "read":
            ctrl.zero_torque()
            print(f"{iface} in zero-torque mode -- move it by hand. Ctrl+C to stop.")
            period = 1.0 / args.rate
            while True:
                t0 = time.time()
                _print_states(iface, ctrl.read_joint_states())
                sleep = period - (time.time() - t0)
                if sleep > 0:
                    time.sleep(sleep)

        elif args.command == "zero":
            print(f"Moving {iface} to zero over {args.duration}s...")
            ctrl.move_joints_over_time([0.0] * 7, args.duration, kp=args.kp, kd=args.kd)

        elif args.command == "move":
            target = [math.radians(d) for d in args.deg]
            print(f"Moving {iface} to {args.deg} deg over {args.duration}s...")
            ctrl.move_joints_over_time(target, args.duration, kp=args.kp, kd=args.kd)

        elif args.command == "gripper":
            if args.open:
                ctrl.open_gripper()
            else:
                ctrl.close_gripper()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print(f"Disabling {iface}...")
        ctrl.disable()
        print("Done.")


if __name__ == "__main__":
    main()
