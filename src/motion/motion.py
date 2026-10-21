"""
Shared motion primitives for the real arm -- joint-space ramps, plus the
limit handling those ramps need.

Two different limit behaviours live here on purpose:

  validate_target()   strict. The target the user actually asked for is
                      never silently corrected -- a typo'd angle fails loudly.
  clamp_to_limits()   lenient, waypoints only. A joint whose *live* position
                      is already outside its configured range would otherwise
                      block every joint, since ArmController.move_joints()
                      validates all 7 atomically; clamping the waypoint lets
                      the move proceed and pulls that joint back toward range.
"""
import math

from arm_can import ArmController, _TAU_MAX_NM
from sim_bridge import SimArmController
from config import CTRL_HZ, JOINT_NAMES
from trajectory import run_trajectory

# How often drive_real_and_mirror_sim() pushes the measured pose to sim. A sim
# call is a ZMQ round trip (often 5-30 ms), so doing it every control tick
# stalls the real arm's command stream; 20 Hz still looks live.
SIM_MIRROR_HZ = 20.0


def validate_target(target_rad: list[float], limits: list[tuple[float, float]]) -> None:
    for q, (lo, hi), name in zip(target_rad, limits, JOINT_NAMES[:7]):
        if not lo <= q <= hi:
            raise ValueError(
                f"{name} target = {math.degrees(q):.2f} deg out of range "
                f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}] deg"
            )


def clamp_to_limits(q: list[float], limits: list[tuple[float, float]]) -> list[float]:
    """Only ever pulls a joint toward its nearest boundary. Since the target
    is validated separately, this never commands a joint further out of range
    than it already is."""
    return [min(max(v, lo), hi) for v, (lo, hi) in zip(q, limits)]


def _send_path(real_ctrl: ArmController, kp, kd):
    """send(q, dq) for run_trajectory(): waypoints clamped into range, and no
    velocity target on a joint whose waypoint got clamped (it isn't following
    the path there)."""
    limits = real_ctrl.arm_limits_rad

    def send(q, dq):
        q_cmd = clamp_to_limits(q, limits)
        dq_cmd = [v if c == w else 0.0 for v, c, w in zip(dq, q_cmd, q)]
        real_ctrl.move_joints(q_cmd, kp=kp, kd=kd, dq=dq_cmd)
    return send


def warn_if_start_out_of_range(start: list[float], limits: list[tuple[float, float]]) -> None:
    offenders = [
        f"{JOINT_NAMES[i]}={math.degrees(q):.2f} deg (limit [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}])"
        for i, (q, (lo, hi)) in enumerate(zip(start, limits)) if not lo <= q <= hi
    ]
    if offenders:
        print(f"[real] currently outside configured limits, pulling back toward range: "
              f"{', '.join(offenders)}")


def ramp_start(real_ctrl: ArmController) -> tuple[list[float], str]:
    """Where a new real move should start: the setpoint the motors are still
    holding from the previous move, else the measured pose. Returns (q, source)."""
    measured = [s.position_rad for s in real_ctrl.read_joint_states()[:7]]
    held = real_ctrl.held_q
    if held is None or max(real_ctrl.held_kp) <= 0:
        return measured, "measured position"
    # A held arm sags below its setpoint by the error that produces its holding
    # torque; restarting from the measured pose zeroes that torque and drops the
    # arm. More error than the torque clamp can produce means the hold is gone.
    # Per joint, since each has its own kp (kp=0 holds nothing, so never counts).
    reach = [tau_max / kp + math.radians(2.0) if kp > 0 else math.inf
             for kp, tau_max in zip(real_ctrl.held_kp, _TAU_MAX_NM)]
    off = [abs(m - h) for m, h in zip(measured, held)]
    worst = max(range(7), key=lambda i: off[i] / reach[i])
    if off[worst] > reach[worst]:
        print(f"[real] {JOINT_NAMES[worst]} is {math.degrees(off[worst]):.1f} deg from its last "
              f"command, more than the motors could be holding -- starting from the measured position")
        return measured, "measured position"
    return list(held), "held setpoint"


def ramp_real(
    real_ctrl: ArmController, target_rad: list[float], duration_s: float,
    kp, kd, rate_hz: float = CTRL_HZ,
) -> None:
    """Minimum-jerk joint-space move (trajectory.py) from ramp_start() to
    target_rad, with each waypoint clamped into range so an out-of-limit
    reading can't block it."""
    validate_target(target_rad, real_ctrl.arm_limits_rad)
    start, _ = ramp_start(real_ctrl)
    warn_if_start_out_of_range(start, real_ctrl.arm_limits_rad)
    run_trajectory(start, target_rad, duration_s, rate_hz,
                   _send_path(real_ctrl, kp, kd)).report(rate_hz)


def drive_real_and_mirror_sim(
    real_ctrl: ArmController, sim_ctrl: SimArmController,
    target_rad: list[float], duration_s: float,
    kp, kd, rate_hz: float = CTRL_HZ,
) -> None:
    """Move the real arm to target_rad like ramp_real(), and every
    1/SIM_MIRROR_HZ read its actual measured position back and write *that*
    into sim -- so sim tracks what the arm really did (lag, limit clamps,
    disturbances), not an idealized trajectory independently run to the same
    target."""
    validate_target(target_rad, real_ctrl.arm_limits_rad)
    start, _ = ramp_start(real_ctrl)
    warn_if_start_out_of_range(start, real_ctrl.arm_limits_rad)

    last_mirror = -math.inf

    def mirror(t: float) -> None:
        nonlocal last_mirror
        if t - last_mirror < 1.0 / SIM_MIRROR_HZ and t < duration_s:
            return
        last_mirror = t
        actual = [s.position_rad for s in real_ctrl.read_joint_states()[:7]]
        try:
            sim_ctrl.set_joint_positions(actual)
        except ValueError as e:
            print(f"[sim] skipped tick: {e}")

    run_trajectory(start, target_rad, duration_s, rate_hz,
                   _send_path(real_ctrl, kp, kd), after_tick=mirror).report(rate_hz)
