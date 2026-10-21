"""
Smooth joint-space trajectories for the real arm: a minimum-jerk profile
sampled against the wall clock.

Why not a linear ramp stepped by tick count (what the ramps used to do):

  * A linear ramp starts and stops at full speed -- a velocity step, felt as
    a kick at both ends. Minimum-jerk (quintic) starts and ends with zero
    velocity and acceleration.
  * Indexing the setpoint by tick count means a late tick (the GUI thread
    holding the GIL, a slow sim call) freezes the setpoint, and the arm moves
    in stop-go steps. Here the setpoint comes from elapsed time, so a late
    tick just lands further along the same path.
  * Each sample also carries the path's velocity, sent as the MIT dq target:
    with dq=0 the kd term brakes the joint the whole way and it lags then
    lurches; with dq = path velocity kd helps it track instead.
"""
import time
from dataclasses import dataclass
from typing import Callable


def min_jerk(start: list[float], target: list[float], duration_s: float,
             t: float) -> tuple[list[float], list[float]]:
    """Position and velocity at time t (s) along the minimum-jerk path from
    start to target. Before 0 it's start, from duration_s on it's target,
    at rest at both ends."""
    if duration_s <= 0 or t >= duration_s:
        return list(target), [0.0] * len(target)
    tau = max(t, 0.0) / duration_s
    s = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)
    ds = 30 * tau ** 2 * (1 - tau) ** 2 / duration_s
    q = [a + (b - a) * s for a, b in zip(start, target)]
    dq = [(b - a) * ds for a, b in zip(start, target)]
    return q, dq


@dataclass
class TrajectoryStats:
    ticks: int
    worst_gap_s: float      # longest time between two consecutive commands

    def report(self, rate_hz: float, label: str = "[real]") -> None:
        """Print a note when the loop stalled for more than 3 periods."""
        if self.worst_gap_s > 3.0 / rate_hz:
            print(f"{label} control loop stalled up to {self.worst_gap_s * 1000:.0f} ms "
                  f"({self.ticks} commands) -- the path kept time, but the arm may have jerked")


def run_trajectory(
    start: list[float], target: list[float], duration_s: float, rate_hz: float,
    send: Callable[[list[float], list[float]], None],
    after_tick: Callable[[float], None] | None = None,
) -> TrajectoryStats:
    """Stream the minimum-jerk path from start to target: send(q, dq) at
    rate_hz on fixed deadlines, then once more at the target (dq = 0), which
    the motors keep holding. after_tick(t) runs after each send."""
    period = 1.0 / rate_hz
    t0 = prev = deadline = time.monotonic()
    ticks, worst_gap = 0, 0.0
    while True:
        now = time.monotonic()
        worst_gap = max(worst_gap, now - prev)
        prev = now
        t = now - t0
        q, dq = min_jerk(start, target, duration_s, t)
        send(q, dq)
        ticks += 1
        if after_tick is not None:
            after_tick(t)
        if t >= duration_s:
            return TrajectoryStats(ticks, worst_gap)
        deadline += period
        slack = deadline - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        elif slack < -period:
            deadline = time.monotonic()   # far behind: resync, don't burst
