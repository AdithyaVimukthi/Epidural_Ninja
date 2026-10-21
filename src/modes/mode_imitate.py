"""
Imitate mode -- real arm to sim shadow.

Puts the real arm in zero-torque/backdrivable mode (like arm_can.py's
`read`) and continuously mirrors its live joint positions onto the simulated
arm. Real is always the source, sim is always the sink. With --arm both,
each tick reads and writes both sides, so left/right shadow in lockstep.

Sim trouble never ends imitate: a joint moved by hand past its config.yaml
limit is shown pinned at that limit, and a failed sim write (CoppeliaSim
frozen past the GUI's ZMQ timeout -- its periodic scene auto-save can stall
it for seconds -- or closed) drops that connection and reconnects about
once a second until sim answers again.
"""
import time

from motion import clamp_to_limits
from session import Session, resolve_sides

# Between reconnect attempts while sim is unreachable.
_SIM_RETRY_S = 1.0
# A sim write slower than this is reported, so freezes show up in the log.
_SIM_SLOW_S = 1.0


class _SimLink:
    """One side's sim connection, reconnected after a failed write."""

    def __init__(self, ctrl, side: str):
        self.ctrl = ctrl
        self.side = side
        self.down_at: float | None = None

    def write(self, q: list[float]) -> None:
        if self.down_at is not None:
            if time.time() - self.down_at < _SIM_RETRY_S:
                return
            try:
                self.ctrl.connect()
            except Exception:
                self.down_at = time.time()
                return
            print(f"[sim] {self.side}: reconnected -- mirroring again")
            self.down_at = None
        t0 = time.time()
        try:
            self.ctrl.set_joint_positions(q)
        except Exception as e:
            # A ZMQ request that timed out leaves its socket unusable: reconnect.
            self.down_at = time.time()
            print(f"[sim] {self.side}: write failed ({type(e).__name__}: {e}) -- real arm "
                  f"unaffected; reconnecting every {_SIM_RETRY_S:g} s")
            return
        took = time.time() - t0
        if took > _SIM_SLOW_S:
            print(f"[sim] {self.side}: CoppeliaSim stalled {took:.1f} s (scene auto-save?)")


def cmd_imitate(session: Session, args) -> None:
    sides = resolve_sides(args.arm)
    print(f"[imitate] real:{'+'.join(sides)} -> sim:{'+'.join(sides)} shadow.")

    real_ctrls = {side: session.real(side) for side in sides}
    sim_links = {side: _SimLink(session.sim(side, args.sim_host, args.sim_port), side)
                 for side in sides}

    for ctrl in real_ctrls.values():
        ctrl.zero_torque()
    print(f"[real] {'/'.join(sides)} in zero-torque mode -- move by hand. "
          f"Mirroring into sim at {args.rate} Hz. Ctrl+C to stop.")

    period = 1.0 / args.rate
    t_end = time.time() + args.duration if args.duration else None
    try:
        while t_end is None or time.time() < t_end:
            t0 = time.time()
            for side in sides:
                ctrl = real_ctrls[side]
                q = [s.position_rad for s in ctrl.read_joint_states()[:7]]
                # Sim refuses out-of-range poses: show a joint pushed past its
                # limit pinned there rather than freezing the whole arm.
                sim_links[side].write(clamp_to_limits(q, ctrl.arm_limits_rad))
            sleep = period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nInterrupted.")

    print("Done.")
