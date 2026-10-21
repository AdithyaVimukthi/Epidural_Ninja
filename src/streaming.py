"""
Streaming joint positions tick by tick, shared by the modes that send a whole
path (`line`, `replay`) or mirror a moving arm (`record`): a send function per
backend, and the thread that mirrors the real arm into sim.
"""
import threading

from motion import clamp_to_limits
from sim_bridge import SimArmController

# Sim-only paths just need to look smooth.
SIM_RATE_HZ = 50.0
MIRROR_RATE_HZ = 30.0


class SimMirror:
    """Writes the latest real-arm reading into sim from its own thread, so a
    stalled ZMQ call (~80ms, occasionally) never holds up the next CAN frame."""

    def __init__(self, side: str, host: str, port: int):
        self.sim = SimArmController(side, host=host, port=port)
        self.sim.connect()
        self.latest: list[float] | None = None
        self.updates = 0
        self.skipped = 0
        self.error: Exception | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"sim-mirror-{side}", daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            self.error = TimeoutError("a sim write was still stuck 2 s after the motion ended, "
                                      "so sim may not show the final pose")
            return
        if self.error is not None or self.latest is None:
            return
        try:
            self._write(self.latest)   # land sim exactly on the arm's final pose
        except Exception as e:
            self.error = e

    def _write(self, q: list[float]) -> None:
        try:
            self.sim.set_joint_positions(q)
            self.updates += 1
        except ValueError:
            self.skipped += 1

    def _run(self) -> None:
        written = None
        period = 1.0 / MIRROR_RATE_HZ
        while not self._stop.is_set():
            q = self.latest
            if q is not None and q is not written:
                try:
                    self._write(q)
                except Exception as e:
                    self.error = e
                    return
                written = q
            self._stop.wait(period)


def real_sender(real_ctrl, kp, kd, mirror: SimMirror | None = None):
    limits = real_ctrl.arm_limits_rad

    def send(q):
        # A path point can sit a hair outside a limit (an IK answer exactly on one),
        # and move_joints() rejects anything out of range -- mid-path.
        real_ctrl.move_joints(clamp_to_limits([float(v) for v in q], limits), kp=kp, kd=kd)
        if mirror is not None:
            # Sag can carry a joint held at its limit just past it, and sim refuses
            # out-of-range poses: show it pinned at the limit rather than frozen.
            measured = [s.position_rad for s in real_ctrl.read_joint_states()[:7]]
            mirror.latest = clamp_to_limits(measured, limits)
    return send


def sim_sender(sim_ctrl):
    def send(q):
        try:
            sim_ctrl.set_joint_positions([float(v) for v in q])
        except ValueError as e:
            print(f"[sim] skipped waypoint: {e}")
    return send
