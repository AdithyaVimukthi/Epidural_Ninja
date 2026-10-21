"""
Jobs the GUI runs on the worker that aren't CLI commands: enabling and
disabling one arm, holding it where it is, letting it go limp, printing its
state, and CoppeliaSim housekeeping (start/pause/stop, load a scene, list the
scene's joints). Bind the arguments with functools.partial; the worker passes
the session.
"""
import time

from arm_can import _print_states
from motion import clamp_to_limits, warn_if_start_out_of_range

from gui.backend import open_sim_client


# ── real arm ──────────────────────────────────────────────────────────────────

def enable(side: str, session) -> None:
    session.real(side)
    print(f"[real] {side} enabled -- it holds nothing until a move or Hold pose")


def disable(side: str, session) -> None:
    session.disable_real(side)


def hold_here(side: str, kp, kd, session) -> None:
    """Hold the arm at its measured pose -- what record does once a take ends."""
    ctrl = session.real(side)
    q = [s.position_rad for s in ctrl.read_joint_states()[:7]]
    warn_if_start_out_of_range(q, ctrl.arm_limits_rad)
    ctrl.move_joints(clamp_to_limits(q, ctrl.arm_limits_rad), kp=kp, kd=kd)
    print(f"[real] holding {side} where it is")


def go_limp(side: str, session) -> None:
    session.real(side).zero_torque()
    print(f"[real] {side} limp (zero torque, plus gravity feedforward on the joints "
          f"enabled in joint_tauG_enable) -- move it by hand")


def print_state(side: str, session) -> None:
    ctrl = session.real(side)
    _print_states(f"{ctrl.iface} ({side})", ctrl.read_joint_states())


# ── CoppeliaSim ───────────────────────────────────────────────────────────────

def _sim(host: str, port: int):
    client = open_sim_client(host, port)
    return client, client.require("sim")


def _state_name(sim, state: int) -> str:
    if state == sim.simulation_stopped:
        return "stopped"
    if state == sim.simulation_paused:
        return "paused"
    return "running"


def simulation(action: str, host: str, port: int, session) -> None:
    """Start, pause or stop the simulation. Kinematic writes (every mode's
    default) work either way; the sim gripper needs it running."""
    _client, sim = _sim(host, port)
    {"start": sim.startSimulation, "pause": sim.pauseSimulation, "stop": sim.stopSimulation}[action]()
    time.sleep(0.2)
    print(f"[sim] simulation {_state_name(sim, sim.getSimulationState())}")


def load_scene(path: str, host: str, port: int, session) -> None:
    _client, sim = _sim(host, port)
    if sim.getSimulationState() != sim.simulation_stopped:
        print("[sim] stopping the simulation first...")
        sim.stopSimulation()
        deadline = time.monotonic() + 10.0
        while sim.getSimulationState() != sim.simulation_stopped:
            if time.monotonic() > deadline:
                raise RuntimeError("the simulation didn't stop within 10 s -- scene not loaded")
            time.sleep(0.1)
    print(f"[sim] loading {path} ...")
    sim.loadScene(path)
    # Handles from the old scene are meaningless now.
    session.drop_sim(quiet=True)
    print("[sim] scene loaded")


def list_joints(host: str, port: int, session) -> None:
    _client, sim = _sim(host, port)
    handles = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type, 0)
    print(f"[sim] {len(handles)} joints in the scene:")
    for h in handles:
        print(f"  {sim.getObjectAlias(h, 0)}")
