"""
The GUI's link to the existing control code: one worker thread that runs the
drive modes (main.dispatch -> modes/*.py) exactly as the CLI shell does, the
capture of everything they print, and the live joint telemetry the GUI shows.

Threading model -- why it's shaped like this:

  * Every real-arm (CAN) call happens on the Worker thread. openarm_can isn't
    thread-safe, so the GUI never touches an ArmController itself; between
    commands the worker polls the connected arms so the live view stays current.
  * Sim state is read by SimMonitor over its own ZMQ connection -- the same
    trick streaming.SimMirror uses -- so the sim view stays live while a
    command is running on the worker.
  * Stop raises KeyboardInterrupt inside the worker thread. Every mode already
    handles that as Ctrl+C in the CLI shell (imitate/record catch it and finish
    cleanly, the rest unwind), so stopping behaves exactly as it does there.
"""
from __future__ import annotations

import ctypes
import dataclasses
import io
import os
import queue
import socket
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from typing import Callable

import zmq
from PySide6.QtCore import QObject, Signal

import _bootstrap  # noqa: F401 -- puts every src/ sub-directory on sys.path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import arm_can
import config as config_module
import mode_record
from session import Session
from sim_bridge import SimArmController

SIDES = ("left", "right")

# A ZMQ call CoppeliaSim hasn't answered in this long means it's gone (closed,
# or stuck in a modal dialog). The stock client would wait forever.
SIM_TIMEOUT_MS = 5000

# Scope history per (source, side): ~60 s at the 100 Hz control rate.
HISTORY_LEN = 6000


# ── console capture ───────────────────────────────────────────────────────────

class LogBuffer(io.TextIOBase):
    """Stands in for sys.stdout/sys.stderr: keeps every write for the GUI
    console (drained on a timer, so a mode printing every tick can't flood the
    event loop) and echoes it to the terminal the GUI was started from."""

    encoding = "utf-8"

    def __init__(self, echo=None):
        super().__init__()
        self._echo = echo
        self._lock = threading.Lock()
        self._chunks: list[str] = []

    def writable(self) -> bool:
        return True

    def isatty(self) -> bool:
        # Modes skip their \r status lines when this is False; the GUI shows
        # its own progress instead.
        return False

    def write(self, text: str) -> int:
        if not text:
            return 0
        with self._lock:
            self._chunks.append(text)
        if self._echo is not None:
            try:
                self._echo.write(text)
                self._echo.flush()
            except Exception:
                pass
        return len(text)

    def drain(self) -> str:
        with self._lock:
            text, self._chunks = "".join(self._chunks), []
        return text


# ── telemetry ─────────────────────────────────────────────────────────────────

@dataclass
class ArmReading:
    q: list[float] | None = None        # 7 arm joints, rad
    gripper: float | None = None        # real: rad; sim: finger opening, m
    vel: list[float] | None = None      # real only, rad/s
    tau: list[float] | None = None      # real only, Nm
    t: float = 0.0                      # time.monotonic() of the reading; 0 = never
    q_cmd: list[float] | None = None    # real only: the last move_joints() command
    t_cmd: float = 0.0

    def fresh(self, max_age: float, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.q is not None and now - self.t <= max_age

    def pose(self, max_age: float = 1.0) -> list[float] | None:
        """The newer of measured and commanded q, if either is recent. During a
        plain real ramp the modes only command (no reads), so the command is the
        best live picture of where the arm is going."""
        now = time.monotonic()
        best, best_t = None, -1.0
        if self.q is not None and now - self.t <= max_age:
            best, best_t = self.q, self.t
        if self.q_cmd is not None and now - self.t_cmd <= max_age and self.t_cmd > best_t:
            best = self.q_cmd
        return best


class Telemetry:
    """Latest reading per (source, side) plus a rolling history for the scope.
    source is "real" or "sim". Written by the worker and the sim monitor, read
    by the GUI thread; every access goes through one lock."""

    def __init__(self):
        self._lock = threading.Lock()
        keys = [(src, side) for src in ("real", "sim") for side in SIDES]
        self._now = {key: ArmReading() for key in keys}
        self._hist = {key: deque(maxlen=HISTORY_LEN) for key in keys}
        self.real_status = {side: "off" for side in SIDES}
        self.sim_status, self.sim_detail = "off", ""

    def real_read(self, side: str, states) -> None:
        now = time.monotonic()
        q = [s.position_rad for s in states[:7]]
        vel = [s.velocity for s in states[:7]]
        tau = [s.torque for s in states[:7]]
        with self._lock:
            r = self._now["real", side]
            r.q, r.vel, r.tau, r.gripper, r.t = q, vel, tau, states[7].position_rad, now
            self._hist["real", side].append((now, q, vel, tau))

    def real_cmd(self, side: str, q) -> None:
        with self._lock:
            r = self._now["real", side]
            r.q_cmd, r.t_cmd = [float(v) for v in q], time.monotonic()

    def real_forget(self, side: str) -> None:
        with self._lock:
            self._now["real", side] = ArmReading()

    def sim_read(self, side: str, q, gripper) -> None:
        now = time.monotonic()
        q = [float(v) for v in q]
        with self._lock:
            r = self._now["sim", side]
            r.q, r.gripper, r.t = q, gripper, now
            self._hist["sim", side].append((now, q, None, None))

    def set_sim_status(self, status: str, detail: str = "") -> None:
        self.sim_status, self.sim_detail = status, detail

    def snapshot(self) -> dict[tuple[str, str], ArmReading]:
        with self._lock:
            return {key: dataclasses.replace(r) for key, r in self._now.items()}

    def history(self, src: str, side: str, since: float) -> list[tuple]:
        """Samples newer than `since` (time.monotonic()), oldest first."""
        with self._lock:
            hist = self._hist[src, side]
            out = []
            for sample in reversed(hist):
                if sample[0] < since:
                    break
                out.append(sample)
        out.reverse()
        return out


# ── session ───────────────────────────────────────────────────────────────────

def real_status(ctrl) -> str:
    if not ctrl._enabled:
        return "disabled"
    if ctrl._limp_kd is not None:
        return "limp"
    if ctrl.held_q is not None and max(ctrl.held_kp) > 0:
        return "holding"
    return "enabled"


class GuiSession(Session):
    """Session whose real controllers also report every reading and command
    to the telemetry, and which can drop one arm or the sim connections."""

    def __init__(self, telemetry: Telemetry):
        super().__init__()
        self.telemetry = telemetry

    def real(self, side: str):
        fresh = side not in self._real
        ctrl = super().real(side)
        if fresh:
            self._instrument(side, ctrl)
            self._sync_gravity(ctrl)
        return ctrl

    @staticmethod
    def _sync_gravity(ctrl) -> None:
        # The GUI may have switched joints after this controller copied
        # TAU_G_ENABLE in its __init__.
        ctrl._tau_g_mask = list(arm_can.TAU_G_ENABLE)
        if any(ctrl._tau_g_mask) and ctrl._dyn is None:
            from dynamics import ArmDynamics
            ctrl._dyn = ArmDynamics(ctrl.side)

    def _instrument(self, side: str, ctrl) -> None:
        # Instance attributes shadow the methods, so the controller's own calls
        # (move_joints_over_time, release_arm, ...) report too.
        read, move, tel = ctrl.read_joint_states, ctrl.move_joints, self.telemetry

        def read_joint_states():
            states = read()
            tel.real_read(side, states)
            return states

        def move_joints(positions_rad, *args, **kwargs):
            move(positions_rad, *args, **kwargs)
            tel.real_cmd(side, positions_rad)

        ctrl.read_joint_states = read_joint_states
        ctrl.move_joints = move_joints

    @property
    def real_ctrls(self) -> dict:
        return dict(self._real)

    def real_status(self, side: str) -> str:
        ctrl = self._real.get(side)
        return "off" if ctrl is None else real_status(ctrl)

    def disable_real(self, side: str) -> None:
        ctrl = self._real.pop(side, None)
        if ctrl is None:
            return
        print(f"[real] disabling {side} arm...")
        try:
            ctrl.disable()
        except Exception as e:
            print(f"[real] WARNING disabling {side} failed: {e}")
        self.telemetry.real_forget(side)

    def drop_sim(self, quiet: bool = False) -> None:
        if self._sim and not quiet:
            print("[sim] dropped the sim connections -- the next command reconnects")
        self._sim.clear()

    def close(self) -> None:
        # Unlike Session.close(), one arm failing to disable doesn't leave the
        # other enabled.
        for side in list(self._real):
            self.disable_real(side)
        self._sim.clear()
        try:
            self.close_loadcell()
        except Exception as e:
            print(f"[loadcell] WARNING closing it failed: {e}")


# ── gravity feedforward switches ──────────────────────────────────────────────

def set_gravity_mask(session: GuiSession, mask) -> list[str]:
    """Switch the gravity feedforward per joint (control.joint_tauG_enable) for
    the rest of this run: arms enabled from now on start with it, connected
    ones switch at once. Returns the connected sides that still need a
    dynamics model -- made while every joint was off, so Pinocchio was never
    loaded; hand one to attach_dynamics().

    Called from the GUI thread, possibly mid-command: it only swaps attributes
    that ArmController reads on every command it sends, and sends nothing.
    The module value is set first so a controller being created right now
    either sees it or is already in the session by the time we look."""
    mask = [float(m) for m in mask]
    arm_can.TAU_G_ENABLE = config_module.TAU_G_ENABLE = mask
    need = []
    for side, ctrl in session.real_ctrls.items():
        ctrl._tau_g_mask = list(mask)
        if any(mask) and ctrl._dyn is None:
            need.append(side)
    return need


def attach_dynamics(session: GuiSession, side: str, dyn) -> None:
    """Give a connected controller the dynamics model it was made without.
    _dyn only ever goes from None to a model, which the worker's reads of it
    handle whichever side of the change they land on."""
    ctrl = session.real_ctrls.get(side)
    if ctrl is not None and ctrl._dyn is None:
        ctrl._dyn = dyn


# ── CoppeliaSim connections that fail fast ────────────────────────────────────

def open_sim_client(host: str, port: int) -> RemoteAPIClient:
    """A RemoteAPIClient that can't hang the GUI: a TCP probe first (a client
    pointed at a closed port waits forever for its first reply), a send/receive
    timeout, and no linger so dropping a dead connection returns at once."""
    try:
        with socket.create_connection((host, port), timeout=1.0):
            pass
    except OSError as e:
        raise ConnectionError(f"CoppeliaSim isn't reachable at {host}:{port} "
                              f"({e.strerror or e}) -- start it and load the OpenArm scene") from None
    client = RemoteAPIClient(host=host, port=port)
    client.socket.setsockopt(zmq.RCVTIMEO, SIM_TIMEOUT_MS)
    client.socket.setsockopt(zmq.SNDTIMEO, SIM_TIMEOUT_MS)
    client.socket.setsockopt(zmq.LINGER, 0)
    return client


def _sim_connect(self: SimArmController) -> None:
    """SimArmController.connect(), through open_sim_client()."""
    self.client = open_sim_client(self.host, self.port)
    self.sim = self.client.require("sim")
    self._resolve_handles()


def sim_error_text(e: Exception) -> str:
    if isinstance(e, zmq.Again):
        return f"CoppeliaSim didn't answer within {SIM_TIMEOUT_MS / 1000:g} s"
    return str(e) or type(e).__name__


def install_hooks() -> None:
    """The only behaviour the GUI changes in the CLI code."""
    # record stops on Enter only when stdin is a terminal; under the GUI the
    # Stop button (-> KeyboardInterrupt, the CLI's Ctrl+C) ends the take.
    mode_record._stdin_is_terminal = lambda: False
    SimArmController.connect = _sim_connect


# ── worker ────────────────────────────────────────────────────────────────────

def _async_raise(thread_id: int | None, exc_type) -> None:
    """Raise exc_type in a thread at its next bytecode; None cancels one that
    hasn't landed yet. This is how Stop delivers Ctrl+C to the worker."""
    if thread_id is None:
        return   # not started
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_ulong(thread_id), None if exc_type is None else ctypes.py_object(exc_type))


@dataclass
class Job:
    label: str
    fn: Callable[[GuiSession], None]


class WorkerSignals(QObject):
    started = Signal(str)
    finished = Signal(str, str, float)   # label, "ok" / "stopped" / "error", seconds


_QUIT = object()


class Worker(threading.Thread):
    """Runs one job at a time against a GuiSession; polls the real arms while idle."""

    POLL_S = 0.05

    def __init__(self, telemetry: Telemetry):
        super().__init__(name="openarm-worker", daemon=True)
        self.signals = WorkerSignals()
        self.telemetry = telemetry
        self.session = GuiSession(telemetry)
        self.poll_real = True
        self._jobs: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._armed = False
        self._current: Job | None = None
        self._t0 = 0.0
        self._poll_error_at = 0.0

    # ── called from the GUI thread ─────────────────────────────────────────

    @property
    def busy(self) -> bool:
        return self._current is not None or not self._jobs.empty()

    @property
    def current_label(self) -> str | None:
        job = self._current
        return job.label if job is not None else None

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._t0 if self._current is not None else 0.0

    def submit(self, label: str, fn: Callable[[GuiSession], None]) -> bool:
        """Queue a job if nothing is running or waiting. False if busy."""
        with self._lock:
            if self.busy:
                return False
            self._jobs.put(Job(label, fn))
        return True

    def enqueue(self, label: str, fn: Callable[[GuiSession], None]) -> None:
        """Queue behind whatever is running -- for disable-all after a Stop."""
        self._jobs.put(Job(label, fn))

    def interrupt(self) -> bool:
        with self._lock:
            if not self._armed:
                return False
            _async_raise(self.ident, KeyboardInterrupt)
            return True

    def shutdown(self, timeout: float = 5.0) -> None:
        """Stop the running job, disable every real arm, end the thread."""
        self.interrupt()
        self._jobs.put(_QUIT)
        self.join(timeout)

    # ── worker thread ──────────────────────────────────────────────────────

    def run(self) -> None:
        while True:
            try:
                try:
                    job = self._jobs.get(timeout=self.POLL_S)
                except queue.Empty:
                    self._poll()
                    continue
                if job is _QUIT:
                    break
                self._run(job)
            except KeyboardInterrupt:
                # A Stop that landed just as a job was finishing.
                self._finish("stopped")
            except Exception:
                traceback.print_exc()
        try:
            self.session.close()
        except Exception as e:
            print(f"Error while disconnecting: {e}")

    def _run(self, job: Job) -> None:
        self._current, self._t0 = job, time.monotonic()
        self.signals.started.emit(job.label)
        outcome = "error"
        try:
            self._set_armed(True)
            try:
                job.fn(self.session)
            finally:
                self._set_armed(False)
            outcome = "ok"
        except KeyboardInterrupt:
            outcome = "stopped"
            print("Stopped.")
            # A Stop can land between a ZMQ request and its reply, which leaves
            # that connection unusable; reconnecting next time costs ~0.1 s.
            self.session.drop_sim(quiet=True)
        except SystemExit:
            pass   # argparse already printed why
        except Exception as e:
            print(f"Error: {sim_error_text(e)}")
            traceback.print_exc(file=sys.__stderr__)
            if isinstance(e, zmq.ZMQError):
                self.session.drop_sim()
        finally:
            self._finish(outcome)

    def _finish(self, outcome: str) -> None:
        self._set_armed(False)
        job, self._current = self._current, None
        if job is None:
            return
        self._update_status()
        self.signals.finished.emit(job.label, outcome, time.monotonic() - self._t0)

    def _set_armed(self, armed: bool) -> None:
        with self._lock:
            self._armed = armed
            if not armed:
                _async_raise(self.ident, None)

    def _poll(self) -> None:
        if self.poll_real:
            for side, ctrl in self.session.real_ctrls.items():
                try:
                    ctrl.read_joint_states()
                except Exception as e:
                    now = time.monotonic()
                    if now - self._poll_error_at > 5.0:
                        print(f"[real] {side}: reading state failed: {e}")
                        self._poll_error_at = now
        self._update_status()

    def _update_status(self) -> None:
        for side in SIDES:
            self.telemetry.real_status[side] = self.session.real_status(side)


# ── sim monitor ───────────────────────────────────────────────────────────────

class SimMonitor(threading.Thread):
    """Reads both simulated arms over its own connection, so the sim view stays
    live while the worker is busy writing to sim."""

    RETRY_S = 2.0

    def __init__(self, telemetry: Telemetry, host: str, port: int, rate_hz: float = 10.0):
        super().__init__(name="openarm-sim-monitor", daemon=True)
        self.telemetry = telemetry
        self.host, self.port = host, port
        self.rate_hz = rate_hz
        self.enabled = True
        self._quit = threading.Event()
        self._reset = threading.Event()

    def configure(self, host: str, port: int) -> None:
        self.host, self.port = host, port
        self._reset.set()

    def reconnect(self) -> None:
        self._reset.set()

    def stop(self) -> None:
        self._quit.set()

    def run(self) -> None:
        ctrls: dict[str, SimArmController] = {}
        while not self._quit.is_set():
            if self._reset.is_set():
                ctrls.clear()
                self._reset.clear()
            if not self.enabled:
                ctrls.clear()
                self.telemetry.set_sim_status("off", "live view off")
                self._quit.wait(0.3)
                continue
            t0 = time.monotonic()
            errors, unreachable = {}, False
            for side in SIDES:
                try:
                    if side not in ctrls:
                        ctrl = SimArmController(side, host=self.host, port=self.port)
                        ctrl.connect()
                        ctrls[side] = ctrl
                    ctrl = ctrls[side]
                    self.telemetry.sim_read(side, ctrl.read_joint_positions(),
                                            ctrl.read_gripper_position())
                except Exception as e:
                    ctrls.pop(side, None)
                    errors[side] = sim_error_text(e)
                    if isinstance(e, (ConnectionError, zmq.ZMQError)):
                        unreachable = True
                        break   # the other side won't do better
            if not errors:
                self.telemetry.set_sim_status("online", f"{self.host}:{self.port}")
            elif unreachable or len(errors) == len(SIDES):
                self.telemetry.set_sim_status("offline", next(iter(errors.values())))
            else:
                # e.g. a scene holding only one arm
                self.telemetry.set_sim_status("partial", "; ".join(f"{s}: {m}" for s, m in errors.items()))
            if errors:
                self._quit.wait(self.RETRY_S)
                continue
            self._quit.wait(max(0.0, 1.0 / self.rate_hz - (time.monotonic() - t0)))
