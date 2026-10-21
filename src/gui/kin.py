"""
Kinematics for the GUI, kept off the event loop: loading the URDF takes about
a second per arm and an IK solve can take several, so both run on one
background thread (Compute). FK for the live view is fast (~0.2 ms) and runs
on the GUI thread, on models Compute built at startup (ViewModels).
"""
from __future__ import annotations

import itertools
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from typing import Callable

import numpy as np
from PySide6.QtCore import QObject, Signal
from spatialmath import SE3

from kinematics import ArmKinematics

from gui.backend import SIDES


class Compute(QObject):
    """Runs fn() on a background thread and hands (result, error) to
    on_done on the GUI thread."""

    _done = Signal(int, object, object)

    def __init__(self):
        super().__init__()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="openarm-compute")
        self._ids = itertools.count()
        self._callbacks: dict[int, Callable] = {}
        self._models: dict[tuple[str, str], ArmKinematics] = {}   # compute thread only
        self._done.connect(self._deliver)

    def model(self, side: str, limits: str = "real") -> ArmKinematics:
        """Only call from inside a submitted fn."""
        key = (side, limits)
        if key not in self._models:
            self._models[key] = ArmKinematics(side, limit_source=limits)
        return self._models[key]

    def submit(self, fn: Callable[[], object], on_done: Callable[[object, Exception | None], None]) -> None:
        job_id = next(self._ids)
        self._callbacks[job_id] = on_done

        def done(fut, job_id=job_id):
            if fut.cancelled():   # shutting down
                return
            error = fut.exception()
            self._done.emit(job_id, None if error else fut.result(), error)

        self._pool.submit(fn).add_done_callback(done)

    def _deliver(self, job_id: int, result, error) -> None:
        callback = self._callbacks.pop(job_id, None)
        if callback is not None:
            callback(result, error)

    def shutdown(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)


class ViewModels:
    """One ArmKinematics per side for FK on the GUI thread."""

    def __init__(self, compute: Compute, on_ready: Callable[[], None] | None = None):
        self._models: dict[str, ArmKinematics] = {}
        self._on_ready = on_ready
        for side in SIDES:
            compute.submit(lambda side=side: ArmKinematics(side, limit_source="real"),
                           lambda model, error, side=side: self._loaded(side, model, error))

    def _loaded(self, side: str, model, error) -> None:
        if error is not None:
            print(f"[gui] couldn't load the {side} kinematic model: {error}")
            return
        self._models[side] = model
        if self._on_ready is not None and len(self._models) == len(SIDES):
            self._on_ready()

    def ready(self, side: str) -> bool:
        return side in self._models

    def chain(self, side: str, q_rad) -> np.ndarray | None:
        """(N, 3) positions of every frame from robot_base to the TCP."""
        model = self._models.get(side)
        if model is None or q_rad is None:
            return None
        return np.asarray(model.robot.fkine_all(np.asarray(q_rad, dtype=float)).t)

    def ee(self, side: str, q_rad) -> SE3 | None:
        model = self._models.get(side)
        if model is None or q_rad is None:
            return None
        return model.robot.fkine(np.asarray(q_rad, dtype=float))


def build_dynamics(side: str):
    """An ArmDynamics (Pinocchio). Imported here, not at the top, so Pinocchio
    loads on the compute thread instead of delaying start-up."""
    from dynamics import ArmDynamics
    return ArmDynamics(side)


class GravityModels:
    """One ArmDynamics per side for showing gravity torque on the GUI thread."""

    def __init__(self, compute: Compute):
        self._models: dict = {}
        for side in SIDES:
            compute.submit(partial(build_dynamics, side),
                           lambda model, error, side=side: self._loaded(side, model, error))

    def _loaded(self, side: str, model, error) -> None:
        if error is not None:
            print(f"[gui] couldn't load the {side} dynamics model: {error}")
            return
        self._models[side] = model

    def torque(self, side: str, q_rad) -> np.ndarray | None:
        """Gravity torque (Nm) at each of the 7 joints at q_rad, before the mask."""
        model = self._models.get(side)
        if model is None or q_rad is None:
            return None
        return np.asarray(model.gravity_torque(np.degrees(q_rad)))


def solve_ik(compute: Compute, side: str, limits: str, pose: list[float], seed_deg) -> object:
    """Same solve cmd_kinematic runs: position-only for 3 values, full pose
    (rpy in degrees, xyz order) for 6. Run it through compute.submit()."""
    km = compute.model(side, limits)
    if len(pose) == 3:
        return km.ikine_position(pose, q0_deg=seed_deg)
    x, y, z, r, p, yaw = pose
    T = SE3(x, y, z) * SE3.RPY([r, p, yaw], unit="deg", order="xyz")
    return km.ikine(T, q0_deg=seed_deg)
