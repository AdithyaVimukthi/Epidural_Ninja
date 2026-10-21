"""
The GUI's side of config/user_config.yaml (see config.py): the store for its
own state, live changes to the real-arm joint limits, and building the file
that's written when the GUI closes.

config.yaml is never written -- it stays the defaults.
"""
from __future__ import annotations

import copy
import math

from PySide6.QtCore import QByteArray, QSettings

import config as config_module
from config import (
    DEFAULT_CFG, GRIPPER_KD, GRIPPER_KP, GUI_STATE, JOINT_NAMES, REAL_ARM_LIMITS_RAD,
    REAL_GRIPPER_RAD,
)

SIDE_KEYS = {"right": "R_right_arm", "left": "R_left_arm"}


class Prefs:
    """QSettings-style value()/setValue() over the gui: section of
    user_config.yaml. Keys like "ui/tab" are nested (gui: ui: tab:). A key the
    file doesn't have yet falls back to the old QSettings store, so the first
    run after switching keeps the previous window layout and choices."""

    def __init__(self, data: dict):
        self.data = copy.deepcopy(data)
        self._legacy = QSettings("OpenArm", "OpenArmGUI")

    def value(self, key: str, default=None):
        node = self.data
        for part in key.split("/"):
            if not isinstance(node, dict) or part not in node:
                return self._legacy.value(key, default)
            node = node[part]
        if isinstance(node, dict) and set(node) == {"base64"}:
            return QByteArray.fromBase64(str(node["base64"]).encode())
        return node

    def setValue(self, key: str, value) -> None:
        if isinstance(value, (QByteArray, bytes)):
            value = {"base64": bytes(QByteArray(value).toBase64()).decode()}
        *parents, last = key.split("/")
        node = self.data
        for part in parents:
            if not isinstance(node.get(part), dict):
                node[part] = {}
            node = node[part]
        node[last] = value


def load_prefs() -> Prefs:
    return Prefs(GUI_STATE)


# ── real-arm joint limits ─────────────────────────────────────────────────────
# Shape used by the GUI, in degrees:
#   {side: {"joints": [[lo, hi]] * 7, "gripper": [open, close]}}

def current_limits_deg() -> dict:
    return {side: {"joints": [[math.degrees(lo), math.degrees(hi)] for lo, hi in REAL_ARM_LIMITS_RAD[side]],
                   "gripper": [math.degrees(REAL_GRIPPER_RAD[side]["open"]),
                               math.degrees(REAL_GRIPPER_RAD[side]["close"])]}
            for side in SIDE_KEYS}


def default_limits_deg() -> dict:
    """The limits config.yaml itself gives (ignoring user_config.yaml)."""
    out = {}
    for side, key in SIDE_KEYS.items():
        block = DEFAULT_CFG[key]
        out[side] = {"joints": [[float(v) for v in block["limits_deg"][n]] for n in JOINT_NAMES[:7]],
                     "gripper": [float(block["gripper"]["open"]), float(block["gripper"]["close"])]}
    return out


def limit_problems(limits: dict) -> list[str]:
    """Why these limits can't be applied (empty = fine)."""
    out = []
    for side, block in limits.items():
        for name, (lo, hi) in zip(JOINT_NAMES[:7], block["joints"]):
            if not lo < hi:
                out.append(f"{side} {name}: min {lo:g}° must be below max {hi:g}°")
        if block["gripper"][0] == block["gripper"][1]:
            out.append(f"{side} gripper: open and close can't be the same angle")
    return out


def apply_limits(limits: dict) -> list[str]:
    """Make these the real-arm limits for the rest of this run. The lists and
    dicts in config are changed in place, so every holder of them (connected
    ArmController / SimArmController, kinematics, replay) sees the new values
    on its next use. Returns a line per change."""
    changes = []
    for side, block in limits.items():
        old = current_limits_deg()[side]
        for j, ((lo, hi), (olo, ohi)) in enumerate(zip(block["joints"], old["joints"])):
            if (round(lo, 3), round(hi, 3)) != (round(olo, 3), round(ohi, 3)):
                changes.append(f"{side} {JOINT_NAMES[j]}: [{olo:g}, {ohi:g}] -> [{lo:g}, {hi:g}] deg")
        if [round(v, 3) for v in block["gripper"]] != [round(v, 3) for v in old["gripper"]]:
            changes.append(f"{side} gripper open/close: {old['gripper'][0]:g}/{old['gripper'][1]:g} -> "
                           f"{block['gripper'][0]:g}/{block['gripper'][1]:g} deg")
        REAL_ARM_LIMITS_RAD[side][:] = [(math.radians(lo), math.radians(hi)) for lo, hi in block["joints"]]
        REAL_GRIPPER_RAD[side].update(open=math.radians(block["gripper"][0]),
                                      close=math.radians(block["gripper"][1]))
    return changes


# ── the file ──────────────────────────────────────────────────────────────────

def build_user_config(kp: list[float], kd: list[float], tau_g_mask, sim_host: str, sim_port: int,
                      gui_state: dict) -> dict:
    """What the GUI edits (config.GUI_MANAGED) as it has it now, plus its own
    state, on top of whatever else user_config.yaml already held -- never a
    copy of the rest of the configuration, which config.yaml owns."""
    out = copy.deepcopy(config_module.USER_CFG)
    control = out.setdefault("control", {})
    control["kp"] = [float(v) for v in kp] + [GRIPPER_KP]
    control["kd"] = [float(v) for v in kd] + [GRIPPER_KD]
    control["joint_tauG_enable"] = [int(bool(v)) for v in tau_g_mask]
    for side, block in current_limits_deg().items():
        arm = out.setdefault(SIDE_KEYS[side], {})
        arm["limits_deg"] = {name: [round(lo, 3), round(hi, 3)]
                             for name, (lo, hi) in zip(JOINT_NAMES[:7], block["joints"])}
        arm["gripper"] = {"open": round(block["gripper"][0], 3), "close": round(block["gripper"][1], 3)}
    sim = out.setdefault("sim", {})
    sim["host"], sim["port"] = sim_host, int(sim_port)
    out["gui"] = copy.deepcopy(gui_state)
    return out
