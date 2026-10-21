"""
Central config loader. All scripts import from here:

    from config import CAN_LEFT, CAN_RIGHT, SEND_IDS, RECV_IDS, ...

Two files, in this folder:

  config.yaml       the defaults. Never written by any program -- edit it by hand.
  user_config.yaml  written by the GUI when it closes: the whole configuration
                    as it was left (gains, gravity switches, joint limits, ...)
                    plus a `gui:` section (window layout, every tab's inputs).
                    Laid over config.yaml key by key, so it wins wherever it
                    has a value -- for the GUI and the CLI alike. Delete it
                    (or Settings -> Reset to config.yaml) to go back to defaults.
"""
import copy
import math
import os
import sys
import xml.etree.ElementTree as ET
import yaml

_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_FILE = os.path.join(_CONFIG_DIR, "config.yaml")
CONFIG_FILE: str = _CONFIG_FILE
# A read-only install (the .deb, under /opt) can't write next to the code: its
# launchers set OPENARM_DATA_DIR (~/OpenArm), and user_config.yaml and a relative
# paths.recordings go there instead. Unset (this folder, the .run install): as before.
DATA_DIR: str | None = os.environ.get("OPENARM_DATA_DIR") or None
if DATA_DIR:
    DATA_DIR = os.path.abspath(os.path.expanduser(DATA_DIR))
    os.makedirs(DATA_DIR, exist_ok=True)
USER_CONFIG_FILE: str = os.path.join(DATA_DIR or _CONFIG_DIR, "user_config.yaml")

if not os.path.exists(_CONFIG_FILE):
    print(f"[config] ERROR: config file not found at {_CONFIG_FILE}", file=sys.stderr)
    sys.exit(1)

with open(_CONFIG_FILE) as _f:
    DEFAULT_CFG: dict = yaml.safe_load(_f)


def _merge(base: dict, over: dict) -> dict:
    """base with over laid on top: nested dicts merge, anything else is replaced."""
    out = dict(base)
    for k, v in over.items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _load_user_config() -> tuple[dict, str | None]:
    """(contents of user_config.yaml or {}, why it was ignored or None)."""
    if not os.path.exists(USER_CONFIG_FILE):
        return {}, None
    try:
        with open(USER_CONFIG_FILE) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        return {}, str(e)
    if not isinstance(data, dict):
        return {}, "not a YAML mapping"
    return data, None


_user, USER_CONFIG_ERROR = _load_user_config()
if USER_CONFIG_ERROR:
    print(f"[config] WARNING: ignoring {USER_CONFIG_FILE} ({USER_CONFIG_ERROR}) -- using config.yaml",
          file=sys.stderr)
USER_CONFIG_LOADED: bool = bool(_user)
# The GUI's own state (window layout, tab inputs) -- only gui/ reads it.
GUI_STATE: dict = _user.pop("gui", None) or {}
# What user_config.yaml itself holds (without gui:), before the merge.
USER_CFG: dict = copy.deepcopy(_user)
cfg: dict = _merge(copy.deepcopy(DEFAULT_CFG), _user)

# What the GUI writes to user_config.yaml (gui/userconfig.py), as dotted key
# prefixes. Nothing else: copying the whole configuration there froze every
# other value (paths.urdf, the gripper switch, the guard thresholds...) over
# later config.yaml edits. Anything else found in user_config.yaml was put
# there by hand (or by `loadcell.py calibrate --save`) -- kept, and announced.
GUI_MANAGED: tuple[str, ...] = ("control.kp", "control.kd", "control.joint_tauG_enable",
                                "R_right_arm", "R_left_arm", "sim.host", "sim.port")


def _overrides(base, over, path=""):
    if isinstance(base, dict) and isinstance(over, dict):
        for k, v in over.items():
            if k in base:
                yield from _overrides(base[k], v, f"{path}.{k}" if path else k)
    elif base != over:
        yield path, base, over


_shadowed = [o for o in _overrides(DEFAULT_CFG, USER_CFG)
             if not any(o[0] == g or o[0].startswith(g + ".") for g in GUI_MANAGED)]
if _shadowed:
    print("[config] note: user_config.yaml overrides config.yaml for " + ", ".join(
        f"{p} ({b!r} -> {o!r})" for p, b, o in _shadowed), file=sys.stderr)


def save_user_config(data: dict, path: str = USER_CONFIG_FILE) -> None:
    """Write data (a full config, optionally with a gui: section) to
    user_config.yaml, atomically. config.yaml is never touched."""
    header = ("# OpenArm user configuration -- written by the GUI when it closes.\n"
              "# Laid over config.yaml (the defaults, never edited) at start-up, for the GUI\n"
              "# and the CLI. Delete this file to go back to config.yaml.\n")
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(header)
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, allow_unicode=True, width=100)
    os.replace(tmp, path)

# ── file paths ────────────────────────────────────────────────────────────────
# config.yaml `paths:` entries are relative to the OpenArm_v3/ project root
# (two levels up from this file: OpenArm_v3/src/config/config.py).
PROJECT_ROOT : str = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


def _path(p: str) -> str:
    return os.path.normpath(os.path.join(PROJECT_ROOT, os.path.expanduser(p)))


URDF_PATH       : str            = _path(cfg["paths"]["urdf"])
SIM_SCENES      : dict[str, str] = {k: _path(v) for k, v in cfg["paths"].get("sim_scenes", {}).items()}
RECORDINGS_DIR  : str            = (os.path.join(DATA_DIR, os.path.expanduser(cfg["paths"]["recordings"]))
                                    if DATA_DIR else _path(cfg["paths"]["recordings"]))
DOCS_DIR        : str            = _path(cfg["paths"].get("docs", "docs"))
OPENARM_CAN_DIR : str            = _path(cfg["paths"]["openarm_can"])


def urdf_joint_limits(joint: str) -> tuple[float, float]:
    """(lower, upper) of a URDF joint -- radians for revolute, metres for prismatic."""
    for j in ET.parse(URDF_PATH).getroot().iter("joint"):
        if j.get("name") == joint:
            lim = j.find("limit")
            if lim is None:
                raise KeyError(f"joint {joint!r} has no <limit> in {URDF_PATH}")
            return float(lim.get("lower")), float(lim.get("upper"))
    raise KeyError(f"joint {joint!r} not found in {URDF_PATH}")


# ── CAN ───────────────────────────────────────────────────────────────────────
CAN_LEFT  : str   = cfg["can"]["left"]
CAN_RIGHT : str   = cfg["can"]["right"]
CAN_FD    : bool  = cfg["can"]["fd"]
CAN_BITRATE  : int = cfg["can"]["bitrate"]
CAN_DBITRATE : int = cfg["can"]["dbitrate"]

IFACE_BY_ARM: dict[str, str] = {"left": CAN_LEFT, "right": CAN_RIGHT}
SIDE_BY_IFACE: dict[str, str] = {iface: side for side, iface in IFACE_BY_ARM.items()}

# ── motors ────────────────────────────────────────────────────────────────────
SEND_IDS    : list[int] = cfg["motors"]["arm"]["send_ids"]
RECV_IDS    : list[int] = cfg["motors"]["arm"]["recv_ids"]
GRIPPER_SND : int       = cfg["motors"]["gripper"]["send_id"]
GRIPPER_RCV : int       = cfg["motors"]["gripper"]["recv_id"]
ARM_MOTOR_TYPES : list[str] = cfg["motors"]["arm"]["types"]
GRIPPER_TYPE    : str       = cfg["motors"]["gripper"]["type"]
# Per side: False where the gripper motor has been removed (the needle tool
# replaces it). config.yaml takes one bool for both arms or {left:, right:}.
_grip = cfg["motors"]["gripper"].get("enabled", True)
GRIPPER_ENABLED : dict[str, bool] = (
    {side: bool(_grip.get(side, True)) for side in ("left", "right")} if isinstance(_grip, dict)
    else {side: bool(_grip) for side in ("left", "right")})

# MIT torque range (Nm) per Damiao type -- tMax in openarm_can's
# dm_motor_constants.hpp. openarm_can encodes tau_ff over +-tMax, so it's also
# the most feedforward a joint can be sent.
MOTOR_T_MAX_NM: dict[str, float] = {
    "DM3507": 5, "DM4310": 10, "DM4310_48V": 10, "DM4340": 28, "DM4340_48V": 28,
    "DM6006": 20, "DM8006": 40, "DM8009": 54, "DM10010L": 200, "DM10010": 200,
    "DMH3510": 1, "DMH6215": 10, "DMG6220": 10,
}
_bad = [t for t in ARM_MOTOR_TYPES + [GRIPPER_TYPE] if t not in MOTOR_T_MAX_NM]
if len(ARM_MOTOR_TYPES) != 7 or _bad:
    print(f"[config] ERROR: motors.arm.types needs 7 known motor types, got {ARM_MOTOR_TYPES!r}"
          + (f" (unknown: {_bad})" if _bad else ""), file=sys.stderr)
    sys.exit(1)
# per arm joint, joint1..joint7
ARM_TAU_MAX_NM : list[float] = [float(MOTOR_T_MAX_NM[t]) for t in ARM_MOTOR_TYPES]

JOINT_NAMES : list[str] = cfg["motors"]["arm"]["names"] + [cfg["motors"]["gripper"]["name"]]

# ── limits: simulation ──────────────────────────────────────────────────────────
# Taken from the URDF (paths.urdf), per side -- for simulation/kinematics use,
# NOT for driving real hardware. See REAL_ARM_LIMITS_RAD below, which
# arm_can.py actually uses.
SIM_ARM_LIMITS_RAD: dict[str, list[tuple[float, float]]] = {
    side: [urdf_joint_limits(f"openarm_{side}_joint{i}") for i in range(1, 8)]
    for side in ("left", "right")
}

# ── limits: real hardware (per side -- right and left are NOT mirrors) ────────
# R_right_arm / R_left_arm in config.yaml hold the as-measured limits for each
# physical arm, including gripper open/close angles (whose sign convention
# also differs per side).
def _side_limits_rad(side_cfg: dict) -> list[tuple[float, float]]:
    limits = side_cfg["limits_deg"]
    return [
        (math.radians(limits[name][0]), math.radians(limits[name][1]))
        for name in JOINT_NAMES[:7]
    ]


def _side_gripper_rad(side_cfg: dict) -> dict[str, float]:
    g = side_cfg["gripper"]
    return {"open": math.radians(g["open"]), "close": math.radians(g["close"])}


# arm-only (excludes gripper), matches SEND_IDS/RECV_IDS order -- keyed by side
REAL_ARM_LIMITS_RAD: dict[str, list[tuple[float, float]]] = {
    "right": _side_limits_rad(cfg["R_right_arm"]),
    "left":  _side_limits_rad(cfg["R_left_arm"]),
}
# {"open": rad, "close": rad} per side
REAL_GRIPPER_RAD: dict[str, dict[str, float]] = {
    "right": _side_gripper_rad(cfg["R_right_arm"]),
    "left":  _side_gripper_rad(cfg["R_left_arm"]),
}

# ── control ───────────────────────────────────────────────────────────────────
# kp/kd in config.yaml are 8 values: joint1..joint7, then the gripper
def _gains(key: str) -> list[float]:
    v = cfg["control"][key]
    if not isinstance(v, list) or len(v) != 8:
        print(f"[config] ERROR: control.{key} needs 8 values (joint1..joint7, gripper), got {v!r}",
              file=sys.stderr)
        sys.exit(1)
    return [float(x) for x in v]


KP         : list[float] = _gains("kp")[:7]
KD         : list[float] = _gains("kd")[:7]
GRIPPER_KP : float       = _gains("kp")[7]
GRIPPER_KD : float       = _gains("kd")[7]
CTRL_HZ    : int         = cfg["control"]["ctrl_hz"]


def per_joint(gain, name: str = "gain") -> list[float]:
    """A gain for the 7 arm joints: one number (used for every joint) or 7 values."""
    if isinstance(gain, (int, float)):
        return [float(gain)] * 7
    gain = [float(g) for g in gain]
    if len(gain) == 1:
        return gain * 7
    if len(gain) != 7:
        raise ValueError(f"{name} needs 1 or 7 values (joint1..joint7), got {len(gain)}")
    return gain

# per-joint 0/1 mask for the gravity feedforward torque (joint1..joint7)
TAU_G_ENABLE : list[float] = [float(v) for v in cfg["control"].get("joint_tauG_enable", [0] * 7)]
if len(TAU_G_ENABLE) != 7:
    print(f"[config] ERROR: control.joint_tauG_enable needs 7 values, got {len(TAU_G_ENABLE)}",
          file=sys.stderr)
    sys.exit(1)

# ── needle tool, load cell, force guard, search, occupancy map ────────────────
# docs/09_loadcell_force_feedback_plan.md. Plain dicts: their users read the
# keys they need (with defaults), so an older user_config.yaml still works.
TOOL_TIP_OFFSET_M : list[float] = [float(v) for v in (cfg.get("tool") or {}).get("tip_offset_m", [0, 0, 0])]
LOADCELL_CFG : dict = dict(cfg.get("loadcell") or {})
GUARD_CFG    : dict = dict(cfg.get("guard") or {})
SEARCH_CFG   : dict = dict(cfg.get("search") or {})
MAP_CFG      : dict = dict(cfg.get("map") or {})
MAPS_DIR     : str  = os.path.join(RECORDINGS_DIR, MAP_CFG.get("dir", "maps"))
AXIS_HOLD_CFG: dict = dict(cfg.get("axis_hold") or {})

# ── CoppeliaSim (ZMQ Remote API) ────────────────────────────────────────────────
SIM_HOST : str = cfg["sim"]["host"]
SIM_PORT : int = cfg["sim"]["port"]
# both fingers, both sides share the same range in the URDF
SIM_GRIPPER_FINGER_RANGE_M: tuple[float, float] = urdf_joint_limits("openarm_left_finger_joint1")


def summary() -> str:
    user = ("not loaded: " + USER_CONFIG_ERROR if USER_CONFIG_ERROR
            else "loaded" if USER_CONFIG_LOADED else "none yet")
    lines = [
        f"Config: {_CONFIG_FILE}",
        f"  USER  {USER_CONFIG_FILE} ({user})",
        f"  URDF  {URDF_PATH}",
        f"  REC   {RECORDINGS_DIR}",
        f"  CAN   left={CAN_LEFT}  right={CAN_RIGHT}  fd={CAN_FD}"
        f"  bitrate={CAN_BITRATE} dbitrate={CAN_DBITRATE}",
        f"  ARM   {ARM_MOTOR_TYPES}  send={[hex(i) for i in SEND_IDS]}"
        f"  recv={[hex(i) for i in RECV_IDS]}",
        f"  GRIP  {GRIPPER_TYPE}  send={hex(GRIPPER_SND)}  recv={hex(GRIPPER_RCV)}",
        f"  GAINS kp={KP}  kd={KD}  ctrl={CTRL_HZ}Hz",
        f"  GRIP  kp={GRIPPER_KP}  kd={GRIPPER_KD}",
        f"  TAU_G enable={[int(v) for v in TAU_G_ENABLE]}",
        f"  TOOL  tip offset {TOOL_TIP_OFFSET_M} m (TCP frame)",
        f"  LOAD  port={LOADCELL_CFG.get('port', 'auto')}  sign={LOADCELL_CFG.get('sign', 1)}"
        f"  scale={LOADCELL_CFG.get('scale_counts_per_n')} counts/N  guard units={GUARD_CFG.get('units', 'counts')}",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    print(summary())
