"""The GUI writes only what it edits to user_config.yaml (config.GUI_MANAGED)."""
import config
from gui.userconfig import build_user_config


def flatten(d, prefix=""):
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            yield from flatten(v, key)
        else:
            yield key


def test_gui_writes_only_what_it_edits(monkeypatch):
    monkeypatch.setattr(config, "USER_CFG", {})
    out = build_user_config(config.KP, config.KD, [1] * 7, "localhost", 23000, {"ui": {"tab": 3}})
    for key in flatten({k: v for k, v in out.items() if k != "gui"}):
        assert any(key == g or key.startswith(g + ".") for g in config.GUI_MANAGED), key
    for frozen in ("paths", "motors", "can", "guard", "loadcell", "tool", "search", "map"):
        assert frozen not in out


def test_hand_edits_in_user_config_survive(monkeypatch):
    monkeypatch.setattr(config, "USER_CFG", {"loadcell": {"scale_counts_per_n": 219000.0}})
    out = build_user_config(config.KP, config.KD, [1] * 7, "localhost", 23000, {})
    assert out["loadcell"] == {"scale_counts_per_n": 219000.0}
