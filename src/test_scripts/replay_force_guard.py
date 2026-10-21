"""
Replay recorded insertions through the ForceGuard offline -- to tune the bone
rule (plan Phase 4) on real force-vs-depth data without moving anything.

Feeds a plunger v3 log (or every attempt in a search folder) tick by tick into
a fresh ForceGuard in *log* mode -- the recorded motion can't react, so every
decision is reported as what it would have done -- with config.yaml guard:
values, overridden by the flags. Prints the events per file and, with --plot,
draws force against measured depth with the events marked.

    python3 test_scripts/replay_force_guard.py ../recordings/plunger/20261020_101500_right.csv
    python3 test_scripts/replay_force_guard.py ../recordings/search/<run>/ --k-bone 8e5 --plot
    python3 test_scripts/replay_force_guard.py <files...> --set f_bone_min=1.5e6 t_persist_s=0.2

Fit the thresholds on half of the trials and check them on the other half.
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402

import numpy as np  # noqa: E402

from config import GUARD_CFG  # noqa: E402
from force_guard import ForceGuard, ForceSample, GuardConfig  # noqa: E402
from insertion import load_log  # noqa: E402


def files_from(paths: list[str]) -> list[str]:
    out = []
    for p in paths:
        if os.path.isdir(p):
            out += sorted(glob.glob(os.path.join(p, "attempt_*.csv")))
        else:
            out.append(p)
    return out


def replay(path: str, cfg: GuardConfig):
    header, c = load_log(path)
    ok = ~np.isnan(c["force"])
    t, cmd, meas, force, seq = (c[k][ok] for k in ("t_s", "cmd_depth_mm", "meas_depth_mm", "force", "force_seq"))
    age = c["force_age_ms"][ok] / 1000
    length = float(np.nanmax(c["cmd_depth_mm"])) + 1.0
    g = ForceGuard(cfg, length_mm=length, plan_speed_mm_s=1.0, mode="log")
    # Only the forward part of the recording: the guard's own retract would
    # differ from the recorded one.
    turn = int(np.argmax(cmd)) + 1
    for i in range(turn):
        g.update(float(t[i]), ForceSample(float(force[i]), float(age[i]), int(seq[i])),
                 float(cmd[i]), float(meas[i]))
        if g.finished:
            break
    return header, (t, cmd, meas, force), g


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("paths", nargs="+", help="v3 log files and/or search folders")
    p.add_argument("--f-contact", type=float)
    p.add_argument("--f-bone-min", type=float)
    p.add_argument("--k-bone", type=float)
    p.add_argument("--stall-ratio", type=float)
    p.add_argument("--t-persist", type=float)
    p.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="any other GuardConfig field")
    p.add_argument("--plot", action="store_true", help="force vs measured depth, events marked")
    args = p.parse_args()

    cfg = GuardConfig.from_dict(GUARD_CFG)
    cfg.f_abort = cfg.f_pull_abort = 1e18      # replay the whole run; trips are reported below anyway
    cfg.stale_s, cfg.max_dropped, cfg.lag_max_mm = 1e9, 10 ** 9, 1e9
    for key, val in (("f_contact", args.f_contact), ("f_bone_min", args.f_bone_min), ("k_bone", args.k_bone),
                     ("stall_ratio", args.stall_ratio), ("t_persist_s", args.t_persist)):
        if val is not None:
            setattr(cfg, key, val)
    for kv in args.set:
        k, _, v = kv.partition("=")
        setattr(cfg, k, None if v.lower() == "none" else float(v))

    files = files_from(args.paths)
    if not files:
        sys.exit("no log files found")
    plots = []
    for f in files:
        header, data, g = replay(f, cfg)
        name = os.path.relpath(f)
        print(f"\n{name}  ({header.get('attempt', '')}; units {header.get('force_units', '?')})")
        print(f"    max force {np.nanmax(data[3]):.6g} at {data[2][int(np.nanargmax(data[3]))]:.2f} mm measured")
        for e in g.events:
            print(f"    would: {e}")
        if not g.events:
            print("    no events")
        plots.append((name, data, g.events))

    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 5))
        for name, (t, cmd, meas, force), events in plots:
            line, = ax.plot(meas, force, lw=1, label=os.path.basename(name))
            for e in events:
                ax.plot(e.meas_depth_mm, e.force, "o", color=line.get_color())
                ax.annotate(e.kind.replace("would_", ""), (e.meas_depth_mm, e.force), fontsize=7)
        for k, c in (("f_contact", "g"), ("f_bone_min", "r")):
            v = getattr(cfg, k)
            if v is not None:
                ax.axhline(v, color=c, ls="--", lw=0.8, label=k)
        ax.set_xlabel("measured depth along the line (mm)")
        ax.set_ylabel("force")
        ax.grid(True, alpha=0.4)
        ax.legend(fontsize=7)
        plt.show()


if __name__ == "__main__":
    main()
