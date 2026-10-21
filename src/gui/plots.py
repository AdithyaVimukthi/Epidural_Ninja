"""
matplotlib canvases for the GUI: the live 3D view of both arms, the joint
plot of a recording, and the rolling scope. All share the window's dark theme.
"""
from __future__ import annotations

import math

import matplotlib
import matplotlib.patches
matplotlib.use("QtAgg")
import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

from gui.backend import SIDES

BG = "#1e2027"
PANEL = "#262932"
FG = "#d7dae0"
MUTED = "#8b919e"
GRID = "#3a3f4b"
SIDE_COLOR = {"left": "#4fc3f7", "right": "#ffb74d"}
GHOST_COLOR = "#e57fff"
PATH_COLOR = "#ef5350"
JOINT_COLORS = ["#4e79a7", "#f28e2b", "#e15759", "#76b7b2", "#59a14f", "#edc948", "#b07aa1"]

# Where both arms can be: robot_base frame, metres.
_XLIM, _YLIM, _ZLIM = (-0.35, 0.65), (-0.6, 0.6), (-0.15, 0.95)
_SHOULDER_Z = 0.698   # height of both arm mounts on the torso (URDF)

matplotlib.rcParams.update({
    "figure.facecolor": BG, "axes.facecolor": PANEL, "savefig.facecolor": BG,
    "axes.edgecolor": GRID, "axes.labelcolor": FG, "text.color": FG,
    "xtick.color": MUTED, "ytick.color": MUTED, "grid.color": GRID,
    "legend.facecolor": PANEL, "legend.edgecolor": GRID, "font.size": 9,
})


class ArmView3D(FigureCanvasQTAgg):
    """Stick figures of both arms, real (solid) and sim (dashed), with overlays
    for the active tab: a ghost pose (dotted), target points, a planned path.
    Drag to orbit, scroll to zoom."""

    def __init__(self, parent=None):
        fig = Figure(figsize=(5, 5))
        super().__init__(fig)
        self.setParent(parent)
        self.ax = ax = fig.add_subplot(111, projection="3d")
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        ax.set_facecolor(BG)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_pane_color((0.15, 0.16, 0.2, 1.0))
            axis._axinfo["grid"]["color"] = GRID
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        self._apply_limits()

        ax.plot([0, 0], [0, 0], [0, _SHOULDER_Z], color=MUTED, lw=6, solid_capstyle="round")
        ax.plot([-0.1, 0.1], [0, 0], [0, 0], color=MUTED, lw=3)
        self._lines = {}
        for side in SIDES:
            c = SIDE_COLOR[side]
            self._lines["real", side], = ax.plot([], [], [], color=c, lw=3, marker="o", ms=4)
            self._lines["sim", side], = ax.plot([], [], [], color=c, lw=1.5, ls="--", marker="o",
                                                ms=3, alpha=0.7)
            self._lines["ghost", side], = ax.plot([], [], [], color=GHOST_COLOR, lw=1.8, ls=":",
                                                  marker="o", ms=3)
            self._lines["target", side], = ax.plot([], [], [], color=GHOST_COLOR, ls="", marker="X",
                                                   ms=10)
        self._lines["path"], = ax.plot([], [], [], color=PATH_COLOR, lw=2, marker="o", ms=4)
        self._lines["approach"], = ax.plot([], [], [], color=PATH_COLOR, lw=1, ls="--")
        # Plunger tab: occupancy-map hits and the insertion attempts' lines.
        self._lines["hits"], = ax.plot([], [], [], color="#ef5350", ls="", marker="o", ms=7, alpha=0.85)
        self._lines["attempts"], = ax.plot([], [], [], color="#ffa726", lw=1.3)
        ax.legend(handles=[
            Line2D([], [], color=SIDE_COLOR["left"], lw=3, label="left"),
            Line2D([], [], color=SIDE_COLOR["right"], lw=3, label="right"),
            Line2D([], [], color=MUTED, lw=3, label="real"),
            Line2D([], [], color=MUTED, lw=1.5, ls="--", label="sim"),
            Line2D([], [], color=GHOST_COLOR, lw=1.8, ls=":", label="target / preview"),
        ], loc="upper left", fontsize=8, framealpha=0.8)
        self.reset_view()
        self._shown = {}

    def _apply_limits(self) -> None:
        self.ax.set_xlim(*_XLIM)
        self.ax.set_ylim(*_YLIM)
        self.ax.set_zlim(*_ZLIM)
        self.ax.set_box_aspect((_XLIM[1] - _XLIM[0], _YLIM[1] - _YLIM[0], _ZLIM[1] - _ZLIM[0]))

    def reset_view(self, elev: float = 20, azim: float = -50) -> None:
        self.ax.view_init(elev=elev, azim=azim)
        self._apply_limits()
        self.draw_idle()

    def set_scene(self, scene: dict) -> None:
        """scene maps a line key -> (N, 3) points or None to hide it. Keys:
        ("real"|"sim"|"ghost"|"target", side), "path", "approach". Redraws only
        if something changed."""
        changed = False
        for key, line in self._lines.items():
            pts = scene.get(key)
            sig = None if pts is None else np.round(np.asarray(pts, dtype=float), 4).tobytes()
            if key in self._shown and self._shown[key] == sig:
                continue
            self._shown[key] = sig
            changed = True
            if pts is None:
                line.set_data_3d([], [], [])
            else:
                pts = np.asarray(pts, dtype=float).reshape(-1, 3)
                line.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
        if changed:
            self.draw_idle()


class RecordingPlot(FigureCanvasQTAgg):
    """Joint angles of a recording against time, one panel per arm."""

    def __init__(self, parent=None):
        super().__init__(Figure(figsize=(6, 4)))
        self.setParent(parent)
        self.show_message("Select a recording")

    def show_message(self, text: str) -> None:
        fig = self.figure
        fig.clear()
        fig.text(0.5, 0.5, text, ha="center", va="center", color=MUTED, fontsize=11)
        self.draw_idle()

    def show_recording(self, rec, names: list[str]) -> None:
        fig = self.figure
        fig.clear()
        arms = rec.arms
        axes = fig.subplots(len(arms), 1, sharex=True, squeeze=False)[:, 0]
        for ax, side in zip(axes, arms):
            deg = np.degrees(rec.q[side])
            for j in range(7):
                ax.plot(rec.t, deg[:, j], color=JOINT_COLORS[j], lw=1.2, label=names[j])
            ax.set_ylabel(f"{side} (deg)", color=SIDE_COLOR[side])
            ax.grid(True, alpha=0.4)
        fig.legend(*axes[0].get_legend_handles_labels(), ncol=7, fontsize=7, loc="upper center",
                   frameon=False)
        axes[-1].set_xlabel("time (s)")
        fig.subplots_adjust(left=0.14, right=0.98, top=0.9, bottom=0.12, hspace=0.12)
        self.draw_idle()


class ScopePlot(FigureCanvasQTAgg):
    """Rolling plot of one quantity for the 7 joints of one arm."""

    def __init__(self, parent=None):
        super().__init__(Figure(figsize=(6, 4)))
        self.setParent(parent)
        self.ax = self.figure.add_subplot(111)
        self.figure.subplots_adjust(left=0.09, right=0.98, top=0.93, bottom=0.11)
        self.ax.grid(True, alpha=0.4)
        self.ax.set_xlabel("time (s, 0 = now)")
        self.lines = [self.ax.plot([], [], color=JOINT_COLORS[j], lw=1.3, label=f"joint{j + 1}")[0]
                      for j in range(7)]
        self.ax.legend(ncol=7, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, 1.09),
                       frameon=False)
        self._msg = self.ax.text(0.5, 0.5, "", transform=self.ax.transAxes, ha="center",
                                 va="center", color=MUTED, fontsize=11)

    def update_data(self, t: np.ndarray, y: np.ndarray | None, visible: list[bool],
                    window_s: float, ylabel: str, message: str = "") -> None:
        self.ax.set_ylabel(ylabel)
        self._msg.set_text(message)
        lo, hi = math.inf, -math.inf
        for j, line in enumerate(self.lines):
            if y is None or len(t) == 0:
                line.set_data([], [])
                continue
            line.set_data(t, y[:, j])
            line.set_visible(visible[j])
            if visible[j]:
                lo, hi = min(lo, float(np.min(y[:, j]))), max(hi, float(np.max(y[:, j])))
        self.ax.set_xlim(-window_s, 0)
        if lo <= hi:
            pad = max(0.05 * (hi - lo), 0.5)
            self.ax.set_ylim(lo - pad, hi + pad)
        self.draw_idle()


# Colours of an insertion attempt's line by outcome, in the map views.
OUTCOME_COLOR = {"bone": "#ef5350", "map_stop": "#ffa726", "depth": "#66bb6a", "abort": "#e57fff",
                 "running": "#4fc3f7", "stopped": MUTED}


class ForcePlot(FigureCanvasQTAgg):
    """Rolling load-cell force with the guard's thresholds as dashed lines."""

    THRESHOLDS = (("f_contact", "#66bb6a"), ("f_soft", "#ffa726"), ("f_bone_min", "#e57fff"),
                  ("f_abort", "#ef5350"))

    def __init__(self, parent=None):
        super().__init__(Figure(figsize=(6, 2.4)))
        self.setParent(parent)
        self.ax = self.figure.add_subplot(111)
        self.figure.subplots_adjust(left=0.13, right=0.98, top=0.92, bottom=0.2)
        self.ax.grid(True, alpha=0.4)
        self.ax.set_xlabel("time (s, 0 = now)")
        self.line, = self.ax.plot([], [], color="#4fc3f7", lw=1.3)
        self._thr = {k: self.ax.axhline(0, color=c, ls="--", lw=1.4, visible=False)
                     for k, c in self.THRESHOLDS}
        self._msg = self.ax.text(0.5, 0.5, "", transform=self.ax.transAxes, ha="center", va="center",
                                 color=MUTED, fontsize=10)

    def update_data(self, t: np.ndarray, y: np.ndarray, units: str, thresholds: dict,
                    window_s: float, message: str = "") -> None:
        self.ax.set_ylabel(f"force ({units})")
        self._msg.set_text(message)
        self.line.set_data(t, y)
        lo, hi = (float(np.min(y)), float(np.max(y))) if len(y) else (0.0, 1.0)
        for k, line in self._thr.items():
            v = thresholds.get(k)
            line.set_visible(v is not None)
            if v is not None:
                line.set_ydata([v, v])
                # Keep a threshold in view once the force is within reach of it.
                if v <= max(hi, 0) * 1.6 + 1e-9:
                    hi = max(hi, v)
        pad = max(0.08 * (hi - lo), 1.0)
        self.ax.set_xlim(-window_s, 0)
        self.ax.set_ylim(lo - pad, hi + pad)
        self.draw_idle()


class MapPlot(FigureCanvasQTAgg):
    """The occupancy map, relative to the entry point E (mm): top view (x-y)
    and side view (x-z, surface at z = 0). Hits are circles of the hit radius;
    each attempt's line runs from its start to how deep it got, coloured by
    outcome."""

    def __init__(self, parent=None):
        super().__init__(Figure(figsize=(6, 3.0)))
        self.setParent(parent)
        self.top, self.side = self.figure.subplots(1, 2)
        self.figure.subplots_adjust(left=0.1, right=0.98, top=0.88, bottom=0.17, wspace=0.32)
        self._sig = None
        self.show_map(None)

    def show_map(self, view: dict | None, title: str = "") -> None:
        sig = None if view is None else (title, view.get("t"), len(view.get("lines", [])),
                                         len(view.get("hits", [])))
        if sig is not None and sig == self._sig:
            return
        self._sig = sig
        for ax, (xl, yl) in ((self.top, ("x − E (mm)", "y − E (mm)")), (self.side, ("x − E (mm)", "z − E (mm)"))):
            ax.clear()
            ax.grid(True, alpha=0.4)
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.set_aspect("equal", adjustable="datalim")
        self.top.set_title("top", fontsize=9)
        self.side.set_title("side", fontsize=9)
        if not view or (view.get("entry") is None and not view.get("lines")):
            self.top.text(0.5, 0.5, "no map yet", transform=self.top.transAxes, ha="center", color=MUTED)
            self.figure.suptitle(title, fontsize=9, color=MUTED)
            self.draw_idle()
            return
        origin = view.get("entry")
        if origin is None:
            origin = np.asarray(view["lines"][0][0])
        origin = np.asarray(origin, dtype=float)
        r = view.get("hit_radius_mm") or 3.0
        self.side.axhline(0, color=MUTED, lw=1)
        for ax, (i, j) in ((self.top, (0, 1)), (self.side, (0, 2))):
            for start, reached, outcome in view.get("lines", []):
                a = (np.asarray(start) - origin) * 1000
                b = (np.asarray(reached) - origin) * 1000
                ax.plot([a[i], b[i]], [a[j], b[j]], color=OUTCOME_COLOR.get(outcome, MUTED), lw=1.6)
                ax.plot([b[i]], [b[j]], color=OUTCOME_COLOR.get(outcome, MUTED), marker="o", ms=3)
            for h in np.asarray(view.get("hits", np.zeros((0, 3)))).reshape(-1, 3):
                p = (h - origin) * 1000
                ax.add_patch(matplotlib.patches.Circle((p[i], p[j]), r, color="#ef5350", alpha=0.35))
                ax.plot([p[i]], [p[j]], "x", color="#ef5350", ms=6)
            ax.plot([0], [0], marker="+", color=FG, ms=10)
        handles = [Line2D([], [], color=c, lw=2, label=k) for k, c in OUTCOME_COLOR.items()
                   if k not in ("stopped",)]
        self.side.legend(handles=handles, fontsize=7, loc="lower left", framealpha=0.7)
        self.figure.suptitle(title, fontsize=9, color=FG)
        self.draw_idle()
