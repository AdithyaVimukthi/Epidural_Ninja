"""
One tab per drive mode, plus the scope and settings. A mode tab builds the
command line the CLI would take and runs it through main.dispatch on the
worker -- so it behaves exactly like typing that command at the openarm>
prompt, and the console shows the command ahead of its output.
"""
from __future__ import annotations

import json
import shlex
import math
import os
import time
from datetime import datetime
from functools import partial

import numpy as np
from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtGui import QDesktopServices, QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QAbstractItemView, QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QInputDialog, QLabel, QLineEdit, QMessageBox,
    QFrame, QPlainTextEdit, QPushButton, QRadioButton, QScrollArea, QSpinBox, QSplitter, QTabWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from config import (
    AXIS_HOLD_CFG, CONFIG_FILE, DATA_DIR, DOCS_DIR, GUARD_CFG, JOINT_NAMES, KD, KP, LOADCELL_CFG, MAP_CFG, MAPS_DIR, PROJECT_ROOT,
    REAL_ARM_LIMITS_RAD, RECORDINGS_DIR, SEARCH_CFG, SIM_HOST, SIM_PORT, CTRL_HZ, USER_CONFIG_FILE, summary,
)
from kinematics import TOOL_AXIS
from mode_line import TORQUE_LOG_DIR, direction_from_angles
import insertion
import loadcell as lcmod
import mode_plunger
from mode_plunger import LOG_DIR as PLUNGER_LOG_DIR
from occupancy_map import OccupancyMap

LOADCELL_DIR = os.path.join(RECORDINGS_DIR, "loadcell")
from recording import (
    REPLAY_MAX_SPEED_DEG_S, RecordingError, list_recordings, load, out_of_range, peak_speed,
    recording_path, speed_factor_within,
)
from session import resolve_sides
from streaming import SIM_RATE_HZ

from gui import actions
from gui.backend import SIDES
from gui.kin import solve_ik
from gui.plots import ForcePlot, MapPlot, RecordingPlot, ScopePlot
from gui.userconfig import (
    Prefs, apply_limits, current_limits_deg, default_limits_deg, limit_problems,
)
from gui.widgets import GainsEditor, JointEditor, LimitsEditor, dspin, hline, ispin, note

# With OPENARM_DATA_DIR (the .deb's read-only /opt install) it lives there instead.
POSES_FILE = os.path.join(DATA_DIR or PROJECT_ROOT, "poses.json")


def num(v) -> str:
    return format(float(v), ".6g")


def nums(values) -> list[str]:
    return [num(v) for v in values]


def mono() -> QFont:
    f = QFont("monospace")
    f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def row(*widgets, stretch: bool = True) -> QHBoxLayout:
    h = QHBoxLayout()
    for w in widgets:
        if isinstance(w, str):
            w = QLabel(w)
        h.addWidget(w)
    if stretch:
        h.addStretch(1)
    return h


def compact_form(parent=None) -> QFormLayout:
    """A form whose fields keep their natural width instead of filling the tab."""
    form = QFormLayout(parent)
    form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.FieldsStayAtSizeHint)
    return form


def button(text: str, slot, tip: str = "") -> QPushButton:
    b = QPushButton(text)
    b.clicked.connect(lambda: slot())
    if tip:
        b.setToolTip(tip)
    return b


def open_path(path: str, is_dir: bool = False) -> None:
    """Open a file or folder in the desktop's default application."""
    if is_dir:
        os.makedirs(path, exist_ok=True)
    QDesktopServices.openUrl(QUrl.fromLocalFile(path))


class Tab(QWidget):
    title = ""
    # Inputs never restored at start-up: toggles that act on their own, which
    # should only ever be switched on on purpose.
    NO_RESTORE: frozenset = frozenset()

    def __init__(self, app):
        super().__init__()
        self.app = app

    def state(self) -> dict:
        """This tab's inputs (spin boxes, check boxes, combos, text fields held
        as attributes), saved to user_config.yaml when the GUI closes."""
        out = {}
        for name, w in vars(self).items():
            if name.startswith("_") or name in self.NO_RESTORE:
                continue
            if isinstance(w, (QDoubleSpinBox, QSpinBox)):
                out[name] = w.value()
            elif isinstance(w, QCheckBox):
                out[name] = w.isChecked()
            elif isinstance(w, QComboBox):
                out[name] = w.currentText()
            elif isinstance(w, QLineEdit):
                out[name] = w.text()
        return out

    def restore(self, state: dict) -> None:
        """Put back what state() saved; anything that no longer fits is skipped."""
        for name, v in (state or {}).items():
            w = getattr(self, name, None)
            if name.startswith("_") or name in self.NO_RESTORE:
                continue
            try:
                if isinstance(w, QDoubleSpinBox):
                    w.setValue(float(v))
                elif isinstance(w, QSpinBox):
                    w.setValue(int(v))
                elif isinstance(w, QCheckBox):
                    if w.isEnabled():
                        w.setChecked(bool(v))
                elif isinstance(w, QComboBox):
                    i = w.findText(str(v))
                    if i >= 0:
                        w.setCurrentIndex(i)
                elif isinstance(w, QLineEdit):
                    w.setText(str(v))
            except (TypeError, ValueError):
                pass

    def overlay(self) -> dict:
        """Extra 3D view lines while this tab is shown (see ArmView3D.set_scene)."""
        return {}

    def on_selection_changed(self) -> None:
        """The toolbar's Arm or Drive changed."""

    def on_models_ready(self) -> None:
        """The FK models finished loading."""


# ── joint ─────────────────────────────────────────────────────────────────────

class PoseLibrary:
    """Named joint targets in POSES_FILE (PROJECT_ROOT/poses.json):
    {name: {"left": [7 deg], "right": [7 deg], "timestamp": ...}}, either arm
    optional. Entries in the older single-arm format ({"deg_values": [...]})
    load onto whichever arms are selected."""

    def __init__(self, path: str):
        self.path = path
        self.poses: dict = {}
        self.reload()

    def reload(self) -> None:
        try:
            with open(self.path) as f:
                self.poses = json.load(f)
        except FileNotFoundError:
            self.poses = {}
        except (OSError, ValueError) as e:
            print(f"[gui] couldn't read {self.path}: {e}")
            self.poses = {}

    def save(self) -> None:
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.poses, f, indent=2)
        os.replace(tmp, self.path)

    @staticmethod
    def targets(entry: dict, sides: list[str]) -> dict[str, list[float]]:
        if "deg_values" in entry:
            return {side: list(entry["deg_values"][:7]) for side in sides}
        return {side: list(entry[side]) for side in sides if side in entry}


class JointTab(Tab):
    title = "Joint"
    NO_RESTORE = frozenset({"jog"})

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        v.addWidget(note("Direct joint-angle control (the joint and home commands). Each arm has its "
                         "own column; with Arm = both they move left then right, as --arm both does. "
                         "Ranges are the real-arm limits, which sim enforces too."))
        self.editor = JointEditor()
        self.editor.released.connect(self._jog)
        v.addWidget(self.editor)
        v.addLayout(row("Set targets:", button("Zero", self._zero),
                        button("From real arm", partial(self._load, "real")),
                        button("From sim", partial(self._load, "sim"))))
        v.addWidget(hline())

        form = compact_form()
        self.duration = dspin(0.1, 120, 2.0, 1, 0.5, "s", "ramp time (--duration)")
        form.addRow("Duration", self.duration)
        self.gripper = QComboBox()
        self.gripper.addItems(["leave", "open", "close"])
        form.addRow("Gripper with the move", self.gripper)
        v.addLayout(form)
        v.addLayout(row(app.run_button("▶  Move", self._move, primary=True),
                        app.run_button("⌂  Home (all zeros)", self._home),
                        app.run_button("Open gripper", partial(self._gripper, "open")),
                        app.run_button("Close gripper", partial(self._gripper, "close"))))

        self.jog = QCheckBox("Live jog (sim only): move as soon as a value is set, over")
        self.jog_duration = dspin(0.05, 5, 0.3, 2, 0.1, "s")
        v.addLayout(row(self.jog, self.jog_duration))
        self.show_ghost = QCheckBox("Show the targets in the 3D view")
        self.show_ghost.setChecked(True)
        v.addWidget(self.show_ghost)

        box = QGroupBox("Saved poses")
        box.setToolTip(POSES_FILE)
        h = QHBoxLayout(box)
        self.pose_combo = QComboBox()
        self.pose_combo.setMinimumWidth(180)
        h.addWidget(self.pose_combo, 1)
        for text, fn in (("Load", self._pose_load), ("Save as…", self._pose_save),
                         ("Delete", self._pose_delete), ("Import…", self._pose_import)):
            h.addWidget(button(text, fn))
        v.addWidget(box)
        v.addStretch(1)

        self.poses = PoseLibrary(POSES_FILE)
        self._refresh_poses()
        self.on_selection_changed()

    def state(self) -> dict:
        return {**super().state(), "targets": {side: self.editor.values(side) for side in SIDES}}

    def restore(self, state: dict) -> None:
        super().restore(state)
        for side, deg in ((state or {}).get("targets") or {}).items():
            if side in SIDES and len(deg) == 7:
                self.editor.set_values(side, deg)

    def on_selection_changed(self) -> None:
        for side in SIDES:
            self.editor.set_side_enabled(side, side in self.app.sides())
        sim_only = self.app.backend() == "sim"
        self.jog.setEnabled(sim_only)
        self.jog.setToolTip("" if sim_only else "only with Drive = sim")
        if not sim_only:
            self.jog.setChecked(False)

    def overlay(self) -> dict:
        if not self.show_ghost.isChecked():
            return {}
        return {("ghost", side): self.app.view_models.chain(side, np.radians(self.editor.values(side)))
                for side in self.app.sides()}

    def _zero(self) -> None:
        for side in self.app.sides():
            self.editor.set_values(side, [0.0] * 7)

    def _load(self, src: str) -> None:
        for side in self.app.sides():
            q, _ = self.app.current_q(side, prefer=src)
            if q is None:
                print(f"[gui] no live {src} reading for the {side} arm")
                continue
            self.editor.set_values(side, np.degrees(q))

    def _joint_argv(self, side: str, backend: str, duration: float) -> list[str]:
        argv = ["joint", "--arm", side, "--mode", backend,
                "--deg", *nums(self.editor.values(side)), "--duration", num(duration)]
        if self.gripper.currentText() != "leave":
            argv += ["--gripper", self.gripper.currentText()]
        return argv + self.app.gain_args() + self.app.sim_args()

    def _move(self) -> None:
        argvs = [self._joint_argv(side, self.app.backend(), self.duration.value())
                 for side in self.app.sides()]
        self.app.run_cli(argvs, "joint")

    def _home(self) -> None:
        argv = ["home", "--arm", self.app.arm(), "--mode", self.app.backend(),
                "--duration", num(self.duration.value())]
        self.app.run_cli([argv + self.app.gain_args() + self.app.sim_args()], "home")

    def _gripper(self, action: str) -> None:
        argv = ["joint", "--arm", self.app.arm(), "--mode", self.app.backend(), "--gripper", action]
        self.app.run_cli([argv + self.app.sim_args()], f"gripper {action}")

    def _jog(self, side: str) -> None:
        if (not self.jog.isChecked() or self.app.backend() != "sim"
                or side not in self.app.sides() or self.app.worker.busy):
            return
        argv = ["joint", "--arm", side, "--mode", "sim", "--deg", *nums(self.editor.values(side)),
                "--duration", num(self.jog_duration.value())] + self.app.sim_args()
        self.app.run_cli([argv], "jog", confirm=False)

    # ── poses ──────────────────────────────────────────────────────────────

    def _refresh_poses(self, select: str | None = None) -> None:
        current = select or self.pose_combo.currentText()
        self.pose_combo.clear()
        self.pose_combo.addItems(sorted(self.poses.poses))
        i = self.pose_combo.findText(current)
        if i >= 0:
            self.pose_combo.setCurrentIndex(i)

    def _pose_load(self) -> None:
        name = self.pose_combo.currentText()
        if not name:
            return
        targets = PoseLibrary.targets(self.poses.poses[name], self.app.sides())
        if not targets:
            print(f"[gui] pose {name!r} has nothing for the {'/'.join(self.app.sides())} arm")
            return
        for side, deg in targets.items():
            self.editor.set_values(side, deg)
        print(f"[gui] loaded pose {name!r} for {'+'.join(targets)} -- press Move to go there")

    def _pose_save(self) -> None:
        name, ok = QInputDialog.getText(self, "Save pose", "Name:", text=self.pose_combo.currentText())
        name = name.strip()
        if not ok or not name:
            return
        entry = self.poses.poses.get(name, {})
        if "deg_values" in entry:
            entry = {}
        entry.update({side: [round(v, 3) for v in self.editor.values(side)] for side in self.app.sides()})
        entry["timestamp"] = datetime.now().isoformat(timespec="seconds")
        self.poses.poses[name] = entry
        self.poses.save()
        self._refresh_poses(select=name)
        print(f"[gui] saved pose {name!r} ({'+'.join(self.app.sides())}) to {self.poses.path}")

    def _pose_delete(self) -> None:
        name = self.pose_combo.currentText()
        if not name or not self.app.ask(f"Delete the saved pose {name!r}?"):
            return
        del self.poses.poses[name]
        self.poses.save()
        self._refresh_poses()

    def _pose_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import poses", PROJECT_ROOT, "JSON (*.json)")
        if not path:
            return
        try:
            with open(path) as f:
                incoming = json.load(f)
        except (OSError, ValueError) as e:
            QMessageBox.warning(self, "Import poses", f"Couldn't read {path}:\n{e}")
            return
        added = 0
        for name, entry in incoming.items():
            if not isinstance(entry, dict) or not ("deg_values" in entry or set(SIDES) & set(entry)):
                continue
            key = name if name not in self.poses.poses else f"{name} (imported)"
            self.poses.poses[key] = entry
            added += 1
        self.poses.save()
        self._refresh_poses()
        print(f"[gui] imported {added} pose(s) from {path}")


# ── cartesian (kinematic) ─────────────────────────────────────────────────────

class CartesianTab(Tab):
    title = "Cartesian"
    NO_RESTORE = frozenset({"nudge_move"})

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        v.addWidget(note("Give an end-effector pose in the robot_base frame; IK turns it into joint "
                         "angles, driven like joint mode (the kinematic command). Preview IK solves "
                         "the same way and shows the result as a ghost in the 3D view. Each arm keeps "
                         "its own target."))
        self.side_buttons: dict[str, QRadioButton] = {}
        group = QButtonGroup(self)
        side_row = QHBoxLayout()
        side_row.addWidget(QLabel("Editing the target for:"))
        for side in SIDES:
            rb = QRadioButton(side)
            group.addButton(rb)
            rb.toggled.connect(lambda on, side=side: on and self._switch_side(side))
            self.side_buttons[side] = rb
            side_row.addWidget(rb)
        side_row.addStretch(1)
        v.addLayout(side_row)

        grid = QGridLayout()
        self.xyz = [dspin(-1.5, 1.5, 0.0, 4, 0.005, "m") for _ in range(3)]
        self.rpy = [dspin(-180, 180, 0.0, 2, 5, "°") for _ in range(3)]
        for i, axis in enumerate("xyz"):
            grid.addWidget(QLabel(axis), 0, 2 * i)
            grid.addWidget(self.xyz[i], 0, 2 * i + 1)
        self.orient = QCheckBox("Orientation (roll pitch yaw, xyz order) -- unchecked = position only, "
                                "orientation left free")
        grid.addWidget(self.orient, 1, 0, 1, 6)
        for i, axis in enumerate(("roll", "pitch", "yaw")):
            grid.addWidget(QLabel(axis), 2, 2 * i)
            grid.addWidget(self.rpy[i], 2, 2 * i + 1)
        v.addLayout(grid)
        for w in self.xyz + self.rpy:
            w.valueChanged.connect(self._target_edited)
        self.orient.toggled.connect(self._orient_toggled)

        form = compact_form()
        self.limits = QComboBox()
        self.limits.addItems(["real", "sim"])
        self.limits.setToolTip("joint limits the IK solve respects (--limits): real = config.yaml, "
                               "sim = URDF. Must be real whenever the real arm moves.")
        form.addRow("IK limits", self.limits)
        self.duration = dspin(0.1, 120, 2.0, 1, 0.5, "s")
        form.addRow("Duration", self.duration)
        v.addLayout(form)

        self.step = dspin(0.1, 100, 10, 1, 1, "mm")
        nudge = [QLabel("Nudge"), self.step]
        for axis in range(3):
            for sign in (-1, 1):
                b = button(f"{'−' if sign < 0 else '+'}{'XYZ'[axis]}", partial(self._nudge, axis, sign))
                b.setFixedWidth(46)
                nudge.append(b)
        self.nudge_move = QCheckBox("move after each nudge")
        v.addLayout(row(*nudge, self.nudge_move))

        v.addLayout(row(button("From current EE", self._from_current,
                               "set this arm's target to where its end effector is now"),
                        button("Preview IK", self._preview),
                        app.run_button("▶  Move to pose", self._move, primary=True)))
        self.result = QPlainTextEdit()
        self.result.setReadOnly(True)
        self.result.setFont(mono())
        self.result.setMinimumHeight(150)
        self.result.setPlaceholderText("IK preview results appear here")
        v.addWidget(self.result, 1)

        self._targets = {side: {"xyz": [0.3, 0.15 if side == "left" else -0.15, 0.3],
                                "rpy": [0.0, 0.0, 0.0], "orient": False} for side in SIDES}
        self._editing: str | None = None
        self._preview_sol: dict = {}
        self._results: dict[str, str] = {}
        self._switch_side(app.sides()[0])
        self.on_selection_changed()

    # ── target bookkeeping ─────────────────────────────────────────────────

    def _switch_side(self, side: str) -> None:
        if self._editing == side:
            return
        self._editing = side
        self.side_buttons[side].setChecked(True)
        self._show(side)

    def _store(self) -> None:
        t = self._targets[self._editing]
        t["xyz"] = [w.value() for w in self.xyz]
        t["rpy"] = [w.value() for w in self.rpy]
        t["orient"] = self.orient.isChecked()

    def _show(self, side: str) -> None:
        t = self._targets[side]
        for w, val in zip(self.xyz + self.rpy, t["xyz"] + t["rpy"]):
            w.blockSignals(True)
            w.setValue(val)
            w.blockSignals(False)
        self.orient.blockSignals(True)
        self.orient.setChecked(t["orient"])
        self.orient.blockSignals(False)
        for w in self.rpy:
            w.setEnabled(t["orient"])

    def _target_edited(self) -> None:
        self._store()
        self._preview_sol.pop(self._editing, None)   # no longer this target's solution

    def _orient_toggled(self, on: bool) -> None:
        for w in self.rpy:
            w.setEnabled(on)
        self._target_edited()

    def pose(self, side: str) -> list[float]:
        t = self._targets[side]
        return list(t["xyz"]) + (list(t["rpy"]) if t["orient"] else [])

    def on_selection_changed(self) -> None:
        sides = self.app.sides()
        for side, rb in self.side_buttons.items():
            rb.setEnabled(side in sides)
        if self._editing not in sides:
            self._switch_side(sides[0])
        real = self.app.backend() != "sim"
        self.limits.model().item(1).setEnabled(not real)
        if real:
            self.limits.setCurrentIndex(0)

    def on_models_ready(self) -> None:
        # Start from where each arm is, so a stray Move goes nowhere.
        for side in SIDES:
            q, _ = self.app.current_q(side)
            T = self.app.view_models.ee(side, q if q is not None else [0.0] * 7)
            if T is not None:
                self._targets[side]["xyz"] = [float(v) for v in T.t]
        self._show(self._editing)

    def overlay(self) -> dict:
        out = {}
        for side in self.app.sides():
            out["target", side] = [self._targets[side]["xyz"]]
            sol = self._preview_sol.get(side)
            if sol is not None:
                out["ghost", side] = self.app.view_models.chain(side, sol.q)
        return out

    # ── actions ────────────────────────────────────────────────────────────

    def _from_current(self) -> None:
        side = self._editing
        q, src = self.app.current_q(side)
        if q is None:
            print(f"[gui] no live reading for the {side} arm -- connect the sim or enable the real arm")
            return
        T = self.app.view_models.ee(side, q)
        if T is None:
            print("[gui] the kinematic model is still loading")
            return
        t = self._targets[side]
        t["xyz"] = [float(v) for v in T.t]
        t["rpy"] = [float(v) for v in T.rpy(unit="deg", order="xyz")]
        self._show(side)
        self._preview_sol.pop(side, None)
        print(f"[gui] {side} target = current EE from {src}: xyz {np.round(T.t, 4)} "
              f"rpy {np.round(t['rpy'], 2)} deg")

    def _nudge(self, axis: int, sign: int) -> None:
        w = self.xyz[axis]
        w.setValue(w.value() + sign * self.step.value() / 1000.0)
        if self.nudge_move.isChecked():
            self._move([self._editing])

    def _preview(self) -> None:
        limits = self.limits.currentText()
        self._results = {}
        self.result.setPlainText("Solving...")
        for side in self.app.sides():
            pose = self.pose(side)
            q, src = self.app.current_q(side)
            seed = None if q is None else [math.degrees(v) for v in q]
            self.app.compute.submit(partial(solve_ik, self.app.compute, side, limits, pose, seed),
                                    partial(self._solved, side, pose, seed, src))

    def _solved(self, side, pose, seed, src, sol, error) -> None:
        if error is not None:
            text = f"[{side}] IK error: {error}"
        else:
            if sol.success and self.pose(side) == pose:
                self._preview_sol[side] = sol
            q_deg = np.degrees(sol.q)
            lines = [f"[{side}] {'solved' if sol.success else 'FAILED'}   residual {sol.residual:.3g}   "
                     f"evaluations {sol.iterations}",
                     "  target  " + " ".join(nums(pose)),
                     "  q (deg) " + " ".join(f"{v:7.2f}" for v in q_deg)]
            if seed is not None:
                change = np.abs(q_deg - np.array(seed))
                j = int(np.argmax(change))
                lines.append(f"  max joint change from the {src}: {change[j]:.2f} deg ({JOINT_NAMES[j]})")
            else:
                lines.append("  seeded from zero (no live reading)")
            if not sol.success:
                lines.append(f"  {sol.reason}")
            text = "\n".join(lines)
        self._results[side] = text
        self.result.setPlainText("\n\n".join(self._results[s] for s in SIDES if s in self._results))

    def _move(self, sides: list[str] | None = None) -> None:
        argvs = [["kinematic", "--arm", side, "--mode", self.app.backend(),
                  "--pose", *nums(self.pose(side)), "--limits", self.limits.currentText(),
                  "--duration", num(self.duration.value())] + self.app.gain_args() + self.app.sim_args()
                 for side in (sides or self.app.sides())]
        self.app.run_cli(argvs, "kinematic")


# ── line ──────────────────────────────────────────────────────────────────────

class LineTab(Tab):
    title = "Line"

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        v.addWidget(note("Drive the end effector along a straight line in three speed-staged sections: "
                         "current pose → start (approach), start → split fraction of depth, then on to "
                         "full depth (the line command). The whole path is IK-planned before anything "
                         "moves. With Arm = both, both arms run the same line, as --arm both does. "
                         "Tool orientation 'path' holds the gripper axis (TCP −X) parallel to the line, "
                         "pointing along it, with roll left free; the approach is then a joint-space "
                         "move that arrives aligned. "
                         "On the real arm, joint torques from reaching the start point to full depth "
                         f"are saved to {TORQUE_LOG_DIR} as <date>_<time>_<side>.csv."))
        form = compact_form()
        self.start = [dspin(-1.5, 1.5, val, 4, 0.005, "m") for val in (0.30, -0.35, 0.30)]
        form.addRow("Start x y z", row(*self.start))
        self.tilt = dspin(0, 180, 0, 1, 5, "°", "angle away from straight down")
        self.rot = dspin(-180, 180, 0, 1, 15, "°", "rotation about vertical: which way the tilt points")
        form.addRow("Direction tilt / rot", row(self.tilt, self.rot, note("0 0 = straight down")))
        self.depth_spin = dspin(0.0005, 1.0, 0.06, 4, 0.005, "m", "distance along the line (--depth)")
        self.split = dspin(0.01, 0.99, 0.6, 2, 0.05, "", "fraction of depth where section 3 begins")
        form.addRow("Depth / split", row(self.depth_spin, self.split))
        self.speeds = [dspin(0.0001, 0.5, val, 4, 0.001, "m/s") for val in (0.02, 0.005, 0.002)]
        form.addRow("Speeds approach / 2 / 3", row(*self.speeds))
        self.orient = QComboBox()
        self.orient.addItems(["path", "free"])
        self.orient.setToolTip("path: tool axis parallel to the line, roll free (--orient path)\n"
                               "free: position-only IK, orientation left to the solver")
        form.addRow("Tool orientation", self.orient)
        self.limits = QComboBox()
        self.limits.addItems(["real", "sim"])
        form.addRow("IK limits", self.limits)
        self.rate_auto = QCheckBox("auto")
        self.rate_auto.setChecked(True)
        self.rate_auto.setToolTip(f"control rate ({CTRL_HZ} Hz) with the real arm, {SIM_RATE_HZ:g} Hz sim only")
        self.rate = dspin(1, 1000, CTRL_HZ, 0, 10, "Hz")
        self.rate.setEnabled(False)
        self.rate_auto.toggled.connect(lambda on: self.rate.setEnabled(not on))
        form.addRow("Tick rate", row(self.rate_auto, self.rate))
        self.plan_step = dspin(0.0002, 0.05, 0.002, 4, 0.0005, "m", "Cartesian spacing of IK knots")
        form.addRow("Plan step", self.plan_step)
        v.addLayout(form)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        v.addWidget(self.summary)
        v.addLayout(row(button("Start = current EE", self._from_current),
                        app.run_button("Plan only (dry run)", partial(self._run, True)),
                        app.run_button("▶  Run line", partial(self._run, False), primary=True)))
        v.addStretch(1)
        self.on_selection_changed()

    def on_selection_changed(self) -> None:
        real = self.app.backend() != "sim"
        self.limits.model().item(1).setEnabled(not real)
        if real:
            self.limits.setCurrentIndex(0)

    def _points(self):
        direction = direction_from_angles(self.tilt.value(), self.rot.value())
        start = np.array([w.value() for w in self.start])
        depth, split = self.depth_spin.value(), self.split.value()
        return direction, start, start + direction * depth * split, start + direction * depth

    def overlay(self) -> dict:
        direction, start, mid, end = self._points()
        out = {"path": [start, mid, end]}
        side = self.app.sides()[0]
        q, _ = self.app.current_q(side)
        chain = self.app.view_models.chain(side, q)
        approach = None
        if chain is not None:
            approach = float(np.linalg.norm(start - chain[-1]))
            out["approach"] = [chain[-1], start]
        self._update_summary(direction, approach)
        return out

    def _update_summary(self, direction, approach_m) -> None:
        depth, split = self.depth_spin.value(), self.split.value()
        s1, s2, s3 = (w.value() for w in self.speeds)
        l2, l3 = depth * split, depth * (1 - split)
        parts = [f"direction ({direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f})"]
        total = l2 / s2 + l3 / s3
        # With --orient path the approach is a joint-space move whose time is
        # only known once planned; this is its lower bound.
        at_least = "≥ " if self.orient.currentText() == "path" else ""
        if approach_m is not None:
            parts.append(f"approach {approach_m * 1000:.1f} mm / {at_least}{approach_m / s1:.1f} s")
            total += approach_m / s1
        parts += [f"to split {l2 * 1000:.1f} mm / {l2 / s2:.1f} s",
                  f"to depth {l3 * 1000:.1f} mm / {l3 / s3:.1f} s",
                  f"total {at_least or '≈ '}{total:.1f} s" + ("" if approach_m is not None else " + approach")]
        if not s1 > s2 > s3:
            parts.append("<span style='color:#fbbf24'>speeds are not decreasing</span>")
        text = "  ·  ".join(parts)
        if text != self.summary.text():
            self.summary.setText(text)

    def _from_current(self) -> None:
        side = self.app.sides()[0]
        q, src = self.app.current_q(side)
        T = self.app.view_models.ee(side, q) if q is not None else None
        if T is None:
            print(f"[gui] no live reading for the {side} arm (or the model is still loading)")
            return
        for w, val in zip(self.start, T.t):
            w.setValue(float(val))
        print(f"[gui] line start = {side} EE from {src}: {np.round(T.t, 4)}")

    def _run(self, dry_run: bool) -> None:
        argv = ["line", "--arm", self.app.arm(), "--mode", self.app.backend(),
                "--start", *nums(w.value() for w in self.start),
                "--angles", num(self.tilt.value()), num(self.rot.value()),
                "--depth", num(self.depth_spin.value()), "--speeds", *nums(w.value() for w in self.speeds),
                "--split", num(self.split.value()), "--orient", self.orient.currentText(),
                "--limits", self.limits.currentText(),
                "--plan-step", num(self.plan_step.value())]
        if not self.rate_auto.isChecked():
            argv += ["--rate", num(self.rate.value())]
        if dry_run:
            argv.append("--dry-run")
        self.app.run_cli([argv + self.app.gain_args() + self.app.sim_args()],
                         "line (dry run)" if dry_run else "line")


# ── plunger ───────────────────────────────────────────────────────────────────

# The guard thresholds the Plunger tab can change live: (GuardConfig field,
# label, can be switched off, tooltip).
THRESHOLD_FIELDS = (
    ("f_contact", "contact", False, "contact: force above this on 3 samples in a row (green line)"),
    ("f_soft", "slow from", True, "insertion slows from contact up to this force, down to s_min of its "
                                  "speed (orange line); off = no slow-down"),
    ("f_bone_min", "bone ≥", True, "bone needs at least this force, plus a steep rise or a stall, for "
                                   "t_persist_s -- then stop and retract (pink line); off = no bone rule"),
    ("f_abort", "stop & retract", False, "safety limit: any force above this stops, releases and retracts "
                                         "the needle at once (red line)"),
    ("f_out", "out <", False, "after a retract the needle counts as out once |force| stays under this"),
)


class PlungerTab(Tab):
    title = "Plunger"
    # Within this of a point's joint angles / EE position counts as being there.
    AT_DEG, AT_MM, AT_TILT_DEG = 1.0, 5.0, 3.0
    NO_RESTORE = frozenset({"map_new"})
    FORCE_WINDOW_S = 20.0
    STATE_COLOR = {"approach": "#4fc3f7", "insert": "#fbbf24", "release": "#ffa726", "retract": "#ffa726",
                   "out_check": "#ffa726", "hold": "#66bb6a", "re-aim": "#e57fff", "planning": "#8b919e",
                   "done": "#8b919e"}

    def __init__(self, app):
        super().__init__(app)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        # Top: everything used while running (buttons, status, force plot),
        # always in view. Bottom: the setup, scrolling on its own. The divider
        # between them can be dragged (remembered with the tab).
        self._split = QSplitter(Qt.Orientation.Vertical)
        self._split.setChildrenCollapsible(False)
        outer.addWidget(self._split)

        # ── run bar ────────────────────────────────────────────────────────
        top = QWidget()
        tv = QVBoxLayout(top)
        tv.setContentsMargins(0, 0, 0, 0)
        bar = QGroupBox("Run")
        bv = QVBoxLayout(bar)
        self.mark_btn = button("⚑  Mark (M)", self._mark,
                               "record the time since point 4 started in its CSV -- e.g. the moment "
                               "the needle touches the sample (the M key works anywhere in the window)")
        self.mark_btn.setEnabled(False)
        self._stop_btn = button("■  Stop (Esc)", app.stop, "stop the running command -- as Esc or the "
                                                          "toolbar's Stop")
        self._stop_btn.setObjectName("stop")
        self._stop_btn.setEnabled(False)
        buttons = [
            app.run_button("▶ 1 Safe", partial(self._run, 1)),
            app.run_button("▶ 2 J3 → 0", partial(self._run, 2)),
            app.run_button("▶ 3 Start", partial(self._run, 3)),
            app.run_button("▶ 4 Plunge", partial(self._run, 4), primary=True),
            app.run_button("⏏ 5 Retract", partial(self._run, 5)),
        ]
        buttons[0].setToolTip("point 1: the safe pose below, joint1 to joint7 one at a time")
        buttons[1].setToolTip("point 2: joint3 back to 0")
        buttons[2].setToolTip("point 3: to x y z below with the tool straight down")
        buttons[3].setToolTip("point 4: straight down by Depth -- goes to point 3 first if the arm is "
                              "elsewhere; with Guard log/on, the load cell is in the loop")
        buttons[4].setToolTip("point 5: back out along the last insertion line (else straight back by Depth)")
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        for c, b in enumerate(buttons):
            b.setMinimumWidth(0)
            grid.addWidget(b, 0, c)
            grid.setColumnStretch(c, 1)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setStyleSheet("font-size: 15px; font-weight: bold;")
        run_all = app.run_button("▶▶ Run 1 → 4", self._run_all, primary=True)
        run_all.setToolTip("points 1, 2, 3 and 4 one after another, stopping at the first that doesn't "
                           "complete; Stop (Esc) ends the whole sequence")
        run_all.setMinimumWidth(0)
        self.then_retract = QCheckBox("then 5")
        self.then_retract.setToolTip("finish Run 1 → 4 with point 5: back the needle out (only if it was "
                                     "left in)")
        grid.addWidget(run_all, 1, 0)
        grid.addWidget(self.then_retract, 1, 1)
        grid.addWidget(self.mark_btn, 1, 2)
        grid.addWidget(self._stop_btn, 1, 3)
        grid.addWidget(self.status, 2, 0, 1, 5)
        bv.addLayout(grid)
        self.guard_status = QLabel()
        self.guard_status.setWordWrap(True)
        self.marks_label = QLabel()
        self.marks_label.setWordWrap(True)
        bv.addWidget(self.guard_status)
        bv.addWidget(self.marks_label)
        tv.addWidget(bar)

        # ── force plot (and the occupancy map one click away) ──────────────
        self._plots = QTabWidget()
        self._plots.setDocumentMode(True)
        page = QWidget()
        pv = QVBoxLayout(page)
        pv.setContentsMargins(0, 4, 0, 0)
        self.force_plot = ForcePlot()
        self.force_plot.setMinimumHeight(200)
        pv.addWidget(self.force_plot, 1)
        self._lc_label = QLabel()
        self._lc_label.setFont(mono())
        pv.addWidget(self._lc_label)
        self.lc_port = QLineEdit(str(LOADCELL_CFG.get("port", "auto")))
        self.lc_port.setToolTip("serial port, or auto (Espressif USB device, else the first ttyACM)")
        self.lc_port.setMaximumWidth(140)
        self._lc_btn = button("Connect", self._lc_toggle)
        # Underscored: recording never restarts by itself when the GUI opens.
        self._lc_record = QCheckBox("Record to CSV")
        self._lc_record.setToolTip(f"every sample the graph shows, appended to {LOADCELL_DIR}/"
                                   f"<date>_<time>_loadcell[_<experiment name>].csv while ticked -- "
                                   f"keeps going on other tabs")
        self._lc_record.toggled.connect(self._record_toggled)
        self.force_window = dspin(5, 120, self.FORCE_WINDOW_S, 0, 5, "s", "time span of the graph, and of a "
                                                                        "saved PNG (the last 120 s are kept)")
        pv.addLayout(row("Load cell", self.lc_port, self._lc_btn, button("Tare", self._lc_tare),
                         "   Window", self.force_window))
        pv.addLayout(row(self._lc_record, button("Save PNG…", self._save_force_png),
                         button("Folder", lambda: open_path(LOADCELL_DIR, is_dir=True))))
        self._record_label = QLabel()
        self._record_label.setWordWrap(True)
        pv.addWidget(self._record_label)
        self._plots.addTab(page, "Force")
        page = QWidget()
        pv = QVBoxLayout(page)
        pv.setContentsMargins(0, 4, 0, 0)
        self.map_plot = MapPlot()
        self.map_plot.setMinimumHeight(200)
        pv.addWidget(self.map_plot, 1)
        pv.addLayout(row(button("Reload from file", self._load_map_file)))
        self._plots.addTab(page, "Occupancy map")
        tv.addWidget(self._plots, 1)
        self._split.addWidget(top)

        self._recorder = None
        self._record_timer = QTimer(self)
        self._record_timer.setInterval(200)
        self._record_timer.timeout.connect(self._record_tick)
        QApplication.instance().aboutToQuit.connect(self._stop_recording)

        # ── setup (scrolls) ────────────────────────────────────────────────
        setup = QWidget()
        v = QVBoxLayout(setup)
        v.setContentsMargins(0, 0, 0, 0)
        v.addWidget(note("Plunging in points, each its own command (buttons above). 1–2 bring the arm from "
                         "zeros to the safe pose; 3 and 4 work from any pose. Point 3 holds the tool (TCP −X) "
                         "straight down; 4 goes straight down from it (to point 3 first if the arm is "
                         "elsewhere); 5 backs the needle out. On the real arm, point 4 records to "
                         f"{PLUNGER_LOG_DIR}."))

        box = QGroupBox("1 · Safe point -- joint1 to joint7, one at a time")
        bl = QVBoxLayout(box)
        self.editor = JointEditor()
        bl.addWidget(self.editor)
        self.joint_speed = dspin(1, 60, 10, 1, 1, "deg/s", "average speed of each joint move, points 1-3")
        bl.addLayout(row("Set from:", button("Zero", self._zero),
                         button("Real arm", partial(self._load, "real")),
                         button("Sim", partial(self._load, "sim")),
                         "   Joint speed", self.joint_speed))
        v.addWidget(box)

        box = QGroupBox("3 · Plunge start -- tool straight down")
        bl = QVBoxLayout(box)
        self.xyz = [dspin(-1.5, 1.5, val, 4, 0.005, "m") for val in (0.30, -0.35, 0.25)]
        self.limits = QComboBox()
        self.limits.addItems(["real", "sim"])
        self.limits.setToolTip("joint limits the IK solve respects (--limits): real = config.yaml, "
                               "sim = URDF. Must be real whenever the real arm moves.")
        bl.addLayout(row("x y z", *self.xyz, "  IK limits", self.limits))
        bl.addLayout(row(button("From current EE", self._from_current),
                         app.run_button("Check point 3 (plan only)", partial(self._run, 3, True))))
        v.addWidget(box)

        box = QGroupBox("4 · Plunge -- straight down from point 3, recorded")
        bl = QVBoxLayout(box)
        form = compact_form()
        self.depth_spin = dspin(0.0005, 0.5, 0.03, 4, 0.005, "m", "how far below point 3 (--depth)")
        self.speed = dspin(0.0001, 0.05, 0.002, 4, 0.0005, "m/s", "descent speed (--speed)")
        form.addRow("Depth / speed", row(self.depth_spin, self.speed))
        self.name = QLineEdit()
        self.name.setPlaceholderText("optional, added to the file name")
        form.addRow("Experiment name", self.name)
        self.plan_step = dspin(0.0002, 0.05, 0.002, 4, 0.0005, "m", "Cartesian spacing of IK knots")
        form.addRow("Plan step", self.plan_step)
        self.axis_hold = QCheckBox("Hold the needle on its line (real arm)")
        self.axis_hold.setChecked(bool(AXIS_HOLD_CFG.get("enabled", True)))
        self.axis_hold.setToolTip("the joints give way under the insertion load and the needle tilts; this "
                                  "measures the tilt and sideways drift every tick and corrects them "
                                  "(motion/axis_hold.py) -- points 4 and 5. Off = --no-axis-hold")
        form.addRow("", self.axis_hold)
        bl.addLayout(form)
        bl.addLayout(row(app.run_button("Check point 4 (plan only)", partial(self._run, 4, True)),
                         button("Open log folder", lambda: open_path(PLUNGER_LOG_DIR, is_dir=True))))
        v.addWidget(box)
        # Only while point 4 runs, so M still types an m in the console otherwise.
        self._mark_key = QShortcut(QKeySequence(Qt.Key.Key_M), app)
        self._mark_key.setContext(Qt.ShortcutContext.ApplicationShortcut)
        self._mark_key.setEnabled(False)
        self._mark_key.activated.connect(self._mark)
        app.job_started.connect(self._job_started)
        app.job_finished.connect(self._job_finished)
        app.job_started.connect(lambda _label: self._stop_btn.setEnabled(True))
        app.job_finished.connect(lambda _label, _outcome: self._stop_btn.setEnabled(self.app.worker.busy))

        # ── needle force feedback (docs/09_loadcell_force_feedback_plan.md) ──
        box = QGroupBox("Needle force feedback -- the load cell in the loop (point 4)")
        bl = QVBoxLayout(box)
        bl.addWidget(note("Guard off: the plain descent. Log: constant speed, force logged; the safety "
                          "trips (force limit, stale load cell) still stop and retract. On: contact, slow-down "
                          "as the force builds, bone stop, full retract out of the block; with Search it then "
                          "re-aims through the same entry point (map-guided) and tries again. The fake load "
                          "cell is a phantom model for sim -- its surface sits 10 mm under point 3."))
        self.guard = QComboBox()
        self.guard.addItems(["off", "log", "on"])
        self.lc_source = QComboBox()
        self.lc_source.addItems(["auto", "real", "fake"])
        self.lc_source.setToolTip("auto: the real load cell with the real arm, the fake one in sim")
        self.phantom = QComboBox()
        self.phantom.addItems(list(LOADCELL_CFG.get("fake_phantoms") or {"sphere": None}))
        self.phantom.setToolTip("fake load cell phantom (config.yaml loadcell.fake_phantoms)")
        bl.addLayout(row("Guard", self.guard, "   Load cell", self.lc_source, "   fake phantom", self.phantom))
        self.insert_speed = dspin(0.0001, 0.05, float(GUARD_CFG.get("insert_speed_mm_s") or 1.0) / 1000, 4,
                                  0.0005, "m/s", "speed after contact (--insert-speed)")
        self.on_bone = QComboBox()
        self.on_bone.addItems(["retract", "hold"])
        self.on_success = QComboBox()
        self.on_success.addItems(["hold", "retract"])
        bl.addLayout(row("Insert speed", self.insert_speed, "   On bone", self.on_bone,
                         "   At depth", self.on_success))
        # Thresholds: start from config.yaml every launch (config.yaml owns
        # them), take effect the moment they change -- plot, next run, and a
        # run already going. Underscored, so the tab doesn't save them.
        units = GUARD_CFG.get("units", "counts")
        self._thr_spins: dict[str, QDoubleSpinBox] = {}
        thr_row = []
        for key, label, can_off, tip in THRESHOLD_FIELDS:
            spin = dspin(0.0 if can_off else 1000.0, 8_400_000.0, 0.0, 0, 50_000.0, "", tip)
            spin.setGroupSeparatorShown(True)
            spin.setMinimumWidth(110)
            if can_off:
                spin.setSpecialValueText("off")
            self._thr_spins[key] = spin
            thr_row += [f"  {label}", spin]
        self._load_thresholds()
        for key, spin in self._thr_spins.items():
            spin.valueChanged.connect(partial(self._threshold_changed, key))
        bl.addLayout(row(f"Thresholds ({units})", *thr_row[:4]))
        bl.addLayout(row("", *thr_row[4:], button("From config.yaml", self._reset_thresholds,
                                                  "back to the config.yaml values")))
        self._thr_note = QLabel()
        self._thr_note.setWordWrap(True)
        bl.addWidget(self._thr_note)
        self.search = QCheckBox("Search")
        self.search.setToolTip("after a bone hit: retract fully out, re-aim through the same entry point, "
                               "try again (needs Guard on)")
        self.max_attempts = ispin(1, 30, int(SEARCH_CFG.get("max_attempts", 6)))
        self.max_tilt = dspin(1, 40, float(SEARCH_CFG.get("max_tilt_deg", 20)), 1, 1, "deg")
        bl.addLayout(row(self.search, "   max attempts", self.max_attempts, "   max tilt", self.max_tilt))
        self.map_name = QLineEdit(str(MAP_CFG.get("name", "phantom")))
        self.map_name.setToolTip(f"occupancy map: {MAPS_DIR}/<name>.json -- one per phantom setup")
        self.map_name.editingFinished.connect(lambda: self._load_map_file())
        self.map_new = QCheckBox("start it afresh")
        self.map_new.setToolTip("the next run starts an empty map (the old file is kept, time-stamped) -- "
                                "do this whenever the block has moved")
        bl.addLayout(row("Occupancy map", self.map_name, self.map_new, button("Clear now…", self._clear_map),
                         button("Folder", lambda: open_path(MAPS_DIR, is_dir=True))))
        v.addWidget(box)
        v.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(setup)
        self._split.addWidget(scroll)
        self._split.setStretchFactor(0, 3)
        self._split.setStretchFactor(1, 2)

        self._file_view: dict | None = None
        self._connecting = False
        self._timer = QTimer(self)
        self._timer.setInterval(150)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        QTimer.singleShot(0, self._load_map_file)
        self.on_selection_changed()

    def state(self) -> dict:
        return {**super().state(), "targets": {side: self.editor.values(side) for side in SIDES},
                "xyz": [w.value() for w in self.xyz], "split": self._split.sizes(),
                "plot_tab": self._plots.currentIndex()}

    def restore(self, state: dict) -> None:
        super().restore(state)
        for side, deg in ((state or {}).get("targets") or {}).items():
            if side in SIDES and len(deg) == 7:
                self.editor.set_values(side, deg)
        xyz = (state or {}).get("xyz") or []
        if len(xyz) == 3:
            for w, val in zip(self.xyz, xyz):
                w.setValue(float(val))
        split = (state or {}).get("split") or []
        if len(split) == 2 and min(split) > 0:
            self._split.setSizes([int(x) for x in split])
        self._plots.setCurrentIndex(int((state or {}).get("plot_tab", 0) or 0))

    def on_selection_changed(self) -> None:
        for side in SIDES:
            self.editor.set_side_enabled(side, side in self.app.sides())
        real = self.app.backend() != "sim"
        self.limits.model().item(1).setEnabled(not real)
        if real:
            self.limits.setCurrentIndex(0)

    def _points(self):
        p3 = np.array([w.value() for w in self.xyz])
        return p3, p3 + np.array([0.0, 0.0, -self.depth_spin.value()])

    def overlay(self) -> dict:
        p3, p4 = self._points()
        out = {"path": [p3, p4]}
        view = self._map_view()
        if view:
            hits = np.asarray(view.get("hits", np.zeros((0, 3)))).reshape(-1, 3)
            out["hits"] = hits if len(hits) else None
            segs = []
            for start, reached, _ in view.get("lines", []):
                segs += [start, reached, [np.nan] * 3]
            out["attempts"] = np.array(segs, dtype=float) if segs else None
        for side in self.app.sides():
            out["ghost", side] = self.app.view_models.chain(side, np.radians(self.editor.values(side)))
        side = self.app.sides()[0]
        q, _ = self.app.current_q(side)
        chain = self.app.view_models.chain(side, q)
        if chain is not None:
            out["approach"] = [chain[-1], p3]
        self._update_status(side, q, p3, p4)
        return out

    def _where(self, side: str, q, p3, p4) -> int | None:
        """Which point the arm is at now, if any."""
        T = self.app.view_models.ee(side, q)
        if T is None:
            return None
        tilt = math.degrees(math.acos(float(np.clip(-(T.R @ TOOL_AXIS)[2], -1.0, 1.0))))
        for point, p in ((4, p4), (3, p3)):
            if np.linalg.norm(T.t - p) * 1000 <= self.AT_MM and tilt <= self.AT_TILT_DEG:
                return point
        q_deg, safe = np.degrees(q), np.array(self.editor.values(side))
        if np.max(np.abs(q_deg - safe)) <= self.AT_DEG:
            return 1
        safe[2] = 0.0
        if np.max(np.abs(q_deg - safe)) <= self.AT_DEG:
            return 2
        if np.max(np.abs(q_deg)) <= self.AT_DEG:
            return 0
        return None

    def _update_status(self, side: str, q, p3, p4) -> None:
        names = {0: "zero", 1: "point 1 (safe)", 2: "point 2 (joint3 at 0)", 3: "point 3 (plunge start)",
                 4: "point 4 (full depth)"}
        if q is None:
            text = f"<span style='color:#8b919e'>{side}: no live reading</span>"
        else:
            at = self._where(side, q, p3, p4)
            if at is None:
                text = f"{side}: <span style='color:#fbbf24'>between points</span>"
            elif at == 4:
                text = f"{side}: at {names[4]} -- done"
            else:
                text = f"{side}: at {names[at]}  →  next: point {at + 1}"
        if text != self.status.text():
            self.status.setText(text)

    def _zero(self) -> None:
        for side in self.app.sides():
            self.editor.set_values(side, [0.0] * 7)

    def _load(self, src: str) -> None:
        for side in self.app.sides():
            q, _ = self.app.current_q(side, prefer=src)
            if q is None:
                print(f"[gui] no live {src} reading for the {side} arm")
                continue
            self.editor.set_values(side, np.degrees(q))

    # ── marks during point 4 ───────────────────────────────────────────────

    def _job_started(self, label: str) -> None:
        if label in ("plunger 4", "plunger all"):
            self._marks: list[str] = []
            self.marks_label.setText("<span style='color:#8b919e'>press Mark (or M) at the moment "
                                     "to note</span>")
            self.mark_btn.setEnabled(True)
            self._mark_key.setEnabled(True)

    def _job_finished(self, label: str, outcome: str) -> None:
        if label in ("plunger 4", "plunger all"):
            self.mark_btn.setEnabled(False)
            self._mark_key.setEnabled(False)
            self._load_map_file()

    def _mark(self) -> None:
        got = mode_plunger.mark()
        if got is None:
            self.marks_label.setText("<span style='color:#fbbf24'>not recording -- the descent hasn't "
                                     "started, or Drive is sim with Guard off (no CSV)</span>")
            return
        number, t, depth_mm = got
        print(f"[plunger] mark {number} at t = {t:.3f} s (commanded depth {depth_mm:.2f} mm)")
        self._marks.append(f"<b>#{number}</b> {t:.3f} s / {depth_mm:.2f} mm")
        self.marks_label.setText("   ".join(self._marks))

    def _from_current(self) -> None:
        side = self.app.sides()[0]
        q, src = self.app.current_q(side)
        T = self.app.view_models.ee(side, q) if q is not None else None
        if T is None:
            print(f"[gui] no live reading for the {side} arm (or the model is still loading)")
            return
        for w, val in zip(self.xyz, T.t):
            w.setValue(float(val))
        print(f"[gui] point 3 = {side} EE from {src}: {np.round(T.t, 4)}")

    def _run(self, point: int, dry_run: bool = False) -> None:
        self.app.run_cli(self._argvs(point, dry_run), f"plunger {point}" + (" (plan only)" if dry_run else ""))

    def _argvs(self, point: int, dry_run: bool = False) -> list[list[str]]:
        """The plunger command line(s) for one point, as its button runs them."""
        base = ["plunger", "--point", str(point), "--mode", self.app.backend(),
                "--joint-speed", num(self.joint_speed.value())]
        if point == 1:
            # Each arm has its own safe pose, so one command per arm.
            argvs = [base + ["--arm", side, "--deg", *nums(self.editor.values(side))]
                     for side in self.app.sides()]
        else:
            argv = base + ["--arm", self.app.arm()]
            if point >= 3:
                argv += ["--xyz", *nums(w.value() for w in self.xyz),
                         "--depth", num(self.depth_spin.value()), "--speed", num(self.speed.value()),
                         "--limits", self.limits.currentText(), "--plan-step", num(self.plan_step.value())]
            if point == 4 and self.name.text().strip():
                argv += ["--name", self.name.text().strip()]
            if point >= 4 and not self.axis_hold.isChecked():
                argv.append("--no-axis-hold")
            if point == 4 and self.guard.currentText() != "off":
                argv += self._guard_args(dry_run)
            if dry_run:
                argv.append("--dry-run")
            argvs = [argv]
        return [a + self.app.gain_args() + self.app.sim_args() for a in argvs]

    def _run_all(self) -> None:
        """Points 1 -> 2 -> 3 -> 4 in one go (then 5 if ticked and the needle
        was left in). Stops at the first point that doesn't complete; Stop
        (Esc) ends the whole sequence."""
        steps = [(p, argv) for p in (1, 2, 3, 4) for argv in self._argvs(p)]
        if self.then_retract.isChecked():
            steps += [(5, argv) for argv in self._argvs(5)]
        parsed = []
        for point, argv in steps:
            try:
                parsed.append((point, argv, self.app.parser.parse_args(argv)))
            except SystemExit:
                return   # argparse printed why
        commands = "\n".join(shlex.join(a) for _, a, _ in parsed)
        confirm = None
        if self.app.backend() != "sim" and self.app.settings_tab.confirm_real.isChecked():
            confirm = f"This drives the REAL arm through points 1 to 4 in one go:\n\n{commands}"
        sides = self.app.sides()

        def job(session):
            for point, argv, args in parsed:
                if point == 5 and not any(side in insertion.NEEDLE_IN for side in sides):
                    print("[plunger] the needle isn't in the material -- skipping point 5")
                    continue
                print(f"openarm> {shlex.join(argv)}")
                if not mode_plunger.cmd_plunger(session, args):
                    print(f"[plunger] point {point} didn't complete -- the sequence stops here")
                    return
            print("[plunger] sequence done")
        self.app.run_job("plunger all", job, confirm=confirm)

    # ── needle force feedback ──────────────────────────────────────────────

    def _load_thresholds(self) -> None:
        """Fill the threshold boxes with what a run would use now, without
        firing their change handlers."""
        for key, spin in self._thr_spins.items():
            v = insertion.guard_value(key)
            if v is None and key == "f_out":
                v = insertion.guard_value("f_contact")
            spin.blockSignals(True)
            spin.setValue(0.0 if v is None else float(v))
            spin.blockSignals(False)

    def _threshold_changed(self, key: str, value: float) -> None:
        can_off = next(off for k, _, off, _ in THRESHOLD_FIELDS if k == key)
        new = None if (can_off and value <= 0.0) else float(value)
        insertion.set_guard_override(key, new)
        base = GUARD_CFG.get(key)
        shown = "off" if new is None else f"{new:,.0f}"
        running = (self.app.worker.current_label or "").startswith("plunger")
        print(f"[guard] {key} = {shown} -- in use now{' (also by the running insertion)' if running else ''}; "
              f"config.yaml has {base if base is None else format(base, ',.0f')} "
              f"(edit it there to keep this after a restart)")
        self._check_thresholds()

    def _reset_thresholds(self) -> None:
        insertion.GUARD_OVERRIDES.clear()
        for key in self._thr_spins:
            insertion.set_guard_override(key, GUARD_CFG.get(key))
        self._load_thresholds()
        print("[guard] thresholds back to config.yaml")
        self._check_thresholds()

    def _check_thresholds(self) -> None:
        v = {k: insertion.guard_value(k) for k, *_ in THRESHOLD_FIELDS}
        problems = []
        if v["f_abort"] is not None and v["f_contact"] is not None and v["f_abort"] <= v["f_contact"]:
            problems.append("stop & retract is at or below contact -- every contact would abort")
        if v["f_bone_min"] is not None and v["f_abort"] is not None and v["f_bone_min"] >= v["f_abort"]:
            problems.append("bone ≥ is at or above stop & retract -- the force limit always trips first")
        if v["f_soft"] is not None and v["f_contact"] is not None and v["f_soft"] <= v["f_contact"]:
            problems.append("slow from is at or below contact -- no slow-down")
        if v["f_abort"] is not None and v["f_abort"] >= 8_000_000:
            problems.append("stop & retract is near the HX711's limit (8.39 M) -- it may saturate first")
        self._thr_note.setText("<span style='color:#fbbf24'>" + "; ".join(problems) + "</span>"
                               if problems else "")

    def _guard_args(self, dry_run: bool) -> list[str]:
        out = ["--guard", self.guard.currentText(), "--insert-speed", num(self.insert_speed.value()),
               "--on-bone", self.on_bone.currentText(), "--on-success", self.on_success.currentText(),
               "--map", self.map_name.text().strip() or "phantom"]
        source = self.lc_source.currentText()
        if source != "auto":
            out += ["--loadcell", source]
        if source == "fake" or (source == "auto" and self.app.backend() == "sim"):
            out += ["--fake-phantom", self.phantom.currentText()]
        if self.search.isChecked():
            out += ["--search", "--max-attempts", str(self.max_attempts.value()),
                    "--max-tilt", num(self.max_tilt.value())]
        if self.map_new.isChecked() and not dry_run:
            out.append("--map-new")
            self.map_new.setChecked(False)   # once, not on every run after
        return out

    def _session(self):
        return self.app.worker.session

    def _lc_toggle(self) -> None:
        if self._connecting:
            return
        if self.app.worker.busy:
            print("[gui] busy -- connect / disconnect the load cell between commands")
            return
        lc = self._session().loadcell_dev
        if lc is not None and lc.is_open:
            self._session().close_loadcell()
            print("[loadcell] disconnected")
            return
        port = self.lc_port.text().strip() or "auto"
        self._connecting = True
        self._lc_btn.setText("Connecting…")
        print(f"[loadcell] connecting ({port})...")
        # Opening waits for the board's first samples (a few seconds): off the
        # event loop, on the compute thread.
        self.app.compute.submit(lambda: lcmod.from_config("real", port=port).open(), self._lc_connected)

    def _lc_connected(self, lc, error) -> None:
        self._connecting = False
        if error is not None:
            print(f"[loadcell] {error}")
            return
        if self.app.worker.busy:   # a command started meanwhile: don't swap it under that
            lc.close()
            print("[gui] busy -- connect the load cell between commands")
            return
        self._session().attach_loadcell(lc)
        print(f"[loadcell] connected: {lc.describe()}")

    def _lc_tare(self) -> None:
        lc = self._display_loadcell()
        if lc is None:
            print("[loadcell] not connected")
            return
        try:
            r = lc.tare(0.5)
            print(f"[loadcell] tared at {r.offset:.0f} counts (sd {r.sd:.0f}, n {r.n}) -- point 4 tares "
                  f"again before every insertion")
        except lcmod.LoadCellError as e:
            print(f"[loadcell] {e}")

    def _display_loadcell(self):
        """The load cell a running insertion uses (maybe the fake one), else the
        connected one, else the last run's (its trace stays up for a look)."""
        live = insertion.LIVE
        if self.app.worker.busy and live.get("loadcell") is not None:
            return live["loadcell"]
        lc = self._session().loadcell_dev
        if lc is not None and lc.is_open:
            return lc
        return live.get("loadcell")

    def _map_view(self) -> dict | None:
        view = insertion.MAP_VIEW
        if view and (self._file_view is None or view.get("t", 0) >= self._file_view.get("t", 0)):
            return view
        return self._file_view

    def _load_map_file(self) -> None:
        path = OccupancyMap.path_for(MAPS_DIR, self.map_name.text().strip() or "phantom")
        if not os.path.exists(path):
            self._file_view = {"t": time.monotonic(), "entry": None, "hits": np.zeros((0, 3)), "lines": []}
            return
        try:
            m = OccupancyMap.load(path)
        except (OSError, ValueError) as e:
            print(f"[map] can't read {path}: {e}")
            return
        self._file_view = {"t": time.monotonic(), "entry": m.entry_point, "hits": m.hit_points(),
                           "hit_radius_mm": m.hit_radius_mm,
                           "lines": [(np.array(ln["from_m"]), np.array(ln["to_m"]), ln.get("outcome", "stopped"))
                                     for ln in m.lines]}

    def _clear_map(self) -> None:
        name = self.map_name.text().strip() or "phantom"
        if self.app.worker.busy:
            print("[gui] busy -- clear the map between commands")
            return
        if not self.app.ask(f"Start the occupancy map '{name}' afresh?\n\nIts hits stop protecting the next "
                            f"insertions. The old file is kept with a time stamp.", "Clear map"):
            return
        OccupancyMap.open(MAPS_DIR, name, self.app.sides()[0], float(MAP_CFG.get("hit_radius_mm", 3.0)),
                          fresh=True).save()
        print(f"[map] '{name}' cleared")
        self._load_map_file()

    # ── recording / saving the force ───────────────────────────────────────

    def _file_stem(self, what: str) -> str:
        name = self.name.text().strip()
        return f"{datetime.now():%Y%m%d_%H%M%S}_{what}" + (f"_{name}" if name else "")

    def _record_toggled(self, on: bool) -> None:
        if not on:
            self._stop_recording()
            return
        path = os.path.join(LOADCELL_DIR, self._file_stem("loadcell") + ".csv")
        try:
            self._recorder = lcmod.CsvRecorder(path, note=self.name.text().strip())
        except OSError as e:
            print(f"[loadcell] can't record to {path}: {e}")
            self._lc_record.blockSignals(True)
            self._lc_record.setChecked(False)
            self._lc_record.blockSignals(False)
            return
        self._record_timer.start()
        print(f"[loadcell] recording to {path}")
        self._record_tick()

    def _record_tick(self) -> None:
        rec = self._recorder
        if rec is None:
            return
        lc = self._display_loadcell()
        rec.drain(lc)
        secs = time.monotonic() - rec.t0
        waiting = "" if lc is not None else "  -- waiting for a load cell (Connect)"
        self._record_label.setText(f"<span style='color:#f87171'>● REC</span>  {rec.rows:,} samples, "
                                   f"{secs:.0f} s  →  {rec.path}{waiting}")

    def _stop_recording(self) -> None:
        rec, self._recorder = self._recorder, None
        self._record_timer.stop()
        if rec is None:
            return
        rec.drain(self._display_loadcell())
        path = rec.close()
        print(f"[loadcell] recording stopped: {rec.rows:,} samples over "
              f"{time.monotonic() - rec.t0:.1f} s -> {path}")
        self._record_label.setText(f"<span style='color:#8b919e'>saved {rec.rows:,} samples → {path}</span>")
        if self._lc_record.isChecked():
            self._lc_record.blockSignals(True)
            self._lc_record.setChecked(False)
            self._lc_record.blockSignals(False)

    def _save_force_png(self, path: str | None = None) -> None:
        if path is None:
            os.makedirs(LOADCELL_DIR, exist_ok=True)
            default = os.path.join(LOADCELL_DIR, self._file_stem("force") + ".png")
            path, _ = QFileDialog.getSaveFileName(self, "Save force graph", default, "PNG image (*.png)")
            if not path:
                return
        if not path.lower().endswith(".png"):
            path += ".png"
        self._tick(force=True)   # the graph as of right now
        name = self.name.text().strip()
        title = self.force_plot.ax.set_title(
            f"load cell, {datetime.now():%Y-%m-%d %H:%M:%S}" + (f"  ·  {name}" if name else ""), fontsize=9)
        try:
            self.force_plot.figure.savefig(path, dpi=150)
        except OSError as e:
            print(f"[loadcell] couldn't save {path}: {e}")
            return
        finally:
            title.set_text("")   # the title is for the file only
            self.force_plot.draw_idle()
        print(f"[loadcell] graph saved -> {path}")

    def _tick(self, force: bool = False) -> None:
        if not (self.isVisible() or force):
            return
        lc = self._display_loadcell()
        if not self._connecting:
            own = self._session().loadcell_dev
            text = "Disconnect" if own is not None and own.is_open else "Connect"
            if self._lc_btn.text() != text:
                self._lc_btn.setText(text)
        if lc is None:
            self._lc_label.setText("not connected -- Connect, or run point 4 with the fake load cell")
            self.force_plot.update_data(np.zeros(0), np.zeros(0), "counts", {}, self.force_window.value(),
                                        "no load cell")
        else:
            r = lc.latest()
            if r is None:
                text = "no samples yet"
            else:
                newtons = f"  {r.newtons:8.3f} N" if r.newtons is not None else ""
                stale = ("  STALE" if r.age_s > 0.5 else "") + ("  SATURATED" if r.saturated else "")
                text = (f"raw {r.raw:>9d}  tared {r.counts:>10.0f}{newtons}  {lc.rate_hz():5.1f} Hz  "
                        f"dropped {r.dropped}{stale}" + (f"  [{lc.error}]" if lc.error else ""))
            self._lc_label.setText(f"{lc.kind}: {text}")
            t, y = lc.series(self.force_window.value())
            same_units = GUARD_CFG.get("units", "counts") == lc.units
            thr = ({k: insertion.guard_value(k) for k in ("f_contact", "f_soft", "f_bone_min", "f_abort")}
                   if same_units else {})
            self.force_plot.update_data(t, y, lc.units, thr, self.force_window.value())
        self._tick_status()
        view = self._map_view()
        name = self.map_name.text().strip() or "phantom"
        self.map_plot.show_map(view, f"map '{name}': {len((view or {}).get('hits', []))} hit(s)")

    def _tick_status(self) -> None:
        live = insertion.LIVE
        running = (self.app.worker.current_label or "").startswith("plunger")
        if not live or not running:
            done = live.get("stop_reason") if live else None
            text = (f"<span style='color:#8b919e'>last run: {done}</span>" if done else
                    "<span style='color:#8b919e'>no guarded run yet</span>")
        else:
            state = live.get("state", "")
            parts = [f"<b style='color:{self.STATE_COLOR.get(state, '#d7dae0')}'>{state.upper()}</b>"]
            if live.get("attempt"):
                parts.append(f"attempt {live['attempt']} (tilt {live.get('tilt', 0):g}°, "
                             f"rot {live.get('rot', 0):g}°)")
            f = live.get("force", math.nan)
            if f == f:
                parts.append(f"force {f:,.0f} {live.get('units', '')}")
            if "cmd_depth_mm" in live:
                parts.append(f"depth {live['cmd_depth_mm']:.1f} mm cmd / {live.get('meas_depth_mm', 0):.1f} meas")
            text = "  ·  ".join(parts)
        if text != self.guard_status.text():
            self.guard_status.setText(text)


# ── imitate ───────────────────────────────────────────────────────────────────

class ImitateTab(Tab):
    title = "Imitate"

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        v.addWidget(note("The real arm goes limp (zero torque, plus gravity feedforward on the joints "
                         "enabled in joint_tauG_enable) and sim mirrors it live as you move it by hand "
                         "(the imitate command). Needs the real arm and CoppeliaSim; ignores Drive."))
        form = compact_form()
        self.rate = dspin(1, 100, 20, 0, 5, "Hz", "mirror rate (--rate)")
        form.addRow("Mirror rate", self.rate)
        self.duration = dspin(0, 3600, 0, 1, 5, "s", "0 = until Stop")
        self.duration.setSpecialValueText("until Stop")
        form.addRow("Duration", self.duration)
        self.hold = QCheckBox("Hold the arm where it is when imitate ends (the CLI leaves it limp)")
        self.hold.setChecked(True)
        form.addRow("", self.hold)
        v.addLayout(form)
        v.addLayout(row(app.run_button("▶  Start imitate", self._start, primary=True),
                        button("■  Stop", app.stop)))
        v.addStretch(1)

    def _start(self) -> None:
        argv = ["imitate", "--arm", self.app.arm(), "--rate", num(self.rate.value())]
        if self.duration.value() > 0:
            argv += ["--duration", num(self.duration.value())]
        after = None
        if self.hold.isChecked():
            kp, kd = self.app.gains()
            sides = self.app.sides()

            def after(session):
                for side in sides:
                    if side in session.real_ctrls:
                        actions.hold_here(side, kp, kd, session)
        arms = " and ".join(self.app.sides())
        self.app.run_cli([argv + self.app.sim_args()], "imitate", after=after,
                         confirm=f"The real {arms} arm goes LIMP as soon as imitate starts.\n\n"
                                 f"Hold it before you continue. Stop (Esc) ends imitate.")


# ── record ────────────────────────────────────────────────────────────────────

class RecordTab(Tab):
    title = "Record"
    NO_RESTORE = frozenset({"overwrite"})

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        v.addWidget(note("Move the real arm by hand and save the motion for Replay (the record command). "
                         "After the countdown the seven arm joints go limp (kp = 0; the gripper is left "
                         "alone). Stop ends the take and saves it; the arm is then held where it stopped "
                         "with the gains from Settings."))
        form = compact_form()
        self.name = QLineEdit()
        self.name.setPlaceholderText("auto: rec_<date>_<time>")
        self.overwrite = QCheckBox("overwrite an existing recording with this name")
        form.addRow("Name", self.name)
        form.addRow("", self.overwrite)
        self.rate = dspin(1, 500, 50, 0, 10, "Hz", "samples per second (--rate)")
        form.addRow("Sample rate", self.rate)
        self.duration = dspin(0, 3600, 0, 1, 5, "s", "0 = until Stop")
        self.duration.setSpecialValueText("until Stop")
        form.addRow("Duration", self.duration)
        self.countdown = dspin(0, 30, 3, 1, 1, "s", "time to take hold of the arm before it goes limp")
        form.addRow("Countdown", self.countdown)
        self.damping = dspin(0, 5, 0, 2, 0.1, "Nm·s/rad", "kd while limp: slows an arm that's let go")
        form.addRow("Damping while limp", self.damping)
        self.no_trim = QCheckBox("keep the still stretches at the start and end (--no-trim)")
        form.addRow("", self.no_trim)
        self.mirror = QCheckBox("mirror the arm into sim live (--mode both)")
        form.addRow("", self.mirror)
        v.addLayout(form)
        self.stop_btn = button("■  Stop && save", app.stop)
        self.stop_btn.setEnabled(False)
        v.addLayout(row(app.run_button("●  Start recording", self._start, primary=True), self.stop_btn))
        self.status = QLabel("")
        self.status.setObjectName("bigstatus")
        v.addWidget(self.status)
        v.addStretch(1)
        self._active = False
        self._countdown = 0.0
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)
        app.job_started.connect(self._started)
        app.job_finished.connect(self._finished)

    def _start(self) -> None:
        name = self.name.text().strip()
        if name and os.path.exists(recording_path(name)) and not self.overwrite.isChecked():
            QMessageBox.warning(self, "Record", f"{recording_path(name)} already exists.\n\n"
                                "Pick another name or tick overwrite.")
            return
        argv = ["record", "--arm", self.app.arm(), "--mode", "both" if self.mirror.isChecked() else "real",
                "--rate", num(self.rate.value()), "--countdown", num(self.countdown.value()),
                "--damping", num(self.damping.value())]
        if name:
            argv += ["--name", name]
        if self.overwrite.isChecked():
            argv.append("--overwrite")
        if self.duration.value() > 0:
            argv += ["--duration", num(self.duration.value())]
        if self.no_trim.isChecked():
            argv.append("--no-trim")
        arms = " and ".join(self.app.sides())
        self.app.run_cli([argv + self.app.gain_args() + self.app.sim_args()], "record",
                         confirm=f"After a {self.countdown.value():g} s countdown the real {arms} arm "
                                 f"goes LIMP.\n\nHold it before you continue. Stop (Esc) ends the take "
                                 f"and saves it.")

    def _started(self, label: str) -> None:
        if label == "record":
            self._active = True
            self._countdown = self.countdown.value()
            self.stop_btn.setEnabled(True)
            self._timer.start()

    def _finished(self, label: str, outcome: str) -> None:
        if label != "record":
            return
        self._active = False
        self._timer.stop()
        self.stop_btn.setEnabled(False)
        self.status.setText({"ok": "Take finished -- see the console for the summary.",
                             "stopped": "Stopped.", "error": "Failed -- see the console."}[outcome])

    def _tick(self) -> None:
        t = self.app.worker.elapsed
        if t < self._countdown:
            self.status.setText(f"<span style='color:#fbbf24'>Hold the arm -- limp in "
                                f"{math.ceil(self._countdown - t)} s</span>")
        else:
            rec = t - self._countdown
            self.status.setText(f"<span style='color:#f87171'>● REC</span>  "
                                f"{int(rec // 60):02d}:{rec % 60:04.1f}   (Stop or Esc to save)")


# ── replay ────────────────────────────────────────────────────────────────────

class ReplayTab(Tab):
    title = "Replay"
    COLUMNS = ["name", "arms", "length", "samples", "rate", "created"]

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        v.addWidget(note(f"Play a recording from {RECORDINGS_DIR} on the backend picked in Drive (the "
                         f"replay command). Each arm first ramps to the recording's first pose (shown as "
                         f"a ghost), then follows it. Which arms play is set here, not by the toolbar."))
        split = QSplitter(Qt.Orientation.Horizontal)
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.itemSelectionChanged.connect(self._selected)
        split.addWidget(self.table)
        self.plot = RecordingPlot()
        self.plot.setMinimumHeight(260)
        split.addWidget(self.plot)
        split.setSizes([360, 560])
        v.addWidget(split, 1)
        self.info = QLabel()
        self.info.setWordWrap(True)
        self.info.setTextFormat(Qt.TextFormat.RichText)
        v.addWidget(self.info)

        form = compact_form()
        self.arm = QComboBox()
        self.arm.addItems(["recorded arms", "left", "right", "both"])
        self.speed = dspin(0.05, 10, 1.0, 2, 0.1, "×", "playback speed (--speed)")
        self.approach = dspin(0.1, 120, 3.0, 1, 0.5, "s", "ramp time to the first pose (--approach)")
        form.addRow("Arms / speed / approach", row(self.arm, self.speed, self.approach))
        self.max_speed = dspin(1, 1000, REPLAY_MAX_SPEED_DEG_S, 0, 10, "deg/s",
                               "the real arm refuses faster joint motion (--max-speed)")
        self.rate_auto = QCheckBox("auto")
        self.rate_auto.setChecked(True)
        self.rate = dspin(1, 1000, CTRL_HZ, 0, 10, "Hz")
        self.rate.setEnabled(False)
        self.rate_auto.toggled.connect(lambda on: self.rate.setEnabled(not on))
        form.addRow("Max speed / tick rate", row(self.max_speed, self.rate_auto, self.rate))
        v.addLayout(form)
        for w in (self.speed, self.max_speed):
            w.valueChanged.connect(self._update_info)
        v.addLayout(row(button("⟳  Refresh", self.refresh), button("Delete", self._delete),
                        button("Open folder", lambda: open_path(RECORDINGS_DIR, is_dir=True)),
                        app.run_button("▶  Play", self._play, primary=True)))
        self._recs: dict = {}
        self._paths: dict[str, str] = {}
        self.refresh()
        app.job_finished.connect(lambda label, outcome: label == "record" and self.refresh(newest=True))

    def refresh(self, newest: bool = False) -> None:
        paths = list_recordings()
        keep = self._selected_name()
        if newest and paths:
            keep = os.path.splitext(os.path.basename(max(paths, key=os.path.getmtime)))[0]
        self._recs, self._paths = {}, {}
        self.table.blockSignals(True)
        self.table.setRowCount(0)
        for path in paths:
            name = os.path.splitext(os.path.basename(path))[0]
            try:
                rec = load(path)
                cells = [name, "+".join(rec.arms), f"{rec.duration:.2f} s", str(len(rec)),
                         f"{rec.rate_hz:g} Hz", rec.created]
            except RecordingError as e:
                rec, cells = None, [name, "unreadable", "", "", "", str(e)]
            r = self.table.rowCount()
            self.table.insertRow(r)
            for c, text in enumerate(cells):
                self.table.setItem(r, c, QTableWidgetItem(text))
            self._recs[name], self._paths[name] = rec, path
        self.table.blockSignals(False)
        if not paths:
            self.plot.show_message("No recordings yet -- make one in the Record tab")
            self.info.setText("")
            return
        names = list(self._recs)
        self.table.selectRow(names.index(keep) if keep in names else 0)
        self._selected()

    def _selected_name(self) -> str | None:
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        if not rows:
            return None
        item = self.table.item(rows[0].row(), 0)
        return item.text() if item is not None else None

    def _selected_rec(self):
        name = self._selected_name()
        return name, self._recs.get(name) if name else None

    def _selected(self) -> None:
        name, rec = self._selected_rec()
        if rec is None:
            self.plot.show_message("Unreadable recording" if name else "Select a recording")
        else:
            self.plot.show_recording(rec, JOINT_NAMES)
        self._update_info()

    def _update_info(self) -> None:
        name, rec = self._selected_rec()
        if rec is None:
            self.info.setText("")
            return
        speed, max_speed = self.speed.value(), self.max_speed.value()
        parts = [f"<b>{name}</b>: {'+'.join(rec.arms)}, {rec.duration:.2f} s, {len(rec)} samples "
                 f"at {rec.rate_hz:g} Hz, recorded {rec.created}"]
        if any(rec.trimmed_s):
            parts[0] += f", trimmed {rec.trimmed_s[0]:.2f} s + {rec.trimmed_s[1]:.2f} s of stillness"
        for side in rec.arms:
            limits = REAL_ARM_LIMITS_RAD[side]
            lo, hi = np.array([l for l, _ in limits]), np.array([h for _, h in limits])
            peak, joint, at = peak_speed(rec.t, np.clip(rec.q[side], lo, hi))
            text = (f"{side}: peak {peak * speed:.1f} deg/s at ×{speed:g} "
                    f"({JOINT_NAMES[joint]}, {at:.2f} s in)")
            if peak * speed > max_speed:
                text += (f" -- <span style='color:#f87171'>over max speed {max_speed:g}; the real arm "
                         f"needs ×{speed_factor_within(max_speed, peak):g} or slower</span>")
            past = out_of_range(rec.q[side], limits)
            if past:
                text += (f" -- <span style='color:#fbbf24'>{', '.join(JOINT_NAMES[j] for j, _, _ in past)} "
                         f"past the limits, clamped on replay</span>")
            parts.append(text)
        self.info.setText("<br>".join(parts))

    def _play_sides(self, rec) -> list[str]:
        choice = self.arm.currentText()
        sides = rec.arms if choice == "recorded arms" else resolve_sides(choice)
        return [side for side in sides if side in rec.arms]

    def overlay(self) -> dict:
        _, rec = self._selected_rec()
        if rec is None:
            return {}
        out = {}
        for side in self._play_sides(rec):
            limits = REAL_ARM_LIMITS_RAD[side]
            q0 = np.clip(rec.q[side][0], [l for l, _ in limits], [h for _, h in limits])
            out["ghost", side] = self.app.view_models.chain(side, q0)
        return out

    def _play(self) -> None:
        name, rec = self._selected_rec()
        if rec is None:
            print("[gui] select a readable recording first")
            return
        argv = ["replay", "--name", name]
        if self.arm.currentText() != "recorded arms":
            argv += ["--arm", self.arm.currentText()]
        argv += ["--mode", self.app.backend(), "--speed", num(self.speed.value()),
                 "--approach", num(self.approach.value()), "--max-speed", num(self.max_speed.value())]
        if not self.rate_auto.isChecked():
            argv += ["--rate", num(self.rate.value())]
        self.app.run_cli([argv + self.app.gain_args() + self.app.sim_args()], "replay")

    def _delete(self) -> None:
        name = self._selected_name()
        if not name or not self.app.ask(f"Delete the recording {name!r}?\n\n{self._paths[name]}"):
            return
        os.remove(self._paths[name])
        print(f"[gui] deleted {self._paths[name]}")
        self.refresh()


# ── scope ─────────────────────────────────────────────────────────────────────

class ScopeTab(Tab):
    title = "Scope"
    QUANTITIES = {"position (deg)": 1, "velocity (deg/s)": 2, "torque (Nm)": 3}

    def __init__(self, app):
        super().__init__(app)
        v = QVBoxLayout(self)
        self.source = QComboBox()
        self.source.addItems(["real", "sim"])
        self.side = QComboBox()
        self.side.addItems(["right", "left"])
        self.quantity = QComboBox()
        self.quantity.addItems(list(self.QUANTITIES))
        self.window_s = dspin(1, 60, 10, 0, 1, "s")
        self.pause = QCheckBox("pause")
        v.addLayout(row("Source", self.source, "Arm", self.side, "Show", self.quantity,
                        "Window", self.window_s, self.pause))
        self.joints = []
        for j in range(7):
            cb = QCheckBox(JOINT_NAMES[j])
            cb.setChecked(True)
            self.joints.append(cb)
        v.addLayout(row(*self.joints))
        self.plot = ScopePlot()
        self.plot.setMinimumHeight(320)
        v.addWidget(self.plot, 1)
        v.addWidget(note("Real-arm values are recorded whenever the arm is read: while idle (Poll when "
                         "idle), and every tick of imitate, record and the real+sim mirrored moves. "
                         "A plain real ramp only commands, so it shows no measurements."))

    def tick(self) -> None:
        if self.pause.isChecked():
            return
        src, side = self.source.currentText(), self.side.currentText()
        label = self.quantity.currentText()
        idx, window = self.QUANTITIES[label], self.window_s.value()
        visible = [cb.isChecked() for cb in self.joints]
        now = time.monotonic()
        rows = [(s[0], s[idx]) for s in self.app.telemetry.history(src, side, now - window)
                if s[idx] is not None]
        if not rows:
            msg = ("sim reports positions only" if src == "sim" and idx > 1
                   else f"no {src} {side} data in the last {window:g} s")
            self.plot.update_data(np.array([]), None, visible, window, label, msg)
            return
        t = np.array([r[0] for r in rows]) - now
        y = np.array([r[1] for r in rows], dtype=float)
        if idx in (1, 2):
            y = np.degrees(y)
        self.plot.update_data(t, y, visible, window, label)


# ── settings ──────────────────────────────────────────────────────────────────

class SettingsTab(Tab):
    title = "Settings"

    def __init__(self, app, settings: Prefs):
        super().__init__(app)
        v = QVBoxLayout(self)

        box = QGroupBox("Gains for real-arm commands (--kp / --kd)")
        bl = QVBoxLayout(box)
        bl.addWidget(note("Sent with joint, home, kinematic, line, plunger and replay; record, imitate and Hold "
                          "pose use them to hold the arm. Passed only when they differ from the saved "
                          "values. The gripper uses its config gains."))
        self.gains = GainsEditor(KP, KD)
        bl.addWidget(self.gains)
        v.addWidget(box)

        box = QGroupBox("Real-arm joint limits")
        bl = QVBoxLayout(box)
        bl.addWidget(note("What every real-arm command is checked and clamped against (sim enforces "
                          "the joint ones too; the sim gripper uses the URDF finger range), and the "
                          "Joint tab's slider ranges. A change applies by itself from the next command "
                          "on -- after the running one, if a command is running -- and is saved to "
                          "user_config.yaml when the GUI closes (or now with Save). The URDF column is "
                          "for reference only."))
        self.limits = LimitsEditor()
        self._start_limits = current_limits_deg()
        self.limits.set_values(self._start_limits)
        bl.addWidget(self.limits)
        self.limits_status = QLabel()
        self.limits_status.setWordWrap(True)
        bl.addWidget(self.limits_status)
        bl.addLayout(row(button("Save", partial(self.apply_limits, save=True),
                                "write user_config.yaml now"),
                         button("Revert", lambda: self.limits.set_values(self._start_limits),
                                "back to the limits the GUI started with"),
                         button("config.yaml values", lambda: self.limits.set_values(default_limits_deg()),
                                "back to the defaults in config.yaml")))
        # Apply a moment after the last edit, so a Revert (every box at once)
        # or a run of arrow clicks applies once.
        self._limits_timer = QTimer(self)
        self._limits_timer.setSingleShot(True)
        self._limits_timer.setInterval(300)
        self._limits_timer.timeout.connect(self.apply_limits)
        self.limits.edited.connect(self._limits_timer.start)
        self._set_limits_status("in use", "#8b919e")
        v.addWidget(box)

        box = QGroupBox("CoppeliaSim")
        form = compact_form(box)
        self.host = QLineEdit(str(settings.value("sim/host", SIM_HOST)))
        self.port = ispin(1, 65535, int(settings.value("sim/port", SIM_PORT)))
        form.addRow("Host / port", row(self.host, self.port, button("Apply", self._apply_sim)))
        self.sim_rate = dspin(1, 30, float(settings.value("sim/rate", 10)), 0, 1, "Hz",
                              "how often the live view reads the sim")
        self.sim_rate.valueChanged.connect(lambda val: setattr(app.sim_monitor, "rate_hz", val))
        form.addRow("Live view rate", self.sim_rate)
        v.addWidget(box)

        box = QGroupBox("Safety")
        bl = QVBoxLayout(box)
        self.confirm_real = QCheckBox("Ask before every command that drives the real arm")
        self.confirm_real.setChecked(str(settings.value("safety/confirm_real", "true")) == "true")
        bl.addWidget(self.confirm_real)
        bl.addWidget(note("Commands that make the arm go limp (imitate, record, Go limp) and Disable all "
                          "motors always ask."))
        v.addWidget(box)

        box = QGroupBox("Configuration files")
        bl = QVBoxLayout(box)
        bl.addWidget(note("config.yaml holds the defaults and is never changed by the GUI. Everything "
                          "set here and in the other tabs (gains, gravity switches, limits, window "
                          "layout, tab inputs) is saved to user_config.yaml when the GUI closes and "
                          "loaded from it at start-up -- the CLI reads it too. Hand edits to either "
                          "file take effect on the next start."))
        text = QPlainTextEdit(summary())
        text.setReadOnly(True)
        text.setFont(mono())
        text.setMaximumHeight(170)
        bl.addWidget(text)
        bl.addLayout(row(button("Open config.yaml", lambda: open_path(CONFIG_FILE)),
                         button("Open user_config.yaml", self._open_user_config),
                         button("Save now", self._save_now, "write user_config.yaml now"),
                         button("Reset to config.yaml…", self._reset,
                                "delete user_config.yaml when the GUI closes, so the next start "
                                "uses config.yaml alone")))
        bl.addLayout(row(button("Open recordings folder", lambda: open_path(RECORDINGS_DIR, is_dir=True)),
                         button("Open docs", lambda: open_path(DOCS_DIR, is_dir=True))))
        v.addWidget(box)
        v.addStretch(1)

    def state(self) -> dict:
        return {}   # its values go into the config itself (see save())

    def _set_limits_status(self, text: str, color: str) -> None:
        self.limits_status.setText(text)
        self.limits_status.setStyleSheet(f"color: {color};")

    def apply_limits(self, save: bool = False) -> bool:
        """Make the editor's limits the ones in use (called after an edit, when
        a command finishes, and before saving). False if they couldn't be:
        invalid, or a command is running (then it's retried when it ends)."""
        self._limits_timer.stop()
        new = self.limits.values()
        problems = limit_problems(new)
        if problems:
            self._set_limits_status("Not applied -- " + "; ".join(problems), "#f87171")
            return False
        # The Joint tab's sliders follow the editor straight away, even while a
        # command runs: they only set targets, nothing can start until that
        # command ends, and these limits are applied as it does.
        self.app.joint_tab.editor.refresh_limits(new)
        self.app.plunger_tab.editor.refresh_limits(new)
        if new == current_limits_deg():
            self._set_limits_status("in use", "#8b919e")
        elif self.app.worker.busy:
            self._set_limits_status("Changed -- the Joint tab shows them already; the arm uses them "
                                    "once the running command finishes", "#fbbf24")
            return False
        else:
            changes = apply_limits(new)
            self.app.limits_changed()
            print("[gui] real-arm limits changed:\n  " + "\n  ".join(changes))
            self._set_limits_status("Applied -- in use from the next command", "#4ade80")
        if save:
            self.app.save_user_config()
        return True

    def _open_user_config(self) -> None:
        if not os.path.exists(USER_CONFIG_FILE):
            self.app.save_user_config()
        open_path(USER_CONFIG_FILE)

    def _save_now(self) -> None:
        self.app.save_user_config()

    def _reset(self) -> None:
        if not self.app.ask("Go back to config.yaml?\n\nuser_config.yaml is deleted when the GUI "
                            "closes (and nothing is saved then), so the next start uses the defaults "
                            "for everything: gains, gravity switches, joint limits, window layout "
                            "and tab inputs.", "Reset to config.yaml"):
            return
        self.app.reset_user_config_on_exit = True
        print(f"[gui] {USER_CONFIG_FILE} will be deleted on exit -- restart to use config.yaml")

    def _apply_sim(self) -> None:
        self.app.set_sim_endpoint(self.host.text().strip() or SIM_HOST, self.port.value())

    def save(self, settings: Prefs) -> None:
        # The endpoint in use -- host/port typed in but never Applied aren't kept.
        host, port = self.app.sim_endpoint()
        settings.setValue("sim/host", host)
        settings.setValue("sim/port", port)
        settings.setValue("sim/rate", self.sim_rate.value())
        settings.setValue("safety/confirm_real", "true" if self.confirm_real.isChecked() else "false")
