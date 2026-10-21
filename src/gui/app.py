"""
Main window: the toolbar (which arm, which backend, Stop), the mode tabs, and
three docks -- connections, live view (3D + joint table) and the console, which
also takes the same commands as the openarm> shell.
"""
from __future__ import annotations

import math
import os
import shlex
import signal
import sys
import time
from functools import partial

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QKeySequence, QPalette, QShortcut, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDockWidget, QFileDialog, QGridLayout, QGroupBox,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
    QScrollArea, QSizePolicy, QSplitter, QTableWidget, QTableWidgetItem, QTabWidget, QToolBar,
    QVBoxLayout, QWidget,
)

import arm_can
from config import (
    IFACE_BY_ARM, JOINT_NAMES, SIM_HOST, SIM_PORT, SIM_SCENES, TAU_G_ENABLE, USER_CONFIG_FILE,
    save_user_config,
)
from kinematics import _load_limits_rad
from main import build_parser, dispatch
from session import resolve_sides

from gui import actions
from gui.backend import (
    SIDES, LogBuffer, SimMonitor, Telemetry, Worker, attach_dynamics, install_hooks,
    set_gravity_mask,
)
from gui.kin import Compute, GravityModels, ViewModels, build_dynamics
from gui.plots import BG, FG, MUTED, SIDE_COLOR, ArmView3D
from gui.tabs import (
    CartesianTab, ImitateTab, JointTab, LineTab, PlungerTab, RecordTab, ReplayTab, ScopeTab,
    SettingsTab, button, mono, num, row,
)
from gui.userconfig import build_user_config, load_prefs
from gui.widgets import GravityBox, StatusDot, WheelGuard, limits_deg, note

HISTORY_FILE = os.path.expanduser("~/.openarm_gui_history")

LOG_COLORS = {"cmd": "#7cc4ff", "error": "#f87171", "warn": "#fbbf24", "ok": "#4ade80",
              "gui": MUTED, "text": FG}

VIEWS = {"Iso": (20, -50), "Front": (0, 0), "Side": (0, -90), "Top": (90, -90)}

TABLE_ROWS = JOINT_NAMES[:7] + ["gripper", "EE x", "EE y", "EE z"]
TABLE_COLS = ["L real °", "L τ Nm", "L sim °", "R real °", "R τ Nm", "R sim °"]


def _log_kind(line: str) -> str:
    s = line.strip()
    if s.startswith("openarm>"):
        return "cmd"
    if s.startswith("Error") or "ERROR" in s or "Traceback" in s or "failed" in s.lower():
        return "error"
    if "WARNING" in s or "NOTE" in s or s.startswith(("Note", "Stopped", "Interrupted")):
        return "warn"
    if s.startswith("Done") or "saved" in s:
        return "ok"
    if s.startswith("[gui]"):
        return "gui"
    return "text"


class CommandLine(QLineEdit):
    """The console's input, with Up/Down history kept in ~/.openarm_gui_history."""

    def __init__(self):
        super().__init__()
        self.setPlaceholderText("same commands as the openarm> shell, e.g.  joint --deg 0 20 0 45 0 0 0"
                                "   (help, clear)")
        try:
            with open(HISTORY_FILE) as f:
                self._history = [ln.rstrip("\n") for ln in f if ln.strip()][-500:]
        except OSError:
            self._history = []
        self._pos = len(self._history)

    def remember(self, line: str) -> None:
        if not self._history or self._history[-1] != line:
            self._history.append(line)
            try:
                with open(HISTORY_FILE, "a") as f:
                    f.write(line + "\n")
            except OSError:
                pass
        self._pos = len(self._history)

    def keyPressEvent(self, e) -> None:
        if e.key() in (Qt.Key.Key_Up, Qt.Key.Key_Down) and self._history:
            step = -1 if e.key() == Qt.Key.Key_Up else 1
            self._pos = min(max(self._pos + step, 0), len(self._history))
            self.setText(self._history[self._pos] if self._pos < len(self._history) else "")
            return
        super().keyPressEvent(e)


class MainWindow(QMainWindow):
    selection_changed = Signal()
    job_started = Signal(str)
    job_finished = Signal(str, str)   # label, outcome

    def __init__(self, log: LogBuffer):
        super().__init__()
        self.setWindowTitle("OpenArm Control")
        self.resize(1600, 980)
        self.log_buffer = log
        # GUI state lives in user_config.yaml's gui: section (config.py, gui/userconfig.py)
        self.settings = load_prefs()
        self.reset_user_config_on_exit = False
        # argparse names every usage/error line after sys.argv[0]; make it read
        # like the shell's commands rather than the GUI's script name.
        argv0, sys.argv[0] = sys.argv[0], "openarm"
        try:
            self.parser = build_parser()
        finally:
            sys.argv[0] = argv0
        self.telemetry = Telemetry()
        self.worker = Worker(self.telemetry)
        self.worker.poll_real = str(self.settings.value("real/poll", "true")) == "true"
        self.worker.signals.started.connect(self._on_started)
        self.worker.signals.finished.connect(self._on_finished)
        self.compute = Compute()
        self.sim_monitor = SimMonitor(self.telemetry, str(self.settings.value("sim/host", SIM_HOST)),
                                      int(self.settings.value("sim/port", SIM_PORT)),
                                      float(self.settings.value("sim/rate", 10)))
        self.sim_monitor.enabled = str(self.settings.value("sim/live", "true")) == "true"
        self._run_buttons: list[QPushButton] = []
        self._overlay_error = ""
        self._docks: dict[str, QDockWidget] = {}
        self.view_models = ViewModels(self.compute, on_ready=self._models_ready)
        self.gravity_models = GravityModels(self.compute)

        # Side docks run full height; the console sits under the tabs only.
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self._build_toolbar()
        self._build_tabs()
        self._build_live_dock()
        self._build_connections_dock()
        self._build_console_dock()
        geometry, state = self.settings.value("ui/geometry"), self.settings.value("ui/state")
        if geometry is not None:
            self.restoreGeometry(geometry)
        if state is None or not self.restoreState(state):
            self.resizeDocks([self._docks["connections"], self._docks["live"]], [300, 520],
                             Qt.Orientation.Horizontal)
            self.resizeDocks([self._docks["console"]], [240], Qt.Orientation.Vertical)

        for interval, slot in ((50, self._drain_log), (100, self._refresh_live),
                               (200, self._tick_scope), (250, self._refresh_status)):
            timer = QTimer(self)
            timer.setInterval(interval)
            timer.timeout.connect(slot)
            timer.start()

        self.worker.start()
        self.sim_monitor.start()
        self._set_busy(False)
        self._refresh_status()
        print("[gui] OpenArm GUI ready. Arm and Drive in the toolbar pick what every tab drives; "
              "Esc stops the running command.")

    # ── API the tabs use ───────────────────────────────────────────────────

    def arm(self) -> str:
        return self.arm_combo.currentText()

    def sides(self) -> list[str]:
        return resolve_sides(self.arm())

    def backend(self) -> str:
        return self.backend_combo.currentText()

    def gains(self) -> tuple[list[float], list[float]]:
        g = self.settings_tab.gains
        return g.kp(), g.kd()

    def gain_args(self) -> list[str]:
        g = self.settings_tab.gains
        if g.is_default():
            return []
        return ["--kp", *(num(v) for v in g.kp()), "--kd", *(num(v) for v in g.kd())]

    def sim_endpoint(self) -> tuple[str, int]:
        return self.sim_monitor.host, self.sim_monitor.port

    def sim_args(self) -> list[str]:
        host, port = self.sim_endpoint()
        out = []
        if host != SIM_HOST:
            out += ["--sim-host", host]
        if port != SIM_PORT:
            out += ["--sim-port", str(port)]
        return out

    def set_sim_endpoint(self, host: str, port: int) -> None:
        self.sim_monitor.configure(host, port)
        # Session caches sim connections by side only, so drop them once idle.
        self.worker.enqueue("reset sim", lambda s: s.drop_sim())
        print(f"[gui] CoppeliaSim endpoint set to {host}:{port}")

    def current_q(self, side: str, prefer: str | None = None) -> tuple[list[float] | None, str]:
        """Live joint angles (rad) of one arm and where they came from. Real
        first when Drive includes real, else sim first; `prefer` pins one."""
        snap = self.telemetry.snapshot()
        order = [prefer] if prefer else (["real", "sim"] if self.backend() != "sim" else ["sim", "real"])
        for src in order:
            r = snap[src, side]
            q = r.pose(max_age=2.0) if src == "real" else (r.q if r.fresh(3.0) else None)
            if q is not None:
                return list(q), f"{src} arm" if src == "real" else "sim"
        return None, ""

    def run_button(self, text: str, slot, primary: bool = False) -> QPushButton:
        """A button that starts a command -- disabled while one is running."""
        b = QPushButton(text)
        if primary:
            b.setObjectName("run")
        b.clicked.connect(lambda: slot())
        self._run_buttons.append(b)
        return b

    def ask(self, text: str, title: str = "OpenArm") -> bool:
        box = QMessageBox(QMessageBox.Icon.Warning, title, text,
                          QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel, self)
        box.setDefaultButton(QMessageBox.StandardButton.Cancel)
        return box.exec() == QMessageBox.StandardButton.Yes

    def run_cli(self, argvs: list[list[str]], label: str, confirm: str | bool | None = None,
                after=None) -> bool:
        """Parse each command line exactly as the shell would and run them in
        order on the worker. confirm: text to always ask, False to never ask,
        None to ask only when the real arm is involved (and Settings says so).
        after(session) runs once the commands end, however they end."""
        if self.worker.busy:
            print("[gui] busy -- wait for the current command or press Stop")
            return False
        parsed = []
        for argv in argvs:
            try:
                parsed.append((argv, self.parser.parse_args(argv)))
            except SystemExit:
                return False   # argparse printed the problem
        if confirm is None:
            commands = "\n".join(shlex.join(a) for a in argvs)
            if any(a.drive_mode in ("imitate", "record") for _, a in parsed):
                # These make the arm go limp -- always ask, whatever Settings says.
                confirm = f"This makes the REAL arm go limp:\n\n{commands}\n\nHold it before you continue."
            elif (any(getattr(a, "mode", "sim") in ("real", "both") for _, a in parsed)
                  and self.settings_tab.confirm_real.isChecked()):
                confirm = f"This drives the REAL arm:\n\n{commands}"
        if confirm and not self.ask(str(confirm)):
            return False

        def job(session):
            try:
                for argv, args in parsed:
                    print(f"openarm> {shlex.join(argv)}")
                    dispatch(session, args)
            finally:
                if after is not None:
                    after(session)

        return self._submit(label, job)

    def run_job(self, label: str, fn, confirm: str | None = None) -> bool:
        if self.worker.busy:
            print("[gui] busy -- wait for the current command or press Stop")
            return False
        if confirm and not self.ask(confirm):
            return False
        return self._submit(label, fn)

    def _submit(self, label: str, fn) -> bool:
        if not self.worker.submit(label, fn):
            print("[gui] busy -- wait for the current command or press Stop")
            return False
        self._set_busy(True)
        return True

    def stop(self) -> None:
        # Esc is an application-wide shortcut, so it also has to do what Esc
        # normally does in a dialog: cancel it.
        modal = QApplication.activeModalWidget()
        if modal is not None and hasattr(modal, "reject"):
            modal.reject()
        if self.worker.interrupt():
            print("[gui] Stop -- interrupting the running command (as Ctrl+C would)")

    def disable_all(self) -> None:
        if not self.ask("Disable every motor now?\n\nThe arms lose all torque and DROP. "
                        "Support them first.", "Disable all motors"):
            return
        self.worker.interrupt()
        self.worker.enqueue("disable all", lambda s: s.close())
        self._set_busy(True)

    # ── building the window ────────────────────────────────────────────────

    def _build_toolbar(self) -> None:
        tb = QToolBar("Target")
        tb.setObjectName("target_toolbar")
        tb.setMovable(False)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, tb)
        tb.addWidget(QLabel("  Arm "))
        self.arm_combo = QComboBox()
        self.arm_combo.addItems(["right", "left", "both"])
        self.arm_combo.setToolTip("--arm for every tab (Replay picks its own)")
        tb.addWidget(self.arm_combo)
        tb.addWidget(QLabel("   Drive "))
        self.backend_combo = QComboBox()
        self.backend_combo.addItems(["sim", "real", "both"])
        for i, tip in enumerate(("CoppeliaSim only -- nothing touches hardware",
                                 "the real arm only",
                                 "the real arm, with its measured pose mirrored into sim")):
            self.backend_combo.setItemData(i, tip, Qt.ItemDataRole.ToolTipRole)
        self.backend_combo.setToolTip("--mode: sim, real or both")
        tb.addWidget(self.backend_combo)
        for combo, key in ((self.arm_combo, "ui/arm"), (self.backend_combo, "ui/backend")):
            i = combo.findText(str(self.settings.value(key, combo.currentText())))
            combo.setCurrentIndex(max(i, 0))
            combo.currentTextChanged.connect(self._selection_changed)
        self.real_badge = QLabel()
        tb.addWidget(self.real_badge)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        tb.addWidget(spacer)
        self.busy_label = QLabel("idle")
        self.busy_label.setMinimumWidth(260)
        tb.addWidget(self.busy_label)
        self.stop_btn = QPushButton("■  STOP   (Esc)")
        self.stop_btn.setObjectName("stop")
        self.stop_btn.setToolTip("Interrupt the running command, as Ctrl+C does in the shell. The real "
                                 "arm keeps holding its last command (imitate leaves it limp unless "
                                 "'hold when it ends' is ticked).")
        self.stop_btn.clicked.connect(self.stop)
        tb.addWidget(self.stop_btn)
        estop = QPushButton("Disable all motors")
        estop.setObjectName("estop")
        estop.setToolTip("Stop, then disable every real motor -- the arms drop")
        estop.clicked.connect(self.disable_all)
        tb.addWidget(estop)
        shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
        shortcut.activated.connect(self.stop)
        self._update_real_badge()

    def _selection_changed(self) -> None:
        self._update_real_badge()
        self.selection_changed.emit()

    def _update_real_badge(self) -> None:
        real = self.backend() != "sim"
        self.backend_combo.setStyleSheet("QComboBox { background: #6b1d1d; color: white; }" if real else "")
        self.real_badge.setText("  <b style='color:#f87171'>REAL HARDWARE</b>" if real else "")

    def _build_tabs(self) -> None:
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setMinimumWidth(560)
        self.settings_tab = SettingsTab(self, self.settings)
        self.replay_tab = ReplayTab(self)
        self.scope_tab = ScopeTab(self)
        self.joint_tab = JointTab(self)
        self.plunger_tab = PlungerTab(self)
        self._tabs = [self.joint_tab, CartesianTab(self), LineTab(self), self.plunger_tab, ImitateTab(self),
                      RecordTab(self), self.replay_tab, self.scope_tab, self.settings_tab]
        for tab in self._tabs:
            tab.restore(self.settings.value(f"tabs/{tab.title}", {}))
        for tab in self._tabs:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QScrollArea.Shape.NoFrame)
            scroll.setWidget(tab)
            self.tabs.addTab(scroll, tab.title)
        self.tabs.setCurrentIndex(int(self.settings.value("ui/tab", 0)))
        self.selection_changed.connect(lambda: [t.on_selection_changed() for t in self._tabs])
        self.setCentralWidget(self.tabs)

    def _current_tab(self):
        return self._tabs[self.tabs.currentIndex()]

    def _build_live_dock(self) -> None:
        dock = QDockWidget("Live view", self)
        dock.setObjectName("live_dock")
        self._docks["live"] = dock
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(4, 4, 4, 4)
        split = QSplitter(Qt.Orientation.Vertical)
        top = QWidget()
        tv = QVBoxLayout(top)
        tv.setContentsMargins(0, 0, 0, 0)
        self.view3d = ArmView3D()
        self.view3d.setMinimumSize(360, 320)
        tv.addWidget(self.view3d, 1)
        self.show_real = QCheckBox("real")
        self.show_sim = QCheckBox("sim")
        self.show_overlay = QCheckBox("tab overlay")
        for cb in (self.show_real, self.show_sim, self.show_overlay):
            cb.setChecked(True)
        views = [button(name, partial(self.view3d.reset_view, *angles)) for name, angles in VIEWS.items()]
        for b in views:
            b.setMinimumWidth(58)
        tv.addLayout(row("Show", self.show_real, self.show_sim, self.show_overlay, *views))
        split.addWidget(top)

        self.table = QTableWidget(len(TABLE_ROWS), len(TABLE_COLS))
        self.table.setHorizontalHeaderLabels(TABLE_COLS)
        self.table.setVerticalHeaderLabels(TABLE_ROWS)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setFont(mono())
        self.table.verticalHeader().setDefaultSectionSize(22)
        self.table.horizontalHeader().setStretchLastSection(False)
        for c in range(len(TABLE_COLS)):
            self.table.setColumnWidth(c, 70)
            head = self.table.horizontalHeaderItem(c)
            head.setForeground(QColor(SIDE_COLOR["left" if c < 3 else "right"]))
        for r in range(len(TABLE_ROWS)):
            for c in range(len(TABLE_COLS)):
                item = QTableWidgetItem("—")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(r, c, item)
        self.table.setToolTip("real: measured (grey = no fresh reading)   τ: measured torque   "
                              "sim: CoppeliaSim   gripper: real in °, sim finger opening in mm   "
                              "EE: tool point in robot_base, mm   red = outside the real-arm limits")
        split.addWidget(self.table)
        split.setSizes([520, 330])
        v.addWidget(split)
        dock.setWidget(w)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

    def _build_connections_dock(self) -> None:
        dock = QDockWidget("Connections", self)
        dock.setObjectName("connections_dock")
        self._docks["connections"] = dock
        w = QWidget()
        v = QVBoxLayout(w)

        box = QGroupBox("Real arm (CAN)")
        grid = QGridLayout(box)
        self.real_dots: dict[str, StatusDot] = {}
        for r, side in enumerate(SIDES):
            name = QLabel(f"{side} ({IFACE_BY_ARM[side]})")
            name.setStyleSheet(f"color: {SIDE_COLOR[side]}; font-weight: bold;")
            grid.addWidget(name, r, 0)
            dot = StatusDot()
            self.real_dots[side] = dot
            grid.addWidget(dot, r, 1)
            grid.addWidget(self.run_button("Enable", partial(self._enable, side)), r, 2)
            grid.addWidget(self.run_button("Disable", partial(self._disable, side)), r, 3)
        grid.addWidget(note("For the arm(s) picked in the toolbar:"), 2, 0, 1, 4)
        grid.addWidget(self.run_button("Hold pose", self._hold), 3, 0, 1, 2)
        grid.addWidget(self.run_button("Go limp", self._limp), 3, 2, 1, 2)
        grid.addWidget(self.run_button("Print state", self._print_state), 4, 0, 1, 2)
        self.poll_real = QCheckBox("Poll when idle")
        self.poll_real.setToolTip("read the enabled arms ~20 times a second between commands")
        self.poll_real.setChecked(self.worker.poll_real)
        self.poll_real.toggled.connect(lambda on: setattr(self.worker, "poll_real", on))
        grid.addWidget(self.poll_real, 4, 2, 1, 2)
        v.addWidget(box)

        self.gravity_box = GravityBox(TAU_G_ENABLE)
        self.gravity_box.changed.connect(self._gravity_changed)
        self.gravity_box.save_requested.connect(self._gravity_save)
        v.addWidget(self.gravity_box)

        box = QGroupBox("CoppeliaSim")
        grid = QGridLayout(box)
        self.sim_dot = StatusDot()
        self.sim_dot.setWordWrap(True)
        grid.addWidget(self.sim_dot, 0, 0, 1, 3)
        self.live_sim = QCheckBox("Live view")
        self.live_sim.setChecked(self.sim_monitor.enabled)
        self.live_sim.toggled.connect(lambda on: setattr(self.sim_monitor, "enabled", on))
        grid.addWidget(self.live_sim, 1, 0)
        grid.addWidget(button("Reconnect", self._sim_reconnect), 1, 1, 1, 2)
        for c, action in enumerate(("start", "pause", "stop")):
            grid.addWidget(self.run_button(action.capitalize(), partial(self._sim_action, action)), 2, c)
        self.scene_combo = QComboBox()
        self.scene_combo.addItems(list(SIM_SCENES))
        self.scene_combo.setToolTip("\n".join(f"{k}: {p}" for k, p in SIM_SCENES.items()))
        grid.addWidget(self.scene_combo, 3, 0, 1, 2)
        grid.addWidget(self.run_button("Load scene", self._load_scene), 3, 2)
        grid.addWidget(self.run_button("List scene joints", self._list_joints), 4, 0, 1, 3)
        v.addWidget(box)
        v.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(w)
        # Scroll vertically only, so the dock must be at least as wide as its content.
        scroll.setMinimumWidth(w.minimumSizeHint().width() + scroll.verticalScrollBar().sizeHint().width())
        dock.setWidget(scroll)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)

    def _build_console_dock(self) -> None:
        dock = QDockWidget("Console", self)
        dock.setObjectName("console_dock")
        self._docks["console"] = dock
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(4, 4, 4, 4)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(mono())
        self.console.setMaximumBlockCount(20000)
        self.console.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        v.addWidget(self.console, 1)
        self.cmd = CommandLine()
        self.cmd.setFont(mono())
        self.cmd.returnPressed.connect(self._on_command)
        prompt = QLabel("openarm>")
        prompt.setFont(mono())
        v.addLayout(row(prompt, self.cmd, button("Clear", self.console.clear),
                        button("Save log…", self._save_log), stretch=False))
        dock.setWidget(w)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)
        self._formats = {}
        for kind, color in LOG_COLORS.items():
            fmt = QTextCharFormat()
            fmt.setForeground(QColor(color))
            if kind == "cmd":
                fmt.setFontWeight(QFont.Weight.Bold)
            self._formats[kind] = fmt

    # ── connection actions ─────────────────────────────────────────────────

    def _enable(self, side: str) -> None:
        self.run_job(f"enable {side}", partial(actions.enable, side))

    def _disable(self, side: str) -> None:
        self.run_job(f"disable {side}", partial(actions.disable, side),
                     confirm=f"Disable the {side} arm's motors? It loses all torque and DROPS.")

    def _hold(self) -> None:
        kp, kd = self.gains()
        sides = self.sides()
        self.run_job("hold", lambda s: [actions.hold_here(side, kp, kd, s) for side in sides])

    def _limp(self) -> None:
        sides = self.sides()
        self.run_job("limp", lambda s: [actions.go_limp(side, s) for side in sides],
                     confirm=f"Make the {' and '.join(sides)} arm limp?\n\nOnly the gravity "
                             f"feedforward joints keep any torque -- support the arm.")

    def _print_state(self) -> None:
        sides = self.sides()
        self.run_job("state", lambda s: [actions.print_state(side, s) for side in sides])

    def _gravity_changed(self, mask: list[int]) -> None:
        for side in set_gravity_mask(self.worker.session, mask):
            self.compute.submit(partial(build_dynamics, side), partial(self._attach_dynamics, side))
        on = [JOINT_NAMES[j] for j, m in enumerate(mask) if m]
        print(f"[gui] gravity feedforward on: {', '.join(on) if on else 'no joints'}")
        holding = [s for s in SIDES if self.telemetry.real_status[s] == "holding"]
        if holding:
            print(f"[gui] the {' and '.join(holding)} arm keeps its current hold until its next "
                  f"command -- Hold pose applies the change now")

    def _attach_dynamics(self, side: str, dyn, error) -> None:
        if error is not None:
            print(f"[gui] couldn't load the {side} dynamics model, so its gravity feedforward "
                  f"stays off: {error}")
            return
        attach_dynamics(self.worker.session, side, dyn)

    def _gravity_save(self, mask: list[int]) -> None:
        if self.save_user_config():
            print(f"[gui] gravity feedforward switches saved: {mask}")

    # ── user_config.yaml ───────────────────────────────────────────────────

    def _store_gui_state(self) -> None:
        """Copy the window's current state into self.settings (-> gui: section)."""
        self.settings.setValue("ui/geometry", self.saveGeometry())
        self.settings.setValue("ui/state", self.saveState())
        self.settings.setValue("ui/arm", self.arm())
        self.settings.setValue("ui/backend", self.backend())
        self.settings.setValue("ui/tab", self.tabs.currentIndex())
        self.settings.setValue("real/poll", "true" if self.worker.poll_real else "false")
        self.settings.setValue("sim/live", "true" if self.sim_monitor.enabled else "false")
        self.settings_tab.save(self.settings)
        for tab in self._tabs:
            self.settings.setValue(f"tabs/{tab.title}", tab.state())

    def save_user_config(self) -> bool:
        """Write everything to user_config.yaml (config.yaml is never touched).
        Limits still waiting in the editor are applied first when possible;
        otherwise the ones in use are what's saved."""
        if not self.worker.busy:
            self.settings_tab.apply_limits()
        self._store_gui_state()
        kp, kd = self.gains()
        host, port = self.sim_endpoint()
        try:
            save_user_config(build_user_config(kp, kd, arm_can.TAU_G_ENABLE, host, port,
                                               self.settings.data))
        except (OSError, ValueError, TypeError) as e:
            print(f"[gui] ERROR: couldn't write {USER_CONFIG_FILE}: {e}")
            return False
        self.gravity_box.mark_saved()
        print(f"[gui] saved settings to {USER_CONFIG_FILE}")
        return True

    def limits_changed(self) -> None:
        """The real-arm limits were just changed in Settings (config's lists,
        updated in place): refresh what copied them."""
        self.joint_tab.editor.refresh_limits()
        self.plunger_tab.editor.refresh_limits()

        def requalify():   # on the compute thread, which owns its IK models
            for (side, source), km in self.compute._models.items():
                if source == "real":
                    km.robot.qlim = _load_limits_rad(side, "real")
        self.compute.submit(requalify, lambda _r, e: e and print(f"[gui] IK limits not updated: {e}"))

    def _sim_action(self, action: str) -> None:
        self.run_job(f"sim {action}", partial(actions.simulation, action, *self.sim_endpoint()))

    def _load_scene(self) -> None:
        name = self.scene_combo.currentText()
        path = SIM_SCENES[name]
        if self.ask(f"Load the {name} scene into CoppeliaSim?\n\n{path}\n\nIt replaces the open "
                    f"scene; unsaved changes there are lost."):
            self.run_job("load scene", partial(actions.load_scene, path, *self.sim_endpoint()))
            self.sim_monitor.reconnect()

    def _list_joints(self) -> None:
        self.run_job("list joints", partial(actions.list_joints, *self.sim_endpoint()))

    def _sim_reconnect(self) -> None:
        self.sim_monitor.reconnect()
        self.worker.enqueue("reset sim", lambda s: s.drop_sim())

    # ── console ────────────────────────────────────────────────────────────

    def _on_command(self) -> None:
        line = self.cmd.text().strip()
        self.cmd.clear()
        if not line:
            return
        self.cmd.remember(line)
        if line in ("exit", "quit"):
            self.close()
            return
        if line == "clear":
            self.console.clear()
            return
        if line in ("help", "-h", "--help"):
            print(f"openarm> {line}")
            print(self.parser.format_help())
            return
        try:
            argv = shlex.split(line)
        except ValueError as e:
            print(f"Error: {e}")
            return
        self.run_cli([argv], argv[0])

    def _drain_log(self) -> None:
        text = self.log_buffer.drain()
        if not text:
            return
        bar = self.console.verticalScrollBar()
        at_bottom = bar.value() >= bar.maximum() - 4
        cursor = self.console.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        for piece in text.splitlines(keepends=True):
            cursor.insertText(piece, self._formats[_log_kind(piece)])
        if at_bottom:
            bar.setValue(bar.maximum())

    def _save_log(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save console log",
                                              os.path.expanduser("~/openarm_log.txt"), "Text (*.txt)")
        if path:
            with open(path, "w") as f:
                f.write(self.console.toPlainText())
            print(f"[gui] log saved to {path}")

    # ── job state ──────────────────────────────────────────────────────────

    def _on_started(self, label: str) -> None:
        self._set_busy(True)
        self.job_started.emit(label)

    def _on_finished(self, label: str, outcome: str, seconds: float) -> None:
        self._set_busy(self.worker.busy)
        if not self.worker.busy:
            self.settings_tab.apply_limits()   # any limit edit made while it ran
        if outcome != "ok" or seconds > 1.0:
            print(f"[gui] {label}: {outcome} after {seconds:.1f} s")
        self.job_finished.emit(label, outcome)

    def _set_busy(self, busy: bool) -> None:
        for b in self._run_buttons:
            b.setEnabled(not busy)
        self.stop_btn.setEnabled(busy)

    def _models_ready(self) -> None:
        for tab in self._tabs:
            tab.on_models_ready()

    # ── periodic refresh ───────────────────────────────────────────────────

    def _refresh_status(self) -> None:
        for side, dot in self.real_dots.items():
            status = self.telemetry.real_status[side]
            dot.set_status(status)
        status, detail = self.telemetry.sim_status, self.telemetry.sim_detail
        self.sim_dot.set_status(status, f"{status}  {detail}" if detail else status, detail)
        for side in SIDES:
            # Gravity feedforward only acts on the real arm, so prefer its pose.
            q, src = self.current_q(side, prefer="real")
            if q is None:
                q, src = self.current_q(side, prefer="sim")
            self.gravity_box.set_torques(side, self.gravity_models.torque(side, q), src)

    def _tick_scope(self) -> None:
        if self._current_tab() is self.scope_tab:
            self.scope_tab.tick()

    def _refresh_live(self) -> None:
        label = self.worker.current_label
        self.busy_label.setText(f"<b>▶ {label}</b>  {self.worker.elapsed:.1f} s" if label else
                                "<span style='color:#8b919e'>idle</span>")
        snap = self.telemetry.snapshot()
        now = time.monotonic()
        scene = {}
        for side in SIDES:
            col = 0 if side == "left" else 3
            real, sim = snap["real", side], snap["sim", side]
            q_real = real.pose(max_age=1.5)
            q_sim = sim.q if sim.fresh(3.0, now) else None
            chain_real = self.view_models.chain(side, q_real)
            chain_sim = self.view_models.chain(side, q_sim)
            self._fill_column(col, side, real.q, None if real.gripper is None else math.degrees(real.gripper),
                              chain_real, stale=not real.fresh(1.0, now))
            self._fill_torque(col + 1, real.tau, stale=not real.fresh(1.0, now))
            self._fill_column(col + 2, side, sim.q, None if sim.gripper is None else sim.gripper * 1000,
                              chain_sim, stale=not sim.fresh(1.5, now))
            scene["real", side] = chain_real if self.show_real.isChecked() else None
            scene["sim", side] = chain_sim if self.show_sim.isChecked() else None
        if self.show_overlay.isChecked():
            try:
                scene.update(self._current_tab().overlay())
            except Exception as e:   # an overlay is a nicety; never let it break the view
                if str(e) != self._overlay_error:
                    print(f"[gui] overlay error: {e}")
                    self._overlay_error = str(e)
        self.view3d.set_scene(scene)

    def _cell(self, r: int, c: int, text: str, stale: bool = False, bad: bool = False) -> None:
        item = self.table.item(r, c)
        if item.text() != text:
            item.setText(text)
        color = "#f87171" if bad else (MUTED if stale else FG)
        if item.foreground().color().name() != color:
            item.setForeground(QColor(color))

    def _fill_column(self, c: int, side: str, q, gripper, chain, stale: bool) -> None:
        lims = limits_deg(side)
        for j in range(7):
            if q is None:
                self._cell(j, c, "—", stale=True)
                continue
            deg = math.degrees(q[j])
            lo, hi = lims[j]
            self._cell(j, c, f"{deg:7.1f}", stale=stale, bad=not lo - 0.5 <= deg <= hi + 0.5)
        self._cell(7, c, "—" if gripper is None else f"{gripper:7.1f}", stale=stale)
        for k in range(3):
            self._cell(8 + k, c, "—" if chain is None else f"{chain[-1][k] * 1000:7.1f}", stale=stale)

    def _fill_torque(self, c: int, tau, stale: bool) -> None:
        for j in range(7):
            self._cell(j, c, "—" if tau is None else f"{tau[j]:6.2f}", stale=stale or tau is None)
        for r in range(7, len(TABLE_ROWS)):
            self._cell(r, c, "", stale=True)

    # ── closing ────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        real_on = [s for s in SIDES if self.telemetry.real_status[s] != "off"]
        if self.worker.busy or real_on:
            parts = []
            if self.worker.busy:
                parts.append(f"'{self.worker.current_label}' is still running and will be stopped.")
            if real_on:
                parts.append(f"The {' and '.join(real_on)} arm's motors will be DISABLED -- the arm "
                             f"drops, as it does when the CLI shell exits. Support it first.")
            if not self.ask("\n\n".join(parts) + "\n\nQuit?", "Quit OpenArm GUI"):
                event.ignore()
                return
        if self.reset_user_config_on_exit:
            try:
                os.remove(USER_CONFIG_FILE)
                print(f"[gui] deleted {USER_CONFIG_FILE} -- next start uses config.yaml")
            except FileNotFoundError:
                pass
            except OSError as e:
                print(f"[gui] ERROR: couldn't delete {USER_CONFIG_FILE}: {e}")
        else:
            self.save_user_config()
        self.worker.shutdown(timeout=8.0)
        if self.worker.is_alive():
            sys.__stderr__.write("WARNING: the worker didn't finish within 8 s -- the real motors may "
                                 "still be enabled; run `python3 comm/arm_can.py status` to check\n")
        self.sim_monitor.stop()
        self.compute.shutdown()
        self._drain_log()
        event.accept()


QSS = """
QLabel#note { color: #8b919e; }
QLabel#bigstatus { font-size: 20px; font-weight: bold; padding: 8px 0; }
QGroupBox { font-weight: bold; border: 1px solid #3a3f4b; border-radius: 6px; margin-top: 12px;
            padding: 8px 6px 6px 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
QPushButton { padding: 5px 12px; border-radius: 4px; border: 1px solid #3a3f4b; background: #2b2f38; }
QPushButton:hover { background: #343945; }
QPushButton:pressed { background: #23262e; }
QPushButton:disabled { color: #5c6270; background: #24272e; }
QPushButton#run { background: #1f6f3a; border-color: #2e8b4e; color: white; font-weight: bold; }
QPushButton#run:hover { background: #26844a; }
QPushButton#run:disabled { background: #22302a; border-color: #2a3a30; color: #6b7a70; }
QPushButton#stop { background: #c62828; border-color: #ef5350; color: white; font-weight: bold;
                   padding: 6px 20px; font-size: 14px; }
QPushButton#stop:hover { background: #d93a3a; }
QPushButton#stop:disabled { background: #4a2626; border-color: #5a2e2e; color: #a88; }
QPushButton#estop { background: #3a1212; border-color: #c62828; color: #ffcdd2; }
QToolBar { spacing: 6px; padding: 4px; border: none; }
QTabBar::tab { padding: 7px 16px; }
QDockWidget::title { padding: 4px 8px; background: #262932; }
"""


def apply_theme(app: QApplication) -> None:
    app.setStyle("Fusion")
    p = QPalette()
    colors = {
        QPalette.ColorRole.Window: BG, QPalette.ColorRole.WindowText: FG,
        QPalette.ColorRole.Base: "#16181d", QPalette.ColorRole.AlternateBase: "#22252c",
        QPalette.ColorRole.ToolTipBase: "#2b2f38", QPalette.ColorRole.ToolTipText: FG,
        QPalette.ColorRole.Text: FG, QPalette.ColorRole.Button: "#2b2f38",
        QPalette.ColorRole.ButtonText: FG, QPalette.ColorRole.BrightText: "#f87171",
        QPalette.ColorRole.Highlight: "#3b82f6", QPalette.ColorRole.HighlightedText: "#ffffff",
        QPalette.ColorRole.Link: "#7cc4ff", QPalette.ColorRole.PlaceholderText: MUTED,
    }
    for role, color in colors.items():
        p.setColor(role, QColor(color))
    for role in (QPalette.ColorRole.Text, QPalette.ColorRole.ButtonText, QPalette.ColorRole.WindowText):
        p.setColor(QPalette.ColorGroup.Disabled, role, QColor("#5c6270"))
    app.setPalette(p)
    app.setStyleSheet(QSS)


def run(app: QApplication, splash=None) -> int:
    log = LogBuffer(echo=sys.__stdout__)
    sys.stdout = sys.stderr = log
    install_hooks()
    apply_theme(app)
    wheel_guard = WheelGuard(app)
    app.installEventFilter(wheel_guard)
    win = MainWindow(log)
    win.show()
    if splash is not None:
        splash.finish(win)
    # Ctrl+C / SIGTERM in the launching terminal close the window the normal
    # way (motors disabled on the way out). Python only sees signals while it
    # runs, so give it a moment every 200 ms.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: win.close())
    ticker = QTimer()
    ticker.timeout.connect(lambda: None)
    ticker.start(200)
    return app.exec()
