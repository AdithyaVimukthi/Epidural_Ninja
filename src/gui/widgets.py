"""
Small widgets shared by the window: a spin-box factory, the two-arm joint-angle
editor, the kp/kd gains editor, an on/off switch, the per-joint gravity
feedforward panel and a status dot.
"""
from __future__ import annotations

import math

from PySide6.QtCore import QEvent, QObject, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (
    QAbstractButton, QAbstractSpinBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QSlider, QSpinBox, QWidget,
)

from config import ARM_TAU_MAX_NM, JOINT_NAMES, REAL_ARM_LIMITS_RAD, SIM_ARM_LIMITS_RAD

from gui.backend import SIDES
from gui.plots import FG, MUTED, SIDE_COLOR

JOINT_ROLES = ["shoulder pan", "shoulder lift", "elbow roll", "elbow pitch",
               "wrist roll", "wrist pitch", "wrist twist"]


class WheelGuard(QObject):
    """App-wide event filter: the mouse wheel only changes a spin box, combo
    box or slider that has been clicked into (has focus). Otherwise the wheel
    scrolls the page as usual -- scrolling down Settings used to nudge
    whatever box passed under the pointer (a sim port, a joint limit, Drive)."""

    _GUARDED = (QAbstractSpinBox, QComboBox, QSlider)

    def eventFilter(self, obj, event) -> bool:
        kind = event.type()
        if kind == QEvent.Type.Polish and isinstance(obj, self._GUARDED):
            obj.setFocusPolicy(Qt.FocusPolicy.StrongFocus)   # no focus by wheel
        elif kind == QEvent.Type.Wheel and isinstance(obj, self._GUARDED) and not obj.hasFocus():
            event.ignore()   # declined: Qt passes it on to the parent (the scroll area)
            return True
        return False


def dspin(lo: float, hi: float, value: float, decimals: int = 2, step: float = 1.0,
          suffix: str = "", tip: str = "") -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setValue(value)
    s.setKeyboardTracking(False)
    s.setAlignment(Qt.AlignmentFlag.AlignRight)
    if suffix:
        s.setSuffix(f" {suffix}")
    if tip:
        s.setToolTip(tip)
    return s


def ispin(lo: int, hi: int, value: int, suffix: str = "") -> QSpinBox:
    s = QSpinBox()
    s.setRange(lo, hi)
    s.setValue(value)
    s.setKeyboardTracking(False)
    s.setAlignment(Qt.AlignmentFlag.AlignRight)
    if suffix:
        s.setSuffix(f" {suffix}")
    return s


def hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFrameShadow(QFrame.Shadow.Sunken)
    return f


def note(text: str) -> QLabel:
    """A muted, wrapping explanation line."""
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setObjectName("note")
    return lbl


def limits_deg(side: str) -> list[tuple[float, float]]:
    return [(math.degrees(lo), math.degrees(hi)) for lo, hi in REAL_ARM_LIMITS_RAD[side]]


class JointEditor(QWidget):
    """A slider + spin box per joint for each arm, ranged to that arm's
    real limits (config.yaml R_*_arm.limits_deg -- sim enforces the same)."""

    released = Signal(str)   # a side whose value the user just finished setting

    _SCALE = 10   # slider ticks per degree

    def __init__(self, parent=None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        self._spins: dict[str, list[QDoubleSpinBox]] = {}
        self._sliders: dict[str, list[QSlider]] = {}
        self._lim_labels: dict[str, list[QLabel]] = {side: [] for side in SIDES}
        self._widgets: dict[str, list[QWidget]] = {side: [] for side in SIDES}
        for c, side in enumerate(SIDES):
            head = QLabel(f"{side.capitalize()} arm")
            head.setStyleSheet(f"color: {SIDE_COLOR[side]}; font-weight: bold;")
            grid.addWidget(head, 0, 1 + c * 3, 1, 3)
            self._widgets[side].append(head)
        for i in range(7):
            name = QLabel(JOINT_NAMES[i])
            name.setToolTip(JOINT_ROLES[i])
            grid.addWidget(name, i + 1, 0)
        for c, side in enumerate(SIDES):
            spins, sliders = [], []
            for i, (lo, hi) in enumerate(limits_deg(side)):
                slider = QSlider(Qt.Orientation.Horizontal)
                slider.setRange(round(lo * self._SCALE), round(hi * self._SCALE))
                slider.setMinimumWidth(110)
                slider.setToolTip(f"{side} {JOINT_NAMES[i]} ({JOINT_ROLES[i]}): [{lo:g}, {hi:g}] deg")
                spin = dspin(lo, hi, min(max(0.0, lo), hi), decimals=1, step=1.0, suffix="°")
                spin.setMinimumWidth(78)
                lim = QLabel(f"[{lo:g}, {hi:g}]")
                lim.setObjectName("note")
                slider.setValue(round(spin.value() * self._SCALE))
                slider.valueChanged.connect(lambda v, s=spin: self._from_slider(s, v))
                spin.valueChanged.connect(lambda v, sl=slider: self._from_spin(sl, v))
                slider.sliderReleased.connect(lambda side=side: self.released.emit(side))
                spin.editingFinished.connect(lambda side=side: self.released.emit(side))
                grid.addWidget(slider, i + 1, 1 + c * 3)
                grid.addWidget(spin, i + 1, 2 + c * 3)
                grid.addWidget(lim, i + 1, 3 + c * 3)
                spins.append(spin)
                sliders.append(slider)
                self._lim_labels[side].append(lim)
                self._widgets[side] += [slider, spin, lim]
            self._spins[side], self._sliders[side] = spins, sliders
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(4, 1)

    def _from_slider(self, spin: QDoubleSpinBox, v: int) -> None:
        spin.blockSignals(True)
        spin.setValue(v / self._SCALE)
        spin.blockSignals(False)

    def _from_spin(self, slider: QSlider, v: float) -> None:
        slider.blockSignals(True)
        slider.setValue(round(v * self._SCALE))
        slider.blockSignals(False)

    def values(self, side: str) -> list[float]:
        return [s.value() for s in self._spins[side]]

    def set_values(self, side: str, deg) -> None:
        """Out-of-range values land on the limit (the spin box clamps)."""
        for spin, slider, v in zip(self._spins[side], self._sliders[side], deg):
            spin.setValue(float(v))
            self._from_spin(slider, spin.value())

    def set_side_enabled(self, side: str, enabled: bool) -> None:
        for w in self._widgets[side]:
            w.setEnabled(enabled)

    def refresh_limits(self, limits: dict | None = None) -> None:
        """Re-range every slider and spin box to the current real limits (after
        they were changed in Settings), or to `limits` ({side: {"joints":
        [[lo, hi]] * 7}}, degrees -- LimitsEditor.values()) when given.
        Values outside land on the limit."""
        for side in SIDES:
            ranges = limits_deg(side) if limits is None else [tuple(j) for j in limits[side]["joints"]]
            for i, ((lo, hi), spin, slider, lim) in enumerate(zip(
                    ranges, self._spins[side], self._sliders[side], self._lim_labels[side])):
                spin.setRange(lo, hi)
                slider.blockSignals(True)
                slider.setRange(round(lo * self._SCALE), round(hi * self._SCALE))
                slider.setValue(round(spin.value() * self._SCALE))
                slider.blockSignals(False)
                slider.setToolTip(f"{side} {JOINT_NAMES[i]} ({JOINT_ROLES[i]}): [{lo:g}, {hi:g}] deg")
                lim.setText(f"[{lo:g}, {hi:g}]")


class GainsEditor(QWidget):
    """kp and kd for joint1..joint7, as passed to --kp/--kd."""

    def __init__(self, kp_default: list[float], kd_default: list[float], parent=None):
        super().__init__(parent)
        self._kp_default, self._kd_default = list(kp_default), list(kd_default)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.addWidget(QLabel("kp (Nm/rad)"), 1, 0)
        grid.addWidget(QLabel("kd (Nm·s/rad)"), 2, 0)
        self._kp, self._kd = [], []
        for j in range(7):
            head = QLabel(JOINT_NAMES[j])
            head.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(head, 0, j + 1)
            kp = dspin(0, 500, kp_default[j], decimals=1, step=5)
            kd = dspin(0, 5, kd_default[j], decimals=2, step=0.1)
            grid.addWidget(kp, 1, j + 1)
            grid.addWidget(kd, 2, j + 1)
            self._kp.append(kp)
            self._kd.append(kd)
        row = QHBoxLayout()
        reset = QPushButton("Reset to saved")
        reset.setToolTip("back to the gains the GUI started with (user_config.yaml, else config.yaml)")
        reset.clicked.connect(self.reset)
        row.addWidget(reset)
        row.addStretch(1)
        grid.addLayout(row, 3, 0, 1, 8)

    def kp(self) -> list[float]:
        return [s.value() for s in self._kp]

    def kd(self) -> list[float]:
        return [s.value() for s in self._kd]

    def is_default(self) -> bool:
        return self.kp() == self._kp_default and self.kd() == self._kd_default

    def reset(self) -> None:
        for s, v in zip(self._kp, self._kp_default):
            s.setValue(v)
        for s, v in zip(self._kd, self._kd_default):
            s.setValue(v)


class ToggleSwitch(QAbstractButton):
    """An on/off slide switch -- a checkable button, so toggled/isChecked and
    Space to flip it all work as usual."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedSize(self.sizeHint())

    def sizeHint(self) -> QSize:
        return QSize(38, 20)

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        on = self.isChecked()
        track = "#22c55e" if on else "#4b5160"
        if not self.isEnabled():
            track = "#2f343d"
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(track))
        p.drawRoundedRect(r, r.height() / 2, r.height() / 2)
        d = r.height() - 4
        x = r.right() - d - 2 if on else r.left() + 2
        p.setBrush(QColor("#f5f7fa" if self.isEnabled() else "#6b7280"))
        p.drawEllipse(QRectF(x, r.top() + 2, d, d))
        if self.hasFocus():
            p.setPen(QColor("#3b82f6"))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(r, r.height() / 2, r.height() / 2)


# Per-joint motor torque range -- arm_can clamps the feedforward to it.
_TAU_MAX_NM = ARM_TAU_MAX_NM


class GravityBox(QGroupBox):
    """A switch per joint for the gravity feedforward (control.joint_tauG_enable,
    one mask for both arms), next to the gravity torque each arm's joint sees
    at its current pose -- what switching it on would add."""

    changed = Signal(list)          # the new 7-value 0/1 mask
    save_requested = Signal(list)

    def __init__(self, mask, parent=None):
        super().__init__("Gravity feedforward (τ_g)", parent)
        self._saved = [int(bool(v)) for v in mask]
        grid = QGridLayout(self)
        grid.setVerticalSpacing(3)
        grid.addWidget(note("Adds the gravity torque at the current pose to each joint switched on "
                            "-- real arm only, both arms, every mode. Limp arms pick a change up at "
                            "once, a holding arm on its next command (Hold pose re-applies it now)."),
                       0, 0, 1, 4)
        for c, (text, color) in enumerate((("", None), ("on", None),
                                           ("L Nm", SIDE_COLOR["left"]), ("R Nm", SIDE_COLOR["right"]))):
            head = QLabel(text)
            head.setAlignment(Qt.AlignmentFlag.AlignRight if c >= 2 else Qt.AlignmentFlag.AlignLeft)
            if color:
                head.setStyleSheet(f"color: {color}; font-weight: bold;")
            grid.addWidget(head, 1, c)
        self._switches: list[ToggleSwitch] = []
        self._values: dict[str, list[QLabel]] = {side: [] for side in SIDES}
        for j in range(7):
            name = QLabel(JOINT_NAMES[j])
            name.setToolTip(JOINT_ROLES[j])
            grid.addWidget(name, j + 2, 0)
            sw = ToggleSwitch()
            sw.setChecked(bool(self._saved[j]))
            sw.setToolTip(f"gravity feedforward on {JOINT_NAMES[j]} ({JOINT_ROLES[j]})")
            sw.toggled.connect(lambda _on: self.changed.emit(self.joint_mask()))
            grid.addWidget(sw, j + 2, 1)
            self._switches.append(sw)
            for c, side in enumerate(SIDES):
                val = QLabel("—")
                val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                val.setMinimumWidth(52)
                grid.addWidget(val, j + 2, 2 + c)
                self._values[side].append(val)
        buttons = QHBoxLayout()
        for text, slot, tip in (
                ("All off", lambda: self.set_joint_mask([0] * 7), "switch every joint off"),
                ("Revert", lambda: self.set_joint_mask(self._saved), "back to the last saved switches"),
                ("Save", lambda: self.save_requested.emit(self.joint_mask()),
                 "write the settings to user_config.yaml now (they're also saved when the GUI "
                 "closes; config.yaml is never changed)")):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            buttons.addWidget(b)
        grid.addLayout(buttons, 9, 0, 1, 4)
        self._source = QLabel()
        self._source.setObjectName("note")
        self._source.setWordWrap(True)
        self._sources: dict[str, str | None] = {}
        grid.addWidget(self._source, 10, 0, 1, 4)
        grid.setColumnStretch(0, 1)

    def joint_mask(self) -> list[int]:
        return [int(sw.isChecked()) for sw in self._switches]

    def set_joint_mask(self, mask) -> None:
        """Set every switch, emitting changed once (if anything changed)."""
        mask = [int(bool(v)) for v in mask]
        if mask == self.joint_mask():
            return
        for sw, v in zip(self._switches, mask):
            sw.blockSignals(True)
            sw.setChecked(bool(v))
            sw.blockSignals(False)
            sw.update()
        self.changed.emit(self.joint_mask())

    def mark_saved(self) -> None:
        self._saved = self.joint_mask()

    def set_torques(self, side: str, tau, source: str) -> None:
        """tau: the 7 gravity torques (Nm) at the arm's `source` pose, or None."""
        mask = self.joint_mask()
        for j, label in enumerate(self._values[side]):
            if tau is None:
                text, color, tip = "—", MUTED, "no live pose for this arm"
            else:
                t = float(tau[j])
                clamped = abs(t) > _TAU_MAX_NM[j]
                text = f"{t:6.2f}"
                color = MUTED if not mask[j] else ("#fbbf24" if clamped else FG)
                tip = (f"{side} {JOINT_NAMES[j]}: gravity torque at the {source} pose"
                       + (f" -- over the motor's {_TAU_MAX_NM[j]:g} Nm, so clamped" if clamped else ""))
            if label.text() != text:
                label.setText(text)
            label.setStyleSheet(f"color: {color};")
            label.setToolTip(tip)
        self._sources[side] = source if tau is not None else None
        known = [f"{s} from {src}" for s, src in self._sources.items() if src]
        self._source.setText(f"τ_g at the {', '.join(known)} pose; grey = switched off"
                             if known else "τ_g appears once an arm has a live pose")



STATUS_COLORS = {
    "off": "#6b7280", "disabled": "#6b7280", "enabled": "#60a5fa", "holding": "#4ade80",
    "limp": "#fbbf24", "online": "#4ade80", "partial": "#fbbf24", "offline": "#f87171",
}


class StatusDot(QLabel):
    def set_status(self, status: str, text: str | None = None, tip: str = "") -> None:
        color = STATUS_COLORS.get(status, "#6b7280")
        self.setText(f'<span style="color:{color}; font-size:14px;">●</span> {text or status}')
        self.setToolTip(tip)


class LimitsEditor(QWidget):
    """Real-arm joint limits for both arms (min/max per joint, gripper
    open/close), in degrees, with the URDF (sim) limits alongside for
    reference. Holds values only -- SettingsTab applies them."""

    edited = Signal()   # any value changed

    def __init__(self, parent=None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(3)
        for c, side in enumerate(SIDES):
            head = QLabel(f"{side.capitalize()} arm  min / max")
            head.setStyleSheet(f"color: {SIDE_COLOR[side]}; font-weight: bold;")
            head.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(head, 0, 1 + c * 2, 1, 2)
        urdf = QLabel("URDF (sim) left / right")
        urdf.setObjectName("note")
        grid.addWidget(urdf, 0, 5)
        self._spins: dict[str, list[tuple[QDoubleSpinBox, QDoubleSpinBox]]] = {side: [] for side in SIDES}
        for r, name in enumerate(JOINT_NAMES[:7] + ["gripper"]):
            label = QLabel(name if r < 7 else "gripper open / close")
            label.setToolTip(JOINT_ROLES[r] if r < 7 else "open and close angles of the gripper motor")
            grid.addWidget(label, r + 1, 0)
            for c, side in enumerate(SIDES):
                pair = []
                for k in range(2):
                    spin = dspin(-360, 360, 0, decimals=1, step=1.0, suffix="°")
                    spin.setMinimumWidth(82)
                    spin.valueChanged.connect(lambda _v: self.edited.emit())
                    grid.addWidget(spin, r + 1, 1 + c * 2 + k)
                    pair.append(spin)
                self._spins[side].append(tuple(pair))
            if r < 7:
                ref = "   ".join(f"[{math.degrees(lo):g}, {math.degrees(hi):g}]"
                                  for lo, hi in (SIM_ARM_LIMITS_RAD[s][r] for s in SIDES))
                lbl = QLabel(ref)
                lbl.setObjectName("note")
                grid.addWidget(lbl, r + 1, 5)
        grid.setColumnStretch(6, 1)

    def values(self) -> dict:
        """{side: {"joints": [[lo, hi]] * 7, "gripper": [open, close]}}, degrees."""
        return {side: {"joints": [[a.value(), b.value()] for a, b in self._spins[side][:7]],
                       "gripper": [self._spins[side][7][0].value(), self._spins[side][7][1].value()]}
                for side in SIDES}

    def set_values(self, limits: dict) -> None:
        for side in SIDES:
            rows = [tuple(v) for v in limits[side]["joints"]] + [tuple(limits[side]["gripper"])]
            for (a, b), (lo, hi) in zip(self._spins[side], rows):
                a.setValue(lo)
                b.setValue(hi)
