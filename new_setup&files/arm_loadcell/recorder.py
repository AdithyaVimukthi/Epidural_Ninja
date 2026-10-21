"""USB load-cell experiment recorder. Run with python recorder.py."""
import csv
from datetime import datetime
from pathlib import Path
import re
import sys
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)
import serial
from serial.tools import list_ports
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.collections import LineCollection


class Recorder(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Load Cell Experiment Recorder")
        self.resize(1050, 720)
        self.device = None
        self.firmware_ready = False
        self.next_probe = 0
        self.buffer = bytearray()
        self.state = "idle"
        self.deadline = 0
        self.file = None
        self.writer = None
        self.csv_path = None
        self.times, self.values = [], []
        self.demo_origin = 0
        self.last_sample = 0

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        connection = QHBoxLayout()
        self.ports = QComboBox()
        self.refresh_button = QPushButton("Refresh ports")
        self.connect_button = QPushButton("Connect")
        self.demo = QCheckBox("Demo (simulated data)")
        for item in (QLabel("USB port:"), self.ports, self.refresh_button,
                     self.connect_button, self.demo):
            connection.addWidget(item)
        layout.addLayout(connection)
        settings = QHBoxLayout()
        self.name = QLineEdit("experiment")
        self.folder = QLineEdit(str(Path(__file__).resolve().parent / "recordings"))
        self.browse = QPushButton("Choose folder")
        for item in (QLabel("Experiment:"), self.name, QLabel("Save folder:"),
                     self.folder, self.browse):
            settings.addWidget(item)
        layout.addLayout(settings)
        threshold_row = QHBoxLayout()
        self.threshold_enabled = QCheckBox("Highlight above threshold")
        self.threshold_enabled.setChecked(True)
        self.threshold = QSpinBox()
        self.threshold.setRange(-8388608, 8388607)
        self.threshold.setValue(0)
        self.threshold.setGroupSeparatorShown(True)
        threshold_row.addWidget(self.threshold_enabled)
        threshold_row.addWidget(QLabel("Threshold (raw ADC counts):"))
        threshold_row.addWidget(self.threshold)
        threshold_row.addStretch()
        layout.addLayout(threshold_row)
        controls = QHBoxLayout()
        self.start_button = QPushButton("Start experiment")
        self.stop_button = QPushButton("Stop and plot")
        self.load_button = QPushButton("Open saved CSV")
        self.save_plot = QPushButton("Save graph PNG")
        for item in (self.start_button, self.stop_button, self.load_button, self.save_plot):
            controls.addWidget(item)
        layout.addLayout(controls)
        self.status = QLabel("Connect your board, or enable Demo to try the app.")
        self.stats = QLabel("Samples: 0")
        self.path_label = QLabel("")
        self.path_label.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addWidget(self.stats)
        layout.addWidget(self.path_label)
        self.figure = Figure(layout="constrained")
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.add_subplot(111)
        layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        layout.addWidget(self.canvas)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.demo.toggled.connect(self.update_controls)
        self.browse.clicked.connect(self.choose_folder)
        self.start_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(self.stop)
        self.load_button.clicked.connect(self.load_csv)
        self.save_plot.clicked.connect(self.export_plot)
        self.threshold.valueChanged.connect(self.refresh_threshold)
        self.threshold_enabled.toggled.connect(self.refresh_threshold)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(20)
        self.refresh_ports()
        self.plot()
        self.update_controls()

    def refresh_ports(self):
        selected = self.ports.currentData()
        self.ports.clear()
        for port in list_ports.comports():
            self.ports.addItem(f"{port.device} — {port.description}", port.device)
        index = self.ports.findData(selected)
        if index >= 0:
            self.ports.setCurrentIndex(index)

    def update_controls(self):
        idle = self.state == "idle"
        self.start_button.setEnabled(idle and (self.firmware_ready or self.demo.isChecked()))
        self.stop_button.setEnabled(self.state == "recording")
        self.connect_button.setEnabled(idle and not self.demo.isChecked())
        self.connect_button.setText("Disconnect" if self.device else "Connect")
        self.ports.setEnabled(idle and self.device is None)
        self.refresh_button.setEnabled(idle and self.device is None)
        self.demo.setEnabled(idle and self.device is None)
        for widget in (self.name, self.folder, self.browse, self.load_button):
            widget.setEnabled(idle)
        self.save_plot.setEnabled(idle and bool(self.values))

    def choose_folder(self):
        path = QFileDialog.getExistingDirectory(self, "Save experiments", self.folder.text())
        if path:
            self.folder.setText(path)

    def toggle_connection(self):
        if self.device:
            self.device.close()
            self.device = None
            self.firmware_ready = False
            self.status.setText("Disconnected")
        else:
            port = self.ports.currentData()
            if not port:
                QMessageBox.warning(self, "No port", "Connect the board and refresh ports.")
                return
            try:
                self.device = serial.Serial(port, 115200, timeout=0, write_timeout=0.2)
                self.buffer.clear()
                self.firmware_ready = False
                self.next_probe = time.monotonic()
                self.status.setText("Connected. Checking firmware…")
            except (serial.SerialException, OSError) as exc:
                self.device = None
                QMessageBox.warning(self, "Connection failed", str(exc))
        self.update_controls()

    def send(self, command):
        self.device.write((command + "\n").encode("ascii"))

    def start(self):
        try:
            folder = Path(self.folder.text()).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            name = re.sub(r"[^A-Za-z0-9_-]+", "_", self.name.text()).strip("_") or "experiment"
            suffix = "_DEMO" if self.demo.isChecked() else ""
            self.csv_path = folder / f"{name}{suffix}_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
            self.file = self.csv_path.open("x", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            self.writer.writerow(["elapsed_ms", "raw_value"])
            self.file.flush()
            self.times, self.values = [], []
            self.plot()
            self.path_label.setText(f"CSV: {self.csv_path}")
            if self.demo.isChecked():
                self.state = "recording"
                self.demo_origin = time.monotonic()
                self.last_sample = self.demo_origin - 0.1
            else:
                self.device.reset_input_buffer()
                self.buffer.clear()
                self.state = "starting"
                self.send("start")
                self.deadline = time.monotonic() + 5
            self.status.setText("Recording simulated data" if self.demo.isChecked() else "Waiting for board to acknowledge start…")
            self.update_controls()
        except (OSError, serial.SerialException) as exc:
            self.fail(str(exc))

    def stop(self):
        if self.demo.isChecked():
            self.finish("Experiment finished")
            return
        try:
            self.send("stop")
            self.state = "stopping"
            self.deadline = time.monotonic() + 5
            self.status.setText("Stopping; saving remaining samples…")
            self.update_controls()
        except (OSError, serial.SerialException) as exc:
            self.fail(str(exc))

    def sample(self, elapsed, raw):
        if self.times and elapsed < self.times[-1]:
            raise ValueError("Board timestamp moved backwards; experiment interrupted.")
        self.writer.writerow([elapsed, raw])
        self.file.flush()  # Preserve each received sample during an interrupted experiment.
        self.times.append(elapsed)
        self.values.append(raw)
        self.stats.setText(f"Samples: {len(self.values):,}   |   Elapsed: {elapsed / 1000:.3f} s   |   Raw: {raw:,}")

    def poll(self):
        try:
            now = time.monotonic()
            if self.demo.isChecked() and self.state == "recording":
                if now - self.last_sample >= 0.1:
                    import math
                    elapsed = int((now - self.demo_origin) * 1000)
                    self.sample(elapsed, int(125000 + 5000 * math.sin(elapsed / 1000)))
                    self.last_sample = now
            elif self.device:
                if self.state == "idle" and not self.firmware_ready and now >= self.next_probe:
                    self.send("status")
                    self.next_probe = now + 2
                self.buffer.extend(self.device.read(min(self.device.in_waiting, 65536)))
                if len(self.buffer) > 131072:
                    raise ValueError("Serial input exceeded limit; check baud rate and firmware.")
                while b"\n" in self.buffer:
                    line, _, self.buffer = self.buffer.partition(b"\n")
                    self.handle_line(line.decode("ascii", errors="replace").strip())
            if self.state in ("starting", "stopping") and now > self.deadline:
                self.fail("Board did not acknowledge the command. Check the firmware and USB connection.")
        except (OSError, serial.SerialException, ValueError) as exc:
            self.fail(str(exc))

    def handle_line(self, line):
        if line == "# READY" and self.state == "idle":
            self.firmware_ready = True
            self.status.setText("Board ready. You can start an experiment.")
            self.update_controls()
        elif self.state == "idle" and re.fullmatch(r"-?\d+", line):
            self.firmware_ready = False
            self.status.setText("Old firmware detected. Upload the updated loadcell_raw.ino, then reconnect.")
            self.update_controls()
        elif line == "# RECORDING" and self.state == "idle":
            self.send("stop")
        elif line == "# STOPPED" and self.state == "idle":
            self.send("status")
        elif line == "# STARTED" and self.state == "starting":
            self.state = "recording"
            self.status.setText("Recording — click Stop and plot to finish.")
            self.update_controls()
        elif line == "# STOPPED" and self.state in ("recording", "stopping"):
            self.finish("Experiment finished")
        elif line == "# READY" and self.state != "idle":
            self.fail("Board restarted during the experiment.")
        elif line.startswith("# ERROR"):
            self.status.setText(line[2:])
        elif line and not line.startswith("#") and self.state in ("recording", "stopping"):
            fields = line.split(",")
            if len(fields) != 2:
                raise ValueError(f"Unexpected serial data: {line[:100]}")
            elapsed, raw = map(int, fields)
            if not 0 <= elapsed <= 0xFFFFFFFF or not -8388608 <= raw <= 8388607:
                raise ValueError("Sample is outside the HX711/timestamp range.")
            self.sample(elapsed, raw)

    def finish(self, message):
        if self.file:
            self.file.close()
        self.file = self.writer = None
        self.state = "idle"
        self.status.setText(f"{message}. Saved {len(self.values):,} samples.")
        self.plot()
        self.update_controls()

    def fail(self, message):
        if self.device:
            try:
                self.send("stop")
                self.device.close()
            except (OSError, serial.SerialException):
                pass
            self.device = None
        self.firmware_ready = False
        self.finish("Interrupted — partial data preserved")
        self.status.setText(f"Error: {message}. Any received data remains in the CSV.")
        QMessageBox.warning(self, "Experiment interrupted", message)

    def refresh_threshold(self):
        self.threshold.setEnabled(self.threshold_enabled.isChecked())
        if self.state == "idle":
            self.plot()

    def plot(self):
        self.axes.clear()
        if self.values:
            seconds = [t / 1000 for t in self.times]
            self.axes.plot(seconds, self.values, linewidth=1, color="#167d9a", label="Raw readings")
            if self.threshold_enabled.isChecked():
                threshold = self.threshold.value()
                red_segments = []
                for x0, x1, y0, y1 in zip(seconds, seconds[1:], self.values, self.values[1:]):
                    if y0 > threshold and y1 > threshold:
                        red_segments.append([(x0, y0), (x1, y1)])
                    elif (y0 > threshold) != (y1 > threshold):
                        crossing = x0 + (x1 - x0) * (threshold - y0) / (y1 - y0)
                        red_segments.append([(x0, y0), (crossing, threshold)] if y0 > threshold
                                            else [(crossing, threshold), (x1, y1)])
                if red_segments:
                    self.axes.add_collection(LineCollection(red_segments, colors="#d62728", linewidths=1.5))
                above = [(x, y) for x, y in zip(seconds, self.values) if y > threshold]
                self.axes.scatter([x for x, _ in above], [y for _, y in above],
                                  color="#d62728", s=12, zorder=3, label=f"Above threshold ({len(above):,})")
                self.axes.axhline(threshold, color="#d62728", linestyle="--", alpha=0.7,
                                  label=f"Threshold: {threshold:,}")
                self.axes.legend(loc="best")
            self.axes.set_title(f"{len(self.values):,} samples | min {min(self.values):,} | max {max(self.values):,} | mean {sum(self.values)/len(self.values):,.1f}")
        else:
            self.axes.set_title("The graph appears when the experiment finishes")
        self.axes.set_xlabel("Elapsed time (seconds)")
        self.axes.set_ylabel("Raw HX711 ADC counts")
        self.axes.grid(True, alpha=0.25)
        self.canvas.draw_idle()

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open experiment", self.folder.text(), "CSV files (*.csv)")
        if not path:
            return
        try:
            times, values = [], []
            with open(path, newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                if reader.fieldnames != ["elapsed_ms", "raw_value"]:
                    raise ValueError("Expected CSV columns: elapsed_ms,raw_value")
                for row in reader:
                    t, value = int(row["elapsed_ms"]), int(row["raw_value"])
                    if t < 0 or (times and t < times[-1]) or not -8388608 <= value <= 8388607:
                        raise ValueError("Invalid sample or timestamp order in CSV.")
                    times.append(t)
                    values.append(value)
            self.times, self.values = times, values
            self.csv_path = Path(path)
            self.path_label.setText(f"CSV: {path}")
            self.stats.setText(f"Samples: {len(values):,}")
            self.status.setText("Loaded saved experiment")
            self.plot()
            self.update_controls()
        except (OSError, ValueError, KeyError, TypeError, csv.Error) as exc:
            QMessageBox.warning(self, "Cannot load CSV", str(exc))

    def export_plot(self):
        default = str(self.csv_path.with_suffix(".png")) if self.csv_path else "experiment.png"
        path, _ = QFileDialog.getSaveFileName(self, "Save graph", default, "PNG image (*.png)")
        if path:
            try:
                self.figure.savefig(path, dpi=180)
                self.status.setText(f"Graph saved: {path}")
            except OSError as exc:
                QMessageBox.warning(self, "Cannot save graph", str(exc))

    def closeEvent(self, event):
        if self.state != "idle":
            answer = QMessageBox.question(self, "Finish experiment?", "Stop recording and close? Received samples have already been saved.")
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
        if self.device:
            try:
                self.send("stop")
                self.device.close()
            except (OSError, serial.SerialException):
                pass
        if self.file:
            self.file.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Recorder()
    window.show()
    sys.exit(app.exec())
