#!/usr/bin/env python3
"""Plunger rig GUI for arduino/uno_r4_sensors.

- Arrow keys (or the on-screen buttons) jog the stepper:
    Up / Right  -> forward ('F')
    Down / Left -> reverse ('R')
    release     -> stop ('S'); Space / Esc also stop
  While a key is held, the command is repeated every 100 ms because the
  sketch stops the motor after 350 ms without an update.
- Plunge / Reset buttons drive the motor until the laser distance reaches
  the entered target (Plunge -> e.g. 50 mm, Reset -> e.g. 80 mm). Any arrow
  key, Stop, Space or Esc aborts the move.
- Live plots of the HX711 raw value and the VL53L1X distance (mm).

Requires: pyserial, matplotlib, tkinter.
"""

import csv
import math
import queue
import statistics
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200
HEARTBEAT_MS = 100      # must stay well under the sketch's 350 ms timeout
RELEASE_DEBOUNCE_MS = 40  # filters X11 key auto-repeat release/press pairs
PLOT_INTERVAL_MS = 100
MAX_POINTS = 20000

# Auto move (Plunge / Reset) safety limits
AUTO_TIMEOUT_S = 30.0       # give up if the target is not reached in time
AUTO_LASER_LOST_S = 1.0     # stop if no valid laser reading for this long
AUTO_WRONG_WAY_MM = 5.0     # stop if distance moves this far the wrong way
DEFAULT_MEDIAN_N = 5        # laser median filter length (valid readings)
DEFAULT_CONFIRM_N = 3       # filtered readings past target needed to stop

FORWARD_KEYS = {"Up", "Right"}
REVERSE_KEYS = {"Down", "Left"}


class SerialReader(threading.Thread):
    """Reads lines from the port and pushes parsed items to a queue."""

    def __init__(self, ser, out_queue):
        super().__init__(daemon=True)
        self.ser = ser
        self.out = out_queue
        self.running = True

    def run(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError, TypeError) as exc:
                if self.running:
                    self.out.put(("error", str(exc)))
                return
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("#") or line.startswith("ms,"):
                self.out.put(("info", line))
                continue
            parts = line.split(",")
            if len(parts) < 5 or not parts[0].isdigit():
                continue
            try:
                ms = int(parts[0])
                load = float(parts[1])      # "nan" parses to NaN
                dist = float(parts[2])
                status = int(parts[3])
            except ValueError:
                continue
            self.out.put(("data", ms, load, dist, status, parts[-1], line))


class PlungerGUI:
    def __init__(self, root):
        self.root = root
        root.title("Plunger Control")
        root.geometry("1100x760")
        root.minsize(800, 560)

        self.ser = None
        self.reader = None
        self.write_lock = threading.Lock()
        self.rx = queue.Queue()

        # Motor command state
        self.held = set()            # keys / buttons currently held
        self.pending_release = {}    # key -> after() id
        self.cmd_dir = 0             # -1 reverse, 0 stop, 1 forward
        self.auto = None             # active Plunge/Reset move, or None
        self.last_dist = None        # latest filtered laser distance (mm)
        self.dist_recent = deque(maxlen=50)  # raw readings for the median filter
        self.last_dist_time = 0.0    # time.monotonic() of last_dist

        # Data buffers (only valid samples are stored)
        self.t0_ms = None
        self.load_t = deque(maxlen=MAX_POINTS)
        self.load_v = deque(maxlen=MAX_POINTS)
        self.dist_t = deque(maxlen=MAX_POINTS)
        self.dist_v = deque(maxlen=MAX_POINTS)
        self.distf_v = deque(maxlen=MAX_POINTS)   # filtered, same times as dist_t
        self.last_t = 0.0
        self.dirty = False

        # CSV recording
        self.rec_file = None
        self.rec_writer = None

        self._build_ui()
        self._bind_keys()
        self.refresh_ports()

        root.after(HEARTBEAT_MS, self._heartbeat)
        root.after(20, self._drain_queue)
        root.after(PLOT_INTERVAL_MS, self._update_plots)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var,
                                       width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports,
                   takefocus=False).pack(side=tk.LEFT)
        self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect,
                                   takefocus=False)
        self.conn_btn.pack(side=tk.LEFT, padx=6)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Label(top, text="Window (s):").pack(side=tk.LEFT)
        self.window_var = tk.IntVar(value=30)
        ttk.Spinbox(top, from_=5, to=600, increment=5, width=5,
                    textvariable=self.window_var).pack(side=tk.LEFT, padx=4)
        self.pause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Pause plots", variable=self.pause_var,
                        takefocus=False).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Clear", command=self.clear_data,
                   takefocus=False).pack(side=tk.LEFT)
        self.rec_btn = ttk.Button(top, text="Record CSV…", command=self.toggle_record,
                                  takefocus=False)
        self.rec_btn.pack(side=tk.LEFT, padx=6)

        self.conn_status = ttk.Label(top, text="Disconnected", foreground="#b00020")
        self.conn_status.pack(side=tk.RIGHT)

        body = ttk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True)

        # Left: plots
        fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_load = fig.add_subplot(2, 1, 1)
        self.ax_dist = fig.add_subplot(2, 1, 2, sharex=self.ax_load)
        self.load_line, = self.ax_load.plot([], [], color="#1f77b4", lw=1.2)
        self.dist_line, = self.ax_dist.plot([], [], color="#d62728", lw=0.8,
                                            marker=".", ms=3, alpha=0.35, label="raw")
        self.distf_line, = self.ax_dist.plot([], [], color="#8b0000", lw=1.8,
                                             label="filtered (median)")
        self.ax_dist.legend(loc="upper right", fontsize=8)
        self.ax_load.set_title("Load cell (HX711 raw)")
        self.ax_load.set_ylabel("raw counts")
        self.ax_dist.set_title("Laser (VL53L1X)")
        self.ax_dist.set_ylabel("distance (mm)")
        self.ax_dist.set_xlabel("time (s)")
        for ax in (self.ax_load, self.ax_dist):
            ax.grid(True, alpha=0.3)
        self.ax_load.ticklabel_format(axis="y", style="plain", useOffset=False)
        fig.tight_layout()
        self.fig = fig
        self.canvas = FigureCanvasTkAgg(fig, master=body)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right: controls + readouts
        side = ttk.Frame(body, padding=10, width=260)
        side.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Label(side, text="Stepper", font=("TkDefaultFont", 12, "bold")).pack(anchor="w")
        ttk.Label(side, text="Hold ↑/→ forward, ↓/← reverse.\n"
                             "Release, Space or Esc to stop.",
                  justify=tk.LEFT).pack(anchor="w", pady=(2, 8))

        pad = ttk.Frame(side)
        pad.pack(pady=4)
        self.btn_fwd = tk.Button(pad, text="▲  Forward", width=14, height=2,
                                 takefocus=0)
        self.btn_stop = tk.Button(pad, text="■  Stop", width=14, height=1,
                                  takefocus=0, command=self.stop_all)
        self.btn_rev = tk.Button(pad, text="▼  Reverse", width=14, height=2,
                                 takefocus=0)
        self.btn_fwd.pack(pady=2)
        self.btn_stop.pack(pady=2)
        self.btn_rev.pack(pady=2)
        self._default_btn_bg = self.btn_fwd.cget("background")
        for btn, name in ((self.btn_fwd, "btn_fwd"), (self.btn_rev, "btn_rev")):
            btn.bind("<ButtonPress-1>", lambda e, n=name: self._press(n))
            btn.bind("<ButtonRelease-1>", lambda e, n=name: self._release_now(n))

        ttk.Label(side, text="Auto move (laser)",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(14, 2))
        auto = ttk.Frame(side)
        auto.pack(fill=tk.X)
        self.plunge_var = tk.StringVar(value="50")
        self.reset_var = tk.StringVar(value="80")
        self.plunge_btn = tk.Button(auto, text="Plunge", width=8, takefocus=0,
                                    command=lambda: self.start_auto("Plunge", self.plunge_var))
        self.reset_btn = tk.Button(auto, text="Reset", width=8, takefocus=0,
                                   command=lambda: self.start_auto("Reset", self.reset_var))
        for r, (btn, var) in enumerate(((self.plunge_btn, self.plunge_var),
                                        (self.reset_btn, self.reset_var))):
            btn.grid(row=r, column=0, sticky="w", pady=2)
            entry = ttk.Entry(auto, textvariable=var, width=7, justify=tk.RIGHT)
            entry.grid(row=r, column=1, padx=(8, 2))
            entry.bind("<Return>", lambda e: self.root.focus_set())
            ttk.Label(auto, text="mm").grid(row=r, column=2, sticky="w")
        filt = ttk.Frame(side)
        filt.pack(fill=tk.X, pady=(6, 0))
        self.median_var = tk.IntVar(value=DEFAULT_MEDIAN_N)
        self.confirm_var = tk.IntVar(value=DEFAULT_CONFIRM_N)
        self.early_var = tk.DoubleVar(value=0.0)
        for r, (label, var, lo, hi) in enumerate((
                ("Median filter (samples)", self.median_var, 1, 25),
                ("Confirm (readings)", self.confirm_var, 1, 10),
                ("Stop early (mm)", self.early_var, 0, 50))):
            ttk.Label(filt, text=label + ":").grid(row=r, column=0, sticky="w", pady=1)
            box = ttk.Spinbox(filt, from_=lo, to=hi, width=4, textvariable=var)
            box.grid(row=r, column=1, padx=(8, 0))
            box.bind("<Return>", lambda e: self.root.focus_set())
        self.fwd_decreases_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(side, text="Forward = distance decreases",
                        variable=self.fwd_decreases_var,
                        takefocus=False).pack(anchor="w", pady=(4, 0))

        grid = ttk.Frame(side)
        grid.pack(fill=tk.X, pady=(14, 4))
        self.readouts = {}
        rows = [("cmd", "Command sent"), ("motor", "Motor (Arduino)"),
                ("load", "Load raw"), ("dist", "Dist raw (filt)"),
                ("status", "Range status"), ("rate", "Sample rate")]
        for r, (key, label) in enumerate(rows):
            ttk.Label(grid, text=label + ":").grid(row=r, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="—")
            ttk.Label(grid, textvariable=var, font=("TkFixedFont", 11)).grid(
                row=r, column=1, sticky="e", padx=(8, 0))
            self.readouts[key] = var
        grid.columnconfigure(1, weight=1)

        ttk.Label(side, text="Device messages",
                  font=("TkDefaultFont", 10, "bold")).pack(anchor="w", pady=(14, 2))
        self.log = tk.Text(side, width=34, height=8, wrap="word",
                           state="disabled", takefocus=0, font=("TkFixedFont", 9))
        self.log.pack(fill=tk.BOTH, expand=True)

        self._rate_count = 0
        self._rate_t = time.monotonic()

    def _bind_keys(self):
        for key in FORWARD_KEYS | REVERSE_KEYS:
            self.root.bind_all(f"<KeyPress-{key}>", lambda e, k=key: self._press(k))
            self.root.bind_all(f"<KeyRelease-{key}>", lambda e, k=key: self._release(k))
        self.root.bind_all("<KeyPress-space>", lambda e: self.stop_all())
        self.root.bind_all("<KeyPress-Escape>", lambda e: self.stop_all())
        self.root.bind("<FocusOut>", self._on_focus_out)
        # Clicking the plot returns keyboard focus to the window for jogging.
        self.canvas.get_tk_widget().bind("<Button-1>", lambda e: self.root.focus_set())

    def _typing(self):
        """True while a text field has focus, so arrow keys edit it instead."""
        w = self.root.focus_get()
        return isinstance(w, (tk.Entry, ttk.Entry, tk.Spinbox, ttk.Spinbox))

    def _log(self, text):
        self.log.configure(state="normal")
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        self.log.configure(state="disabled")

    # --------------------------------------------------------- connection
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            acm = [p for p in ports if "ACM" in p]
            self.port_var.set(acm[0] if acm else ports[0])

    def toggle_connect(self):
        if self.ser:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2, write_timeout=0.2)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.ser = None
            return
        self.ser.reset_input_buffer()
        self.reader = SerialReader(self.ser, self.rx)
        self.reader.start()
        self.conn_btn.configure(text="Disconnect")
        self.conn_status.configure(text=f"Connected: {port}", foreground="#1b7f2a")
        self._log(f"Connected to {port} @ {BAUD}")
        self.root.focus_set()

    def disconnect(self):
        # Detach the port first so nothing below can re-enter disconnect().
        ser, self.ser = self.ser, None
        reader, self.reader = self.reader, None
        self.stop_all()  # no-op send now; just resets local motor state
        if ser:
            try:
                with self.write_lock:
                    ser.write(b"S")  # best effort; port may already be gone
            except (serial.SerialException, OSError):
                pass
        if reader:
            reader.running = False
            if reader is not threading.current_thread():
                reader.join(timeout=1.0)
        if ser:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        self.conn_btn.configure(text="Connect")
        self.conn_status.configure(text="Disconnected", foreground="#b00020")
        self.readouts["motor"].set("—")

    def _send(self, ch):
        if not self.ser:
            return
        try:
            with self.write_lock:
                self.ser.write(ch.encode("ascii"))
        except (serial.SerialException, OSError) as exc:
            self._log(f"Write error: {exc}")
            self.disconnect()

    # ------------------------------------------------------- motor control
    def _press(self, key):
        if key in FORWARD_KEYS | REVERSE_KEYS and self._typing():
            return
        if self.auto:
            self._end_auto("aborted by user")
        pending = self.pending_release.pop(key, None)
        if pending:
            self.root.after_cancel(pending)
        if key not in self.held:
            self.held.add(key)
            self._update_motor()

    def _release(self, key):
        # Defer the release so an auto-repeat press that follows immediately
        # cancels it instead of briefly stopping the motor.
        if key in self.pending_release:
            self.root.after_cancel(self.pending_release[key])
        self.pending_release[key] = self.root.after(
            RELEASE_DEBOUNCE_MS, lambda: self._release_now(key))

    def _release_now(self, key):
        self.pending_release.pop(key, None)
        self.held.discard(key)
        self._update_motor()

    def _update_motor(self):
        if self.auto and not self.held:
            return  # a stale key release must not stop an auto move
        fwd = any(k in self.held for k in FORWARD_KEYS | {"btn_fwd"})
        rev = any(k in self.held for k in REVERSE_KEYS | {"btn_rev"})
        direction = 1 if fwd and not rev else -1 if rev and not fwd else 0
        if direction != self.cmd_dir:
            self.cmd_dir = direction
            self._send({1: "F", -1: "R", 0: "S"}[direction])
            self._show_cmd()

    def _show_cmd(self):
        text = {1: "FORWARD", -1: "REVERSE", 0: "STOP"}[self.cmd_dir]
        if self.auto and self.cmd_dir:
            text = f"{self.auto['name'].upper()} {text[:3]}"
        self.readouts["cmd"].set(text)
        active = "#9be29b"
        self.btn_fwd.configure(background=active if self.cmd_dir == 1 else self._default_btn_bg)
        self.btn_rev.configure(background=active if self.cmd_dir == -1 else self._default_btn_bg)

    def stop_all(self):
        if self.auto:
            self._end_auto("stopped")
        for after_id in self.pending_release.values():
            self.root.after_cancel(after_id)
        self.pending_release.clear()
        self.held.clear()
        self.cmd_dir = 0
        self._send("S")
        self._show_cmd()

    def _on_focus_out(self, event):
        if event.widget is self.root:
            self.stop_all()

    # ---------------------------------------------------- plunge / reset
    def start_auto(self, name, target_var):
        if not self.ser:
            messagebox.showwarning(name, "Connect to the Arduino first.")
            return
        try:
            target = float(target_var.get())
        except ValueError:
            messagebox.showerror(name, f"Enter a number for the {name} distance (mm).")
            return
        if self.last_dist is None or time.monotonic() - self.last_dist_time > AUTO_LASER_LOST_S:
            messagebox.showerror(name, "No valid laser reading - cannot run auto move.")
            return
        self.stop_all()
        self.root.focus_set()
        start = self.last_dist
        if abs(start - target) < 0.5:
            self._log(f"{name}: already at {start:.0f} mm")
            return
        decrease = target < start
        try:
            early = max(0.0, float(self.early_var.get()))
        except (tk.TclError, ValueError):
            early = 0.0
        forward = decrease == self.fwd_decreases_var.get()
        self.auto = {"name": name, "target": target, "decrease": decrease,
                     "stop_at": target + early if decrease else target - early,
                     "best": start, "start": time.monotonic(), "hits": 0,
                     "confirm": max(1, self._int_var(self.confirm_var, DEFAULT_CONFIRM_N))}
        self._log(f"{name}: {start:.0f} -> {target:.0f} mm "
                  f"(median {self._median_n()}, confirm {self.auto['confirm']})")
        self.cmd_dir = 1 if forward else -1
        self._send("F" if forward else "R")
        self._show_cmd()
        btn = self.plunge_btn if name == "Plunge" else self.reset_btn
        btn.configure(background="#f6d36b")

    def _check_auto(self, dist):
        """dist is the median-filtered distance."""
        a = self.auto
        past = dist <= a["stop_at"] if a["decrease"] else dist >= a["stop_at"]
        # Stop only after several consecutive filtered readings past target,
        # so a single noisy reading cannot end the move early.
        a["hits"] = a["hits"] + 1 if past else 0
        if a["hits"] >= a["confirm"]:
            self._end_auto(f"reached {dist:.1f} mm", ok=True)
            # Report where it settles so 'Stop early' can be tuned.
            self.root.after(1500, lambda: self._log(
                f"  settled at {self.last_dist:.1f} mm (target {a['target']:.0f})"))
            return
        if a["decrease"]:
            a["best"] = min(a["best"], dist)
            wrong_way = dist - a["best"]
        else:
            a["best"] = max(a["best"], dist)
            wrong_way = a["best"] - dist
        if wrong_way > AUTO_WRONG_WAY_MM:
            self._end_auto("distance moving the wrong way - "
                           "check 'Forward = distance decreases'")

    def _end_auto(self, reason, ok=False):
        a, self.auto = self.auto, None
        self.cmd_dir = 0
        self._send("S")
        self._show_cmd()
        self.plunge_btn.configure(background=self._default_btn_bg)
        self.reset_btn.configure(background=self._default_btn_bg)
        self._log(f"{a['name']}: {reason}" + ("" if ok else " (STOPPED)"))

    @staticmethod
    def _int_var(var, default):
        try:
            return int(var.get())
        except (tk.TclError, ValueError):
            return default

    def _median_n(self):
        return min(max(1, self._int_var(self.median_var, DEFAULT_MEDIAN_N)),
                   self.dist_recent.maxlen)

    def _heartbeat(self):
        if self.auto:
            now = time.monotonic()
            if now - self.auto["start"] > AUTO_TIMEOUT_S:
                self._end_auto(f"timeout after {AUTO_TIMEOUT_S:.0f} s")
            elif now - self.last_dist_time > AUTO_LASER_LOST_S:
                self._end_auto("no valid laser reading")
        if self.cmd_dir != 0:
            self._send("F" if self.cmd_dir > 0 else "R")
        self.root.after(HEARTBEAT_MS, self._heartbeat)

    # ------------------------------------------------------------- data
    def _drain_queue(self):
        latest = None
        try:
            while True:
                item = self.rx.get_nowait()
                kind = item[0]
                if kind == "info":
                    self._log(item[1])
                elif kind == "error":
                    self._log(f"Serial error: {item[1]}")
                    self.disconnect()
                else:
                    self._handle_sample(*item[1:])
                    latest = item
        except queue.Empty:
            pass

        if latest:
            _, _, load, dist, status, motor, _ = latest
            self.readouts["load"].set("nan" if math.isnan(load) else f"{int(load)}")
            self.readouts["dist"].set("nan" if math.isnan(dist) else
                                      f"{int(dist)} ({self.last_dist:.0f}) mm")
            self.readouts["status"].set(str(status))
            self.readouts["motor"].set(motor)

        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self.readouts["rate"].set(f"{self._rate_count / (now - self._rate_t):.1f} Hz")
            self._rate_count = 0
            self._rate_t = now

        self.root.after(20, self._drain_queue)

    def _handle_sample(self, ms, load, dist, status, motor, line):
        self._rate_count += 1
        if self.t0_ms is None or ms < (self.t0_ms + self.last_t * 1000) - 1000:
            # first sample, or the board reset (millis went backwards)
            self.t0_ms = ms - int(self.last_t * 1000)
        t = (ms - self.t0_ms) / 1000.0
        self.last_t = t
        if not math.isnan(load):
            self.load_t.append(t)
            self.load_v.append(load)
        if not math.isnan(dist):
            self.dist_t.append(t)
            self.dist_v.append(dist)
            self.dist_recent.append(dist)
            n = self._median_n()
            filtered = statistics.median(list(self.dist_recent)[-n:])
            self.distf_v.append(filtered)
            self.last_dist = filtered
            self.last_dist_time = time.monotonic()
            if self.auto:
                self._check_auto(filtered)
        self.dirty = True
        if self.rec_writer:
            self.rec_writer.writerow([f"{time.time():.3f}", ms,
                                      "nan" if math.isnan(load) else int(load),
                                      "nan" if math.isnan(dist) else int(dist),
                                      status, motor])

    def clear_data(self):
        for d in (self.load_t, self.load_v, self.dist_t, self.dist_v, self.distf_v):
            d.clear()
        self.t0_ms = None
        self.last_t = 0.0
        self.dirty = True

    def _update_plots(self):
        if self.dirty and not self.pause_var.get():
            self.dirty = False
            try:
                window = max(1, int(self.window_var.get()))
            except (tk.TclError, ValueError):
                window = 30
            xmax = max(self.last_t, window)
            xmin = xmax - window
            self._set_line(self.ax_load, self.load_line, self.load_t, self.load_v, xmin)
            self._set_line(self.ax_dist, self.distf_line, self.dist_t, self.distf_v, xmin)
            self._set_line(self.ax_dist, self.dist_line, self.dist_t, self.dist_v, xmin)
            self.ax_dist.set_xlim(xmin, xmax)
            self.canvas.draw_idle()
        self.root.after(PLOT_INTERVAL_MS, self._update_plots)

    @staticmethod
    def _set_line(ax, line, ts, vs, xmin):
        xs = [t for t in ts if t >= xmin]
        ys = list(vs)[len(vs) - len(xs):]
        line.set_data(xs, ys)
        if ys:
            lo, hi = min(ys), max(ys)
            margin = (hi - lo) * 0.1 or max(abs(hi) * 0.01, 1)
            ax.set_ylim(lo - margin, hi + margin)

    # -------------------------------------------------------- recording
    def toggle_record(self):
        if self.rec_file:
            self.rec_file.close()
            self.rec_file = self.rec_writer = None
            self.rec_btn.configure(text="Record CSV…")
            self._log("Recording stopped")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=time.strftime("plunger_%Y%m%d_%H%M%S.csv"),
            filetypes=[("CSV", "*.csv")])
        self.root.focus_set()
        if not path:
            return
        self.rec_file = open(path, "w", newline="")
        self.rec_writer = csv.writer(self.rec_file)
        self.rec_writer.writerow(
            ["pc_time", "ms", "hx711_raw", "distance_mm", "range_status", "motor"])
        self.rec_btn.configure(text="Stop recording")
        self._log(f"Recording to {path}")

    def on_close(self):
        try:
            self.disconnect()
            if self.rec_file:
                self.rec_file.close()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    PlungerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
