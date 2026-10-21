"""
The needle load cell: a 1 kg cell on an HX711, read by a XIAO ESP32S3 over USB
serial. Plan: docs/09_loadcell_force_feedback_plan.md (Phase 1-2).

Two firmwares are understood, told apart when the port opens:

  v1  new_setup&files/arm_loadcell/loadcell_raw/loadcell_raw.ino -- the
      recorder.py firmware. `start` / `stop`; samples are "<elapsed_ms>,<raw>".
  v2  new_setup&files/arm_loadcell/loadcell_stream/loadcell_stream.ino --
      `hello` answers "# LOADCELL v2 ...", `stream on` / `stream off`; samples
      are "<seq>,<board_us>,<raw>". The sequence number shows every lost sample.

A background thread reads the port, so the 100 Hz control loop only ever takes
the newest sample (latest(), never blocks). Values:

  raw      HX711 counts, as sent -- never changed, so logs can be re-analysed
  counts   sign * (raw - tare offset): tared, positive = pushing the needle in
  newtons  counts / scale_counts_per_n, once calibrated (else None)

Tare comes from the samples already received (tare(seconds) takes the median
of the last `seconds`), so it never waits on the port.

FakeLoadCell has the same interface for testing without the board: it makes
89 Hz samples from a simple phantom (a silicone layer with bone inside) and
the needle-tip position the insertion loop reports through observe().

CLI (from src/):
    python3 comm/loadcell.py ports                     # list serial ports
    python3 comm/loadcell.py monitor [--seconds S]     # live reading
    python3 comm/loadcell.py noise --seconds 5         # baseline noise + rate
    python3 comm/loadcell.py calibrate --masses 0 50 100 200 [--load hang] [--save]
    python3 comm/loadcell.py fake                      # FakeLoadCell walk-through
"""
from __future__ import annotations

import argparse
import math
import os
import random
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _bootstrap  # noqa: F401,E402 -- puts every src/ sub-directory on sys.path

import numpy as np  # noqa: E402

from config import LOADCELL_CFG  # noqa: E402

G = 9.80665
RAW_MIN, RAW_MAX = -8_388_608, 8_388_607
# HX711 at 80 SPS runs ~89 Hz on this board (11-12 ms between samples).
NOMINAL_PERIOD_S = 0.01124
ESPRESSIF_VID = 0x303A


class LoadCellError(RuntimeError):
    pass


@dataclass(frozen=True)
class Sample:
    t: float        # host time.monotonic() when it arrived
    seq: int
    raw: int


@dataclass(frozen=True)
class Reading:
    raw: int
    counts: float               # tared, signed (+ = pushing the needle in)
    newtons: float | None       # None until calibrated
    age_s: float
    seq: int
    dropped: int
    t: float

    @property
    def saturated(self) -> bool:
        """The HX711 is at the end of its range: the real force is beyond what
        it can report (a hand press reached it on 2026-10-12)."""
        return self.raw >= RAW_MAX or self.raw <= RAW_MIN

    def value(self, units: str) -> float:
        if units == "counts":
            return self.counts
        if self.newtons is None:
            raise LoadCellError("force in N needs a calibrated load cell (loadcell.scale_counts_per_n)")
        return self.newtons


@dataclass(frozen=True)
class TareResult:
    offset: float
    sd: float
    peak_to_peak: float
    n: int


class _LoadCellBase:
    """History, tare and scaling shared by the real and the fake load cell."""

    kind = "base"

    def __init__(self, sign: int = 1, scale_counts_per_n: float | None = None, history_s: float = 120.0):
        if sign not in (1, -1):
            raise ValueError("sign must be +1 or -1")
        self.sign = sign
        self.scale = scale_counts_per_n
        self.offset = 0.0
        self.tare_result: TareResult | None = None
        self.dropped = 0
        self.error: str | None = None
        self._lock = threading.Lock()
        self._hist: deque[Sample] = deque(maxlen=int(history_s / NOMINAL_PERIOD_S) + 100)
        self._seq = 0

    # ── reading ────────────────────────────────────────────────────────────

    @property
    def units(self) -> str:
        return "N" if self.scale else "counts"

    def _push(self, raw: int, seq: int | None = None, t: float | None = None) -> None:
        with self._lock:
            if seq is None:
                self._seq += 1
                seq = self._seq
            self._hist.append(Sample(time.monotonic() if t is None else t, seq, int(raw)))

    def latest(self) -> Reading | None:
        with self._lock:
            s = self._hist[-1] if self._hist else None
            dropped = self.dropped
        if s is None:
            return None
        counts = self.sign * (s.raw - self.offset)
        return Reading(raw=s.raw, counts=counts, newtons=counts / self.scale if self.scale else None,
                       age_s=time.monotonic() - s.t, seq=s.seq, dropped=dropped, t=s.t)

    def history(self, seconds: float) -> list[Sample]:
        """Samples from the last `seconds`, oldest first."""
        since = time.monotonic() - seconds
        with self._lock:
            out = []
            for s in reversed(self._hist):
                if s.t < since:
                    break
                out.append(s)
        out.reverse()
        return out

    def series(self, seconds: float, units: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(t relative to now, value) over the last `seconds` -- for plots."""
        hist = self.history(seconds)
        if not hist:
            return np.zeros(0), np.zeros(0)
        now = time.monotonic()
        t = np.array([s.t - now for s in hist])
        counts = self.sign * (np.array([s.raw for s in hist], dtype=float) - self.offset)
        if (units or self.units) == "N" and self.scale:
            return t, counts / self.scale
        return t, counts

    def rate_hz(self, seconds: float = 2.0) -> float:
        hist = self.history(seconds)
        if len(hist) < 2:
            return 0.0
        return (len(hist) - 1) / max(hist[-1].t - hist[0].t, 1e-6)

    @property
    def healthy(self) -> bool:
        r = self.latest()
        return r is not None and r.age_s < 0.5

    # ── tare / scaling ─────────────────────────────────────────────────────

    def tare(self, seconds: float = 0.5, min_samples: int = 5) -> TareResult:
        """Zero on the median of the last `seconds` of samples."""
        hist = self.history(seconds)
        if len(hist) < min_samples:
            raise LoadCellError(f"tare needs {min_samples}+ samples from the last {seconds:g} s, "
                                f"got {len(hist)} -- is the load cell streaming?")
        raws = [s.raw for s in hist]
        result = TareResult(offset=float(statistics.median(raws)),
                            sd=float(statistics.pstdev(raws)) if len(raws) > 1 else 0.0,
                            peak_to_peak=float(max(raws) - min(raws)), n=len(raws))
        self.offset = result.offset
        self.tare_result = result
        return result

    def to_counts(self, newtons: float) -> float:
        if not self.scale:
            raise LoadCellError("not calibrated")
        return newtons * self.scale

    def observe(self, tip_xyz=None, axis=None) -> None:
        """Where the needle tip is now. Only the fake load cell uses it."""

    def describe(self) -> str:
        cal = f"{self.scale:g} counts/N" if self.scale else "uncalibrated (counts)"
        return f"{self.kind} load cell, sign {self.sign:+d}, {cal}"


# ── the real board ────────────────────────────────────────────────────────────

def find_port() -> str | None:
    """The XIAO's port: an Espressif USB device first, else the first ttyACM."""
    try:
        from serial.tools import list_ports
    except ImportError as e:
        raise LoadCellError("pyserial is not installed (pip install pyserial)") from e
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device
    for p in ports:
        if "ACM" in p.device:
            return p.device
    return None


class LoadCell(_LoadCellBase):
    kind = "real"

    def __init__(self, port: str = "auto", baud: int = 115200, sign: int = 1,
                 scale_counts_per_n: float | None = None, history_s: float = 120.0):
        super().__init__(sign, scale_counts_per_n, history_s)
        self.port = port
        self.baud = baud
        self.protocol: str | None = None     # "v1" / "v2" once known
        self.phase = "closed"                # closed, probe, starting, streaming
        self.lines: deque[str] = deque(maxlen=20)   # recent non-sample lines, for errors
        self._ser = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._streaming = threading.Event()
        self._last_board: int | None = None

    def open(self, timeout: float = 8.0) -> "LoadCell":
        try:
            import serial
        except ImportError as e:
            raise LoadCellError("pyserial is not installed (pip install pyserial)") from e
        port = find_port() if self.port in (None, "", "auto") else self.port
        if port is None:
            raise LoadCellError("no load-cell port found (no Espressif / ttyACM device) -- "
                                "is the XIAO plugged in?")
        try:
            self._ser = serial.Serial(port, self.baud, timeout=0.05, write_timeout=0.5)
        except serial.SerialException as e:
            if "Permission denied" in str(e):
                raise LoadCellError(
                    f"permission denied on {port} -- add yourself to the dialout group "
                    f"(sudo usermod -aG dialout $USER, then log out and in) or, for now, "
                    f"sudo setfacl -m u:$USER:rw {port}") from e
            raise LoadCellError(f"can't open {port}: {e}") from e
        self.port = port
        self.phase = "probe"
        self._stop.clear()
        self._streaming.clear()
        self._thread = threading.Thread(target=self._run, name="loadcell", daemon=True)
        self._thread.start()
        if not self._streaming.wait(timeout):
            last = " | ".join(self.lines) or "nothing"
            self.close()
            raise LoadCellError(f"{port}: no samples within {timeout:g} s (board said: {last})")
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                if self.protocol == "v2":
                    ser.write(b"stream off\n")
                elif self.protocol == "v1":
                    ser.write(b"stop\n")
                ser.flush()
            except Exception:
                pass
            try:
                ser.close()
            except Exception:
                pass
        self.phase = "closed"

    @property
    def is_open(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def describe(self) -> str:
        proto = f"firmware {self.protocol}" if self.protocol else "firmware ?"
        return f"{super().describe()}, {self.port} ({proto})"

    # ── reader thread ──────────────────────────────────────────────────────

    def _send(self, text: str) -> None:
        try:
            self._ser.write((text + "\n").encode("ascii"))
        except Exception as e:
            self.error = f"write failed: {e}"

    def _run(self) -> None:
        buf = bytearray()
        next_probe = 0.0
        while not self._stop.is_set():
            try:
                data = self._ser.read(max(1, self._ser.in_waiting))
            except Exception as e:
                self.error = f"serial read failed: {e}"
                self.phase = "closed"
                return
            if data:
                buf += data
                if len(buf) > 65536:
                    buf.clear()
                while b"\n" in buf:
                    raw_line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    self._handle(raw_line.decode("ascii", errors="replace").strip())
            now = time.monotonic()
            if self.phase == "probe" and now >= next_probe:
                self._send("hello")
                next_probe = now + 0.5

    def _handle(self, line: str) -> None:
        if not line:
            return
        if line[0] in "-0123456789":
            self._sample(line)
            return
        self.lines.append(line)
        up = line.upper()
        if up.startswith("# LOADCELL V2"):
            self.protocol = "v2"
            if self.phase == "probe":
                self.phase = "starting"
                self._send("stream on")
        elif self.phase == "probe" and (up.startswith("# ERROR UNKNOWN") or up in ("# RECORDING", "# STOPPED")):
            self.protocol = "v1"
            self.phase = "starting"
            self._last_board = None
            self._send("start")
        elif up in ("# STARTED", "# STREAMING"):
            self.phase = "streaming"
        elif up == "# READY" and self.phase in ("starting", "streaming"):
            # The board restarted under us -- shake hands again.
            self.error = "board restarted"
            self.phase = "probe"
        elif up.startswith("# ERR"):
            self.error = line[2:].strip()

    def _sample(self, line: str) -> None:
        try:
            fields = [int(v) for v in line.split(",")]
        except ValueError:
            return
        if not 0 <= fields[0] <= 0xFFFFFFFF:
            return
        if len(fields) == 2 and self.protocol in (None, "v1"):
            board_ms, raw = fields
            if self.protocol is None:   # v1 board already recording from an earlier session
                self.protocol = "v1"
            if self._last_board is not None and board_ms > self._last_board:
                gap = (board_ms - self._last_board) / 1000.0
                if gap > 2.2 * NOMINAL_PERIOD_S:
                    self.dropped += max(0, round(gap / NOMINAL_PERIOD_S) - 1)
            self._last_board = board_ms
            seq = None
        elif len(fields) == 3 and self.protocol in (None, "v2"):
            seq, _, raw = fields
            self.protocol = "v2"
            if self._last_board is not None and self._last_board + 1 < seq < self._last_board + 10_000:
                self.dropped += seq - self._last_board - 1   # (a bigger jump is a board restart)
            self._last_board = seq
        else:
            return
        if not RAW_MIN <= raw <= RAW_MAX:
            return
        self.phase = "streaming"
        self.error = None
        self._push(raw, seq)
        self._streaming.set()


# ── the fake one ──────────────────────────────────────────────────────────────

@dataclass
class Bone:
    """A hard region inside the phantom, robot_base metres."""
    kind: str                         # "sphere" or "box"
    center: np.ndarray | None = None  # sphere
    radius: float = 0.0
    lo: np.ndarray | None = None      # box corners
    hi: np.ndarray | None = None

    def penetration(self, p: np.ndarray) -> float:
        """How far inside, metres (0 outside)."""
        if self.kind == "sphere":
            return max(0.0, self.radius - float(np.linalg.norm(p - self.center)))
        if np.all(p >= self.lo) and np.all(p <= self.hi):
            return float(min(np.min(p - self.lo), np.min(self.hi - p)))
        return 0.0


@dataclass
class Phantom:
    """A flat silicone block (top at surface_z) with bones inside. Forces in
    raw counts: tissue k_tissue per mm below the surface going in, -k_friction
    per mm coming out, plus k_bone per mm pushed into bone."""
    surface_z: float
    bones: list[Bone] = field(default_factory=list)
    k_tissue: float = 40_000.0
    k_friction: float = 40_000.0
    k_bone: float = 3_000_000.0
    offset: float = 25_000.0          # raw counts with nothing touching (syringe weight)
    noise: float = 8_000.0            # sd, counts

    @classmethod
    def from_config(cls, conf: dict, point3) -> "Phantom":
        """Placed below plunger point 3: the surface surface_below_mm under it;
        each bone given relative to the surface point straight below point 3
        (x, y in robot_base mm, z = mm *below* the surface)."""
        p3 = np.asarray(point3, dtype=float)
        surface_z = p3[2] - conf.get("surface_below_mm", 10.0) / 1000
        origin = np.array([p3[0], p3[1], surface_z])

        def at(v):
            x, y, depth = (float(a) / 1000 for a in v)
            return origin + np.array([x, y, -depth])

        bones = []
        for b in conf.get("bones", []) or []:
            if "sphere" in b:
                s = b["sphere"]
                bones.append(Bone("sphere", center=at(s["center_mm"]), radius=s["radius_mm"] / 1000))
            elif "box" in b:
                a, c = at(b["box"]["lo_mm"]), at(b["box"]["hi_mm"])
                bones.append(Bone("box", lo=np.minimum(a, c), hi=np.maximum(a, c)))
        keys = ("k_tissue", "k_friction", "k_bone", "offset", "noise")
        return cls(surface_z=surface_z, bones=bones, **{k: float(conf[k]) for k in keys if k in conf})

    def force(self, tip: np.ndarray, going_out: bool) -> float:
        depth_mm = (self.surface_z - tip[2]) * 1000
        if depth_mm <= 0:
            return 0.0
        f = -self.k_friction * depth_mm if going_out else self.k_tissue * depth_mm
        f += self.k_bone * 1000 * max((b.penetration(tip) for b in self.bones), default=0.0)
        return f

    def describe(self) -> str:
        parts = [f"surface z {self.surface_z:.4f} m"]
        for b in self.bones:
            if b.kind == "sphere":
                parts.append(f"bone sphere r {b.radius * 1000:.1f} mm at "
                             f"{np.array2string(b.center, precision=4)}")
            else:
                parts.append(f"bone box {np.array2string(b.lo, precision=4)}..{np.array2string(b.hi, precision=4)}")
        return ", ".join(parts)


class FakeLoadCell(_LoadCellBase):
    """Same interface as LoadCell; samples come from `phantom` and the tip
    position passed to observe() -- call it every control tick."""

    kind = "fake"

    def __init__(self, phantom: Phantom, rate_hz: float = 1 / NOMINAL_PERIOD_S, sign: int = 1,
                 scale_counts_per_n: float | None = None, seed: int = 0, history_s: float = 120.0):
        super().__init__(sign, scale_counts_per_n, history_s)
        self.phantom = phantom
        self.period = 1.0 / rate_hz
        self._rng = random.Random(seed)
        self._tip: np.ndarray | None = None
        self._going_out = False
        self._next_t: float | None = None
        self._mute_until = 0.0
        self.port = "fake"
        self.protocol = "fake"

    def open(self, timeout: float = 0.0) -> "FakeLoadCell":
        self._next_t = time.monotonic()
        return self

    def close(self) -> None:
        self._next_t = None

    @property
    def is_open(self) -> bool:
        return self._next_t is not None

    def mute(self, seconds: float) -> None:
        """Stop producing samples for a while -- a USB dropout, for tests."""
        self._mute_until = time.monotonic() + seconds

    def observe(self, tip_xyz=None, axis=None) -> None:
        if self._next_t is None:
            return
        if tip_xyz is not None:
            tip = np.asarray(tip_xyz, dtype=float)
            if self._tip is not None:
                dz = tip[2] - self._tip[2]
                if abs(dz) > 1e-7:
                    self._going_out = dz > 0
            self._tip = tip
        now = time.monotonic()
        if now - self._next_t > 1.0:      # nobody observed for a while: don't backfill
            self._next_t = now - self.period
        while self._next_t <= now:
            t = self._next_t
            self._next_t += self.period
            if t < self._mute_until:
                with self._lock:
                    self._seq += 1
                    self.dropped += 1
                continue
            f = 0.0 if self._tip is None else self.phantom.force(self._tip, self._going_out)
            raw = self.phantom.offset + f + self._rng.gauss(0.0, self.phantom.noise)
            self._push(int(min(max(raw, RAW_MIN), RAW_MAX)), t=t)

    def describe(self) -> str:
        return f"{super().describe()}; phantom: {self.phantom.describe()}"


# ── recording to CSV ──────────────────────────────────────────────────────────

class CsvRecorder:
    """Every sample of a load cell, appended to a CSV as it comes. Call
    drain(lc) a few times a second (the GUI does, every 200 ms): it writes
    the samples that arrived since the last call and flushes, so a crash
    keeps everything up to the last drain. The load cell may change between
    calls (connect / a fake-load-cell run); `source` says which one each row
    is from. Always exactly 4 header lines, then the column names:

        # openarm-loadcell v1
        # created: 2026-10-12T17:10:00
        # note: <experiment name>
        # loadcell: <the first load cell recorded>
        t_s,seq,raw,tared,newtons,tare_offset,source

    so numpy reads it with
    np.genfromtxt(path, delimiter=",", names=True, skip_header=4, dtype=None, encoding=None).
    t_s: seconds since the recording started (host clock, when the sample
    arrived). tared = sign x (raw - tare_offset), reproducible from raw.
    newtons is empty until the cell is calibrated."""

    MAGIC = "# openarm-loadcell v1"
    HEADER_LINES = 4

    def __init__(self, path: str, note: str = ""):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.path = path
        self.note = note.replace("\n", " ")
        self.rows = 0
        self.t0 = time.monotonic()
        self.created = datetime.now().isoformat(timespec="seconds")
        self._last_t = self.t0
        self._f = open(path, "x")
        self._header_done = False

    def _header(self, desc: str) -> None:
        self._f.write(f"{self.MAGIC}\n# created: {self.created}\n# note: {self.note}\n"
                      f"# loadcell: {desc}\nt_s,seq,raw,tared,newtons,tare_offset,source\n")
        self._header_done = True

    def drain(self, lc) -> int:
        if self._f is None or lc is None:
            return 0
        if not self._header_done:
            self._header(lc.describe())
        span = min(time.monotonic() - self._last_t + 1.0, 120.0)
        new = [smp for smp in lc.history(span) if smp.t > self._last_t]
        for smp in new:
            tared = lc.sign * (smp.raw - lc.offset)
            newtons = f"{tared / lc.scale:.5f}" if lc.scale else ""
            self._f.write(f"{smp.t - self.t0:.4f},{smp.seq},{smp.raw},{tared:.1f},{newtons},"
                          f"{lc.offset:.1f},{lc.kind}\n")
        if new:
            self._last_t = new[-1].t
            self.rows += len(new)
        self._f.flush()
        return len(new)

    def close(self) -> str:
        if self._f is not None:
            if not self._header_done:
                self._header("none (no load cell while recording)")
            self._f.close()
            self._f = None
        return self.path

    @property
    def is_open(self) -> bool:
        return self._f is not None


# ── construction from config ──────────────────────────────────────────────────

def from_config(kind: str = "real", point3=None, phantom: str | None = None, port: str | None = None):
    """A load cell set up from config.yaml `loadcell:` (not opened)."""
    c = LOADCELL_CFG
    sign = int(c.get("sign", 1))
    scale = c.get("scale_counts_per_n") or None
    if kind == "fake":
        presets = c.get("fake_phantoms", {}) or {}
        name = phantom or c.get("fake_phantom", "sphere")
        if name not in presets:
            raise LoadCellError(f"no fake phantom {name!r} in loadcell.fake_phantoms "
                                f"(have: {', '.join(presets) or 'none'})")
        if point3 is None:
            raise LoadCellError("the fake load cell needs point 3 to place its phantom")
        return FakeLoadCell(Phantom.from_config(presets[name], point3), sign=sign, scale_counts_per_n=scale)
    return LoadCell(port or c.get("port", "auto"), int(c.get("baud", 115200)), sign=sign,
                    scale_counts_per_n=scale)


# ── CLI ───────────────────────────────────────────────────────────────────────

def _open(args) -> LoadCell:
    lc = from_config("real", port=args.port)
    print(f"[loadcell] opening {lc.port if lc.port not in (None, '', 'auto') else find_port()}...", flush=True)
    lc.open()
    print(f"[loadcell] {lc.describe()}", flush=True)
    return lc


def _cmd_ports(args) -> None:
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid is not None or args.all]
    for p in ports:
        vid = f"{p.vid:04x}:{p.pid:04x}" if p.vid is not None else "-"
        print(f"  {p.device:<16} {vid:<10} {p.description}")
    if not ports:
        print("  no USB serial ports (--all lists the built-in ttyS ones too)")
    print(f"auto -> {find_port()}")


def _cmd_monitor(args) -> None:
    lc = _open(args)
    try:
        time.sleep(0.6)
        lc.tare(0.5)
        print(f"[loadcell] tared at {lc.offset:.0f} counts; Ctrl+C to stop")
        t_end = time.monotonic() + args.seconds if args.seconds else math.inf
        while time.monotonic() < t_end:
            r = lc.latest()
            force = f"{r.newtons:8.3f} N" if r.newtons is not None else ""
            print(f"  raw {r.raw:>9d}  tared {r.counts:>10.0f}  {force}  "
                  f"{lc.rate_hz():5.1f} Hz  age {r.age_s * 1000:4.0f} ms  dropped {r.dropped}"
                  + ("  SATURATED -- ease off, beyond the ADC range" if r.saturated else "")
                  + (f"  [{lc.error}]" if lc.error else ""), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        lc.close()


def _cmd_noise(args) -> None:
    lc = _open(args)
    try:
        print(f"[loadcell] keep the needle untouched; collecting {args.seconds:g} s...", flush=True)
        time.sleep(args.seconds + 0.2)
        hist = lc.history(args.seconds)
        raws = np.array([s.raw for s in hist], dtype=float)
        gaps = np.diff([s.t for s in hist]) * 1000
        print(f"  samples {len(raws)}  rate {lc.rate_hz(args.seconds):.2f} Hz  dropped {lc.dropped}")
        print(f"  raw mean {raws.mean():.0f}  sd {raws.std():.0f}  p2p {raws.max() - raws.min():.0f}  "
              f"drift (last-first 0.5 s) {np.median(raws[-45:]) - np.median(raws[:45]):.0f}")
        print(f"  host arrival gap ms: median {np.median(gaps):.1f}  max {gaps.max():.1f}")
        print(f"  suggested guard.f_contact >= {max(5 * raws.std(), 50_000):.0f} counts "
              f"(5 sd, at least 50k)")
    finally:
        lc.close()


def _cmd_calibrate(args) -> None:
    lc = _open(args)
    loads_n, means = [], []
    try:
        time.sleep(0.6)
        for m in args.masses:
            how = "pressed onto the needle the way an insertion loads it" if args.load == "push" else \
                  "hanging from the needle hub, syringe pointing down"
            input(f"  {m:g} g {how} -- press Enter when still... ")
            time.sleep(args.settle)
            raws = [s.raw for s in lc.history(args.average)]
            if len(raws) < 5:
                raise LoadCellError("too few samples -- is the board streaming?")
            loads_n.append(m / 1000 * G)
            means.append(float(np.median(raws)))
            print(f"    median {means[-1]:.0f} counts (sd {np.std(raws):.0f}, n {len(raws)})")
    except KeyboardInterrupt:
        print("\n[loadcell] calibration cancelled")
        return
    finally:
        lc.close()
    if len(set(loads_n)) < 2:
        print("[loadcell] need at least two different masses")
        return
    slope, intercept = np.polyfit(loads_n, means, 1)
    fit = np.polyval([slope, intercept], loads_n)
    span = max(abs(means[-1] - means[0]), 1.0)
    worst = float(np.max(np.abs(np.array(means) - fit)))
    # Hanging masses pull the needle away from the syringe -- the opposite of
    # an insertion -- so they flip the sign.
    sign = int(np.sign(slope)) * (1 if args.load == "push" else -1)
    scale = abs(float(slope))
    print(f"\n  scale {scale:.1f} counts/N   sign {sign:+d}   intercept {intercept:.0f}")
    print(f"  worst residual {worst:.0f} counts = {100 * worst / span:.2f} % of the span")
    print(f"  1 kg = {scale * G:.0f} counts; 4.35 M counts (the PLA / bracket-failure peak) = "
          f"{4.35e6 / scale:.1f} N = {4.35e6 / scale / G:.2f} kg")
    print("\n  config.yaml:\n    loadcell:\n"
          f"      scale_counts_per_n: {scale:.1f}\n      sign: {sign}")
    if args.save:
        from config import USER_CONFIG_FILE, cfg, save_user_config
        import copy
        import yaml
        data = {}
        if os.path.exists(USER_CONFIG_FILE):
            with open(USER_CONFIG_FILE) as f:
                data = yaml.safe_load(f) or {}
        if not data:
            data = copy.deepcopy(cfg)
        data.setdefault("loadcell", {}).update({"scale_counts_per_n": round(scale, 1), "sign": sign})
        save_user_config(data)
        print(f"  saved to {USER_CONFIG_FILE} (close the GUI first, or it overwrites this on exit)")


def _cmd_fake(args) -> None:
    p3 = np.array([0.30, -0.30, 0.25])
    lc = from_config("fake", point3=p3, phantom=args.phantom).open()
    print(f"[loadcell] {lc.describe()}")
    surface = lc.phantom.surface_z
    for depth_mm in (-2, 0, 5, 10, 15, 20, 25, 30):
        lc.observe(np.array([p3[0], p3[1], surface - depth_mm / 1000]))
        time.sleep(0.05)
        lc.observe()
        print(f"  tip {depth_mm:>4} mm below the surface: raw {lc.latest().raw:>9d}")


def main() -> None:
    p = argparse.ArgumentParser(description="needle load cell (HX711 on a XIAO ESP32S3)")
    p.add_argument("--port", default=None, help="serial port (default: config loadcell.port, auto)")
    sub = p.add_subparsers(dest="cmd", required=True)
    pp = sub.add_parser("ports", help="list USB serial ports")
    pp.add_argument("--all", action="store_true", help="include the built-in ttyS ports")
    m = sub.add_parser("monitor", help="print the live reading")
    m.add_argument("--seconds", type=float, default=0.0, help="stop after S s (default: Ctrl+C)")
    n = sub.add_parser("noise", help="baseline noise, rate and dropped samples")
    n.add_argument("--seconds", type=float, default=5.0)
    c = sub.add_parser("calibrate", help="fit counts per newton from known masses")
    c.add_argument("--masses", type=float, nargs="+", required=True, metavar="G",
                   help="masses in grams, the first usually 0 (e.g. 0 50 100 200 500)")
    c.add_argument("--load", choices=["push", "hang"], default="push",
                   help="push: the mass presses the needle in like an insertion; hang: it hangs "
                        "from the hub, pulling the needle out (default push)")
    c.add_argument("--settle", type=float, default=1.0, help="seconds to wait after Enter")
    c.add_argument("--average", type=float, default=2.0, help="seconds averaged per mass")
    c.add_argument("--save", action="store_true", help="write the result to user_config.yaml")
    f = sub.add_parser("fake", help="show what the fake load cell reads at a few depths")
    f.add_argument("--phantom", default=None, help="loadcell.fake_phantoms entry")
    args = p.parse_args()
    try:
        {"ports": _cmd_ports, "monitor": _cmd_monitor, "noise": _cmd_noise,
         "calibrate": _cmd_calibrate, "fake": _cmd_fake}[args.cmd](args)
    except LoadCellError as e:
        print(f"[loadcell] {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
