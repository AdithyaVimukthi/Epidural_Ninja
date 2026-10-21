"""Load-cell protocol parsing (v1 / v2), tare and the fake load cell -- no board."""
import time

import numpy as np
import pytest

import loadcell as lcmod
from loadcell import FakeLoadCell, LoadCell, LoadCellError, Phantom


class FakeSerial:
    def __init__(self):
        self.sent = []
        self.in_waiting = 0

    def write(self, data):
        self.sent.append(data.decode().strip())


def board(protocol_lines):
    lc = LoadCell(port="test")
    lc._ser = FakeSerial()
    lc.phase = "probe"
    for line in protocol_lines:
        lc._handle(line)
    return lc


def test_v1_handshake_and_samples():
    lc = board(["# READY", "# ERROR unknown command"])
    assert lc.protocol == "v1" and lc._ser.sent[-1] == "start"
    lc._handle("# STARTED")
    for ms, raw in [(0, 100), (11, 110), (22, 120)]:
        lc._handle(f"{ms},{raw}")
    r = lc.latest()
    assert r.raw == 120 and r.seq == 3 and lc.dropped == 0
    lc._handle("78,130")            # 56 ms gap -> about 4 samples lost
    assert lc.dropped == 4


def test_v1_already_recording():
    lc = board(["12345,500"])
    assert lc.protocol == "v1" and lc.phase == "streaming" and lc.latest().raw == 500


def test_v2_handshake_and_dropped():
    lc = board(["# LOADCELL v2 hx711 gain128"])
    assert lc.protocol == "v2" and lc._ser.sent[-1] == "stream on"
    lc._handle("# STREAMING")
    lc._handle("10,1000,-5")
    lc._handle("11,12000,-6")
    lc._handle("15,56000,-7")
    assert lc.latest().seq == 15 and lc.latest().raw == -7 and lc.dropped == 3


def test_garbage_and_errors():
    lc = board(["# LOADCELL v2 hx711 gain128", "# STREAMING", "1,2,3", "x,y", "99999999999,1,2"])
    assert lc.latest().raw == 3
    lc._handle("# ERR not_ready")
    assert lc.error == "ERR not_ready"
    lc._handle("2,3,4")
    assert lc.error is None


def test_board_restart_rehandshakes():
    lc = board(["# LOADCELL v2 hx711 gain128", "# STREAMING", "1,2,3"])
    lc._handle("# READY")
    assert lc.phase == "probe" and lc.error == "board restarted"


def test_tare_sign_and_scale():
    lc = board(["# LOADCELL v2 x", "# STREAMING"])
    for i, raw in enumerate([1000, 1010, 990, 1005, 995, 1000]):
        lc._handle(f"{i + 1},0,{raw}")
    tr = lc.tare(1.0)
    assert tr.offset == pytest.approx(1000) and tr.n == 6
    lc._handle("7,0,3000")
    assert lc.latest().counts == pytest.approx(2000) and lc.latest().newtons is None
    lc.sign, lc.scale = -1, 200.0
    assert lc.latest().counts == pytest.approx(-2000) and lc.latest().newtons == pytest.approx(-10.0)
    with pytest.raises(LoadCellError):
        board([]).tare(0.5)


def test_fake_load_cell_phantom():
    p3 = np.array([0.3, -0.3, 0.25])
    ph = Phantom.from_config({"surface_below_mm": 10, "noise": 0, "offset": 0,
                              "bones": [{"sphere": {"center_mm": [0, 0, 25], "radius_mm": 4}}]}, p3)
    lc = FakeLoadCell(ph).open()
    surface = p3[2] - 0.010

    def reading(depth_mm, wait=0.03):
        lc.observe(np.array([p3[0], p3[1], surface - depth_mm / 1000]))
        time.sleep(wait)
        lc.observe()
        return lc.latest().raw

    assert reading(-5) == 0
    assert reading(10) == pytest.approx(10 * ph.k_tissue, rel=1e-6)
    assert reading(21.5) > 21.5 * ph.k_tissue + 0.4 * ph.k_bone     # 0.5 mm into the bone
    assert reading(15) < 0                                          # coming back out: friction
    lc.mute(0.05)
    before = lc.dropped
    time.sleep(0.06)
    lc.observe()
    assert lc.dropped > before


def test_from_config_presets():
    fake = lcmod.from_config("fake", point3=[0.3, -0.3, 0.25], phantom="slab")
    assert fake.kind == "fake" and fake.phantom.bones[0].kind == "box"
    with pytest.raises(LoadCellError):
        lcmod.from_config("fake", point3=[0.3, -0.3, 0.25], phantom="nope")


def test_csv_recorder(tmp_path):
    lc = board(["# LOADCELL v2 x", "# STREAMING"])
    rec = lcmod.CsvRecorder(str(tmp_path / "rec.csv"), note="trial 1")
    for i, raw in enumerate([1000, 1010, 990], start=1):
        lc._handle(f"{i},0,{raw}")
    assert rec.drain(lc) == 3
    assert rec.drain(lc) == 0                    # nothing new
    lc.offset, lc.scale = 1000.0, 200.0
    lc._handle("4,0,3000")
    assert rec.drain(lc) == 1
    path = rec.close()
    lines = open(path).read().splitlines()
    assert lines[0] == "# openarm-loadcell v1" and lines[2] == "# note: trial 1"
    assert lines[3].startswith("# loadcell: real load cell")
    d = np.genfromtxt(path, delimiter=",", names=True, skip_header=4, dtype=None, encoding=None)
    assert list(d["raw"]) == [1000, 1010, 990, 3000] and list(d["source"]) == ["real"] * 4
    rows = [ln.split(",") for ln in lines if not ln.startswith("#")]
    assert rows[0] == ["t_s", "seq", "raw", "tared", "newtons", "tare_offset", "source"]
    assert [r[2] for r in rows[1:]] == ["1000", "1010", "990", "3000"]
    assert rows[-1][3:6] == ["2000.0", "10.00000", "1000.0"] and rows[-1][6] == "real"
    assert rows[1][4] == ""                      # not calibrated yet then
    assert rec.drain(lc) == 0 and not rec.is_open



def test_csv_recorder_without_load_cell_keeps_4_header_lines(tmp_path):
    rec = lcmod.CsvRecorder(str(tmp_path / "empty.csv"))
    assert rec.drain(None) == 0
    lines = open(rec.close()).read().splitlines()
    assert len(lines) == 5 and lines[3] == "# loadcell: none (no load cell while recording)"
