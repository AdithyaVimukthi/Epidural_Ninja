"""ForceGuard driven by a simulated needle -- no hardware."""
import random

import pytest

from force_guard import (APPROACH, HOLD, INSERT, OUT_CHECK, ForceGuard, ForceSample, GuardConfig)

DT = 0.01          # 100 Hz control ticks
SAMPLE_DT = 0.0112  # ~89 Hz load cell


def phantom_force(depth_mm, going_out, surface=10.0, bone=None, k_tissue=40_000, k_bone=3_000_000,
                  k_friction=30_000):
    d = depth_mm - surface
    if d <= 0:
        return 0.0
    f = -k_friction * d if going_out else k_tissue * d
    if bone is not None and depth_mm > bone:
        f += k_bone * (depth_mm - bone)
    return f


class Needle:
    """Rigid needle on a line: the measured depth is the commanded one, unless
    `stop_at` (a rigid wall) holds it back; force from phantom_force."""

    def __init__(self, guard, length, plan_speed, bone=None, stop_at=None, k_arm=None, noise=5000,
                 stale_after=None, seed=1):
        self.g, self.length, self.v = guard, length, plan_speed
        self.bone, self.stop_at, self.k_arm = bone, stop_at, k_arm
        self.rng = random.Random(seed)
        self.noise = noise
        self.stale_after = stale_after
        self.tau_depth, self.rate, self.t = 0.0, 0.0, 0.0
        self.seq, self.next_sample, self.last_sample_t = 0, 0.0, 0.0
        self.value = 0.0
        self.prev_meas = 0.0
        self.trace = []

    def force(self, m, going_out):
        return phantom_force(m, going_out, bone=self.bone)

    def meas(self):
        m = self.tau_depth
        if self.stop_at is not None:
            m = min(m, self.stop_at)
        return m

    def step(self):
        self.tau_depth = min(max(self.tau_depth + self.rate * self.v * DT, 0.0), self.length)
        m = self.meas()
        going_out = m < self.prev_meas - 1e-9
        self.prev_meas = m
        while self.next_sample <= self.t and (self.stale_after is None or self.t < self.stale_after):
            f = self.force(m, going_out)
            if self.k_arm is not None:
                f += self.k_arm * (self.tau_depth - m)
            self.value = f + self.rng.gauss(0, self.noise)
            self.seq += 1
            self.last_sample_t = self.next_sample
            self.next_sample += SAMPLE_DT
        sample = ForceSample(self.value, self.t - self.last_sample_t, self.seq)
        self.rate = self.g.update(self.t, sample, self.tau_depth, m)
        self.trace.append((self.t, self.g.state, self.tau_depth, m, self.value, self.rate))
        self.t += DT

    def run(self, seconds=200):
        while not self.g.finished and self.t < seconds:
            self.step()
        return self


def make(mode="on", length=50.0, **kw):
    cfg = GuardConfig(**kw)
    return ForceGuard(cfg, length_mm=length, plan_speed_mm_s=2.0, approach_speed_mm_s=2.0,
                      insert_speed_mm_s=1.0, retract_speed_mm_s=2.0, mode=mode)


def kinds(g):
    return [e.kind for e in g.events]


def test_free_air_reaches_depth_without_contact():
    g = make(length=8.0)
    n = Needle(g, 8.0, 2.0).run()
    assert g.outcome == "depth" and g.contact_depth_mm is None
    assert g.state == HOLD and kinds(g) == ["depth"]
    assert n.tau_depth == pytest.approx(8.0)


def test_silicone_only_reaches_depth_and_slows_after_contact():
    g = make(length=40.0)
    n = Needle(g, 40.0, 2.0).run()
    assert kinds(g) == ["contact", "depth"]
    assert g.contact_depth_mm == pytest.approx(10.0, abs=1.0)   # the surface, not 2.5 mm in
    insert_rates = [r for _, s, *_, r in n.trace if s == INSERT]
    approach_rates = [r for _, s, *_, r in n.trace if s == APPROACH]
    assert max(approach_rates) == pytest.approx(1.0)
    assert max(insert_rates) <= 0.5 + 1e-9          # insert 1 mm/s on a 2 mm/s plan
    assert min(insert_rates) < max(insert_rates)    # slowed by f_soft as the force grew


def test_bone_stops_releases_retracts_and_confirms_out():
    g = make(length=50.0)
    n = Needle(g, 50.0, 2.0, bone=31.0).run()
    assert kinds(g) == ["contact", "bone", "out"]
    assert g.outcome == "bone" and g.out_confirmed
    assert 31.0 <= g.hit_depth_mm < 31.6               # decided within ~0.6 mm of touching bone
    assert n.tau_depth == pytest.approx(0.0, abs=1e-6)  # back at the line's start
    assert max(d for _, _, d, *_ in n.trace) < 32.0     # never went far into bone


def test_hold_after_bone():
    g = ForceGuard(GuardConfig(), length_mm=50, plan_speed_mm_s=2.0, insert_speed_mm_s=1.0,
                   after_bone="hold")
    n = Needle(g, 50.0, 2.0, bone=31.0).run()
    assert g.outcome == "bone" and g.state == HOLD and not g.out_confirmed
    assert n.tau_depth > 30.0


def test_log_mode_keeps_going_through_bone_but_notes_it():
    g = make(mode="log", length=40.0, f_abort=1e12)
    n = Needle(g, 40.0, 2.0, bone=31.0).run()
    assert g.outcome == "depth"
    assert "would_bone" in kinds(g)
    assert all(r == 1.0 for _, s, *_, r in n.trace if s in (APPROACH, INSERT))


def test_log_mode_still_aborts_on_force():
    g = make(mode="log", length=50.0, f_abort=1_500_000)
    Needle(g, 50.0, 2.0, bone=31.0).run()
    assert g.outcome == "abort" and "f_abort" in g.abort_reason
    assert g.out_confirmed


def test_stale_load_cell_aborts():
    g = make(length=40.0)
    Needle(g, 40.0, 2.0, stale_after=7.0).run()
    assert g.outcome == "abort" and "stale" in g.abort_reason
    assert "not_out" in kinds(g)        # can't confirm out without a load cell


def test_stall_on_compliant_arm_is_bone():
    # The needle stops at a rigid wall at 25 mm while its command keeps going;
    # the arm spring (k_arm per mm of lag) builds the force. Stiffness per mm of
    # *measured* travel is undefined there -- the stall rule catches it.
    g = make(length=50.0, k_bone=None)
    Needle(g, 50.0, 2.0, stop_at=25.0, k_arm=200_000).run()
    assert g.outcome == "bone"
    assert "stalled" in g.events[1].detail


def test_map_stop_before_known_hit():
    g = ForceGuard(GuardConfig(), length_mm=50, plan_speed_mm_s=2.0, insert_speed_mm_s=1.0,
                   map_stop_at_mm=26.0)
    n = Needle(g, 50.0, 2.0, bone=31.0).run()
    assert kinds(g) == ["contact", "map_stop", "out"]
    assert max(d for _, _, d, *_ in n.trace) == pytest.approx(26.0, abs=0.1)


def test_pull_abort():
    class Yanked(Needle):
        def force(self, m, going_out):
            return -2_000_000.0 if self.g.state == INSERT else super().force(m, going_out)

    g = make(length=40.0)
    Yanked(g, 40.0, 2.0).run()
    assert g.outcome == "abort" and "pulled" in g.abort_reason


def test_out_check_waits_for_force_to_settle():
    g = make(length=50.0)
    n = Needle(g, 50.0, 2.0, bone=31.0)
    n.run()
    out_t = [t for t, s, *_ in n.trace if s == OUT_CHECK]
    assert out_t and (out_t[-1] - out_t[0]) >= 0.29


def test_saturated_load_cell_aborts():
    g = make(length=40.0, f_abort=1e12)      # only the saturation flag can stop it
    n = Needle(g, 40.0, 2.0)
    while g.state != INSERT:
        n.step()
    g.update(n.t, ForceSample(8_324_218.0, 0.005, n.seq + 1, 0, saturated=True), n.tau_depth, n.meas())
    assert g.outcome == "abort" and "saturated" in g.abort_reason


def test_out_check_tolerates_zero_shift_below_f_out():
    class Shifted(Needle):
        def force(self, m, going_out):
            # after the bone, the zero sits 150k up (overload creep)
            return super().force(m, going_out) + (150_000 if self.g.state in ("retract", "out_check") else 0)

    g = make(length=50.0, f_out=200_000)
    Shifted(g, 50.0, 2.0, bone=31.0).run()
    assert g.out_confirmed
    g2 = make(length=50.0)                      # f_out unset -> f_contact (100k): not out
    Shifted(g2, 50.0, 2.0, bone=31.0).run()
    assert not g2.out_confirmed and "not_out" in kinds(g2)


def test_live_threshold_change_reaches_a_running_guard():
    import insertion
    insertion.GUARD_OVERRIDES.clear()
    try:
        g = make(length=50.0)
        n = Needle(g, 50.0, 2.0)            # silicone only: would reach depth
        while not (g.state == INSERT and n.value > 400_000):
            n.step()
        insertion.set_guard_override("f_abort", 300_000)     # the GUI box, mid-run
        assert insertion._apply_overrides(g.cfg) == ["f_abort 3000000.0 -> 300000.0"]
        n.run()
        assert g.outcome == "abort" and "f_abort 300000" in g.abort_reason
        assert insertion.guard_value("f_abort") == 300_000
    finally:
        insertion.GUARD_OVERRIDES.clear()
