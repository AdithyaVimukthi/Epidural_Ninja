# Needle Load Cell in the Loop: How to Use It

This guide covers what was built from
[09_loadcell_force_feedback_plan.md](09_loadcell_force_feedback_plan.md):
- reading the load cell
- the force guard on plunger point 4
- the bone-avoidance search with the occupancy map
- point 5 (retract out)
- the GUI controls for all of the above

Status on 2026-10-12: everything below is implemented and **tested in sim**
(CoppeliaSim with a fake load cell, from the CLI and from the GUI), and there
are 31 unit tests. **The real board has been checked through this code**
(§8, last rows). **Not yet tested:**
- the real arm (it's disconnected)
- the new firmware (not flashed)

Run everything from `src/` with the project venv
(`../../.venv/bin/python`, or `../run_gui.sh` for the GUI).

## 1. One-time setup

```bash
sudo usermod -aG dialout $USER     # serial access, then log out and back in
# (for right now, until the board is re-plugged:  sudo setfacl -m u:$USER:rw /dev/ttyACM0)
../../.venv/bin/pip install "pyserial>=3.5,<4"   # already done in the project venv
```

**Firmware.** The driver understands two firmwares and picks the right one when
the port opens:

| Firmware | Sketch | Notes |
|---|---|---|
| v1 | `new_setup&files/arm_loadcell/loadcell_raw/loadcell_raw.ino` | What's on the board now; `recorder.py` needs it. |
| v2 | `new_setup&files/arm_loadcell/loadcell_stream/loadcell_stream.ino` | Adds a sequence number to every sample (exact lost-sample count) and reports a dead HX711 after 200 ms instead of 2 s. **Recommended** before arm work. |

**Tests.** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../../.venv/bin/python -m pytest tests -q`.
The variable stops ROS 2's pytest plugin from breaking pytest's start-up.

## 2. Load cell on its own (`comm/loadcell.py`)

```bash
python3 comm/loadcell.py ports                 # which port is the XIAO (auto picks the Espressif one)
python3 comm/loadcell.py monitor               # live raw / tared / N, rate, dropped samples
python3 comm/loadcell.py noise --seconds 10    # baseline sd, rate, a suggested f_contact
python3 comm/loadcell.py calibrate --masses 0 50 100 200 500 [--load hang] [--save]
python3 comm/loadcell.py fake --phantom sphere # what the fake load cell reads at a few depths
```

**Calibration.** For `--load`, choose:
- `push` (default): the mass presses on the needle the way an insertion loads it.
- `hang`: the mass hangs from the hub, pulling the needle out. This flips the
  sign automatically.

`calibrate` prints `scale_counts_per_n` and `sign`. It also prints what the old
4.35 M-count PLA / bracket-failure peak works out to in N and kg, which
answers the overload question in the plan (§2, point 6).

Put both values in `config.yaml` → `loadcell:`. `--save` writes them to
`user_config.yaml` instead; close the GUI first, or it overwrites the file on
exit.

**Units.**
- `raw`: HX711 counts, never changed.
- `counts`: tared, with + meaning pushing the needle in.
- `N`: only once calibrated.

The guard works in `guard.units` (counts until you calibrate). If you switch to
`N`, convert every force in `guard:` too.

## 3. Plunger point 4 with the guard

Points 1–3 are unchanged. Point 4 has a new switch:

| `--guard` | What point 4 does |
|---|---|
| `off` (default) | Exactly as before |
| `log` | Constant speed whatever the force; force and depth logged. **The safety trips still abort, release and retract** (force limit, pull-out, stale load cell, dropped samples, lag). Bone / map stops are only noted as `would_*`. Use this to collect the Phase-3 data. |
| `on` | 1. Tare. 2. Approach. 3. **Contact**: the surface depth is taken where the force started rising. 4. **Insert**: slower, and slowing further as the force builds. 5. **Bone**: stop. 6. **Release**: the arm's stored push is taken off the needle. 7. **Retract**: back along the same line until the needle is confirmed out. |

```bash
# real arm, real load cell, data collection
python3 main.py plunger --point 4 --xyz 0.2 -0.095 0.26 --depth 0.06 --mode real --guard log --name pla_01
# real arm, full guard + search
python3 main.py plunger --point 4 --xyz 0.2 -0.095 0.26 --depth 0.06 --mode real --guard on --search --map block_A
# sim, fake load cell (default in sim), sphere bone
python3 main.py plunger --point 4 --xyz 0.2 -0.095 0.26 --depth 0.06 --guard on --search --map simtest --map-new
```

| Flag | Default | Meaning |
|---|---|---|
| `--loadcell real\|fake` | real with the arm, fake in sim | Fake = phantom model (`loadcell.fake_phantoms`), placed with its surface 10 mm under point 3. With `--mode sim --loadcell real` you can press the syringe by hand while the sim needle descends. |
| `--fake-phantom` | `sphere` | `sphere`: straight down hits it, a 15° tilt misses. `slab`: a wide shelf. `none`: silicone only. |
| `--speed` | 0.002 | Approach speed, m/s (until contact) |
| `--insert-speed` | `guard.insert_speed_mm_s` (1 mm/s) | Speed after contact, m/s. It drops further, to `s_min`, as the force nears `f_soft`. |
| `--on-bone` | `retract` | `retract`: fully out of the block. `hold`: stay on the bone. |
| `--on-success` | `hold` | At the target depth: `hold` there, or `retract` out |
| `--f-contact`, `--f-abort` | config | Quick overrides, in guard units |
| `--search` | off | After bone: re-aim and try again (§4). Needs `--guard on`. |
| `--map NAME` / `--map-new` | `map.name` | Occupancy map (§5). `--map-new` starts it afresh. |
| `--map-skip-blocked` | off | Don't even run a first straight-down line that the map already blocks |
| `--max-attempts`, `--max-tilt` | 6, 20° | Search limits |

**Tare.** Before every attempt the arm holds still for `tare_s` (0.5 s) and
tares on the median. It refuses to start if either:
- the sd is above `tare_noise_max`, or
- the offset moved more than `tare_jump_max` since the previous attempt (the
  needle may still be touching something).

**Point 5: retract out.** A successful run holds at depth, and `--on-bone hold`
holds on the bone, so the needle stays in the block. Point 5 backs it out along
exactly that line. If there is no record of one, it backs straight out along
the tool axis by `--depth`. Point 3 refuses to move while a guarded point 4 has
left the needle in, so you can't drag it sideways through the block.

```bash
python3 main.py plunger --point 5 --mode real          # after a run that held at depth
```

### Axis hold: keeping the needle straight under load (real arm)

**The problem, measured 2026-10-13.** Seven real plunges into silicone. The IK
plan held the needle exactly vertical, re-solved at 0.000° at every knot, but
the measured needle tilted 2.3–4.4° and its tip drifted 16–27 mm off the line.
The joints are servo "springs" (kp 40 Nm/rad), and the needle sits 93 mm to the
side of the wrist, so the insertion force twists the wrist. In one run joint 6
was 4.8° behind its command, which alone tilted the needle 3.8°.

**The fix: `motion/axis_hold.py`, on by default on the real arm.** Every tick it
measures the needle's tilt and the tip's sideways offset, and adds a slowly
integrating correction to the joint commands until both are gone:
- **Motion along the needle is left soft**, which the load-cell guard relies
  on (contact, stall, release).
- **Limits:** the correction is capped at 8° per joint and 5°/s, and kept
  within the joint limits.
- **When a joint reaches its limit,** the others stop counting on it, and the
  run summary warns.
- **Where it applies:** plain and guarded point 4, search attempts (frozen
  during re-aim moves) and point 5.

**Checked against a spring-joint model** of the arm that reproduces the real
runs (4.2° / 30 mm without the hold):

| Insertion | Without hold | With hold |
|---|---|---|
| 2 mm/s | 2.5° tilt, 17 mm sideways | 0.25°, 1.7 mm |
| 2 mm/s, twice as stiff | 3.4°, 23 mm | 0.47°, 3.2 mm |

Faster insertions leave more lag. **Not yet run on the real arm.**

With the needle on its line, it also reaches deeper for the same command,
because less is lost to bending. The force at a given commanded depth is
therefore higher than before.

**Logs:** the plain point-4 and v3 logs have two new columns, `tilt_deg` and
`axis_hold_deg` (the largest current joint correction). The run summary prints
peak tilt, peak sideways offset and peak correction.

**Settings:** `config.yaml axis_hold:` (ki_per_s, max_correction_deg,
max_rate_deg_s). Turn it off with `--no-axis-hold`, or untick the GUI's
*Hold the needle on its line*.

**After any point 4**, plain ones included, the needle counts as still in the
block. Points 3 and 4 refuse to move until **point 5** has backed it out along
its line, so a joint-space move can't drag it sideways through the silicone.
This is remembered within the GUI and the `openarm>` shell, not across
separate one-shot commands.

## 4. The search (`--search`)

1. **Attempt 1** goes straight down from point 3 (angles 0 0). Its contact
   gives the **entry point E** on the surface and the insertion length along
   the needle (`--depth` minus the stand-off).
2. **After a bone hit or a map stop, with the needle confirmed out**, the next
   direction is chosen from the map:
   - Candidates are tilts of 5/10/15/20° every 45°.
   - Any line passing within `hit_radius + line_margin` of a known hit is
     blocked.
   - The order is smallest tilt first, then the most clearance, then furthest
     from all hits.
3. The arm **re-aims** with a joint-space move to a start point `h_safe`
   (15 mm) above E on the new line:
   - The move is checked so the tip stays `h_min` (5 mm) above the surface.
     If it doesn't, the start is raised in 10 mm steps up to 45 mm.
   - Any force during the move reverses it back to where it started.
   - It then re-tares.
4. **The run ends** at the target depth, at `max_attempts`, when no reachable
   unblocked direction is left, or on any safety trip. A safety trip never
   leads to an automatic retry.

Each run writes a folder `recordings/search/<date>_<time>_<side>[_name]/`
containing:
- `attempt_NN.csv`
- `summary.csv` (one row per attempt)
- `map.json` (a snapshot of the map)

## 5. Occupancy map

The map file is `recordings/maps/<name>.json`, one per phantom setup, in
robot_base coordinates:
- **What it holds:** each bone hit is the needle-tip position when bone was
  decided, treated as a sphere of `hit_radius_mm` (3 mm). It also stores E and
  every attempt's line, for the views.
- **Map stop:** on every tick, if the *commanded* tip comes within
  `map_stop_mm` (2 mm) of a hit sphere, the needle stops. It never touches a
  known bone again, including a re-run of the same line in a later session.
- **Clear it whenever the block moves:** use `--map-new`, or *Clear now…* in
  the GUI. The old file is kept with a time stamp.
- **Block-moved warning:** if a contact lands more than `entry_tol_mm` (3 mm)
  from the stored E, it warns that the block may have moved.

## 6. GUI (Plunger tab)

Below point 4 there are three new boxes:

- **Needle force feedback** has the following controls:
  - Guard (off / log / on)
  - Load cell (auto / real / fake) and the fake phantom
  - Insert speed
  - On bone and At depth
  - **Thresholds**, in guard units: contact, slow from, bone ≥, **stop &
    retract** (`f_abort`) and out <. These take effect **the moment they
    change**: the plot lines move, the next run uses them, and a run already
    going picks them up on its next tick (logged as a `setting` event). A
    warning shows if they conflict, for example bone ≥ above stop & retract.
    They start from `config.yaml` on every launch. To keep a value, put it in
    `config.yaml`; *From config.yaml* resets the boxes.
  - Search, with max attempts and max tilt
  - Occupancy-map name, *start it afresh*, *Clear now…*
  - *▶ Retract out (point 5)*

  A status line shows the live state, attempt, direction, force and depth. The
  usual *▶ Plunge to point 4* runs with these settings; with Guard off it's
  the old plunge.

  The guard, load cell, search and map sections belong to `config.yaml`: the
  GUI no longer copies them into `user_config.yaml` on exit. It used to, which
  froze old values over later `config.yaml` edits. At start-up a note shows if
  `user_config.yaml` still overrides any of them.
- **Load cell** has Port, Connect / Disconnect and Tare, a live readout, and a
  force plot with the `f_contact` / `f_soft` / `f_bone_min` / `f_abort`
  lines. During a fake-load-cell run it shows the fake one.
  - **Record to CSV** appends every sample to
    `recordings/loadcell/<date>_<time>_loadcell[_<experiment name>].csv`
    while ticked. It keeps going on other tabs, flushes every 200 ms, and
    never starts by itself when the GUI opens. Columns:
    `t_s,seq,raw,tared,newtons,tare_offset,source`, after exactly 4 header
    lines. Read it with
    `np.genfromtxt(path, delimiter=",", names=True, skip_header=4, dtype=None, encoding=None)`.
  - **Graph window** (5–120 s) sets the plot's time span.
  - **Save graph PNG…** saves the graph as shown, at 150 dpi, titled with the
    date and the experiment name.
- **Occupancy map** has top and side views relative to E. Hits are drawn as
  red circles of the hit radius, and attempt lines are coloured by outcome:
  red bone, orange map stop, green depth. The 3D live view also shows hits
  and attempt lines.

Mark (M) still works during guarded runs and lands in the v3 log.

## 7. Logs (v3)

`recordings/plunger/<stamp>_<side>[_name].csv` (single run) or
`attempt_NN.csv` (search). One row per control tick:

```
t_s,mark,state,rate,cmd_depth_mm,meas_depth_mm,lag_mm,force_raw,force,force_age_ms,force_seq,
joint1_deg..joint7_deg,joint1_nm..joint7_nm
```

- The header records the arm, mode, attempt and direction, the line, speeds,
  gains, the load cell and its tare, the units, every guard threshold, marks
  and events.
- Depths are mm along the attempt's line from its start.
- `state` is the guard state.
- `rate` is the path speed as a multiple of the planned speed: negative while
  retracting.
- Torques are empty in sim.
- Read a log with `insertion.load_log(path)` or `pandas.read_csv(path, comment="#")`.

**Tuning the bone rule (Phase 4).**

```bash
python3 test_scripts/replay_force_guard.py ../recordings/plunger/*_pla_*.csv --plot
python3 test_scripts/replay_force_guard.py <silicone logs> --k-bone 8e5 --f-bone-min 1.5e6
```

This replays the recorded forward motion through a fresh guard. It reports
what the guard would have decided and plots force against measured depth. Fit
on half the trials and check on the other half.

## 8. What the sim tests showed

| Test | Result |
|---|---|
| Sphere phantom, guard on | Contact placed at 10.6 mm (true surface 10.0). Bone decided at 31.3 mm, 0.3 mm into it. Retract and out confirmed. Hit mapped at z 0.2287 (bone top 0.229). |
| Same map, `--search` | Straight down stopped by the map at 26.3 mm, never touching the bone. 5° and 10° directions blocked. 15° / rot 0 planned; the re-aim kept the tip 10.5 mm up. Contact within 0.01 mm of E; full depth reached. |
| Slab phantom (wide shelf), `--search` | 6 hits, every one mapped and retracted out. One re-aim that would have dipped to 3.2 mm was raised to start 25 mm up. It **gave up at max_attempts**: see the limitation below. |
| GUI end to end | Point 3, then guarded search from the Plunger tab: bone, re-aim 15°, depth. Live plot, status and map views update. |
| Regressions | Plain point 4 (`--guard off`) and line mode: same output and timing as before |
| **Real load cell**, firmware v1, untouched | Detected automatically; 89.3 Hz; 0 dropped samples; host arrival gap at most 12.3 ms; noise sd **~700–1,500 counts** |
| **Real load cell**, hand presses | Pushing the needle toward the syringe reads **positive** (sign +1 correct). Firm presses reached **3.7–5.4 M** counts. Harder ones **saturated the HX711** (raw 8,388,607), about 4× rating if this is a 1 mV/V 1 kg cell. Afterwards the zero settled about **+40–75 k** higher. → Saturation is now a safety abort, and the "needle out" check has its own `f_out` (200 k). |

Limitations and notes:

- **Hit-point map vs. a wide bone.** Each hit only blocks a 3 mm sphere, and
  the search tries smallest tilts first. A broad bone shelf therefore costs
  one hit per direction on the 15° ring before it reaches 20° (sim slab test).
  Options, if real bone behaves like that:
  - raise `hit_radius_mm` or `line_margin_mm`
  - lower `max_attempts` for safety
  - switch the map to a surface / voxel model (the "voxel grid" option that
    was declined earlier)
- **Bone thresholds are provisional.** `f_bone_min`, `k_bone` and `f_soft` are
  tuned to the fake phantom (stiffness 40k counts/mm tissue, 3 M counts/mm
  bone). Set them from real data (Phase 3–4) before trusting `--guard on` on
  the arm; use `--guard log` until then.
- **CoppeliaSim autosave stalls sim for about 4 s every 2 minutes.** It saves
  the 94 MB scene; set `autoSaveDelay = 0` in `~/.CoppeliaSim/usrset.txt` to
  stop it. The governed executor no longer lets path time jump after a stall:
  the step is capped at 3 ticks.
- **The 1 kg cell is easy to overload by hand.** Calibrate it (§2) to find out
  what 3–5 M counts really is, and consider a 3–5 kg cell. Re-check the zero
  with `noise` after any hard load.
- **Sim has no arm compliance.** The release step and the lag / stall checks
  only really get exercised on the real arm.

## 9. Arm model with the needle tool (`openarm_dual_needle.urdf`)

`config.yaml paths.urdf` now points to `sim_files/udrf/openarm_dual_needle.urdf`.
The original `openarm_dual.urdf` is unchanged. Only the **right arm** changes;
the left arm keeps its gripper:

| Removed | Added |
|---|---|
| Gripper hand, both fingers and their joints, the gripper TCP | `openarm_right_needle_tool` on the same mount: bracket, 1 kg bar load cell, syringe clamp, syringe and needle. Mesh and inertia come from the CAD. |
| | `openarm_right_needle_hub` (syringe nozzle end) and `openarm_right_needle_tip`. Both have the tool's axes, with −X pointing down the needle. |

**Why it matters:**
- **The tool weighs ~108 g (CAD estimate) instead of the gripper's 422 g.**
  The old model over-compensated gravity by up to ~1 Nm per joint, about 20%
  of the holding torque in a bent pose.
- **The needle is about 93 mm to the side of the wrist axis**, parallel to
  it. So the same point-3 coordinates now give a different arm
  configuration than before.

**The TCP is chosen automatically.** The kinematics and dynamics use
`openarm_{side}_needle_tip` if the URDF has it, otherwise the gripper TCP. To
go back, point `paths.urdf` at `openarm_dual.urdf`.

**How the URDF is made:**
1. `src/tools/extract_needle_tool.py` (needs gmsh; the instructions are in the
   file) takes the tool out of `new_setup&files/arm_with loadcell.step`. The
   STEP is the whole robot at its zero pose; registering it against the URDF
   meshes gives +30.0 / −1.0 / +1.2 mm. The script writes
   `sim_files/udrf/needle_tool/` (an STL in the tool frame, and per-part
   volume, centre of mass and inertia in a JSON file).
2. `src/tools/make_needle_urdf.py` builds the URDF from those files, with no
   CAD software needed. **Rerun it with your measurements:**

```bash
python3 tools/make_needle_urdf.py --needle-mm <tip below the syringe nozzle> --tool-mass-g <weighed> [--fluid-ml <ml>]
```

**Checks done:**
- At the zero pose the hub sits at (29.7, −60.0, 107.1) mm, against
  (29.7, −60.1, 107.1) mm from the CAD.
- The tool axis points straight down.
- Point 3 (0.2, −0.095, 0.26) with its 60 mm descent still plans.
- A guarded search in sim (bone, retract, re-aim 15°, depth) works.

**Still to do:**
- **The CoppeliaSim scene** (`dual_openarm_org.ttt`) still shows the gripper.
  The sim arm moves correctly (joint angles), but the needle isn't drawn.
- **Needle length:** the URDF was generated with `--needle-mm 0` (tip at the
  syringe nozzle) until it's measured.

## 10. Before the real arm (from the plan's Phase 0)

1. **Needle tip:** done in the URDF (§9). Pass the measured `--needle-mm`.
   `tool.tip_offset_m` is now only for small corrections on top.
2. **Gripper motor gone:** `motors.gripper.enabled: {left: true, right: false}`
   is now set. arm_can never touches motor 0x08 on the right arm. This is
   untested on the arm, so check `python3 comm/arm_can.py status` first.
3. **Gravity model:** done (§9). Weigh the tool and pass `--tool-mass-g`.
4. **Bracket:** rebuild it. `f_abort` assumes the old one.
5. **Order on the arm:**
   1. `comm/loadcell.py noise`
   2. `--guard log` in free air
   3. press the syringe by hand to trip an abort
   4. unplug the USB mid-descent to trip the stale abort
   5. foam
   6. silicone
   7. silicone over PLA
   8. tune with the replay tool
   9. `--guard on`, then `--search`

**`user_config.yaml` (fixed 2026-10-12).** The GUI used to copy the *whole*
configuration into it when closing. That froze values like `paths.urdf`, the
gripper switch and the guard thresholds over later `config.yaml` edits. It
now writes only what the GUI edits (gains, gravity switches, joint limits,
sim host/port, window state). At start-up a note lists anything else in that
file that overrides `config.yaml`.
