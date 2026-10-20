# main.py — Unified Control Entry Point

`src/main.py` ties [arm_can.py](02_arm_can_script.md) (real hardware),
[sim_bridge.py](04_sim_bridge_script.md) (CoppeliaSim) and
[kinematics.py](03_kinematics_script.md) (FK/IK) behind one CLI and one
interactive shell. Everything the other scripts do per-backend, this does in
one place, with a choice of which backend(s) actually receive the motion.

Run everything from `OpenArm_v3/src/`. For any command touching real hardware,
bring the CAN interfaces up first — see
[01_hardware_setup.md](01_hardware_setup.md); `main.py` does not do it for you.

## The modes

| Mode | What it does | Backends |
|------|--------------|----------|
| `joint` | Direct joint-angle control, in degrees | sim / real / both |
| `home` | Shortcut for `joint --deg 0 0 0 0 0 0 0` | sim / real / both |
| `kinematic` | Give an EE pose, solve IK, drive the result | sim / real / both |
| `line` | Drive the EE along a straight line in 3 speed-staged sections | sim / real / both |
| `plunger` | Plunging experiment, one point per command (1–4, plus 5 = retract out); logs joint angles + torques; point 4 can run with the needle load cell in the loop (`--guard`, `--search`, see [10_loadcell_force_feedback.md](10_loadcell_force_feedback.md)) | sim / real / both |
| `imitate` | Real arm → sim shadow, backdriven by hand | real reads → sim writes |
| `record` | Move the real arm by hand and save the motion to a file | real / both (sim follows live) |
| `replay` | Play a saved recording back | sim / real / both |

## Two ways to run it

### One-shot

Connects, runs one command, disconnects — same shape as the other scripts here.

```bash
python3 main.py joint --deg 0 -30 0 60 0 0 0
python3 main.py home --arm both --mode both
python3 main.py line --start 0.302 -0.346 0.28 --angles 0 0 --depth 0.04
```

### Interactive shell

Run with **no arguments**. Connections to whichever arm(s)/backend(s) you use
are opened once and stay open across commands, so consecutive moves don't
re-connect or re-enable motors each time:

```
$ python3 main.py
OpenArm interactive shell -- connections stay open between commands.
Commands: joint / kinematic / imitate / line / home / record / replay (same flags as the one-shot CLI).
Type 'help' for full usage, 'exit'/'quit' (or Ctrl+D) to disconnect and leave.

openarm> joint --arm right --deg 10 -20 0 45 0 0 0 --mode both
openarm> line --start 0.30 -0.35 0.30 --angles 15 0 --depth 0.06 --dry-run
openarm> home --arm both --mode both
openarm> exit
```

Every subcommand takes the exact same flags interactively as it does one-shot —
just drop the `python3 main.py` prefix.

| In the shell | Effect |
|---|---|
| ↑ / ↓ | previous / next command (history persists across sessions in `~/.openarm_shell_history`) |
| ← / → | move the cursor within the line |
| `help` | print full usage |
| `Ctrl+C` | cancel the current line, or interrupt the running command — **does not exit the shell** |
| `Ctrl+D`, `exit`, `quit` | disable real motors, disconnect, leave |

A bad command (typo, unreachable target, sim not running) prints an error and
returns to the prompt; it never kills the session.

**The shell does not hot-reload.** If you edit any `.py` file while a shell is
open, that session keeps running the old code — `exit` and relaunch to pick
changes up.

## Flags available in every mode

| Flag | Default | Meaning |
|------|---------|---------|
| `--arm {left,right,both}` | `right` | Which arm. `both` runs left then right in sequence — except in `imitate`, `record` and `replay`, which handle both sides within the same tick loop, so they move in lockstep. `replay` defaults to every arm in the recording instead of `right`. |
| `--sim-host HOST` | `localhost` (`config.yaml` `sim.host`) | CoppeliaSim ZMQ host. |
| `--sim-port PORT` | `23000` (`config.yaml` `sim.port`) | CoppeliaSim ZMQ port. |
| `-h`, `--help` | — | Usage for that mode. |

`--arm left` maps to `can1`, `--arm right` to `can0` (`config.yaml` `can.*`).

`--sim-host`/`--sim-port` only take effect the **first** time a given arm
connects to sim within a shell session; afterwards the cached connection is
reused.

---

## `joint` — direct joint-angle control

```bash
python3 main.py joint --deg 0 -30 0 60 0 0 0                        # sim only (default), right arm
python3 main.py joint --arm left --mode real --deg 0 -30 0 60 0 0 0
python3 main.py joint --arm both --mode both --deg 0 0 0 0 0 0 0 --duration 3
python3 main.py joint --mode real --gripper open
python3 main.py joint --mode both --deg 0 20 0 45 0 0 0 --kp 30 --kd 1.0
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--mode {sim,real,both}` | `sim` | Which backend(s) receive the motion — see [How `--mode both` works](#how---mode-both-works). Defaults to `sim` so nothing touches real hardware unless you ask. |
| `--deg D1 … D7` | — | Seven target angles in degrees, `joint1`…`joint7`. |
| `--gripper {open,close}` | — | Drive the gripper to that side's configured open/close position. Applied after the joint move. |
| `--duration SECONDS` | `2.0` | Ramp time. |
| `--kp KP` | `config.yaml` `control.kp` | Position gain, Nm/rad. **Real arm only** — ignored for sim. |
| `--kd KD` | `config.yaml` `control.kd` | Velocity gain, Nm·s/rad. **Real arm only** — ignored for sim. |

You must pass `--deg`, `--gripper`, or both — with neither it errors rather
than doing nothing silently.

## `home` — go to the zero pose

Exactly `joint --deg 0 0 0 0 0 0 0`; it takes the same flags minus `--deg` and
`--gripper`.

```bash
python3 main.py home
python3 main.py home --arm both --mode both --duration 3
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--mode {sim,real,both}` | `sim` | As above. |
| `--duration SECONDS` | `2.0` | Ramp time. |
| `--kp KP` / `--kd KD` | from `config.yaml` | Real arm only. |

---

## `kinematic` — EE pose → IK → drive

Solves for joint angles that put the end effector at a target pose, then drives
them. Once IK has produced the angles, the move itself is exactly `joint` mode —
the same MIT position ramp, the same strict target validation, the same
closed-loop mirroring under `--mode both`.

```bash
python3 main.py kinematic --pose 0.3 0.1 0.2                       # sim (default), position-only
python3 main.py kinematic --pose 0.3 0.1 0.2 0 0 0                 # full pose, rpy in degrees
python3 main.py kinematic --pose 0.302 -0.346 0.241 --mode sim     # preview in sim first...
python3 main.py kinematic --pose 0.302 -0.346 0.241 --mode both    # ...then drive the real arm
python3 main.py kinematic --arm left --pose 0.3 0.1 0.2 --mode real --duration 4 --kp 30
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--pose V …` | **required** | Target EE pose in the shared `robot_base` frame. **3 values** `x y z` (metres) solve position only, leaving orientation free. **6 values** `x y z roll pitch yaw` (metres + degrees) solve the full pose. |
| `--mode {sim,real,both}` | `sim` | Which backend(s) receive the move — see [How `--mode both` works](#how---mode-both-works). |
| `--limits {real,sim}` | `real` | Which `config.yaml` limit set constrains the solve: `real` = `R_left_arm`/`R_right_arm.limits_deg`, `sim` = `Sim_limits_deg`. **Must be `real` with `--mode real`/`both`** — a solve under the wider sim limits can return angles the real arm can't reach, so that combination is refused before anything connects. |
| `--duration SECONDS` | `2.0` | Ramp time for the resulting move. |
| `--kp KP` | `config.yaml` `control.kp` | Position gain, Nm/rad. **Real arm only.** |
| `--kd KD` | `config.yaml` `control.kd` | Velocity gain, Nm·s/rad. **Real arm only.** |

**The solver is seeded with the arm's current position.** A 7-DOF arm has
infinitely many solutions for one pose; seeding means it settles on the one
nearest where the arm already is — the smallest move — rather than whatever a
zero or random seed lands on. The same target therefore yields different
(equally valid) solutions depending on where you start from. The output reports
the seed, the solution, and `max joint change from current` so you can see how
big a move it implies.

**With `--mode real` or `both`, the seed is the real arm's measured position**,
read over CAN, not the sim's. Seeding from sim would find the solution nearest a
pose the real arm may not actually be in, so the real move could be far larger
than the reported joint change.

**Preview unfamiliar poses in sim first.** Unlike `joint`, you didn't choose
these angles yourself. Run the pose with `--mode sim`, check the solution and
`max joint change from current`, then repeat with `--mode both`. A sim preview
seeds from the sim's pose, so it only predicts the real move if sim and real
start in the same configuration — which is the case right after any
`--mode both` command, since that leaves sim mirroring the real arm.

Position-only (3 values) converges over a noticeably larger reachable volume
than a full 6-value pose — the arm can often *touch* a point but not with an
arbitrary orientation.

If the target is unreachable within the active limits, it reports
`success = False` with the residual and moves nothing.

---

## `line` — EE along a straight line, in 3 speed-staged sections

You give only the line's start point, two angles for its direction, and how far
to travel. The path is split into three sections, each with its own speed.

```bash
python3 main.py line --start 0.302 -0.346 0.28 --angles 0 0 --depth 0.04              # sim (default)
python3 main.py line --start 0.28 -0.33 0.30 --angles 20 0 --depth 0.05 --dry-run
python3 main.py line --start 0.30 -0.35 0.30 --angles 15 90 --depth 0.06 \
                     --speeds 0.05 0.01 0.004 --split 0.7
python3 main.py line --start 0.302 -0.346 0.28 --angles 0 0 --depth 0.04 --mode both --dry-run
python3 main.py line --start 0.302 -0.346 0.28 --angles 0 0 --depth 0.04 --mode both
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--start X Y Z` | **required** | Where the line begins, metres, `robot_base` frame. |
| `--angles TILT ROT` | **required** | Direction, degrees — see below. |
| `--depth D` | **required** | Distance to travel along the line, metres. |
| `--mode {sim,real,both}` | `sim` | Which backend(s) run the path — see [Running on the real arm](#running-on-the-real-arm). |
| `--speeds S1 S2 S3` | `0.02 0.005 0.002` | Metres/second for each section. Warns (does not fail) if they aren't strictly decreasing. |
| `--split F` | `0.6` | Fraction of depth where section 2 ends and section 3 begins. Must be strictly between 0 and 1. |
| `--orient {path,free}` | `path` | `path`: the tool axis is held parallel to the line from `--start` to full depth, pointing along the direction of travel (roll about it free), and the approach is a joint-space move — see [Tool orientation](#tool-orientation). `free`: position-only IK throughout, straight-line approach. |
| `--limits {real,sim}` | `real` | Which limit set constrains the IK solves. **Must be `real` with `--mode real`/`both`** — that combination is refused before anything connects. |
| `--rate HZ` | `config.yaml` `control.ctrl_hz` (100) with the real arm, `50` sim only | Execution tick rate. |
| `--plan-step M` | `0.002` | Cartesian spacing between IK-solved knots, metres. |
| `--dry-run` | off | Plan and print the path, move nothing. With `--mode real`/`both` it still connects to the arm to read its pose, so the plan starts from where the real arm actually is. |
| `--kp KP` | `config.yaml` `control.kp` | Position gain, Nm/rad. **Real arm only.** |
| `--kd KD` | `config.yaml` `control.kd` | Velocity gain, Nm·s/rad. **Real arm only.** |

### The three sections

| Section | From | To | Speed |
|---|---|---|---|
| 1 approach | wherever the EE is now | `--start` | `S1` (with `--orient path`: a joint-space move, also capped at 15 °/s on its furthest-moving joint) |
| 2 to split | `--start` | `--split` of the way in | `S2` |
| 3 to depth | that point | full `--depth` | `S3` |

Defaults decrease, so motion slows the deeper it goes.

### Direction convention

```
dir = ( sin(tilt)·cos(rot),  sin(tilt)·sin(rot),  −cos(tilt) )
```

`TILT` is degrees away from straight-down; `ROT` sets which way that tilt
points, measured about the vertical axis.

| `--angles` | Direction | Meaning |
|---|---|---|
| `0 0` | `( 0.000, 0.000, −1.000)` | straight down |
| `15 0` | `( 0.259, 0.000, −0.966)` | 15° toward +X |
| `15 90` | `( 0.000, 0.259, −0.966)` | 15° toward +Y |
| `30 180` | `(−0.500, 0.000, −0.866)` | 30° toward −X |
| `20 45` | `( 0.242, 0.242, −0.940)` | 20°, between +X and +Y |
| `90 0` | `( 1.000, 0.000, 0.000)` | horizontal, along +X |

### What it prints

Before anything moves it prints the geometry and a plan table (`--dry-run` stops
there); after the run, a result table. This is a real `--mode sim` run of
`line --arm right --start 0.302 -0.346 0.27 --angles 0 0 --depth 0.02 --speeds 0.1 0.02 0.008`:

```
[line] tilt=0.0 deg  rot=0.0 deg  -> direction [ 0.  0. -1.]
       start        [ 0.302 -0.346  0.27 ]
        60% of depth [ 0.302 -0.346  0.258]
       full depth   [ 0.302 -0.346  0.25 ]
[sim] connecting to CoppeliaSim at localhost:23000 for right arm...
[right] current EE [ 0.3024 -0.3464  0.241 ] (from sim position)
    planning 1 approach: 15 knots over 29.0mm...
    planning 2 to 60%: 7 knots over 12.0mm...
    planning 3 to depth: 5 knots over 8.0mm...
[right] plan:
      section            length       speed     time  knots
      1 approach         29.0mm    100.0mm/s    0.29s     15
      2 to 60%           12.0mm     20.0mm/s    0.60s      7
      3 to depth          8.0mm      8.0mm/s    1.00s      5
      total                                    1.89s     27
[sim] right 1 approach: 29.0mm at 100.0mm/s (0.29s)...
[sim] right 2 to 60%: 12.0mm at 20.0mm/s (0.60s)...
[sim] right 3 to depth: 8.0mm at 8.0mm/s (1.00s)...
[right] result:
      checkpoint              tracking error
      start 1 approach                0.00mm
      start 2 to 60%                  0.01mm
      start 3 to depth                0.00mm
      end                             0.00mm   final EE [ 0.302 -0.346  0.25 ]
      longest gap between commands 96.7ms (tick period 20.0ms)
Done.
```

**Tracking error** is the distance between where the EE measurably is and where
the schedule had it at that same moment, checked at the start of each section
and at the end. Being on schedule at both ends of a section is what shows the
section ran at its commanded speed. In sim it stays near zero, since sim is set
directly to each command; on the real arm it includes both the lag behind the
path and the gravity sag described in
[Between commands](#between-commands-the-arm-stays-held).

**Longest gap between commands** is the longest the command stream went without
a new setpoint. On the real arm the motors keep holding the last one through a
gap, so a long gap shows up as the path pausing and then catching up, not as the
arm going limp. In sim-only mode a large value just means a ZMQ stall made the
sim hitch, as in the run above.

With `--mode real` or `both`, a few more lines appear around the run:

```
[right] current EE [ 0.3024 -0.3464  0.241 ] (from real arm held setpoint)
    [right] arm drift since planning: 0.00 deg
[sim] connecting mirror to CoppeliaSim at localhost:23000 for right arm...
[real+sim] right: streaming at 100 Hz, sim mirrored from its own thread at 30 Hz; joint torques logged from --start on
      ...
[line] right torque log: 141 samples from --start (1.40s) -> .../recordings/line_torque/20260922_215405_right.csv
      ...
      sim mirror: 26 updates, 0 skipped
```

### Running on the real arm

`--mode real` streams the planned path to the real arm through the same MIT
position commands every other real move uses; `--mode both` also mirrors the
arm's measured position into sim. Because the path takes seconds and the arm is
physical, a few things work differently from a sim run:

- **Planned from where the real arm starts** — the setpoint its motors are
  holding from the previous move, or its measured pose if nothing is held (see
  [Between commands](#between-commands-the-arm-stays-held)). Planning from the
  sim's pose would aim the first real command wherever sim happened to be.
- **Re-checked just before moving.** Planning takes seconds, so the starting
  point is read again right before streaming. If any joint moved more than 2°
  meanwhile — bumped while unheld, or the motors let go — the run is refused
  with nothing moved; smaller drift is absorbed into the first segment rather
  than arriving as a step.
- **Streams at the control rate** — 100 Hz from `config.yaml` — by default,
  like every other real move.
- **Scheduled by wall-clock time.** Each tick sends where the path should be at
  that moment, so a late tick produces a small step to the right place, never a
  burst of catch-up waypoints that would briefly move faster than commanded.
- **Sim mirror on its own thread** (`--mode both`). Sim writes go over ZMQ and
  occasionally stall; inline, each stall would freeze the arm's setpoint for as
  long as it lasted and then jump it forward. In testing, with an 80 ms stall
  injected on every 5th sim write, the real command stream's longest gap stayed
  at ~10 ms.
- **Waypoints are clamped into range** before sending. An IK answer sitting
  exactly on a joint limit can come back a hair outside it, which
  `move_joints()` would otherwise reject partway through the path.

**The arm stays held at depth.** When the path finishes, the motors keep
servoing to its final setpoint, with the same gravity sag as between any two
commands — which is also why the `end` tracking error on the real arm isn't zero.

### Torque log (real arm)

With `--mode real` or `both`, every run records the arm's measured joint
torques from the moment the schedule reaches `--start` (the end of section 1)
until the path ends at full depth. The approach isn't recorded. There's one
sample per tick, taken right after that tick's command. Each arm gets its own
file, named by the time it reached `--start`:

```
recordings/line_torque/<YYYYmmdd>_<HHMMSS>_<side>.csv      e.g. 20260922_143005_right.csv
```

```
# openarm-line-torque v1
# created: 2026-09-22T14:30:05
# arm: right
# line: --start 0.302 -0.346 0.28 --angles 0 0 --depth 0.04 --speeds 0.02 0.005 0.002 --split 0.6 --orient path
# rate_hz: 100
# gains: kp 40 40 40 40 40 40 40  kd 0.5 0.5 0.5 0.5 0.5 0.5 0.5
t_s,section,cmd_depth_mm,joint1_deg,...,joint7_deg,joint1_nm,...,joint7_nm
0.0001,2,0.003,0.6689,12.2066,...
```

| Column | Meaning |
|---|---|
| `t_s` | Seconds since the schedule reached `--start`. |
| `section` | `2` (start → split) or `3` (split → full depth). |
| `cmd_depth_mm` | How far past `--start` the path had the EE at that moment (commanded, not measured). |
| `jointN_deg` | Measured joint angle. |
| `jointN_nm` | Measured joint torque, Nm, as the motor reports it. This is the total torque, so it includes the torque holding the arm up against gravity. |

Samples are kept in memory during the run and written when it ends. If Ctrl+C
or an error cuts the line short, whatever was recorded is still saved. A run
stopped before reaching `--start` writes nothing. Sim-only runs and `--dry-run`
don't record. The folder is kept separate from `recordings/`, so these files
don't show up in `replay --list`. To load one:

```python
np.genfromtxt(path, delimiter=",", names=True, skip_header=6)   # or pd.read_csv(path, comment="#")
```

### Planning happens before anything moves

The entire path is IK-solved up front — for an insertion you don't want to find
out knot 400 is unreachable once you're already in. If any knot fails, it
reports which one and **moves nothing**:

```
    [right] path not reachable -- 1 approach: knot 359/1217 at
            [ 0.6111 -0.1082  0.5004] unreachable (residual 0.000583)
    [right] nothing moved.
```

Each solve is seeded from the previous one, so the arm follows a single
continuous branch of the redundant solution space instead of flipping between
elbow configurations mid-line.

### Why `--plan-step` is separate from `--rate`

IK runs at a fixed Cartesian resolution, not at the execution tick rate. A
section covering 24 mm at 2 mm/s takes 12 s, which at 50 Hz would be 600 IK
solves just to fill the time — planning would take minutes. At `--plan-step
0.002` it's 12 solves, and execution interpolates in joint space between them,
which stays smooth because consecutive knots are millimetres apart.

Lower `--plan-step` for tighter path-following at the cost of planning time;
raise `--rate` for smoother playback at no planning cost.

### Tool orientation

With `--orient path` (the default), the gripper's approach axis is held
**parallel to the line**, pointing the way the EE travels, from `--start` to
full depth. That axis is the **−X axis of `openarm_{left,right}_hand_tcp_visual`**
(the URDF puts each TCP 0.08 m along −X of its hand link, so −X runs wrist →
between the fingertips); `kinematics.TOOL_AXIS` holds it. `--angles 0 0` therefore
means the gripper points straight down and pushes straight down.

Only the axis direction is constrained, not roll about it. That is 5 constraints
on a 7-DOF arm, leaving 2 redundant DOF for the solver to fit the narrow real
limits (joint 6 is only ±45°). `ArmKinematics.ikine_axis()` does the solve. Its
orientation residual is `(tool axis − line direction)`, which is zero only when
they point the same way, never anti-parallel.

Planning order differs from `--orient free`:

1. **Aligned start poses.** IK for the TCP at `--start` with the tool on the
   line, from 15 seeds (the current configuration, zero, mid-range and 12
   random). The distinct solutions are sorted nearest-first to where the arm is now.
2. **The line, per start pose.** Sections 2–3 are planned knot by knot from
   each candidate in turn, seeded from the previous knot. Two safeguards keep
   one continuous arm configuration:
   * each solve is first pulled slightly toward its seed, then polished
     without that pull, so it lands on the *nearest* solution exactly;
   * a knot more than 10° (any joint) from the previous one rejects that
     candidate, since interpolating across such a hop would leave the line.

   The first candidate whose whole line plans is used.
3. **Approach.** A single joint-space move from the current configuration to
   that start pose. Its time is the longer of `approach distance / S1` and
   `furthest joint travel / 15 °/s`. The EE path of this move is **not** a
   straight line. It arrives at `--start` already aligned.

Why not a Cartesian approach that turns the tool on the way? With the real
limits, most branches hit a joint limit partway through the turn and then need
a 50–130° reconfiguration. Planning the line first avoids that.

The printout lists each candidate tried, and the result table gains an
**axis error** column (measured vs scheduled tool axis):

```
[right] tool axis now [ 0.7995  0.0096 -0.6006], 53.1 deg off the line
    planning 2..3 with the tool on the line: 31 knots over 60.0mm; approach = joint-space move to its start (furthest joint at most 15 deg/s)
    11 start configuration(s) with the tool on the line; trying nearest first
    start configuration 1/11: 2 to 60%: knot 16/18 at [ 0.3   -0.35   0.268] needs a 106.7deg joint jump from the previous knot (max 10) -- the tool can't stay on the line continuously
    ...
    start configuration 5/11: line planned; approach turns its furthest joint 78.8deg
...
[right] result:
      checkpoint              tracking error   axis error
      start 1 approach                0.00mm       0.00deg
      start 2 to 60%                  0.00mm       0.00deg
      start 3 to depth                0.00mm       0.00deg
      end                             0.00mm       0.00deg   final EE [ 0.3  -0.35  0.24]
```

Holding orientation shrinks the reachable set a lot. A line whose direction
points back toward the body (e.g. `--angles 30 180` on the right arm) has no
aligned pose at `--start` at all, and some lines leave the aligned workspace
partway. Either way nothing moves, and the message suggests `--orient free`.

---

## `plunger` — 4-point plunging experiment

The arm starts at all zeros. Each `--point N` is its own command, run from
wherever the previous one left the arm. Points 1–2 bring it from zeros to a
safe pose. **Points 3 and 4 work from any pose**: point 4 goes to point 3
first when the arm isn't there (needs `--xyz`). Either way you can go
1 → 2 → 3 → 4 and stop
and look between points.

| Point | What moves | Inputs |
|---|---|---|
| 1 safe point | joint1, then joint2, … joint7, **one at a time**, each finishing before the next starts | `--deg D1..D7` |
| 2 joint3 → 0 | joint3 only, back to 0 | — |
| 3 plunge start | one joint-space move to `--xyz` with the **tool pointing straight down** (tool axis = TCP −X along robot_base −Z, roll free) | `--xyz X Y Z`, `--depth` |
| 4 plunge | straight down `--depth` from point 3 at `--speed`, tool held down; **joint angles and torques recorded** (real arm). **From any pose:** if the EE isn't at point 3 pointing down, it does point 3's move first | `--depth`, `--speed`, `--xyz` (point 3) |

```bash
openarm> plunger --point 1 --deg 0 30 0 60 0 0 0 --mode real
openarm> plunger --point 2 --mode real
openarm> plunger --point 3 --xyz 0.30 -0.30 0.25 --depth 0.03 --mode real --dry-run   # check only
openarm> plunger --point 3 --xyz 0.30 -0.30 0.25 --depth 0.03 --mode real
openarm> plunger --point 4 --xyz 0.30 -0.30 0.25 --depth 0.03 --speed 0.002 --mode real --name trial1
```

| Flag | Default | Meaning |
|---|---|---|
| `--point {1,2,3,4}` | **required** | Which point to go to. |
| `--deg D×7` | — | Point 1: the safe pose, degrees. Validated against the real limits before anything moves. |
| `--xyz X Y Z` | — | Point 3: plunge start, metres, robot_base. Point 4: refuse unless the EE is within 5 mm of it and the tool within 3° of straight down. |
| `--depth D` | — | Points 3–4: plunge depth below point 3, metres. Point 3 needs it so that the configuration it picks is one the whole descent plans from continuously (same planner as `line --orient path`). |
| `--speed S` | `0.002` | Point 4 descent speed, m/s (constant). |
| `--joint-speed` | `10` | Points 1–3: average joint speed of each move, deg/s (every move at least 1 s; minimum-jerk on the real arm). |
| `--name` | — | Point 4: appended to the log file name. |
| `--dry-run` | off | Points 3–4: plan and report, move nothing. |
| `--limits`, `--rate`, `--plan-step`, `--kp`, `--kd` | | As in `line`. |

Point 3's move is joint-space, so the EE does **not** travel in a straight line
to it -- check the path is clear (the GUI draws point 3 and 4 and the ghost of
the safe pose). **Points 3 and 4 plan from the arm's measured position** (its encoders), not from the setpoint the motors hold, which sits a few mm away under gravity. On the real arm, the descent therefore starts where the needle really is. The printout shows both: the *commanded* EE, which decides whether the arm is at point 3, and the *measured* EE, where the line starts, with its distance from point 3. The difference between the held setpoint and the measured position is carried into the descent as a starting correction (`motion/axis_hold.py`, `preload`), so the first command is what the motors already hold: no jump. With the axis hold on, the needle is then kept on that line.

### Load-cell guard, search and point 5

Point 4 takes `--guard log|on` (the needle load cell in the loop: contact,
bone stop, retract out) and `--search` (re-aim through the same entry point,
map-guided, and try again). Point 5 retracts the needle out along the last
insertion line. All of it is described in
[10_loadcell_force_feedback.md](10_loadcell_force_feedback.md). With the
default `--guard off`, point 4 is exactly as described here.

### Plunge log (real arm)

Point 4 records every control tick from the start of the descent to full
depth, saved even if Stop cuts it short:

```
recordings/plunger/<YYYYmmdd>_<HHMMSS>_<side>[_<name>].csv

# openarm-plunger v2
# created: 2026-10-06T14:30:05
# arm: right
# point3_m: 0.30000 -0.30000 0.25000
# depth_m: 0.03  speed_m_s: 0.002
# rate_hz: 100
# gains: kp 40 40 40 40 40 40 40  kd 0.5 0.5 0.5 0.5 0.5 0.5 0.5
# marks_s: 6.2135
t_s,mark,cmd_depth_mm,meas_depth_mm,joint1_deg,...,joint7_deg,joint1_nm,...,joint7_nm
```

`t_s` counts from the start of the descent, `cmd_depth_mm` is where the path
had the EE below point 3, `meas_depth_mm` where the measured joint angles put
it (FK). The timestamp in the name is when the descent started. Read it with
`np.genfromtxt(path, delimiter=",", names=True, skip_header=8)`. Sim has no
torques, so `--mode sim` records nothing.

**Marks.** While point 4 runs, the GUI's **Mark** button (or the **M** key)
notes the time since the descent started -- press it the moment the needle
touches the sample. `marks_s` lists every mark exactly, in seconds on the same
clock as `t_s` (`none` if there were none); the `mark` column holds the mark's
number (1, 2, ...) on the first sample at or after it and 0 everywhere else,
so `d["t_s"][d["mark"] > 0]` gives the rows to draw a line at. Marks are
GUI-only; the CLI shell has no way to press one.

## `imitate` — real arm → sim shadow

Puts the real arm in zero-torque (backdrivable) mode and continuously mirrors
its live joint positions into CoppeliaSim. Real is always the source, sim
always the sink. Move the physical arm by hand and watch the sim follow.

```bash
python3 main.py imitate                          # right arm, until Ctrl+C
python3 main.py imitate --arm left --duration 30
python3 main.py imitate --arm both --rate 50
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--rate HZ` | `20` | How often to read the real arm and write to sim. |
| `--duration SECONDS` | run until `Ctrl+C` | Stop after N seconds. |

Requires real hardware — the CAN interfaces must be up. With `--arm both`, each
tick reads and writes both sides, so left and right shadow in lockstep.

Sim trouble never ends imitate:

* A joint pushed by hand past its `config.yaml` limit is shown in sim **pinned
  at that limit** (sim refuses out-of-range poses; it used to skip the whole
  tick, freezing the sim arm until the joint came back).
* A failed sim write (CoppeliaSim frozen past the GUI's 5 s ZMQ timeout, or
  closed) prints `[sim] right: write failed (...)`, drops that connection and
  reconnects about once a second; `[sim] right: reconnected` follows when it
  answers again.
* A write that took over 1 s is reported as `CoppeliaSim stalled N s`.

CoppeliaSim's scene **auto-save** (`autoSaveDelay = 2` minutes in
`~/.CoppeliaSim/usrset.txt` by default) freezes its ZMQ server for ~4 s every
2 minutes while it writes the scene. The sim arm pauses for that long. Set
`autoSaveDelay = 0` (with CoppeliaSim closed) to remove it.

---

## `record` — move the arm by hand, save the motion

Makes the real arm's seven joints limp so you can guide it by hand, samples its
joint angles until you stop, then holds the arm where it ended up and saves the
take to a file that [`replay`](#replay--play-a-recording-back) plays back.

```bash
python3 main.py record --name demo                          # right arm, until Enter or Ctrl+C
python3 main.py record --name demo --mode both              # ...with sim following live
python3 main.py record --arm left --name reach1 --duration 20
python3 main.py record --arm both --name two_hands --countdown 5
python3 main.py record --name demo --overwrite
```

**Run it from the interactive shell.** A one-shot `python3 main.py record ...`
disables the motors as the process exits, so the arm goes limp again right after
the take is saved. In the shell it stays held, ready for `replay`.

| Flag | Default | Meaning |
|------|---------|---------|
| `--name NAME` | `rec_<date>_<time>` | What to call the take. A bare name is saved as `OpenArm_v3/recordings/NAME.csv`; a name with a directory in it (`./takes/demo`, `~/demo.csv`) is used as a path. |
| `--overwrite` | off | Replace an existing recording of the same name. Without it, an existing name is refused before the arm is touched. |
| `--mode {real,both}` | `real` | `both` also mirrors the arm into sim, from a separate thread, while you move it. There is no `sim`: the motion comes from the real arm. |
| `--rate HZ` | `50` | Samples per second. |
| `--duration SECONDS` | until Enter or `Ctrl+C` | Stop by itself after this long — the easy way to record alone (see below). |
| `--countdown SECONDS` | `3` | Time to get hold of the arm before its joints go limp. `0` releases at once. |
| `--damping KD` | `0` | Velocity gain while limp, Nm·s/rad, `0`–`5`. Resists fast motion without holding any position, so an arm that's let go drops more slowly — and feels heavier to guide. Not yet tried on the physical arm: start small. |
| `--no-trim` | off | Keep the still stretches at the start and end — see [Trimming](#trimming). |
| `--kp KP` / `--kd KD` | from `config.yaml` | Gains that hold the arm where it stopped. |

Plus `--arm` (default `right`) and `--sim-host`/`--sim-port`.

### What happens

1. **Checks.** Bad flags, or a `--name` that already exists, fail before
   anything connects.
2. **Countdown** — `support the arm now; joints go limp in 3... 2... 1...`.
   Take hold of the arm. `Ctrl+C` here cancels with nothing released.
3. **Release.** The seven arm joints get `kp = 0`. **With no gravity
   compensation, the arm falls if nobody is holding it.** The gripper keeps
   whatever it was last told: it is not released, recorded or replayed.
4. **Sampling** at `--rate`, each sample timestamped. In a terminal a status line
   counts up in place.
5. **Stop** on Enter, `--duration` or `Ctrl+C` — the take is saved either way. If
   reading the arm fails partway, the joints are still held, what was recorded is
   saved, and then the error is shown. Enter works when the shell runs in a
   terminal; with piped input, use `--duration` or `Ctrl+C`.
6. **Hold.** The joints are immediately commanded to stay where they are, so you
   can let go. Like every hold, the arm then sags a little under gravity — see
   [Between commands](#between-commands-the-arm-stays-held). A joint you pushed
   past its configured limit is pulled back to that limit here.
7. **Trim, save, summary.**

Output, from a test run against a simulated arm:

```
openarm> record --name demo
[record] right at 50 Hz -> .../OpenArm_v3/recordings/demo.csv
[record] stops on Enter / Ctrl+C -- the take is saved either way
[record] support the arm now; joints go limp in 3... 2... 1...
[real] right limp -- move by hand
[record]    7.8 s    391 samples  (Enter to stop)
[record] stopped by Enter at 8.02 s, 402 samples
[real] holding right where it stopped (kp 40, kd 0.5)
[record] trimmed 1.48 s of stillness from the start and 3.92 s from the end, 2.62 s kept
[right]           joint1  joint2  joint3  joint4  joint5  joint6  joint7
        min deg      0.0    10.0     0.0    30.0     0.0    -8.0     0.0
        max deg     20.0    10.0     0.0    70.0     0.0     0.0     0.0
[right] peak joint speed 16.7 deg/s (joint4, 0.12 s in)
[record] saved .../OpenArm_v3/recordings/demo.csv
[record] play it back with: replay --name demo
Done.
```

The summary gives, per arm, the range each joint covered and the peak joint
speed (averaged over 0.1 s, so one noisy reading doesn't count). It adds a
`NOTE` when the peak is over `replay`'s default `--max-speed` — naming the
`--speed` that would fit — or when a joint went past its configured limit (the
file keeps the real reading; `replay` clamps it).

### Trimming

You usually spend a few seconds settling before you move the arm, and a few
more getting back to the keyboard afterwards. Those still stretches are cut:
everything before any joint first moves more than 1° from where the take
started, and everything after every joint is back within 1° of where it ended,
keeping 0.2 s of stillness at each end so playback starts and ends at rest.
Pauses in the middle of a take are kept. The output says how much was cut;
`--no-trim` keeps everything.

### Recording alone

Enter stops the take, but if you let go of the arm to reach the keyboard it
falls — and **the fall is recorded as the end of the take**. Instead:

- keep one hand on the arm while you press Enter, or
- use `--duration` and keep holding until `holding ... where it stopped`
  appears; the still time you spend waiting is trimmed off.

If a fall does end up in a take, the summary's peak joint speed shows it — a
falling forearm is far past 90 deg/s within a fraction of a second — and
`replay` refuses to play it on the real arm. Re-record rather than slowing the
fall down with `--speed`.

### The recording file

A CSV with one row per sample: seconds since the first kept sample, then each
recorded arm's seven joint angles in degrees, as measured. The `#` lines at the
top are metadata; `created` is when the arm was released.

```
# openarm-recording v1
# created: 2026-09-15T11:38:26
# rate_hz: 50
# trimmed_s: 1.480 3.920
t_s,right_joint1_deg,right_joint2_deg,right_joint3_deg,right_joint4_deg,right_joint5_deg,right_joint6_deg,right_joint7_deg
0.0000,0.0000,10.0000,0.0000,30.0000,0.0000,0.0000,0.0000
0.0200,0.0000,10.0000,0.0000,30.0000,0.0000,0.0000,0.0000
```

A two-arm take has the seven `left_…` columns before the seven `right_…` ones.
From Python, `recording.load(path)` returns it with angles in radians; with numpy
directly, `np.genfromtxt(path, delimiter=",", names=True, skip_header=4)`. A file
edited or generated by hand plays as long as the header stays intact and `t_s`
keeps increasing.

A recording belongs to the arm it was made on. The left and right arms' joint
conventions differ, so there is no playing a right-arm take on the left arm.

## `replay` — play a recording back

```bash
python3 main.py replay --list                                # what's saved
python3 main.py replay --name demo                           # preview in sim (default)
python3 main.py replay --name demo --mode both               # real arm, sim mirroring it
python3 main.py replay --name demo --mode real --speed 0.5   # half speed
python3 main.py replay --name two_hands --arm left --mode both
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--name NAME` | — | The recording: a name from `--list`, or a path. Required unless `--list`. |
| `--list` | off | List the saved recordings — name, arms, length, samples, when made — and stop. |
| `--arm {left,right,both}` | every arm in the recording | Play only some of a recording's arms. Naming an arm the recording doesn't have is an error. |
| `--mode {sim,real,both}` | `sim` | `sim` plays on the sim arm; `real` on the real arm; `both` on the real arm, with sim mirroring its measured position from a separate thread. |
| `--speed F` | `1` | Playback speed factor: `0.5` takes twice as long, `2` half as long. |
| `--approach SECONDS` | `3` | Time to move from where the arm is to the recording's first pose. |
| `--max-speed DEG_S` | `90` | The real arm refuses to replay if any joint would move faster than this, approach included. Sim only warns. |
| `--rate HZ` | `config.yaml` `control.ctrl_hz` (100) with the real arm, `50` sim only | Tick rate. |
| `--kp KP` / `--kd KD` | from `config.yaml` | Real arm only. |

Plus `--sim-host`/`--sim-port`.

### What happens

1. **The recording is checked before anything connects.** Samples outside the
   arm's configured limits are clamped to the limit, and reported. The peak
   joint speed at `--speed` is worked out; over `--max-speed`, a real replay is
   refused with nothing moved, and the message names the fastest `--speed` that
   fits.
2. **Approach.** Each arm moves in a straight joint-space line from where it is
   to the recording's first pose, over `--approach` seconds. The real arm starts
   from the setpoint it is holding (see
   [Between commands](#between-commands-the-arm-stays-held)), so it doesn't
   drop; sim starts from its current pose. An approach faster than
   `--max-speed` is refused, naming the `--approach` that would fit. Like
   `joint`, nothing checks the way there for collisions — if the arm is far from
   where the recording starts, bring it close first.
3. **Playback.** The samples are streamed against wall-clock time at `--rate`,
   interpolated between samples, `--speed` times as fast. A late tick sends
   where the path should be by now rather than falling behind.
4. **Arms recorded together play together**, in the same ticks — not left then
   right as `--arm both` does elsewhere.
5. **End.** The real arm stays held on the last sample. In sim-only mode, a
   scene joint limit that stopped the sim arm short is reported, as in
   [Troubleshooting](#troubleshooting).

Output, replaying the take above with `--mode real` on the same simulated arm —
held 20° from the take's first pose, and sagging about 6° at joint4 under
gravity, close to what the real arm does:

```
openarm> replay --name demo --mode real
[replay] .../OpenArm_v3/recordings/demo.csv
         right, recorded 2026-09-15T11:38:26, 2.62 s, 132 samples
[right] peak joint speed 16.7 deg/s at --speed 1 (joint4, 0.12 s into the recording)
[right] approach from real arm held setpoint: up to 20.0 deg (joint4) in 3 s, 6.7 deg/s
[real] right: 3 s approach + 2.62 s playback at 100 Hz
[right] result:
      end pose error 6.12 deg (joint4) -- measured as playback ended, so it includes lag and gravity sag
      longest gap between commands 10.1ms (tick period 10.0ms)
Done.
```

**End pose error** is measured the moment the last sample is sent. On the real
arm it includes the lag behind the moving setpoint and the gravity sag, so it
isn't zero even when playback is perfect; in sim it should be about zero.

When the recording is too fast for the real arm:

```
[right] peak joint speed 200.0 deg/s at --speed 1 (joint4, 1.06 s into the recording)
[right] that's over --max-speed 90 deg/s; --speed 0.44 or lower stays under it
[replay] nothing moved. Slow it down with --speed, or raise --max-speed if that motion is intended.
```

### Record, preview, replay

```
openarm> record --name demo --mode both      # guide the arm; Enter to stop
openarm> replay --name demo                  # watch it in sim first
openarm> replay --name demo --mode both      # then on the real arm
```

A sim preview starts from wherever the sim arm is, so its approach matches the
real arm's only when sim was left mirroring the real arm — which
`record --mode both` (or any `--mode both` command) does.

---

## How `--mode both` works

`sim` and `real` each drive that one backend to the target and nothing else.

`both` is **closed-loop**, and not simply "send the same target twice":

1. The real arm is commanded along the ramp as usual.
2. After every tick, its *actual measured* position is read back over CAN.
3. That measured position — not the nominal waypoint — is written into sim.

So sim shows what the arm really did: lag, limit clamps, disturbances. It is
the same read-real/write-sim pattern as `imitate`, just driven by a commanded
ramp instead of a hand-backdriven arm. Driving both backends open-loop to the
same nominal target would instead show an idealized copy that hides exactly the
tracking error you want to see.

Sim writes go over ZMQ and occasionally stall for tens of milliseconds. `line`,
`record` and `replay` mirror from a separate thread, so a stall never delays a
CAN frame to or from the real arm. `joint`, `home` and `kinematic` still mirror
inline, so under `--mode both` a sim stall pauses their real command stream for
as long as it lasts.

Sim refuses poses outside the configured joint limits, and a real arm can
measure just past one — sagging while held at a limit, or pushed there by hand.
The threaded mirror shows that joint pinned at the limit; the inline one skips
the update (`[sim] skipped tick`), so sim briefly freezes.

Real motors are always disabled on the way out — end of the one-shot command,
`exit`/`Ctrl+D` from the shell, or `SIGTERM` either way.

## Between commands: the arm stays held

When a real move finishes, the motors keep servoing to its last commanded
position until the next command, so the arm stays up between commands in the
shell and at the end of a `line` path. (If a motor's CAN timeout register were
set, it would instead let go after that long without a command.)

It does hang slightly *below* that setpoint. Control is pure position (PD) with
no gravity compensation, so the only thing holding the arm up is position error:
it settles where `kp × error` balances gravity. The effect is large — measured on
the real arm ([posvsreal.txt](posvsreal.txt)), a 60° target settles at 53.3° with
`kp 40` and at 39° with `kp 10`.

**Every real move starts from the setpoint the motors are holding, not the
measured position.** Starting from the measured (sagged) position puts the new
setpoint exactly where the arm already hangs, which zeroes the error — and with
it the torque holding the arm up — so the arm drops by the sag before the move
gets going. That was the "jerk, then move" at the start of every command.

One exception: no held arm can sit further from its setpoint than the motors'
torque range allows (about `TMAX / kp` radians; TMAX is 54, 28 or 10 Nm per joint). A joint past that means the
motors have let go — a fault, or `imitate` releasing them — and starting from
the stale setpoint would snap the arm back, so the move starts from the measured
position instead and prints a warning naming the joint.

Two consequences:

- **Positions come out short of the target** by that same sag, on every move;
  `kinematic` and `line` show it as tracking error. Removing it needs gravity
  feed-forward — `dynamics.py` already computes the torque, see
  [05_pinocchio_integration.md](05_pinocchio_integration.md) — rather than a
  change here.
- **Changing `--kp` between commands still gives a small step** at the start of
  the move, since the same error then produces a different torque.

## Joint limits and safety

Two different behaviours, on purpose:

| Check | Applies to | Behaviour |
|---|---|---|
| Commanded target | what you typed | **Strict.** Out of range fails loudly — a typo'd angle never lands quietly on the nearest boundary. |
| Ramp waypoint | intermediate points | **Clamped** into range. |

The clamping exists because `ArmController.move_joints()` validates all seven
joints atomically: a joint whose *live* position is already outside its
configured range (hand-moved past a boundary, say) would otherwise block every
other joint from moving at all. Clamping only ever pulls such a joint back
toward its nearest boundary, and the strict target check means it can never be
commanded further out of range than it already is. When this happens you get:

```
[real] currently outside configured limits, pulling back toward range: joint1=3.82 deg (limit [-80.0, 0.0])
```

Both sim and real now validate against the same per-side real-hardware limits
(`R_left_arm`/`R_right_arm.limits_deg`), so a target valid in one is valid in
the other — see [04_sim_bridge_script.md](04_sim_bridge_script.md).

## Troubleshooting

**"Done." but the arm didn't move (sim).** CoppeliaSim silently clamps writes
to each joint's *own* interval configured in the scene, which is not
necessarily the one in `config.yaml`. This is now reported:

```
[sim] WARNING left joint4: asked 0.00 deg, got 40.00 deg (scene limit [40.0, 180.0])
```

Known case: the scene's `openarm_left_joint4` is `[40°, 180°]` where the right
arm's is `[0°, 140°]` and `config.yaml` says `[0, 140]` for both, so the left
arm cannot be homed until either the scene or the config is corrected.

**Nothing happens on real hardware / connection errors.** The CAN interfaces
come up `DOWN` after every reboot and after unplugging the adapter. Bring them
up per [01_hardware_setup.md](01_hardware_setup.md), then restart the
process — a node started while the interface was down is holding a dead handle.

**Sim commands hang or fail to connect.** CoppeliaSim must already be running
with the OpenArm scene loaded; `main.py` does not launch it. Check the scene's
joint names with `python3 sim_bridge.py list`.

**IK fails on a pose you believe is reachable.** Try position-only (3 values
instead of 6) — a full 6-DOF pose target is much more constrained. Poses are in
the shared `robot_base` frame, not each arm's local mount frame.

**Edits to the code don't take effect.** A running interactive shell keeps the
code it started with. `exit` and relaunch.

## Where each mode lives

`main.py` is only the CLI, the shell, and dispatch. Each mode is its own
module, with the shared pieces alongside:

| File | Contents |
|---|---|
| `main.py` | argparse definitions, interactive shell, dispatch |
| `mode_joint.py` | `joint` (also backs `home`) |
| `mode_kinematic.py` | `kinematic` |
| `mode_imitate.py` | `imitate` |
| `mode_line.py` | `line`, plus `direction_from_angles()` |
| `mode_plunger.py` | `plunger` (reuses `mode_line`'s planner and executor) |
| `mode_record.py` | `record` |
| `mode_replay.py` | `replay` |
| `recording.py` | the recording file: save/load, trimming, speed and limit checks |
| `session.py` | per-side real/sim connection cache |
| `motion.py` | real-arm ramps, target validation, waypoint clamping |
| `streaming.py` | per-tick send functions and the threaded sim mirror, shared by `line`, `record` and `replay` |

To add a mode: write `mode_<name>.py` exposing `cmd_<name>(session, args)`, add
a subparser in `build_parser()`, and a branch in `dispatch()`.
