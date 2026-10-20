# gui_main.py — Desktop GUI

`src/gui_main.py` is a PySide6 front end for everything
[main.py](07_main_script.md) does. Every mode (joint, home, kinematic, line, plunger,
imitate, record, replay) runs on sim, the real arm, or both, with a live 3D
view, a joint table, a scope and a console.

The GUI doesn't reimplement any motion. Each button builds the command line
the CLI would take, parses it with main.py's own parser and runs it through
`main.dispatch()`. So a button does exactly what typing that command at the
`openarm>` shell does, and the console prints the command before its output:

```
openarm> joint --arm right --mode sim --deg 0 0 0 10 0 0 0 --duration 1.5
[sim] moving right to [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0] deg over 1.5s...
Done.
```

## Starting it

```bash
cd OpenArm_v3_2_GUI
./run_gui.sh                         # uses the project venv (../.venv)
# or
cd src && ../../.venv/bin/python gui_main.py
```

For anything touching real hardware, bring the CAN interfaces up first (see
[01_hardware_setup.md](01_hardware_setup.md)). For sim, start CoppeliaSim and
load one of the scenes in `sim_files/`. You can also load one from the GUI:
**Connections → Load scene**.

## Layout

| Area | What's there |
|------|--------------|
| Toolbar | **Arm** (`--arm`: right / left / both) and **Drive** (`--mode`: sim / real / both) for every tab. It turns red and shows *REAL HARDWARE* whenever real is included. **STOP** (Esc) and **Disable all motors** are always on the right. |
| Connections (left) | Per-arm **Enable** / **Disable** with status (off, enabled, holding, limp). **Hold pose**, **Go limp** and **Print state** for the toolbar's arm(s). **Gravity feedforward** switches per joint (see below). CoppeliaSim status plus Start / Pause / Stop simulation, **Load scene** and **List scene joints**. |
| Tabs (centre) | One per mode, plus Scope and Settings (below). |
| Live view (right) | 3D stick figures of both arms, real solid and sim dashed. The active tab adds its overlay: joint targets, IK preview, line path or a recording's first pose. Below it is a table of every joint: measured angle, torque, sim angle, gripper and the end-effector position in mm. Red cells are outside the real-arm limits. |
| Console (bottom) | Everything the modes print, colour-coded. The input line takes the same commands as the `openarm>` shell (`help`, `clear`, `exit` too), with Up/Down history. |

All docks can be moved, floated or closed. Everything is remembered between
runs -- see [Saved settings](#saved-settings-user_configyaml).

## The tabs

| Tab | Command | Notes |
|-----|---------|-------|
| Joint | `joint`, `home` | A slider and spin box per joint, ranged to that arm's real limits. Each arm has its own column; with Arm = both they run left then right, as `--arm both` does. You can fill the targets from the live real or sim pose. **Live jog** (Drive = sim only) moves as soon as you release a slider. Named poses are saved in `OpenArm_v3_2_GUI/poses.json`; **Import…** also reads the older `{"deg_values": [...]}` format. |
| Cartesian | `kinematic` | Each arm has its own xyz (+ optional roll/pitch/yaw) target. **Preview IK** runs the same solve in the background and draws the result as a ghost. **Nudge** steps the target ±X/Y/Z and can move after each step. IK limits `sim` is disabled whenever the real arm is driven. |
| Line | `line` | Start, tilt/rot, depth, three speeds, split, tool orientation (`path` = gripper axis held parallel to the line, `free` = position only; `--orient`). The planned path is drawn in 3D, with an estimated time per section. **Plan only** is `--dry-run`. On the real arm, joint torques from the start point to full depth are saved to `recordings/line_torque/` ([torque log](07_main_script.md#torque-log-real-arm)). |
| Plunger | `plunger` | The four plunging points, one button each: **1** safe pose (its own joint editor per arm, joints moved one at a time 1→7), **2** joint3 to 0, **3** plunge start x y z with the tool straight down (**Check** = plan only), **4** straight down by Depth at Speed. The status line says which point the arm is at and which is next. Point 3, 4 and the safe-pose ghost are drawn in 3D. On the real arm, point 4 saves joint angles and torques to `recordings/plunger/` ([plunge log](07_main_script.md#plunge-log-real-arm)). While it runs, **Mark** (or the **M** key) writes the time since point 4 started into the CSV -- e.g. the moment the needle touches the sample. |
| Imitate | `imitate` | The real arm goes limp and sim mirrors it. Stop ends it; by default the GUI then holds the arm where it is (the CLI leaves it limp). |
| Record | `record` | The countdown and a REC timer are shown. **Stop & save** (or Esc) ends the take and saves it, the way Enter / Ctrl+C does in the CLI. Afterwards the Replay tab opens the new recording. |
| Replay | `replay` | Lists `recordings/`, plots the selected take and flags peak speed against `--max-speed` and any joints past the limits. The first pose is drawn as a ghost. This tab picks its own arms (default: every recorded arm). |
| Scope | — | A rolling plot of position, velocity or torque for one arm, from the live feed. |
| Settings | — | kp/kd per joint (passed as `--kp/--kd` only when they differ from the saved values), the **real-arm joint limits** (editable, see below), the CoppeliaSim host/port, the live-view rate, **Ask before every command that drives the real arm**, and the config files. |

## Gravity feedforward switches

The **Gravity feedforward (τ_g)** panel in Connections has a switch for each
joint. It's the live version of `control.joint_tauG_enable` in config.yaml
(one mask for both arms, applied to the real arm in every mode). Starting
from the saved value:

- **Flipping a switch applies at once**, even while a command runs. Arms
  enabled later start with it. Connected arms change what they send from
  their next command, and a limp arm (imitate, record, Go limp) re-sends its
  command on every read, so it picks the change up immediately.
- **A holding arm keeps its current hold** until its next command. **Hold
  pose** re-applies it right away. Turning a joint off on a holding or limp
  arm removes that joint's support, so it sags a little, or drops if limp.
- **If an arm was enabled with every joint off**, it has no dynamics model
  yet. The GUI loads one (Pinocchio) in the background the first time a
  switch goes on.
- The **L Nm / R Nm** columns show the gravity torque at each joint for the
  arm's current pose (the real arm's, else sim's). That's what the
  feedforward adds when the switch is on. Grey = switched off; amber = over
  the motor's torque range (54 / 28 / 10 Nm for joint1–2 / 3–4 / 5–7), which arm_can clamps to.
- **All off** switches everything off. **Revert** goes back to the last
  saved switches. **Save** writes user_config.yaml now; the switches are also
  saved there when the GUI closes.

## Joint limits

Settings → **Real-arm joint limits** has a min/max per joint for each arm and
the gripper's open/close angles, with the URDF (sim) limits beside them for
reference. They're what every real-arm command is validated and clamped
against, what sim enforces, the IK limits for `--limits real`, and the Joint
tab's slider ranges.

- **A change applies by itself** a moment after you make it, from the next
  command on. If a command is running it waits and applies when that command
  finishes. The line under the table says which (grey in use, green applied,
  amber waiting, red not applied), and the console lists what changed.
- **The Joint tab follows at once**, even while a command is running: its
  sliders, spin boxes and `[min, max]` labels take the new range, so you can
  set targets out to the new limits straight away. Nothing can start until
  the running command ends, and the new limits are in force by then.
- Min must be below max; an invalid entry isn't applied until it's fixed.
- The gripper open/close angles are the real gripper's. The sim gripper is a
  pair of linear fingers and always uses the URDF finger range.
- **Save** writes user_config.yaml now (it's also written on close).
  **Revert** goes back to the limits the GUI started with; **config.yaml
  values** goes back to the defaults.

## Saved settings (user_config.yaml)

`config/config.yaml` holds the defaults and is **never written** by the GUI.
When the GUI closes it writes `config/user_config.yaml`: the whole
configuration as it was left (gains, gravity switches, joint limits, sim
endpoint, and everything else from config.yaml) plus a `gui:` section with
the window layout, toolbar choices, safety setting and every tab's inputs
(Joint tab targets included). At start-up `config.py` lays it over
config.yaml, so the GUI -- and the CLI -- start exactly where you left off.

- Toggles that act on their own start off every time: live jog, "move after
  each nudge" and "overwrite an existing recording".
- **Save now** / **Open user_config.yaml** in Settings → Configuration files.
- **Reset to config.yaml…** deletes user_config.yaml when the GUI closes
  (nothing is saved that time), so the next start uses the defaults.
- Because user_config.yaml holds every key, a later hand edit to config.yaml
  only shows up after a reset (or edit user_config.yaml instead). A broken
  user_config.yaml is ignored with a warning.

## Stop, Disable, and quitting

- **STOP / Esc** interrupts the running command exactly as Ctrl+C does in the
  shell: the worker thread gets a `KeyboardInterrupt`. The real arm keeps
  holding its last command. Imitate and record catch it and finish cleanly,
  and record saves what it has.
- **Disable all motors** stops, then disables every real motor. The arms
  lose all torque and drop, so support them first. It always asks.
- **Quitting** disables the real motors, as leaving the CLI shell does. If an
  arm is enabled, the GUI asks first. Ctrl+C or SIGTERM in the terminal that
  started the GUI closes it the same way.

Enabling an arm doesn't make it hold a pose. Use **Hold pose**, or any move,
and it holds from then on; after that, commands start from the held setpoint
exactly as in the shell.

## How it's put together

```
src/gui_main.py        entry point (splash, then gui.app.run)
src/gui/backend.py     Worker thread, GuiSession, Telemetry, SimMonitor, console capture
src/gui/actions.py     GUI-only jobs: enable, hold, go limp, sim start/stop/load scene
src/gui/kin.py         FK for the live view; IK previews on a background thread
src/gui/plots.py       3D view, recording plot, scope (matplotlib)
src/gui/widgets.py     joint editor, gains editor, on/off switch, gravity panel, status dot
src/gui/tabs.py        one tab per mode + scope + settings
src/gui/app.py         main window, docks, console, theme
```

- **One worker thread owns all hardware.** openarm_can isn't thread-safe, so
  every CAN call (every command, and the ~20 Hz poll between commands) runs
  on it. `GuiSession` is main.py's `Session` with each real controller's
  `read_joint_states` / `move_joints` also reporting to the telemetry, so the
  live view follows the arm during commands too. A plain real ramp only
  sends commands, so during one the view shows the commanded pose.
- **Sim is read on its own connection** (`SimMonitor`), the same way
  `streaming.SimMirror` writes. The sim view stays live while a command runs.
  Its extra ZMQ traffic makes sim ramps roughly 10% slower than the CLI's,
  which already take longer than `--duration` because of ZMQ round trips.
- **Two behaviours differ from the CLI**, both in `backend.install_hooks()`:
  1. Record stops on Stop instead of on Enter.
  2. CoppeliaSim connections probe the port first and time out after 5 s
     without a reply, rather than hanging forever when the sim isn't running.

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| CoppeliaSim shows *offline: isn't reachable at localhost:23000* | CoppeliaSim isn't running, or its ZMQ Remote API is on another port. Set it in Settings → CoppeliaSim and press Apply. |
| *partial: left: joint(s) … not found in scene* | The loaded scene doesn't hold that arm. **List scene joints** shows what's there. |
| *CoppeliaSim didn't answer within 5 s* | The sim is stuck, often behind a modal dialog. The connection is dropped and the next command reconnects. **Reconnect** forces it. |
| Real arm Enable fails | The CAN interface is down or unplugged. See [01_hardware_setup.md](01_hardware_setup.md). |
| A button is greyed out | A command is running (see the toolbar). Wait for it, or press STOP. |
| config.yaml edits don't show up | config.yaml is read at start-up, and user_config.yaml overrides it. Edit user_config.yaml, or Settings → Reset to config.yaml and restart. |
