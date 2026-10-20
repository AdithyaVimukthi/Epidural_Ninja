# arm_can.py — CAN Read/Write Script

`src/arm_can.py` is the low-level read/write interface to one arm (7 joints +
gripper) over one SAVVYCANFD channel. It can be imported as a library
(`ArmController`) or run directly as a CLI. Config (CAN mapping, motor IDs,
joint limits, default gains) comes from `src/config.yaml` via `src/config.py`
— see [01_hardware_setup.md](01_hardware_setup.md) for the hardware side of
this.

Run everything from `OpenArm_v3/src/`, and bring the CAN interfaces up first
(see 01_hardware_setup.md) — `arm_can.py` does not do this for you.

## CLI Usage

```bash
python3 arm_can.py status --arm right                          # one-shot read
python3 arm_can.py params --arm right                          # motor PMAX/VMAX/TMAX registers vs config types (motors stay disabled)
python3 arm_can.py read --arm right --rate 20                   # live monitor, backdrivable
python3 arm_can.py zero --arm right --duration 2                # ramp all 7 joints to 0 rad
python3 arm_can.py move --arm right --deg 0 -30 0 60 0 0 0 --duration 2
python3 arm_can.py gripper --arm right --open
python3 arm_can.py gripper --arm right --close
```

`--arm {left,right}` and `--iface canN` must come **after** the subcommand
(`status`, `read`, `zero`, `move`, `gripper`), not before — see
[Why --arm goes after the subcommand](#why---arm-goes-after-the-subcommand)
below. `--arm right` currently maps to `can0`, `--arm left` to `can1` (per the
verified mapping in 01_hardware_setup.md); `--iface` overrides that mapping
directly if you need to point at a raw interface name.

| Command | What it does |
|---------|---------------|
| `status` | Enable, take one reading of all 8 joints, disable. Quick sanity check. |
| `read` | Put the arm in zero-torque (backdrivable) mode and print position/velocity/torque for all 8 joints in a loop. Move it by hand. Ctrl+C to stop. |
| `zero` | Ramp all 7 arm joints to 0 rad over `--duration` seconds (default 2s). |
| `move --deg D1..D7` | Ramp all 7 arm joints to the given target angles (degrees) over `--duration` seconds. |
| `gripper --open` / `--close` | Send the gripper to its configured open/close limit. |

`zero` and `move` also accept `--kp`/`--kd` to override the default gains from
`config.yaml` (20.0 Nm/rad, 0.5 Nm·s/rad) for that one call.

**Motors are always disabled before the process exits** — on normal
completion, on Ctrl+C (SIGINT), and on `kill`/SIGTERM (e.g. from a supervisor
or `timeout`). This means the arm goes limp rather than staying servoed to a
stale target if the script is interrupted.

## Library Usage

```python
from arm_can import ArmController

arm = ArmController("can0")        # or ArmController.for_side("right")
arm.enable()

states = arm.read_joint_states()   # list[JointState]: name, position_rad, velocity, torque
print(states[0].position_deg)

arm.move_joints([0.0] * 7)                       # one MIT position-hold command
arm.move_joints_over_time([0.0] * 7, 2.0)        # ramp to target over 2s
arm.set_gripper(-1.0)
arm.zero_torque()                                # backdrivable
arm.release_arm(kd=0.0)                          # backdrivable, arm joints only

arm.disable()
```

## What's Happening Inside

### Connection and the enable sequence

`ArmController.__init__` opens the CAN socket (`oa.OpenArm(iface, can_fd=True)`)
and registers the 7 arm motors + 1 gripper motor by their send/recv CAN IDs
from `config.yaml`. It sets `CallbackMode.IGNORE` immediately — incoming
frames are discarded until we're ready for them.

`enable()` then runs the exact sequence the hardware requires:

```python
self.arm.enable_all()                                  # 1. tell every motor to enter MIT mode
self.arm.recv_all()                                     # 2. drain the enable-acknowledgement frames
self.arm.set_callback_mode_all(oa.CallbackMode.STATE)   # 3. now start parsing frames as state updates
```

Each Damiao motor replies to `enable` with one CAN frame. If `STATE` mode were
active before that reply is drained, the library would parse it as a bogus
state update instead of an enable ack. `IGNORE → enable → recv_all → STATE` is
the only ordering that avoids that — this is called out in
01_hardware_setup.md and is not optional.

`disable()` runs `disable_all()` then `recv_all()` to drain the matching
disable acks.

### Reading

```python
self.arm.refresh_all()   # ask every motor for a fresh state frame
self.arm.recv_all()      # pull the replies off the socket into each Motor object
```

then `Motor.get_position()` / `get_velocity()` / `get_torque()` are read
straight out of the library's internal cache — no additional CAN traffic.
`read_joint_states()` wraps this for all 7 arm motors plus the gripper into a
list of `JointState(name, position_rad, velocity, torque)` in a fixed order
(`joint1..joint7, gripper`), with `position_deg` as a convenience property.

### Writing (MIT mode)

Every write goes through `oa.MITParam(kp, kd, q, dq, tau)` — a full-state MIT
command per motor: position target `q`, velocity target `dq`, feed-forward
torque `tau`, and the PD gains `kp`/`kd` that turn position/velocity error into
torque. `move_joints()` always sends `dq=0, tau=0` — it's pure PD position
hold; `zero_torque()` sends `kp=kd=tau=0` — no holding force, so the arm is
backdrivable. `release_arm(kd)` sends `kp=0` to the 7 arm joints only, leaving
the gripper on its last command; a non-zero `kd` damps hand motion without
holding any position (`main.py record` uses it).

**A command is one PD step, not a persistent setpoint.** The motor executes
one control cycle per CAN frame and then does nothing further until the next
frame arrives — there is no "hold this position until told otherwise" mode.
That's why `move_joints_over_time()` exists instead of sending the target
once:

```python
start = [current position of each of the 7 joints]
every tick (rate_hz, default 100 Hz from config.yaml, on fixed deadlines):
    t = time since start                              # wall clock, not tick count
    q, dq = min_jerk(start, target, duration, t)      # motion/trajectory.py
    self.move_joints(q, kp=kp, kd=kd, dq=dq)          # one MIT frame per joint
until t >= duration, then once more at the target with dq = 0
```

Three things this buys you:
1. Continuous resending, so the arm actually gets to the target and holds
   there, instead of taking one step and stopping.
2. A ramp instead of a jump — sending a far-away target directly means a large
   instantaneous position error, and with `kp=20` that's a large instantaneous
   torque demand (clamped to the motor's torque range, but still a hard snap).
   Interpolating in software keeps each step's position error small.
3. Smooth motion. The minimum-jerk path starts and ends at rest (no kick at
   either end); the setpoint comes from elapsed time, so a late tick lands
   further along the path instead of freezing it (the old tick-counted linear
   ramp moved in stop-go steps); and the path velocity goes out as the MIT
   `dq` target, so `kd` helps the joint track instead of braking it toward 0.
   Peak speed is 1.875x the average (distance / duration).

`move()` and `zero()` on the CLI both go through `move_joints_over_time()`.
Only `read`'s `zero_torque()` call and `gripper`'s `set_gripper()` send a
single one-shot frame — zero-torque has nothing to ramp, and the gripper is
one joint with a fixed open/close limit rather than an arbitrary target.

### Safety: joint limits (per side, not mirrored)

Every `q` passed to `move_joints()` / `move_joints_over_time()` / `set_gripper()`
is checked **before** anything is sent to the motors. Out-of-range values
raise `ValueError` instead of being clamped silently — a command with a
typo'd angle fails loudly rather than moving to the nearest limit.

Right and left are **not** mirror images on the real hardware, so the limits
used depend on which arm an `ArmController` is talking to:

- `config.yaml` has two limit sets: `Sim_limits_deg` (the old symmetric URDF
  numbers — simulation/kinematics use only) and `R_right_arm` /
  `R_left_arm` (as-measured real limits, including gripper open/close angles,
  whose sign convention also differs per side — e.g. right gripper is
  open=-65°/close=0°, left is open=0°/close=65°).
- `config.py` loads the real per-side limits into `REAL_ARM_LIMITS_RAD` and
  `REAL_GRIPPER_RAD`, each keyed by `"left"`/`"right"`.
- `ArmController` resolves its own `side` at construction — from an explicit
  `side=` argument, or inferred from `can_interface` via the configured
  can0/can1 mapping (`config.SIDE_BY_IFACE`) — and picks its limits from
  those dicts accordingly. A non-standard `--iface` that isn't can0/can1
  raises immediately, since there'd be no way to know which limit set
  applies.
- `open_gripper()` / `close_gripper()` send that side's exact configured
  open/close angle; `set_gripper(position_rad)` still accepts an arbitrary
  target but clamps it to `[min(open, close), max(open, close)]` for that
  side.

`arm_can.py` always drives the real per-side limits — `Sim_limits_deg` is
not read by this script at all.

### Why `--arm` goes after the subcommand

`--arm`/`--iface` are defined on a shared `common` parser and attached to each
subcommand via `parents=[common]`, not on the top-level parser. This looks
like it should also work the other way round (`arm_can.py --arm right
status`), but argparse has a gotcha here: if the *same* option were also
defined directly on the top-level parser, a subparser's own unset default
(`None`) silently overwrites whatever the top-level parser already parsed for
that same destination, once subparser parsing runs. Defining it in exactly one
place — the shared `common` parser, attached to each subcommand — sidesteps
the bug entirely, at the cost of requiring `--arm`/`--iface` after the
subcommand name.

### Shutdown

```python
signal.signal(signal.SIGTERM, _on_sigterm)   # turn SIGTERM into a KeyboardInterrupt
...
try:
    ...  # the requested command
except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    ctrl.disable()   # always runs: normal exit, Ctrl+C, or kill
```

Python turns Ctrl+C (SIGINT) into `KeyboardInterrupt` automatically, but a
plain SIGTERM (what `kill`, a process supervisor, or `timeout` sends by
default) normally just ends the process with no chance to clean up. The
`signal.signal` call re-routes SIGTERM through the same `KeyboardInterrupt`
path so `finally` — and therefore `disable()` — runs either way.
