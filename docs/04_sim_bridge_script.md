# sim_bridge.py — CoppeliaSim Read/Write Script

`src/sim_bridge.py` is the read/write interface to one arm (7 joints + 2
gripper fingers) in a live CoppeliaSim scene (`sim_files/dual_openarm_org.ttt`
or `dual_openarm_switched.ttt`), over CoppeliaSim's ZMQ Remote API. It can be
imported as a library (`SimArmController`) or run directly as a CLI. Config
(host/port, gripper finger range) comes from `src/config.yaml` via
`src/config.py`, same as the other scripts here. See
[02_arm_can_script.md](02_arm_can_script.md) for the equivalent script that
drives the real hardware, and [03_kinematics_script.md](03_kinematics_script.md)
for the pure-kinematics model — this script does no kinematics at all, it
only talks to CoppeliaSim.

Run everything from `OpenArm_v3/src/`, and have CoppeliaSim already open with
the OpenArm scene loaded and the ZMQ Remote API server running (on by default
since CoppeliaSim 4.x) — `sim_bridge.py` does not launch CoppeliaSim for you.

## CLI Usage

```bash
python3 sim_bridge.py list                                       # every joint alias found in the scene
python3 sim_bridge.py status --arm left                          # one-shot read
python3 sim_bridge.py read --arm left --rate 20                  # live monitor
python3 sim_bridge.py move --arm left --deg 0 -30 0 60 0 0 0     # kinematic write, ramped over 2s
python3 sim_bridge.py move --arm left --deg 0 -30 0 60 0 0 0 --dynamics
python3 sim_bridge.py gripper --arm left --open
python3 sim_bridge.py gripper --arm left --close
```

`--arm {left,right}` and `--host`/`--port` must come **after** the subcommand
(`list`, `status`, `read`, `move`, `gripper`), not before — same argparse
reason as `arm_can.py`, see
[Why --arm goes after the subcommand](02_arm_can_script.md#why---arm-goes-after-the-subcommand).
`--host`/`--port` default to `config.yaml`'s `sim.host`/`sim.port`
(`localhost:23000`) and only need overriding if CoppeliaSim is running
somewhere else.

| Command | What it does |
|---------|---------------|
| `list` | Print every joint object alias found anywhere in the scene. Run this first against a real scene to confirm the expected names actually exist — see [Naming and discovery](#naming-and-discovery) below. |
| `status` | Take one reading of all 7 joints + 2 gripper fingers. Quick sanity check. |
| `read` | Continuously print joint positions in a loop. Ctrl+C to stop. |
| `move --deg D1..D7` | Set all 7 arm joints to the given target angles (degrees). Kinematic write by default, ramped over `--duration` seconds (default 2s); `--dynamics` drives via `setJointTargetPosition` instead — see [Two ways to write a joint](#two-ways-to-write-a-joint). |
| `gripper --open` / `--close` | Drive both finger joints to the configured open/close extreme (metres, `config.yaml` `sim.gripper_finger_range_m`). |

Joint angles for `move` are checked against that side's `R_left_arm` /
`R_right_arm` `limits_deg` (the same as-measured real-hardware limits
`arm_can.py` uses) before anything is sent — an out-of-range value raises
`ValueError` rather than being silently clamped or sent anyway.

## Library Usage

```python
from sim_bridge import SimArmController

sim_arm = SimArmController("left")     # or SimArmController("right", host=..., port=...)
sim_arm.connect()

states = sim_arm.read_joint_states()    # list[SimJointState]: name, position (rad or m)
q = sim_arm.read_joint_positions()      # just the 7 arm-joint radians

sim_arm.set_joint_positions(q)                  # instant kinematic write (teleport)
sim_arm.move_joints_over_time(q, 2.0)           # ramped kinematic write over 2s
sim_arm.move_joints(q)                          # dynamic write (setJointTargetPosition)

sim_arm.open_gripper()
sim_arm.close_gripper()

print(sim_arm.list_scene_joints())      # {alias: handle} for every joint in the scene
```

## What's Happening Inside

### Naming and discovery

`config.yaml`'s dual-arm URDF (`sim_files/udrf/openarm_dual.urdf`) names the
arm joints `openarm_left_joint1`..`openarm_left_joint7` (and `_right_`), and
the two gripper joints `openarm_{side}_finger_joint1`/`finger_joint2`.
CoppeliaSim's URDF importer preserves these as each object's **alias** (its
own leaf name, not the full scene path) — but the importer can nest objects
anywhere in the tree, so `sim_bridge.py` never assumes a path. Instead,
`list_scene_joints()` walks the *whole* scene once:

```python
all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type, 0)
by_alias = {sim.getObjectAlias(h, 0): h for h in all_joints}
```

and `_resolve_handles()` looks up the 7 expected arm-joint names (and, best
effort, the 2 finger names) in that dict. If an expected arm-joint name is
missing, it raises immediately with a pointer to run `sim_bridge.py list` —
this is the first thing to run against an unfamiliar or renamed scene, since
it prints exactly what the importer actually called things.

### Two ways to write a joint

CoppeliaSim exposes two different ways to move a joint, and `sim_bridge.py`
wraps both because they suit different scene setups:

- **`set_joint_positions()` / `set_joint_position` (kinematic)** — teleports
  the joint straight to a value, ignoring dynamics entirely. Works on any
  scene, including a purely visual one with no physics/motion-control set up.
  This is the default for `move` on the CLI, and — like `arm_can.py`'s
  `move_joints_over_time()` — `move_joints_over_time()` ramps it in small
  steps over `--duration` seconds instead of jumping, so motion reads as
  continuous rather than a snap.
- **`move_joints()` / `setJointTargetPosition` (dynamic)** — sets a target for
  CoppeliaSim's own motion-control loop to drive toward, respecting the
  joint's configured dynamics (mass, motor force/velocity limits, etc.). Only
  does anything if the simulation is running *and* the joint's control loop
  is enabled in the scene; otherwise CoppeliaSim just ignores the call. Opt
  into this with `--dynamics` on the CLI.

Neither mode is "more correct" in general — kinematic is what you want to
sanity-check a pose or drive the sim off the real arm's readings (via
`arm_can.py`) with no physics involved; dynamic is what you want if the scene
is set up to simulate motor behavior and you want it to actually push against
that.

### The gripper is linear, not rotary

Unlike the real hardware's gripper (a rotary joint, degrees, with per-side
`open`/`close` angles in `config.yaml`'s `R_left_arm`/`R_right_arm`), the
simulated gripper in `openarm_dual.urdf` is two independent **prismatic**
joints (`openarm_{side}_finger_joint1`/`finger_joint2`, metres, range
`[0.0, 0.044]`) with no `mimic` constraint between them — so `set_gripper()`
writes the same target to both fingers directly rather than deriving one from
the other. `config.yaml`'s `sim.gripper_finger_range_m: [0.0, 0.044]` defines
`[closed, open]` for `open_gripper()`/`close_gripper()`; this is a guess based
on the URDF's `lower="0.0"` bound, not something confirmed against the mesh,
so check visually which end actually corresponds to "open" in your scene and
flip the config values if it's backwards.

### Joint limits: the real-arm limits, same as `arm_can.py`

`sim_bridge.py` validates against `config.yaml`'s `R_left_arm`/`R_right_arm`
`limits_deg` — the same as-measured, per-side real-hardware limits
`arm_can.py` uses — read directly via `config.py`'s `REAL_ARM_LIMITS_RAD`.
No mirroring is needed (unlike the old `Sim_limits_deg` source, or
`kinematics.py`'s `sim` `limit_source`) since each side's real numbers are
already independently correct. This means a target valid in the sim is now
also guaranteed valid on the real arm (and vice versa) — deliberately made
consistent so `main.py`'s `--mode both`/`imitate` don't hit a target that's
in range for one backend and not the other. `Sim_limits_deg` still exists in
`config.yaml` for `kinematics.py`'s optional `--limits sim` IK source, but
`sim_bridge.py` no longer reads it at all.

### Connecting

`connect()` (library) / the CLI's `main()` both do the same two-step handshake:

```python
client = RemoteAPIClient(host=host, port=port)   # opens the ZMQ connection
sim = client.require('sim')                       # the 'sim' API namespace
```

`RemoteAPIClient` defaults to `localhost:23000`, CoppeliaSim's standard ZMQ
Remote API port — `config.yaml`'s `sim.host`/`sim.port` are set to match and
only need changing for a non-default port or a CoppeliaSim instance on
another machine. Unlike `arm_can.py`, there's no enable/disable handshake or
explicit "connection closed" step — the ZMQ client is stateless per call, so
there's nothing to clean up on exit beyond letting the process end.
