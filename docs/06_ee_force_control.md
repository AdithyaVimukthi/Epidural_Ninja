# End-Effector Force Control — CoppeliaSim Test Rig (No Gravity → Gravity → Real Arm)

Status: **sim-side implemented (`dynamics.py`, `sim_force_control.py`), real-arm port not yet done.**

Goal: command a specific force/torque at the end effector, expressed in the
world frame, via `tau = J(q)^T @ F_world`. Validated in CoppeliaSim first —
starting with gravity **off** so the ideal-case math can be checked with
nothing else in play — before adding gravity compensation and finally
porting to the real arm's `tau_ff` channel in
[arm_can.py](../src/arm_can.py). See
[05_pinocchio_integration.md](05_pinocchio_integration.md) for why Pinocchio
specifically.

## The pieces

| File | What it does |
|---|---|
| [src/dynamics.py](../src/dynamics.py) | Pure math, no sim/hardware I/O. Pinocchio-based `ArmDynamics`: `gravity_torque(q_deg)`, `jacobian_world(q_deg)`, `ee_force_to_torque(q_deg, F_world, gravity_comp)`. |
| [src/sim_force_control.py](../src/sim_force_control.py) | CoppeliaSim-side: gravity toggle, joint torque-control-mode toggle, the streaming control loop, force-sensor readback. |

`dynamics.py` is standalone and already verified against `openarm_dual.urdf`
directly (no CoppeliaSim needed) — sanity-checked left/right symmetry:

```bash
python3 dynamics.py --arm left  --gravity 0 -30 0 45 0 20 0
python3 dynamics.py --arm right --gravity 0 -30 0 45 0 20 0
```

## Part 1 — Add a cube and a force sensor in CoppeliaSim

Do this by hand in the GUI, once, on a **copy** of the scene (`File > Save
As` → e.g. `dual_openarm_force_test.ttt`) so the normal teleop scene stays
untouched. Placement is the one part worth eyeballing rather than scripting
blind.

### 1. The cube (what the arm pushes against)

1. `Add` menu → `Primitive Shape` → `Cuboid`. Size it small, e.g. 0.05 x 0.05 x 0.05 m.
2. Rename its alias (double-click the name in the scene hierarchy, or
   right-click → `Rename`) to `force_test_cube` — the exact name doesn't
   matter to the scripts (only the force sensor's alias does, see below),
   but a clear name helps in `sim_force_control.py list`.
3. Double-click it (or right-click → `Object/item properties`) and set its
   **Position** to somewhere inside the left arm's reach — e.g. run
   `python3 kinematics.py --arm left --fk 0 -30 0 45 0 20 0` first and use
   that `xyz` as a starting point, then nudge it in the viewport once you
   can see the arm.
4. In the same dialog, tick **Body is respondable** (so it can be touched)
   and **leave "Body is dynamic" unchecked**. With gravity off and no
   friction, a *dynamic* cube would fly off the instant it's touched — an
   immovable "wall" is what makes the force reading mean something for this
   test. Make it dynamic later, once the basic test works, if you want to
   see the arm actually push it.

### 2. The force sensor + probe (what measures the force that actually lands)

1. `Add` menu → `Force sensor` → pick the default option.
2. It's inserted at the world origin as its own small object. Rename its
   alias to `openarm_force_test_sensor` — the `test` command's `--sensor`
   flag needs this exact name.
3. In the scene hierarchy (tree panel), **drag the sensor onto
   `openarm_left_joint7`** so it becomes that joint's child — this puts it
   in series with the wrist, so it measures whatever force is transmitted
   from the arm into everything below it. (Attaching it to the actual hand
   link would put it closer to the true fingertip TCP, but its exact alias
   isn't confirmed in this scene — run `python3 sim_force_control.py list`
   first if you'd rather find and use that instead of `joint7`.)
4. Open the sensor's `Object/item properties` and zero out its position
   relative to its new parent, then nudge it a few cm along the local Z axis
   so it sits just past the wrist, clear of the joint geometry.
5. `Add` menu → `Primitive Shape` → `Sphere` (radius ~1 cm) for the probe
   tip. Drag it onto the force sensor so it becomes the sensor's child, and
   position it so it just touches — or very slightly overlaps (~2-5 mm) —
   the cube.
6. On the probe: tick **both** "Body is respondable" **and** "Body is
   dynamic", and give it a small mass (e.g. 0.05 kg) in the dynamics
   properties. Unlike the cube, the probe *must* be dynamic — the sensor
   reads the constraint force needed to hold a dynamic body in place against
   whatever it's colliding with; a static probe would always read zero.

### 3. Confirm it all landed correctly

```bash
python3 sim_force_control.py list
```

Look for `[shape] force_test_cube`, `[forcesensor] openarm_force_test_sensor`,
and your probe shape. If any is missing, it wasn't parented/renamed the way
the steps above assume — fix it in the GUI before moving on, the same way
`sim_bridge.py list` is the standard first check against an unfamiliar scene.

Save the scene.

## Part 2 — Stage 1: ideal-case test, no gravity

```bash
python3 sim_force_control.py gravity --off                                    # zero gravity
python3 sim_force_control.py mode --arm left --torque                         # 7 joints -> force/torque control
python3 sim_force_control.py sim --start                                      # press Play

python3 sim_force_control.py test --arm left --force 0 0 2 --duration 5 \
    --sensor openarm_force_test_sensor
```

This streams `tau = J(q)^T @ [0,0,2,0,0,0]` (2 N along world +z, no gravity
term) to the left arm's joints at 50 Hz, reading `q` back from the sim each
cycle. Watch the printed `measured F_world (sensor)` line — once the probe
settles against the cube, it should converge toward `[0, 0, 2, ~0, ~0, ~0]`,
i.e. the sensor (rotated into world axes) agrees with what was commanded.

What to try:

- Different directions (`--force 2 0 0`, `--force 0 0 -2`, etc.) and confirm
  the sensor tracks each one.
- Point the probe away from the cube (no contact) — the arm should just
  drift/accelerate slowly in the commanded direction (`F = ma`, nothing
  opposing it), which is the expected behavior with no gravity and no
  contact force to balance `tau`.
- `Ctrl+C` stops the stream early and zeros `tau_ff` on all joints before
  exiting (see `run_force_test`'s `finally` block) — the arm won't be left
  fighting a stale torque.

When done, put the scene back the way `sim_bridge.py`/normal teleop expects:

```bash
python3 sim_force_control.py mode --arm left --position
python3 sim_force_control.py gravity --on
python3 sim_force_control.py sim --stop
```

## Part 3 — Stage 2: add gravity back in sim

Same setup, but with gravity on and `--gravity-comp` so `tau_g(q)` is added
before the Jacobian-transpose term:

```bash
python3 sim_force_control.py gravity --on
python3 sim_force_control.py mode --arm left --torque
python3 sim_force_control.py sim --start

python3 sim_force_control.py test --arm left --force 0 0 2 --gravity-comp \
    --duration 5 --sensor openarm_force_test_sensor
```

The sensor reading should still converge to `[0, 0, 2, ...]`, same as
Stage 1 — the point of this step is confirming `tau_g(q)` correctly cancels
gravity's contribution instead of leaking into the measured force. If the
reading is off only when `--gravity-comp` is on, the gravity term (not the
Jacobian term, already validated in Stage 1) is the place to look —
consistent with the URDF-inertial and joint-order caveats in
[05_pinocchio_integration.md](05_pinocchio_integration.md).

## Part 4 — Real arm (not yet implemented)

Once Stage 2 is solid, the same `dynamics.ArmDynamics.ee_force_to_torque()`
call ports to hardware by feeding its output into
[arm_can.py](../src/arm_can.py)'s existing `tau_ff` channel —
`oa.MITParam(kp=0, kd=0, q_target=0, dq_target=0, tau_ff=tau[i])` per joint,
the same MIT-mode field `move_joints()` already uses for position control.
This still needs, per
[08_gravity_compensation.md](../../openarm_old/docs/08_gravity_compensation.md)'s
safety section: a per-joint `tau_ff` clamp, a master enable/disable toggle,
and a step-rate limiter — none of that exists yet. Do not skip Stage 1/2 on
the real arm.
