# Using Pinocchio in This Project

[stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio) is a
rigid-body dynamics library (FK, Jacobians, RNEA/CRBA, mass matrix, gravity
and Coriolis terms) that loads directly from a URDF, the same
`sim_files/udrf/openarm_dual.urdf` this project already has. It doesn't
replace `kinematics.py` — it fills the gap next to it: **dynamics**, which
`roboticstoolbox` covers only partially and which
[08_gravity_compensation.md](../../openarm_old/docs/08_gravity_compensation.md)
in `openarm_old` identifies as unimplemented.

## Where it fits vs. the current stack

| Need | Current tool | Pinocchio |
|---|---|---|
| FK / IK (`kinematics.py`) | `roboticstoolbox` + `spatialmath`, working | could cross-check, not required |
| Jacobian | `robot.jacob0()`, working | `pin.computeJointJacobians` |
| **Gravity torque** `tau_ff = gravity(q)` | not implemented (the open item in the gravity-compensation plan) | `pin.computeGeneralizedGravity(model, data, q)` — one call |
| Full inverse dynamics (mass, Coriolis, gravity) | not available | `pin.rnea(model, data, q, v, a)` |
| Mass/inertia matrix (for computed-torque control, if ever needed) | not available | `pin.crba(model, data, q)` |

The gravity-compensation doc's "Open Decisions" table lists
`roboticstoolbox.rne()` on the URDF as option (b) — "most rigorous but
joint order/signs may not match the motor convention." Pinocchio is the
same category of solution (rigorous, URDF-driven RNEA) but is the
purpose-built tool for exactly this: it's what most real robot controllers
(computed-torque, gravity feed-forward, whole-body control) use for this
calculation, it's actively maintained, and its C++ core makes it fast enough
to call every control cycle (relevant if this ever moves from a one-shot
feed-forward term to something recomputed at 100 Hz+ in `ArmWorker._ctrl_loop`).

## Install

```bash
pip install pin      # PyPI package is named "pin" (Pinocchio >=3.0), not "pinocchio"
```

Add `pin` to whatever requirements file ends up tracking `OpenArm_v3/src`'s
dependencies alongside `roboticstoolbox`, `spatialmath`, `numpy`, `scipy`.

## Minimal example against this project's URDF

```python
import pinocchio as pin
import numpy as np

URDF_PATH = "sim_files/udrf/openarm_dual.urdf"

model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

q = pin.neutral(model)          # full dual-arm configuration vector
tau_g = pin.computeGeneralizedGravity(model, data, q)   # gravity torque per joint, N·m
```

`tau_g` comes back indexed over **every** joint in `openarm_dual.urdf`
(both arms + any fixed body joints), not per-arm — mirror the subchain
approach `kinematics.py` already uses (`ARM_LINKS`, jindex renumbering) to
extract just the 7 values for one arm, or build a single-arm Pinocchio
model directly from a reduced URDF/subtree if a cleaner split is wanted.

## Gotchas specific to this project

- **Joint order/signs**: `config.yaml` documents a sign-mirroring convention
  for the right arm (`joint1`/`joint2` flipped between `sim` and `real`
  limit sources). Pinocchio's joint order comes straight from URDF traversal
  order — verify it against the CAN motor mapping in `arm_can.py` the same
  way `kinematics.py` had to reconcile `jindex` before trusting any
  `tau_g` value on hardware. Validate with the zero-stiffness float test
  already specified in the gravity-compensation doc (§5, "Validation
  procedure") before trusting a sign.
- **Inertials**: gravity torque is only as correct as the URDF's `<inertial>`
  blocks. Spot-checked here — `openarm_dual.urdf` does carry mass/COM/inertia
  per link (e.g. `link0` mass `13.89`, per-link `<inertia>` tensors), so the
  data needed is present; it just hasn't been fed through a gravity solver
  yet.
- **Frame convention**: `kinematics.py` roots both arms at the shared
  `robot_base` frame rather than each arm's own mount link, specifically so
  left/right poses share a world frame. Gravity computation doesn't need
  that — gravity torque per joint is frame-independent of the chain's root
  choice as long as the model's own gravity vector (`model.gravity`,
  default `(0,0,-9.81)`) matches the URDF's actual "up" axis.

## Suggested integration

Keep `kinematics.py` as-is — FK/IK there already works and is
hardware-validated per the CLI in
[03_kinematics_script.md](03_kinematics_script.md). Add Pinocchio only where
there's a real gap:

1. New module (e.g. `src/gravity.py`) that loads `openarm_dual.urdf` with
   Pinocchio and exposes a `gravity_torque(arm, q_deg) -> np.ndarray` matching
   `ArmKinematics`'s per-arm, degrees-in convention.
2. Wire its output into `tau_ff` in the CAN control loop
   ([02_arm_can_script.md](02_arm_can_script.md)), guarded by the safety
   items the gravity-compensation doc already calls for: a per-joint clamp,
   a master enable/disable toggle, and a step-rate limiter on `q_target`.
3. Validate with the zero-stiffness float test before enabling any stiffness
   on top of it.
