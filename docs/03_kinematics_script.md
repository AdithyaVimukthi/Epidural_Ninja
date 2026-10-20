# kinematics.py — Forward/Inverse Kinematics Script

`src/kinematics.py` models one OpenArm 7-DOF arm (left or right) using
[roboticstoolbox-python](https://github.com/petercorke/robotics-toolbox-python),
built from `sim_files/udrf/openarm_dual.urdf`. It can be imported as a library
(`ArmKinematics`) or run directly as a CLI. Only the link geometry and joint
axes come from the URDF — joint limits come from `src/config.yaml`, not the
URDF, so limits stay editable in one place. See
[02_arm_can_script.md](02_arm_can_script.md) for the CAN-side script that
actually drives the hardware; this script does no CAN I/O at all, it's pure
kinematics.

Run everything from `OpenArm_v3/src/`.

## CLI Usage

```bash
python3 kinematics.py --arm left                                       # limits + FK/IK demo
python3 kinematics.py --arm right --limits sim                         # use simulation limits instead
python3 kinematics.py --arm left  --fk 0 -30 0 45 0 20 0                # forward kinematics
python3 kinematics.py --arm right --ik 0.3 0.1 0.2 0 0 0                # full-pose inverse kinematics
python3 kinematics.py --arm right --ik 0.3 0.1 0.2                      # position-only IK, orientation free
```

| Flag | What it does |
|------|---------------|
| `--arm {left,right}` | Which arm to model (default `left`). |
| `--limits {real,sim}` | Which `config.yaml` limit set to enforce (default `real`, see below). |
| `--fk q1 q2 q3 q4 q5 q6 q7` | Joint angles in **degrees** → prints the end-effector pose (xyz in m, rpy in deg). |
| `--ik x y z [roll pitch yaw]` | Target pose → solves for joint angles. 3 values = position only (xyz in m, any orientation); 6 values = full pose (rpy in **degrees**). |

With neither `--fk` nor `--ik`, it prints the active joint limits and runs a
small built-in demo (FK at the zero pose, then an FK→IK round-trip check).

`--fk` checks the given angles against the active limits and prints a
`WARNING` if any are out of range — it still computes the pose either way, it
doesn't stop you. `--ik` reports `success = False` if the solver couldn't get
within tolerance (target unreachable, or only reachable outside the joint
limits) — it also prints `residual` (how far off, in metres/radians) and the
closest `q` it found, so a near-miss is distinguishable from "nowhere close."

All poses (`--fk` output, `--ik` input) are in the shared `robot_base` frame —
the same frame for both arms — not each arm's own local mount frame; see
"Which frame poses are in" below.

## Library Usage

```python
from kinematics import ArmKinematics

km = ArmKinematics("left")                    # real-arm limits by default
km = ArmKinematics("right", limit_source="sim")   # or explicitly pick sim limits

T = km.fkine([0, -30, 0, 45, 0, 20, 0])       # degrees -> spatialmath.SE3 pose
print(T.t, T.rpy(unit="deg"))                  # xyz, roll/pitch/yaw (deg)

sol = km.ikine(T)                              # SE3 pose -> IKResult
if sol.success:
    q_deg = __import__("numpy").degrees(sol.q)
else:
    print(sol.reason, sol.residual)            # how far off the closest attempt landed

sol = km.ikine_position([0.3, 0.1, 0.2])       # xyz only -> orientation left free
sol = km.ikine(T, mask=[1, 1, 1, 0, 0, 0])     # same thing, spelled out

J = km.jacobian([0, -30, 0, 45, 0, 20, 0])     # 6x7 geometric Jacobian (world frame)

km.in_limits(sol.q)                            # bool, radians in
km.clip(sol.q)                                 # clamp to the active limits
print(km.limits_report())                      # human-readable limit dump
```

`fkine`/`jacobian` take joint angles in **degrees**; `ikine`/`in_limits`/`clip`
work in **radians** (matching `robot.qlim` and the IK solver's units). This
mismatch is deliberate — degrees are what you'd type on a CLI or read off a
spec sheet, radians are what the solver and limit arrays use internally.

## What's Happening Inside

### Building a single-arm chain from the dual-arm URDF

`openarm_dual.urdf` describes both arms plus grippers as one branching tree
(18 joints total: 7 arm + 2 finger joints, times two). `ArmKinematics.__init__`
loads the whole thing with `rtb.Robot.URDF(...)`, then extracts just one arm's
kinematic chain with:

```python
ets = full.ets(start="robot_base", end="openarm_left_hand_tcp_visual")
```

`robot.ets(start, end)` walks the **unique tree path** between two named
links — it is not a link-index range. `robot_base → body_link0` is a shared
trunk, and the left/right arms are sibling branches off `body_link0` (neither
is an ancestor of the other), so the path to one arm's TCP never touches the
other arm's links: the extracted `ETS` for the right arm has exactly 7 joints
(`jindex` 9..15, matching its position in the full 18-joint tree) and zero
left-arm terms, and vice versa. `hand_tcp_visual` is the tool-center-point
frame at the gripper. Finger joints aren't included in the chain — this
script only ever models the 7 arm joints, not the gripper open/close motion.

The extracted `ETS` keeps the joint indices it had in the *full* tree (e.g.
9..15 for the right arm), so those get renumbered to 0..6 before wrapping it
as a standalone `rtb.Robot` — otherwise `rtb.Robot()` rejects it with
`"joint index N out of range"`.

#### Which frame poses are in

`start="robot_base"` matters, not just "any link that identifies the arm."
`ets(start=X, end=Y)` includes the transform *into* X as its first hop, so
using `start="openarm_left_link0_respondable"` would still root the chain at
that link's *parent* (`body_link0`) — the frame it's actually expressed in is
one hop further back than the name suggests. Rooting explicitly at
`robot_base` avoids that ambiguity and keeps both arms' poses expressed in
the same, single, world-aligned frame — which also matters practically: a
pose that looks reachable in a simulator (typically expressed in the world/
`robot_base` frame) needs to be interpreted in that same frame here, or IK
against it will fail for the wrong reason (frame mismatch, not
unreachability).

### Joint limits: two sources, real by default

`config.yaml` defines two limit sets, and this script can use either
(`--limits` / `limit_source=`, default `"real"`):

- **`real`** — `R_left_arm.limits_deg` / `R_right_arm.limits_deg`: the
  as-measured limits for each physical arm, used exactly as given. Right and
  left are **not** mirror images on the real hardware (e.g. right joint1 is
  `[0°, 80°]`, left joint1 is `[-80°, 0°]`), so there's no sign-flipping logic
  here — each side's numbers are read straight from its own config block.
- **`sim`** — `Sim_limits_deg`: one generic, symmetric set (the original URDF
  numbers). Since only one set exists for both arms, the right arm's
  `joint1`/`joint2` limits are mirrored at load time (`lo, hi = -hi, -lo`),
  per the "Right arm mirrors joint1/joint2 signs" convention documented in
  `config.yaml`; joints 3-7 are identical on both sides so no mirroring is
  needed there.

Either way, the resulting `(2, 7)` array (`[lower_rad; upper_rad]`) is written
to `self.robot.qlim`, which is what `ikine()` (`joint_limits=True`) and
`in_limits()`/`clip()` read from. `limits_report()` also prints the real-arm
gripper open/close angles when `limit_source="real"`, since `config.yaml`
carries those alongside the per-arm joint limits.

Because `config.yaml` is read fresh on every `ArmKinematics(...)` construction,
editing the limits there and re-running picks the change up automatically —
nothing in this script hardcodes limit values.

### Forward and inverse kinematics

`fkine()` is a direct `robot.fkine(q)` call — standard product-of-transforms
FK, no iteration involved.

`ikine()` minimizes pose error (`(T_target.inv() * fkine(q)).log(twist=True)`,
a 6-vector combining position and orientation error) with
`scipy.optimize.least_squares`, using box bounds `bounds=self.qlim` — so a
solution is never returned outside the active joint limits, it's constrained
into them by construction rather than checked after the fact. It tries the
given `q0_deg` (or zero) first, then up to `restarts` (default 8) random
seeds within the limits, stopping as soon as one converges within `tol`
(default `1e-4`, i.e. 0.1 mm / ~0.006°); `IKResult.residual` reports how close
the best attempt got, and `.reason` explains a failure.

This was originally built on roboticstoolbox's own `ikine_LM`, which turned
out to fail unreliably — including with dozens of random restarts — whenever
the true solution sits exactly on a joint-limit boundary. That's not a corner
case here: because `R_left_arm`/`R_right_arm`'s real limits are narrow (as
little as 80-90° of travel on joint1/joint2), a meaningful slice of the
reachable workspace only has solutions with one or more joints pinned at
their limit. `least_squares`'s trust-region-reflective algorithm is designed
to handle box constraints correctly, including boundary-active optima, so it
finds those reliably where `ikine_LM` did not.

`jacobian()` returns the geometric Jacobian in the world frame
(`robot.jacob0`), relating joint velocities to end-effector linear+angular
velocity — useful for velocity-level control or manipulability checks, though
this script doesn't do anything with it beyond exposing it.

### Position-only IK (orientation left free)

A 7-DOF arm has one redundant degree of freedom against a full 6-DOF pose
target, and constraining all 6 (3 position + 3 orientation) makes some
otherwise-reachable points fail to solve — the arm can *touch* the point, just
not with that exact orientation. `--ik x y z` (3 values instead of 6) solves
for position only, via `ikine()`'s `mask` argument
(`mask=[1, 1, 1, 0, 0, 0]`): the x/y/z error is still driven to zero, but
orientation error is excluded from the cost entirely, so the solver is free to
land on whatever orientation gets there. `ikine_position(xyz)` is the library
equivalent. This generally converges more easily and for a larger reachable
volume than a full 6-value target.
