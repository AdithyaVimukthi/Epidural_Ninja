# Hardware Setup

## Physical Components

| Component | Details |
|-----------|---------|
| Robot arms | OpenArm v2.0 — dual arm (left + right) |
| CAN adapter | SAVVYCANFD 2CH CANFD USB (two independent channels) |
| Arm motors | per arm: joint1–2 Damiao DM8009, joint3–4 DM4340, joint5–7 DM4310 |
| Gripper motor | 1 × Damiao DM4310 per arm |
| Protocol | CAN-FD at 5 Mbps data bitrate |

## CAN Interface Mapping

| Channel | Linux Interface | Arm |
|---------|----------------|-----|
| CH1 | `can0` | Right arm |
| CH2 | `can1` | Left arm |

The SAVVYCANFD adapter appears as two separate SocketCAN interfaces. Both are configured for CAN-FD (flexible data-rate) at 5 Mbps.

## Bringing Up the CAN Interfaces (needed every time)

SocketCAN interfaces are **not persistent** — `can0`/`can1` come up as `DOWN`
after every reboot, and also reset if the USB CAN-FD adapter is
unplugged/replugged or the machine sleeps. There's no udev rule or
systemd-networkd config in this project that does it automatically, so run
this manually before any real-hardware session (`bridge.launch.py` without
`use_fake_hardware:=true`, or `gui.py`/`main.py`/etc. against real CAN):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up
```

Check it worked:

```bash
ip link show can0
ip link show can1
# both should report "state UP"
```

If a node that owns real CAN (`arm_driver_node`, `gui.py`, `main.py`, etc.)
was started while the interface was down, restarting the interface alone
won't fix it — the node already failed/is holding a dead handle. Kill and
restart that node/launch after bringing the interface up, not before.

## Motor IDs

Both arms use identical CAN IDs. The host sends commands **to** `send_ids` and receives responses **from** `recv_ids`.

| Joint | Name | Send ID | Recv ID |
|-------|------|---------|---------|
| 1 | joint1 — shoulder pan | 0x01 | 0x11 |
| 2 | joint2 — shoulder lift | 0x02 | 0x12 |
| 3 | joint3 — elbow roll | 0x03 | 0x13 |
| 4 | joint4 — elbow pitch | 0x04 | 0x14 |
| 5 | joint5 — wrist roll | 0x05 | 0x15 |
| 6 | joint6 — wrist pitch | 0x06 | 0x16 |
| 7 | joint7 — wrist twist | 0x07 | 0x17 |
| 8 | gripper | 0x08 | 0x18 |

These IDs were discovered by running the CAN diagnostic tool:

```bash
openarm-can-diagnosis can0 -fd
openarm-can-diagnosis can1 -fd
```

Both arms responded on all 8 IDs at 5 Mbps, confirming the wiring and CAN-FD configuration.

## Motor Specifications

MIT-mode ranges, as read from each motor's PMAX/VMAX/TMAX registers on both
arms (`python3 comm/arm_can.py params --arm right`):

| Joints | Type | TMAX (torque range) | VMAX | PMAX |
|--------|------|---------------------|------|------|
| joint1–2 | DM8009 | ±54 Nm | ±45 rad/s | ±12.5 rad |
| joint3–4 | DM4340 | ±28 Nm | ±10 rad/s | ±12.5 rad |
| joint5–7, gripper | DM4310 | ±10 Nm | ±30 rad/s | ±12.5 rad |

KP range is 0 – 500 Nm/rad and KD 0 – 5 Nm·s/rad on every type.

The types in `config.yaml` (`motors.arm.types`) **must match the motors**.
openarm_can encodes `tau_ff` (and decodes the reported torque and velocity)
over ±TMAX of the *declared* type, while the motor decodes over its own
register. Declaring every joint DM4310 made joint4 (really a DM4340) apply
28/10 = 2.8× the commanded gravity feedforward, so it overshot its target by
~40% (60° → 84.6° at kp 15); joints 1–2 would have got 5.4×.

## Joint Limits (from URDF openarm_v2.0)

These are software limits enforced before any command is sent. The right arm mirrors joint1/joint2 signs but uses the same numerical limits.

| Joint | Min (deg) | Max (deg) |
|-------|-----------|-----------|
| joint1 | -200.0 | 80.0 |
| joint2 | -190.0 | 10.0 |
| joint3 | -90.0 | 90.0 |
| joint4 | 0.0 | 140.0 |
| joint5 | -90.0 | 90.0 |
| joint6 | -45.0 | 45.0 |
| joint7 | -45.0 | 45.0 |
| gripper | -90.0 | 0.0 |

## Library: openarm_can

The `openarm_can` Python library provides the low-level CAN interface.

- **Version installed**: 1.2.8
- **Location**: `/usr/lib/python3/dist-packages/openarm_can/`
- **Type**: C++ library with nanobind Python bindings
- **Package**: `python3-openarm-can` (system APT package)

All scripts add this path explicitly at the top:

```python
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import openarm_can as oa
```

### Key API Classes

```python
oa.OpenArm(iface, can_fd)           # main arm object
oa.MotorType.DM4310                 # motor type enum (also DM4340, DM8009, ...)
oa.CallbackMode.IGNORE              # discard incoming frames
oa.CallbackMode.STATE               # parse incoming frames into state
oa.MITParam(kp, kd, q, dq, tau_ff) # one motor's control command
```

### Initialization Sequence (required for all scripts)

```python
arm = oa.OpenArm("can0", True)   # True = CAN-FD
arm.init_arm_motors([getattr(oa.MotorType, t) for t in ARM_MOTOR_TYPES], SEND_IDS, RECV_IDS)
arm.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SND, GRIPPER_RCV)
arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
arm.enable_all()
arm.recv_all()
arm.set_callback_mode_all(oa.CallbackMode.STATE)
```

The `IGNORE → enable → recv_all → STATE` sequence is required: motors send a response on enable that must be consumed before switching to STATE mode, otherwise the response frame is misinterpreted as a state update.
