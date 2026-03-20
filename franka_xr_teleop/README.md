# franka_xr_teleop

Franka Panda teleoperation bridge using **XRoboToolkit PC Service SDK** as the only operator input source.

This module is the Franka-specific interface layer between XR-Robotics tracking/controller data and the libfranka 1 kHz servo loop. It is focused on safe teleoperation and command execution.

## Scope

- Receive operator state through XR-Robotics SDK callbacks (`PXREAInit` / `PXREADeviceStateJson`)
- Map XR controller pose into robot-frame Cartesian teleop commands
- Enforce deadman/clutch/timeout/workspace/rate-limit safety
- Execute bounded Cartesian deltas through libfranka
- Publish executed action + robot observation for later recording

Non-goals:
- dataset export
- camera recording
- model training

## Repo Layout

```text
franka_xr_teleop/
  cpp/
    teleop_bridge/
      main.cpp
      xrobotics_source.cpp
      xrobotics_source.h
      teleop_mapper.cpp
      teleop_mapper.h
      teleop_state_machine.cpp
      teleop_state_machine.h
      franka_controller.cpp
      franka_controller.h
      safety.cpp
      safety.h
      gripper.cpp
      gripper.h
      observation_pub.cpp
      observation_pub.h
      common_types.h
      CMakeLists.txt
  configs/
    teleop.yaml
    robot.yaml
    safety.yaml
    xr_frame.yaml
  schemas/
    xr_command.schema.json
    robot_observation.schema.json
```

## Input Mapping (Controller-only v1)

The bridge maps the **right controller** fields from XR-Robotics state JSON to teleop semantics:
- `grip`: deadman / arm-enable trigger
- `trigger`: gripper command `[0,1]` mapped to binary open/close with hold-on-contact semantics
- `primaryButton` (A): available for future bindings
- `secondaryButton` (B): available for future bindings
- `pose` (`x,y,z,qx,qy,qz,qw`): target position/orientation input

## Message Contract (Internal)

Incoming normalized command fields used by the bridge:
- `timestamp_ns`
- `sequence_id`
- `teleop_enabled`
- `clutch_pressed`
- `target_position_xyz`
- `target_orientation_xyzw`
- `gripper_command`

`timestamp_ns` is stamped on command receive using workstation monotonic time so timeout-to-hold logic is clock-safe.

Outgoing observation stream (UDP JSON):
- robot state: `q`, `dq`, TCP pose, gripper width, gripper state
- executed action: cartesian delta, gripper command
- status: control mode, teleop state, packet age, fault flags

## Teleop State Machine

States:
- `DISCONNECTED`
- `CONNECTED_IDLE`
- `TELEOP_ARMED`
- `TELEOP_ACTIVE`
- `FAULT`

Motion only occurs in `TELEOP_ACTIVE` when:
- XR stream is healthy (packet age within timeout)
- deadman is true
- robot is not in reflex/fault mode

## Prerequisites

- Ubuntu 22.04
- Same Franka setup as `franka-sanity-checks` (`libfranka 0.9.2`, server version 5)
- XRoboToolkit PC Service installed/running (for example `/opt/apps/roboticsservice/runService.sh`)

## Build

```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DXROBOTICS_SERVICE_ROOT=/opt/apps/roboticsservice
cmake --build build -j"$(nproc)"
```

If SDK paths are non-default, pass `-DXROBOTICS_SDK_ROOT=/path/to/SDK`.

## Run

Dry-run (SDK input only, no robot motion):

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge --dry-run
```

Live teleop:

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge \
  --robot-ip 192.168.2.200 \
  --obs-port 28081
```

Save the robot's current joint positions as the startup home pose:

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge \
  --robot-ip 192.168.2.200 \
  --save-home
```

This reads the current measured joint state and updates `configs/teleop.yaml` at
`teleop.start_joint_positions_rad`. No motion is commanded in this mode.

Hold-only mode (data path active, no robot motion):

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge \
  --robot-ip 192.168.2.200 \
  --obs-port 28081 \
  --no-motion
```

## Safety Notes

The 1 kHz callback remains RT-safe:
- no parsing/network I/O in callback
- no dynamic allocation in callback
- lock-free handoff of latest operator command

Safety shaping includes:
- packet timeout -> hold
- clutch recenter
- max translation/rotation speed limits
- per-step delta limits
- workspace clamping
- jump rejection
