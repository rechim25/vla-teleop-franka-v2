# franka_xr_teleop

Franka Panda teleoperation bridge for XR inputs.

This module is the Franka-specific interface layer between XRoboToolkit-style operator commands and the libfranka 1 kHz servo loop. It is focused on safe teleoperation and command execution only.

## Scope

- Receive XR operator commands on workstation (UDP)
- Map XR frame commands into robot-frame Cartesian teleop commands
- Enforce deadman/clutch/timeout/workspace/rate-limit safety
- Execute bounded Cartesian deltas through libfranka
- Publish executed action + robot observation for later recording

Non-goals in this phase:
- dataset export
- camera recording
- model training

## Repo Layout

```text
franka_xr_teleop/
  cpp/
    teleop_bridge/
      main.cpp
      xr_receiver.cpp
      xr_receiver.h
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
  tools/
    inspect_xr_packets.py
    replay_teleop_session.py
```

## Message Contract

Incoming XR command packet fields:
- `timestamp_ns`
- `sequence_id`
- `teleop_enabled`
- `clutch_pressed`
- `target_position_xyz`
- `target_orientation_xyzw`
- `gripper_command`

Current transport packet (binary, little-endian):
- C struct equivalent in `xr_receiver.cpp` (`XRWirePacket`, 72 bytes)

Outgoing observation stream (UDP JSON):
- robot state: `q`, `dq`, TCP pose, gripper width
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
- deadman (`teleop_enabled`) is true
- robot is not in reflex/fault mode

## Build

Uses the same libfranka stack as `franka-sanity-checks` (`0.9.2`, server version 5).

```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

## Run

Milestone 1 (receive + validate packets, no robot motion):

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge --dry-run --listen-port 28080
```

Milestone 3 path (live teleop bridge to robot):

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge \
  --robot-ip 192.168.2.200 \
  --listen-port 28080 \
  --obs-port 28081
```

Hold-only validation mode (no robot movement while keeping all data paths live):

```bash
./build/cpp/teleop_bridge/franka_xr_teleop_bridge \
  --robot-ip 192.168.2.200 \
  --listen-port 28080 \
  --obs-port 28081 \
  --no-motion
```

## XR Tooling

Inspect incoming XR packets:

```bash
python3 tools/inspect_xr_packets.py
```

Replay saved teleop JSONL to bridge UDP input:

```bash
python3 tools/replay_teleop_session.py /path/to/session.jsonl --ip 127.0.0.1 --port 28080 --hz 90
```

## Safety Notes

The 1 kHz callback is RT-safe by design:
- no parsing/network I/O in callback
- no dynamic allocation in callback
- latest-command lock-free handoff

Current safety shaping includes:
- packet timeout -> hold
- clutch recenter behavior
- max translation/rotation speed limits
- per-step delta limits
- workspace clamping
- jump rejection

Tune limits in `configs/safety.yaml` and mirror those values in code before production runs.
