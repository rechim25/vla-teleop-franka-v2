# Quest 3 Real Robot Testing

## 1. Build and run on real Franka

### Prerequisites
- Franka reachable on FCI network (example: `192.168.2.200`).
- XRoboToolkit PC Service running and Quest connected.
- `franka_xr_teleop` already configured (`robot.yaml`, `teleop.yaml`, `safety.yaml`, `xr_frame.yaml`).

### Quest 3 headset connection (ADB over USB)
Use this when connecting Quest 3 to the PC service through USB tunnel mode.

Prerequisites:
- Quest Developer Mode enabled.
- USB debugging authorized on the headset.

Commands:
```bash
adb devices
adb reverse --remove-all
adb reverse tcp:63901 tcp:63901
adb reverse --list
```

Expected:
- `adb devices` shows the headset with state `device`.
- `adb reverse --list` includes `tcp:63901 tcp:63901`.

In the Quest app, connect to:
```text
127.0.0.1
```

### Build
```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXROBOTICS_SERVICE_ROOT=/opt/apps/roboticsservice
cmake --build build -j$(nproc)
```

### Realtime permission (required after every rebuild)
```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
sudo setcap cap_sys_nice=eip ./build/cpp/teleop_bridge/franka_xr_teleop_bridge
getcap ./build/cpp/teleop_bridge/franka_xr_teleop_bridge
```

Expected `getcap` output:
```text
./build/cpp/teleop_bridge/franka_xr_teleop_bridge cap_sys_nice=eip
```

### Run teleop (real motion)
```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
./build/cpp/teleop_bridge/franka_xr_teleop_bridge --robot-ip 192.168.2.200 --obs-port 28081
```

Notes:
- Startup default is skip-home (no automatic homing move before teleop).
- To force startup homing, add `--move-to-home`.

### Dry run (no robot motion)
```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
./build/cpp/teleop_bridge/franka_xr_teleop_bridge --robot-ip 192.168.2.200 --no-motion --obs-port 28081
```

## 2. Operator controls during teleop

- Hold right grip above threshold to enable teleop (deadman).
- Release right grip to immediately stop commanding motion and re-anchor.
- Right trigger controls gripper width.
- Hold `B` while teleop is active for clutch/recenter (freeze/re-anchor).
- Press `B` while deadman is not held to toggle local recording start/stop.
- Press right axis click to discard current active recording.

## 3. Where logs are

- Console logs: in the terminal where the bridge runs.
- Session logs (when recording is toggled on):  
  `/home/radu/vla-teleop-franka-v2/franka_xr_teleop/logs/franka_xr_teleop/`

## 4. Recover from reflex error

Typical errors:
- `motion aborted by reflex! ["cartesian_reflex"]`
- `robot_mode == kReflex`

Recovery steps:
1. Stop the teleop bridge (`Ctrl+C`).
2. Clear collision cause physically (remove contact/load issue, ensure free workspace).
3. In Franka Desk, unlock and run **Error Recovery**.
4. Re-run teleop bridge. It will also attempt `automaticErrorRecovery()` on startup.
5. If reflex repeats, reduce aggressiveness before retry:
   - Lower `teleop.scale_factor` (for example `0.2` to `0.3`).
   - Lower IK limits (`ik.max_joint_velocity_radps`, `ik.max_joint_step_rad`).
   - Verify realtime permission (`getcap`) is still set.

Fast check command:
```bash
cd /home/radu/vla-teleop-franka-v2/franka_xr_teleop
./build/cpp/teleop_bridge/franka_xr_teleop_bridge --robot-ip 192.168.2.200 --no-motion
```

## 5. Updated Architecture (current implementation)

The teleop stack is a 4-stage pipeline:

1. Quest input
- Quest app sends right-controller pose/buttons through XRoboToolkit PC Service.
- Bridge ingests packets via SDK callback, parses controller pose and buttons, timestamps locally.

2. Intent builder (anchored-relative mapping)
- Deadman is right grip threshold.
- On deadman engage, mapper anchors robot TCP + controller pose.
- Commands are relative to anchor, transformed by `xr_to_robot_rotation`, scaled, and deadbanded.
- `B` clutch holds motion and continuously re-anchors while held.
- Release deadman resets mapping anchor.

3. Planner (non-realtime, ~100 Hz)
- Reads latest XR command and latest robot state.
- Applies stream timeout gating, workspace clamp, jump reject, and Cartesian speed/step shaping.
- Solves IK increment:
  - `position` mode: true 3D position Jacobian solve.
  - `pose` mode: 6D position+orientation solve.
- Publishes target joints + diagnostic flags.

4. Realtime servo loop (robot control rate)
- Reads latest planned target each cycle.
- Applies stale-target gating and per-cycle joint step limits.
- If not safe/fresh/active, commands hold (`state.q`).
- Publishes observation stream (state, teleop flags, safety flags, executed action).

Auxiliary loops:
- Startup sequence: connect robot/gripper, configure conservative behavior, recover reflex if needed, move to home.
- Gripper loop (non-realtime): rate-limited width commands from right trigger.
- Optional JSONL recorder: local session files toggled from controller buttons.
