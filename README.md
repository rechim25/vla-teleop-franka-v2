# libfranka 0.9.2 Franka Panda Sanity Check

This project provides a single standalone C++ executable to verify:
- connectivity to a Franka Panda controller,
- basic state reads (`readOnce()` + `read()`),
- one tiny smooth joint-space command using `libfranka` (no ROS).

It is intentionally minimal and aimed at a first safety/sanity milestone before teleoperation and data collection.

## Version and Platform Constraints

- Ubuntu 22.04
- Robot server version: 5
- `libfranka` version: `0.9.2` (allowed: `>=0.9.1` and `<0.10.0`)
- Do **not** use `libfranka >= 0.10.0` with this setup

## Install Dependencies and libfranka

Follow [INSTALL.md](./INSTALL.md) first. It includes:
- required Ubuntu packages,
- `libfranka 0.9.2` build/install steps,
- verification commands,
- build commands for this project.

## What This Program Does

- Connects to robot by IP (`franka::Robot`)
- Prints:
  - `serverVersion()`
  - one full `RobotState` snapshot
  - current `q` and `O_T_EE`
- Supports two modes:
  - `read-only`: state reads only (`readOnce` + bounded `read`)
  - `tiny-motion`: one tiny smooth joint-space move from current state
  - `recover-only`: one-shot `automaticErrorRecovery()` when robot is in Reflex mode
- Before motion, explicitly configures:
  - `setCollisionBehavior(...)`
  - `setJointImpedance(...)`
  - `setCartesianImpedance(...)`
- Handles and reports:
  - `ControlException`
  - `RealtimeException`
  - `NetworkException`
  - `IncompatibleVersionException`
  - `CommandException`
- On control abort, prints `current_errors` and `last_motion_errors` from the final log state.
- Optional one-shot recovery: `--auto-recover` triggers `automaticErrorRecovery()` once after a control abort.

## Safety Checklist Before First Motion

1. Enable FCI in Franka Desk.
2. Connect laptop directly to the Franka Control LAN port.
3. Boot a PREEMPT_RT kernel.
4. Ensure realtime permissions are configured so control can run with RT priority.
5. Set CPU governor to `performance`.
6. Run `read-only` mode first.
7. Run `communication_test` from libfranka examples before tiny motion.
8. For first motion, keep `--delta-rad` very small (default `0.01` rad).

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

If `find_package(Franka)` fails, use the fallback command in [INSTALL.md](./INSTALL.md) with `-DFranka_DIR=/usr/local/lib/cmake/Franka`.

## Run

After completing [INSTALL.md](./INSTALL.md):

Read-only connectivity check:

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode read-only --read-samples 200
```

Tiny motion (single smooth move):

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode tiny-motion --joint-index 3 --delta-rad 0.01 --duration-s 3.0
```

Tiny motion with one-shot error recovery after abort:

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode tiny-motion --auto-recover
```

Clear Reflex mode only (no motion):

```bash
./build/panda_libfranka_sanity --robot-ip 192.168.2.200 --mode recover-only
```

If you see `communication_constraints_violation` and the robot enters `robot_mode: "Reflex"`, run `recover-only` first, then re-run `read-only`, then `tiny-motion`.

## Notes on Motion Policy

- Motion is single-shot and non-oscillatory.
- The target is computed from the current measured state.
- Time scaling uses a smooth quintic profile (`C2` at start/end), so velocity and acceleration start at zero and end at zero.
- No torque control and no external teleop loop in this milestone.
- The control callback performs only arithmetic; no logging, sleeping, blocking I/O, or dynamic allocation.
