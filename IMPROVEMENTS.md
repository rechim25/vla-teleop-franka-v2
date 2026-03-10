# Franka XR Teleop Improvements

Branch: `fix/improve-franka-teleop-1`

This document summarizes the smoothing and stability changes made to the Franka libfranka teleop path to better match the behavior quality of XRoboToolkit sample implementations.

## 1) Nullspace Home-Pull Gating Near Setpoint

Problem addressed:
- The IK solver always added a nullspace term toward `q_home`, even when the end-effector target was already reached.
- This can create continuous micro-joint motion (audible grinding/hunting) at hold.

Changes:
- Added IK deadbands for tiny Cartesian/orientation errors.
- Scaled nullspace gain by task error magnitude so nullspace pull fades out near setpoint.

Code:
- `franka_xr_teleop/cpp/teleop_bridge/franka_controller.cpp` (`SolveIkStep`)
- `franka_xr_teleop/cpp/teleop_bridge/common_types.h` (`IkConfig`)
- `franka_xr_teleop/cpp/teleop_bridge/config_loader.cpp` (new config parsing/validation)
- `franka_xr_teleop/configs/teleop.yaml` (new IK parameters)

New IK parameters:
- `ik.nullspace_activation_position_error_m`
- `ik.nullspace_activation_orientation_error_rad`
- `ik.position_error_deadband_m`
- `ik.orientation_error_deadband_rad`

## 2) XR Target Deadband + Low-Pass Filtering in Planner

Problem addressed:
- Raw XR controller micro-jitter propagated directly into Cartesian target updates.
- No arm-level filtering/deadband before IK.

Changes:
- Added deadman trigger hysteresis (press/release thresholds).
- Added low-pass filtering and deadband on desired Cartesian pose before IK solve.
- Observation `executed_action` is now derived from filtered target (what we actually command).

Code:
- `franka_xr_teleop/cpp/teleop_bridge/franka_controller.cpp` (`PlannerLoop`)
- `franka_xr_teleop/cpp/teleop_bridge/common_types.h` (`TeleopRuntimeConfig`)
- `franka_xr_teleop/cpp/teleop_bridge/config_loader.cpp` (new config parsing/validation)
- `franka_xr_teleop/configs/teleop.yaml` (new teleop parameters)

New teleop parameters:
- `control_trigger_release_threshold`
- `target_pose_filter_alpha`
- `target_position_deadband_m`
- `target_orientation_deadband_rad`

## 3) Safety Speed/Step Limits Wired and Enforced

Problem addressed:
- `safety.yaml` had translational/rotational speed/step limits, but runtime config and safety filter did not use them.

Changes:
- Added these fields to `SafetyLimits` runtime config.
- Loaded/validated them from YAML.
- Enforced per-cycle Cartesian translation and rotation limits in `SafetyFilter::FilterTargetPose`.

Code:
- `franka_xr_teleop/cpp/teleop_bridge/common_types.h` (`SafetyLimits`)
- `franka_xr_teleop/cpp/teleop_bridge/config_loader.cpp` (`LoadSafetyConfig` + validation)
- `franka_xr_teleop/cpp/teleop_bridge/safety.h` (updated API)
- `franka_xr_teleop/cpp/teleop_bridge/safety.cpp` (rate/step limiting implementation)

Used safety parameters (now active):
- `max_translation_speed_mps`
- `max_rotation_speed_rps`
- `max_step_translation_m`
- `max_step_rotation_rad`

## 4) Planner-to-Servo Interpolation at 1 kHz

Problem addressed:
- Planner runs at ~100 Hz, servo callback at 1 kHz.
- Without interpolation, servo chases staircase-like target updates.

Changes:
- Added in-callback interpolation between planner targets over one planner horizon.
- Keeps 1 kHz output smoother while still respecting joint velocity/step clamps.

Code:
- `franka_xr_teleop/cpp/teleop_bridge/franka_controller.cpp` (inside `robot.control(...)` callback)

## Build Verification

The updated code builds successfully:

```bash
cmake -S franka_xr_teleop -B franka_xr_teleop/build -DCMAKE_BUILD_TYPE=Release
cmake --build franka_xr_teleop/build -j"$(nproc)"
```
