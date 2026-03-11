```text
Quest Controller
  |
  v
XRoboToolkit PC Service SDK callback (PXREADeviceStateJson)
[xrobotics_source.cpp]
  - parse right.pose: x,y,z,qx,qy,qz,qw
  - parse grip/trigger/buttons
  - stamp local monotonic timestamp_ns
  |
  v
command_buffer (LatestValueBuffer<XRCommand>)
[common_types.h]
  - 2-slot lock-free latest sample (overwrite old)

Franka 1 kHz control callback (robot.control)
[franka_controller.cpp]
  - read libfranka RobotState
  - convert to RobotSnapshot
  |
  v
robot_state_buffer (LatestValueBuffer<RobotSnapshot>)
[common_types.h]
  - same 2-slot latest sample handoff
```

```text
Planner Thread (~100 Hz, PlannerLoop)
[franka_controller.cpp]
  reads:
    XRCommand   <- command_buffer.ReadLatest()
    RobotSnapshot <- robot_state_buffer.ReadLatest()

  flow:
    1) TeleopStateMachine gating
       (stream healthy + deadman + robot ok)
    2) TeleopMapper
       - anchor on activation
       - xr_delta_pos = current_xr_pos - anchor_xr_pos
       - robot_delta_pos = R_xr_to_robot * xr_delta_pos * scale
       - orientation delta via quaternion error (angle-axis), rotated to robot frame
       -> desired robot TCP pose
    3) SafetyFilter
       - packet timeout
       - workspace clamp
       - jump reject
    4) IK solve (DLS + nullspace/manipulability)
       -> raw q_target
    5) JointTargetSmoother
       -> smoothed q_target
  publishes:
    PlannedTarget -> planned_target_buffer
```

```text
Franka 1 kHz callback (execution)
[franka_controller.cpp]
  reads:
    PlannedTarget <- planned_target_buffer.ReadLatest()

  if motion allowed + active + fresh:
    q_cmd = step from state.q_d toward planned.target_q
            with per-cycle clamp
  else:
    q_cmd = state.q_d (hold)

  returns q_cmd to libfranka
  also publishes RobotObservation (for UDP output thread)
```

Key property:
- Buffers are **latest-value**, not queues.  
- Planner and 1 kHz loop are decoupled: planner decides target; 1 kHz loop safely executes it at servo rate.