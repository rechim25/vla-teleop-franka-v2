# Architecture Overview

You now have a 4-stage pipeline:

1. Quest input
- Quest app sends right-controller pose and buttons through XRoboToolkit PC Service.
- We use `right_grip` as the “move enabled” trigger and `right_trigger` for gripper.

2. Intent builder (safe target generation)
- When you first squeeze grip, the system “anchors” current robot pose and current controller pose.
- After that, hand motion is interpreted as relative motion from that anchor (not absolute world jumps).
- It applies the XR-to-robot frame mapping and scale factor.
- If grip is released, anchoring resets so re-engage starts from the robot’s current pose.

3. Planner (non-realtime)
- Runs in a normal thread (~100 Hz).
- Takes latest XR command + latest robot state.
- Produces a small next joint target using IK-like math with safety shaping:
  - position-only or full pose mode
  - singularity-aware damping
  - nullspace bias toward home posture
  - workspace/jump/timeout checks
- Publishes the newest joint target + status flags.

4. Robot servo loop (hard realtime)
- Runs at robot control rate (1 kHz).
- Reads latest planner target and sends smooth `JointPositions` commands.
- If target is stale, invalid, or motion disabled, it holds position.
- Observation stream publishes current state, mode, teleop state, and safety flags.

Startup and gripper:
- On startup it connects robot, tries gripper, recovers reflex if needed, moves to configured home pose, then starts teleop.
- Gripper runs in its own non-realtime loop: maps trigger to width, rate-limits commands, and avoids tiny command spam.

That’s the main design: Quest expresses operator intent, planner turns intent into safe incremental robot goals, realtime loop executes smoothly and safely.