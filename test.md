1khz thread.

PREVIOUSLY:

1. Compute joint error: `target_delta = planned.target_q - state.q_d`
2. Apply small deadzone
3. Scale by RT gain: `filtered_delta = rt_alpha * target_delta`
4. Clamp directly to the max per-cycle step: `command_delta = clamp(filtered_delta, -max_step, +max_step)`
5. Send: `q_cmd = state.q_d + command_delta`
Problem with that:

NEW:

1. Compute joint error: `target_delta = planned.target_q - state.q_d`
2. Apply small deadzone
3. Apply RT gain: `filtered_delta = rt_alpha * target_delta`
4. Convert that into a desired joint velocity: `desired_velocity = filtered_delta / dt`
5. Limit that velocity by max joint velocity
6. Limit how much velocity is allowed to change from the previous cycle using:
`max_joint_acceleration_radps2 * dt`
7. Convert the limited velocity back into a step: `unclamped_delta = accel_limited_velocity * dt`
8. Apply the final hard step clamp. Send: `q_cmd = state.q_d + command_delta`