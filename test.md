Before, each cycle did this for each joint:

Compute joint error:
target_delta = planned.target_q - state.q_d
Apply small deadzone
Scale by RT gain:
filtered_delta = rt_alpha * target_delta
Clamp directly to the max per-cycle step:
command_delta = clamp(filtered_delta, -max_step, +max_step)
Send:
q_cmd = state.q_d + command_delta
Problem with that:

command_delta could change abruptly from one 1 kHz cycle to the next
so even tiny target changes could create sharp high-rate micro-corrections
that is what produced the buzz
Now, after the change, each cycle does this:

Compute joint error:
target_delta = planned.target_q - state.q_d
Apply deadzone
Apply RT gain:
filtered_delta = rt_alpha * target_delta
Convert that into a desired joint velocity:
desired_velocity = filtered_delta / dt
Limit that velocity by max joint velocity
Limit how much velocity is allowed to change from the previous cycle using:
max_joint_acceleration_radps2 * dt
Convert the limited velocity back into a step:
unclamped_delta = accel_limited_velocity * dt
Apply the final hard step clamp
Send:
q_cmd = state.q_d + command_delta