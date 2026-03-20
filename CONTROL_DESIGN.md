## Control Path Q&A

### Q: How does it work now compared to how it worked previously? Step by step with equations where needed

Previously, the control path had multiple smoothing stages:

1. XR pose was low-pass filtered:

   $$
   p_f[k] = (1-\alpha_{xr})\,p_f[k-1] + \alpha_{xr}\,p_{raw}[k]
   $$

2. The filtered XR pose was mapped to a Cartesian TCP target with deadband and scaling:

   $$
   \Delta x_{robot} = s \, R_{xr\to robot}\, \mathrm{deadband}(x_{xr}-x_{anchor})
   $$

3. The planner solved one IK step:

   $$
   e =
   \begin{bmatrix}
   K_p^{pos}\,\mathrm{deadband}(p_{des}-p_d) \\
   K_p^{rot}\,\mathrm{deadband}(r_{des}-r_d)
   \end{bmatrix}
   $$

   $$
   dq = J^\top (J J^\top + \lambda^2 I)^{-1} e + k_{ns} N (q_{home} - q_d)
   $$

   $$
   q_{ik} = q_d + \mathrm{clamp}(dq\,dt_p,\ \pm q_{step,max})
   $$

4. The planner then smoothed the joint target again:

   $$
   v_{raw} = \frac{q_{ik}-q_{prev}}{dt_p}
   $$

   $$
   v_{blend} = \alpha_p v_{raw} + (1-\alpha_p) v_{prev}
   $$

5. The 1 kHz RT loop then smoothed the planned target yet again:

   $$
   \Delta q_{target} = q_{planned} - q_d
   $$

   $$
   \Delta q_{filtered} = \alpha_{rt}\,\Delta q_{target}
   $$

   $$
   q_{cmd} = q_d + \Delta q_{cmd}
   $$

That meant the Panda was repeatedly chasing tiny changing position targets, which can create audible buzzing under joint impedance.

Now, the front half is unchanged:

1. XR pose is filtered and mapped to a TCP target.
2. The planner still solves one IK step and computes `q_target`.
3. The planner publishes that raw joint goal directly:

   $$
   q_{goal} \leftarrow q_{ik}
   $$

The difference is entirely in execution. The 1 kHz RT loop now owns one stateful trajectory generator with internal state:

- `q_ref`
- `dq_ref`
- `ddq_ref`

At each RT step:

1. If teleop is inactive, it resets and holds.
2. It checks a hold band:

   $$
   e_{hold,i} = q_{goal,i} - q_{ref,i}
   $$

   If all joints satisfy

   $$
   |e_{hold,i}| \le \epsilon_q
   \quad\text{and}\quad
   |dq_{ref,i}| \le \epsilon_v
   $$

   then it enters hold:

   $$
   dq_{ref}=0,\quad ddq_{ref}=0,\quad q_{cmd}=q_{ref}
   $$

3. Otherwise it runs a PD-like acceleration servo with jerk limiting:

   $$
   e_i = q_{goal,i} - q_{ref,i}
   $$

   $$
   ddq_{des,i} = \mathrm{clamp}(K_p e_i - K_d dq_{ref,i},\ \pm a_{max})
   $$

   $$
   ddq_{ref,i}^{+} = ddq_{ref,i} + \mathrm{clamp}(ddq_{des,i}-ddq_{ref,i},\ \pm j_{max} dt)
   $$

   $$
   dq_{ref,i}^{+} = \mathrm{clamp}(dq_{ref,i} + ddq_{ref,i}^{+} dt,\ \pm v_{max})
   $$

   $$
   q_{ref,i}^{+} = q_{ref,i} + \mathrm{clamp}(dq_{ref,i}^{+} dt,\ \pm q_{step,max})
   $$

   and finally:

   $$
   q_{cmd} = q_{ref}^{+}
   $$

So previously the command was effectively a stream of tiny filtered position nudges, while now it is a single continuous RT-generated reference trajectory with bounded velocity, acceleration, and jerk.

### Q: where is ik done now?

IK is still done in the planner thread, not in the 1 kHz RT loop.

The actual IK solve is in `SolveIkStep(...)` in `cpp/teleop_bridge/franka_controller.cpp`. That function takes the desired TCP pose, current robot commanded state, and Jacobian, and computes the next joint goal `q_target`.

It is called from the planner loop, and the result is published as `planned.target_q`.

What changed is only what happens after IK:

- Before: planner IK output was smoothed in the planner, then smoothed again in RT.
- Now: planner IK output becomes the raw joint goal, and the RT loop only runs the trajectory generator.

So the split is:

- IK: planner thread
- Smooth execution: RT trajectory generator
- No IK in the RT loop

### Q: I tested and it is still very slow. Investigate why it is still so slow without changing code

The main reason it is still slow is that the RT servo is no longer the dominant limiter. The planner is.

The key bottleneck is that the planner still computes only one clipped IK step per planner tick:

$$
q_{goal} = q_d + \mathrm{clamp}(dq \, dt_p,\ \pm \min(q_{step,max}, v_{max} dt_p))
$$

With the current tuned values:

- `planner_rate_hz = 100`, so

  $$
  dt_p = 0.01 \text{ s}
  $$

- `max_joint_velocity_radps = 0.4`
- `max_joint_step_rad = 0.0048`

That means the planner-side joint goal can move ahead by at most:

$$
\min(0.0048,\ 0.4 \cdot 0.01) = 0.004 \text{ rad per planner tick}
$$

So the planner is still only giving the RT loop a local goal about `4 mrad` ahead every `10 ms`, which is a planner-side speed cap of `0.4 rad/s`.

The RT trajectory generator then computes:

$$
ddq_{des} = \mathrm{clamp}(K_p (q_{goal} - q_{ref}) - K_d dq_{ref},\ \pm a_{max})
$$

If the planner only gives it a gap of about `0.004 rad`, and `K_p = 80`, then the largest acceleration that the RT servo can demand from that position error is:

$$
ddq_{des,max} \approx 80 \cdot 0.004 = 0.32 \text{ rad/s}^2
$$

That is far below the configured acceleration limit of `3.0 rad/s^2`. So even though the RT limits were increased, the RT loop never gets a large enough position error to use them fully.

This means the dominant bottleneck is not the RT generator any more. It is the planner still feeding a very local, conservative joint goal.

There are also secondary contributors:

- `scale_factor = 0.5` still halves the operator motion before IK.
- `xr_pose_lowpass_alpha = 0.45` and `xr_translation_deadband_m = 0.0025` still make input conservative.
- `position_gain = 2.1` and `task_translation_deadband_m = 0.0018` still keep Cartesian IK error modest.

So the current diagnosis is:

1. The RT servo has enough headroom.
2. The planner still advances `q_goal` only one small clipped step per tick.
3. That keeps the RT position error small.
4. Small RT position error means small commanded acceleration.
5. Mapping gain and deadbands make the overall teleop feel even slower.
