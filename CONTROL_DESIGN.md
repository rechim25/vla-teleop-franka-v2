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
