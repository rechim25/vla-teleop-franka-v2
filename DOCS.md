The architecture is as follows:
 
 
1. XR publisher: sends VR controller poses, rate varies
 
2. Planner thread (low-frequency):
- safety filter: checks if packet is timed-out, clamps to max translation and orientation delta.
- first order position filtering
- orientation filtering (slerp)
- translation and orientation deadband to ignore small input motion
- The output of this is mapped into robot frame and scaled
- IK solver
- JointSmoother: smooths IK output with first order velocity blend + vel/acc/step limits
 
3. 1khz thread:
 
- first order filtering (NEW)
- some clamping mechanism to make sure delta not too large

