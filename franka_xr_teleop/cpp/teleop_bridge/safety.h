#pragma once

#include "common_types.h"

namespace teleop {

class SafetyFilter {
 public:
  SafetyFilter(const SafetyLimits& limits, double planner_rate_hz);

  bool IsStreamHealthy(double packet_age_s) const;

  bool FilterTargetPose(const Pose& current_pose,
                        const Pose& desired_pose,
                        double packet_age_s,
                        FaultFlags* faults,
                        Pose* safe_pose) const;

 private:
  double ClampLinearStep() const;
  double ClampAngularStep() const;
  std::array<double, 3> ClampWorkspace(const std::array<double, 3>& p, bool* clamped) const;

  SafetyLimits limits_{};
  double planner_dt_s_ = 0.01;
};

}  // namespace teleop
