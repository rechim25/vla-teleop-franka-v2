#pragma once

#include "common_types.h"

namespace teleop {

class SafetyFilter {
 public:
  explicit SafetyFilter(const SafetyLimits& limits);

  void Reset();
  const SafetyLimits& limits() const { return limits_; }
  bool IsStreamHealthy(double packet_age_s) const;

  TeleopAction ComputeSafeAction(const Pose& current_pose,
                                 const Pose& desired_pose,
                                 double desired_gripper,
                                 double dt_s,
                                 double packet_age_s,
                                 FaultFlags* faults);

 private:
  std::array<double, 3> ClampWorkspace(const std::array<double, 3>& p, bool* clamped) const;

  SafetyLimits limits_{};
  TeleopAction prev_action_{};
};

}  // namespace teleop
