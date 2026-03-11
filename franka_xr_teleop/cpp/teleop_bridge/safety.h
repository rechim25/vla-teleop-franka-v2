#pragma once

#include "common_types.h"

namespace teleop {

class SafetyFilter {
 public:
  explicit SafetyFilter(const SafetyLimits& limits);

  bool IsStreamHealthy(double packet_age_s) const;

  bool FilterTargetPose(const Pose& current_pose,
                        const Pose& desired_pose,
                        double packet_age_s,
                        double dt_s,
                        FaultFlags* faults,
                        Pose* safe_pose) const;

 private:
  std::array<double, 3> ClampWorkspace(const std::array<double, 3>& p, bool* clamped) const;

  SafetyLimits limits_{};
};

}  // namespace teleop
