#include "safety.h"

#include <cmath>

#include "math_utils.h"

namespace teleop {

SafetyFilter::SafetyFilter(const SafetyLimits& limits) : limits_(limits) {}

bool SafetyFilter::IsStreamHealthy(double packet_age_s) const {
  return packet_age_s >= 0.0 && packet_age_s <= limits_.packet_timeout_s;
}

std::array<double, 3> SafetyFilter::ClampWorkspace(const std::array<double, 3>& p,
                                                   bool* clamped) const {
  std::array<double, 3> out = p;
  for (size_t i = 0; i < 3; ++i) {
    const double before = out[i];
    out[i] = std::clamp(out[i], limits_.workspace_min[i], limits_.workspace_max[i]);
    if (std::abs(out[i] - before) > 1e-12) {
      *clamped = true;
    }
  }
  return out;
}

bool SafetyFilter::FilterTargetPose(const Pose& current_pose,
                                    const Pose& desired_pose,
                                    double packet_age_s,
                                    FaultFlags* faults,
                                    Pose* safe_pose) const {
  if (!IsStreamHealthy(packet_age_s)) {
    faults->packet_timeout = true;
    *safe_pose = current_pose;
    return false;
  }

  bool workspace_clamped = false;
  *safe_pose = desired_pose;
  safe_pose->p = ClampWorkspace(desired_pose.p, &workspace_clamped);
  faults->workspace_clamped = workspace_clamped;

  const Eigen::Vector3d position_error = ToEigen(safe_pose->p) - ToEigen(current_pose.p);
  const Eigen::Vector3d rotation_error =
      QuaternionErrorAngleAxis(ToEigenQuat(current_pose.q), ToEigenQuat(safe_pose->q));
  if (position_error.norm() > limits_.jump_reject_translation_m ||
      rotation_error.norm() > limits_.jump_reject_rotation_rad) {
    faults->jump_rejected = true;
    *safe_pose = current_pose;
    return false;
  }

  return true;
}

}  // namespace teleop
