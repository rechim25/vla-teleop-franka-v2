#include "safety.h"

#include <algorithm>
#include <cmath>

#include "math_utils.h"

namespace teleop {

SafetyFilter::SafetyFilter(const SafetyLimits& limits, double planner_rate_hz)
    : limits_(limits), planner_dt_s_(1.0 / std::max(planner_rate_hz, 1.0)) {}

double SafetyFilter::ClampLinearStep() const {
  const double by_speed = limits_.max_translation_speed_mps * planner_dt_s_;
  return std::min(limits_.max_step_translation_m, by_speed);
}

double SafetyFilter::ClampAngularStep() const {
  const double by_speed = limits_.max_rotation_speed_rps * planner_dt_s_;
  return std::min(limits_.max_step_rotation_rad, by_speed);
}

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

  const double max_translation_step = ClampLinearStep();
  if (position_error.norm() > max_translation_step && max_translation_step > 1e-12) {
    const Eigen::Vector3d clamped_error =
        position_error * (max_translation_step / position_error.norm());
    safe_pose->p = ToArray3(ToEigen(current_pose.p) + clamped_error);
  }

  const double max_rotation_step = ClampAngularStep();
  const double rotation_norm = rotation_error.norm();
  if (rotation_norm > max_rotation_step && max_rotation_step > 1e-12) {
    const Eigen::Vector3d axis = rotation_error / rotation_norm;
    const Eigen::Quaterniond q_current = ToEigenQuat(current_pose.q);
    const Eigen::Quaterniond q_step = Eigen::Quaterniond(Eigen::AngleAxisd(max_rotation_step, axis));
    safe_pose->q = ToArrayQuat(q_step * q_current);
  }

  return true;
}

}  // namespace teleop
