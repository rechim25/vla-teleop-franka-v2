#include "safety.h"

#include <algorithm>
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

double ClampStepByRate(double max_step, double max_rate, double dt_s) {
  if (max_step <= 0.0) {
    return 0.0;
  }
  if (max_rate <= 0.0 || dt_s <= 0.0) {
    return max_step;
  }
  return std::min(max_step, max_rate * dt_s);
}

bool SafetyFilter::FilterTargetPose(const Pose& current_pose,
                                    const Pose& desired_pose,
                                    double packet_age_s,
                                    double control_dt_s,
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

  const double dt_s = std::max(control_dt_s, 1e-6);

  const double max_translation_step_m = ClampStepByRate(
      limits_.max_step_translation_m, limits_.max_translation_speed_mps, dt_s);
  if (max_translation_step_m <= 0.0) {
    safe_pose->p = current_pose.p;
  } else {
    const Eigen::Vector3d limited_position_error = ToEigen(safe_pose->p) - ToEigen(current_pose.p);
    const double translation_norm = limited_position_error.norm();
    if (translation_norm > max_translation_step_m && translation_norm > 1e-12) {
      safe_pose->p = ToArray3(
          ToEigen(current_pose.p) + limited_position_error * (max_translation_step_m / translation_norm));
    }
  }

  const double max_rotation_step_rad = ClampStepByRate(
      limits_.max_step_rotation_rad, limits_.max_rotation_speed_rps, dt_s);
  if (max_rotation_step_rad <= 0.0) {
    safe_pose->q = current_pose.q;
  } else {
    const Eigen::Vector3d limited_rotation_error =
        QuaternionErrorAngleAxis(ToEigenQuat(current_pose.q), ToEigenQuat(safe_pose->q));
    const double rotation_norm = limited_rotation_error.norm();
    if (rotation_norm > max_rotation_step_rad && rotation_norm > 1e-12) {
      const Eigen::Quaterniond current_q = ToEigenQuat(current_pose.q);
      const Eigen::Quaterniond delta_q = Eigen::Quaterniond(
          Eigen::AngleAxisd(max_rotation_step_rad, limited_rotation_error / rotation_norm));
      safe_pose->q = ToArrayQuat(delta_q * current_q);
    }
  }

  return true;
}

}  // namespace teleop
