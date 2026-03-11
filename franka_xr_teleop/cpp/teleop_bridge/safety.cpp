#include "safety.h"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>

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
                                    double dt_s,
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

  double max_translation_delta_m = std::numeric_limits<double>::infinity();
  if (limits_.max_step_translation_m > 0.0) {
    max_translation_delta_m = std::min(max_translation_delta_m, limits_.max_step_translation_m);
  }
  if (limits_.max_translation_speed_mps > 0.0 && dt_s > 0.0) {
    max_translation_delta_m =
        std::min(max_translation_delta_m, limits_.max_translation_speed_mps * dt_s);
  }
  if (!std::isfinite(max_translation_delta_m)) {
    max_translation_delta_m = position_error.norm();
  }

  double max_rotation_delta_rad = std::numeric_limits<double>::infinity();
  if (limits_.max_step_rotation_rad > 0.0) {
    max_rotation_delta_rad = std::min(max_rotation_delta_rad, limits_.max_step_rotation_rad);
  }
  if (limits_.max_rotation_speed_rps > 0.0 && dt_s > 0.0) {
    max_rotation_delta_rad = std::min(max_rotation_delta_rad, limits_.max_rotation_speed_rps * dt_s);
  }
  if (!std::isfinite(max_rotation_delta_rad)) {
    max_rotation_delta_rad = rotation_error.norm();
  }

  Eigen::Vector3d bounded_translation = position_error;
  const double position_norm = position_error.norm();
  if (position_norm > max_translation_delta_m && max_translation_delta_m > 0.0) {
    bounded_translation *= (max_translation_delta_m / position_norm);
  }
  safe_pose->p = ToArray3(ToEigen(current_pose.p) + bounded_translation);

  Eigen::Vector3d bounded_rotation = rotation_error;
  const double rotation_norm = rotation_error.norm();
  if (rotation_norm > max_rotation_delta_rad && max_rotation_delta_rad > 0.0) {
    bounded_rotation *= (max_rotation_delta_rad / rotation_norm);
  }

  const double bounded_angle = bounded_rotation.norm();
  Eigen::Quaterniond bounded_delta_q = Eigen::Quaterniond::Identity();
  if (bounded_angle > 1e-12) {
    bounded_delta_q = Eigen::AngleAxisd(bounded_angle, bounded_rotation / bounded_angle);
  }
  safe_pose->q = ToArrayQuat(bounded_delta_q * ToEigenQuat(current_pose.q));

  const Eigen::Vector3d shaped_position_error = ToEigen(safe_pose->p) - ToEigen(current_pose.p);
  const Eigen::Vector3d shaped_rotation_error =
      QuaternionErrorAngleAxis(ToEigenQuat(current_pose.q), ToEigenQuat(safe_pose->q));
  if (shaped_position_error.norm() > limits_.jump_reject_translation_m ||
      shaped_rotation_error.norm() > limits_.jump_reject_rotation_rad) {
    faults->jump_rejected = true;
    *safe_pose = current_pose;
    return false;
  }

  return true;
}

}  // namespace teleop
