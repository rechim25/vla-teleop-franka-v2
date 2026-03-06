#include "safety.h"

#include <algorithm>
#include <cmath>

namespace teleop {
namespace {

std::array<double, 4> NormalizeQuat(const std::array<double, 4>& q) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

std::array<double, 4> QuatConjugate(const std::array<double, 4>& q) {
  return {-q[0], -q[1], -q[2], q[3]};
}

std::array<double, 4> QuatMultiply(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  return {
      a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
      a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
      a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
      a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  };
}

std::array<double, 3> QuatToRotVec(const std::array<double, 4>& q_in) {
  const auto q = NormalizeQuat(q_in);
  const double w = std::clamp(q[3], -1.0, 1.0);
  const double angle = 2.0 * std::acos(w);
  const double s = std::sqrt(std::max(1.0 - w * w, 0.0));
  if (s < 1e-8) {
    return {0.0, 0.0, 0.0};
  }
  return {(q[0] / s) * angle, (q[1] / s) * angle, (q[2] / s) * angle};
}

double Norm3(const std::array<double, 3>& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::array<double, 3> ClampNorm3(const std::array<double, 3>& v, double max_norm) {
  const double n = Norm3(v);
  if (n <= max_norm || n < 1e-12) {
    return v;
  }
  const double s = max_norm / n;
  return {v[0] * s, v[1] * s, v[2] * s};
}

}  // namespace

SafetyFilter::SafetyFilter(const SafetyLimits& limits) : limits_(limits) {}

void SafetyFilter::Reset() {
  prev_action_ = TeleopAction{};
}

bool SafetyFilter::IsStreamHealthy(double packet_age_s) const {
  return packet_age_s >= 0.0 && packet_age_s <= limits_.packet_timeout_s;
}

std::array<double, 3> SafetyFilter::ClampWorkspace(const std::array<double, 3>& p, bool* clamped) const {
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

TeleopAction SafetyFilter::ComputeSafeAction(const Pose& current_pose,
                                             const Pose& desired_pose,
                                             double desired_gripper,
                                             double dt_s,
                                             double packet_age_s,
                                             FaultFlags* faults) {
  TeleopAction out{};
  out.gripper_command = std::clamp(desired_gripper, 0.0, 1.0);

  if (!IsStreamHealthy(packet_age_s)) {
    faults->packet_timeout = true;
    prev_action_ = TeleopAction{};
    return out;
  }

  bool workspace_clamped = false;
  const std::array<double, 3> safe_target = ClampWorkspace(desired_pose.p, &workspace_clamped);
  faults->workspace_clamped = workspace_clamped;

  std::array<double, 3> raw_translation = {
      safe_target[0] - current_pose.p[0],
      safe_target[1] - current_pose.p[1],
      safe_target[2] - current_pose.p[2],
  };

  const std::array<double, 4> q_err = QuatMultiply(NormalizeQuat(desired_pose.q),
                                                   QuatConjugate(NormalizeQuat(current_pose.q)));
  std::array<double, 3> raw_rotvec = QuatToRotVec(q_err);

  if (Norm3(raw_translation) > limits_.jump_reject_translation_m ||
      Norm3(raw_rotvec) > limits_.jump_reject_rotation_rad) {
    faults->jump_rejected = true;
    prev_action_ = TeleopAction{};
    return out;
  }

  const double max_trans_step =
      std::min(limits_.max_step_translation_m, limits_.max_translation_speed_mps * dt_s);
  const double max_rot_step =
      std::min(limits_.max_step_rotation_rad, limits_.max_rotation_speed_rps * dt_s);

  raw_translation = ClampNorm3(raw_translation, max_trans_step);
  raw_rotvec = ClampNorm3(raw_rotvec, max_rot_step);

  constexpr double kAlpha = 0.25;
  for (size_t i = 0; i < 3; ++i) {
    out.delta_translation_m[i] =
        (1.0 - kAlpha) * prev_action_.delta_translation_m[i] + kAlpha * raw_translation[i];
    out.delta_rotation_rad[i] =
        (1.0 - kAlpha) * prev_action_.delta_rotation_rad[i] + kAlpha * raw_rotvec[i];
  }

  prev_action_ = out;
  return out;
}

}  // namespace teleop
