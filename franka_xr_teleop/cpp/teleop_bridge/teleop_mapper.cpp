#include "teleop_mapper.h"

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

std::array<double, 4> RotVecToQuat(const std::array<double, 3>& r) {
  const double angle = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  if (angle < 1e-8) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  const double half = 0.5 * angle;
  const double s = std::sin(half) / angle;
  return {r[0] * s, r[1] * s, r[2] * s, std::cos(half)};
}

}  // namespace

TeleopMapper::TeleopMapper(const TeleopBridgeConfig& config) : config_(config) {}

void TeleopMapper::Reset(const Pose& robot_pose, const XRCommand& xr_cmd) {
  initialized_ = true;
  anchor_robot_pose_ = robot_pose;
  anchor_xr_pos_ = xr_cmd.target_position_xyz;
  anchor_xr_quat_ = NormalizeQuat(xr_cmd.target_orientation_xyzw);
  last_clutch_pressed_ = xr_cmd.clutch_pressed;
}

std::array<double, 3> TeleopMapper::RotateXrVectorToRobot(const std::array<double, 3>& v) const {
  std::array<double, 3> out{};
  for (size_t i = 0; i < 3; ++i) {
    out[i] = config_.xr_to_robot_rotation[i][0] * v[0] +
             config_.xr_to_robot_rotation[i][1] * v[1] +
             config_.xr_to_robot_rotation[i][2] * v[2];
  }
  return out;
}

bool TeleopMapper::MapToTarget(const Pose& current_robot_pose,
                               const XRCommand& xr_cmd,
                               bool teleop_active,
                               bool clutch_pressed,
                               Pose* mapped_target_pose,
                               TeleopAction* raw_action) {
  if (!initialized_) {
    Reset(current_robot_pose, xr_cmd);
  }

  if (!teleop_active) {
    Reset(current_robot_pose, xr_cmd);
    return false;
  }

  if (clutch_pressed) {
    Reset(current_robot_pose, xr_cmd);
    return false;
  }

  if (last_clutch_pressed_ && !clutch_pressed) {
    // Clutch released: re-anchor from current state to avoid jump.
    Reset(current_robot_pose, xr_cmd);
  }
  last_clutch_pressed_ = clutch_pressed;

  const std::array<double, 3> xr_delta = {
      xr_cmd.target_position_xyz[0] - anchor_xr_pos_[0],
      xr_cmd.target_position_xyz[1] - anchor_xr_pos_[1],
      xr_cmd.target_position_xyz[2] - anchor_xr_pos_[2],
  };

  const std::array<double, 3> robot_delta = RotateXrVectorToRobot(xr_delta);

  const std::array<double, 4> q_xr = NormalizeQuat(xr_cmd.target_orientation_xyzw);
  const std::array<double, 4> q_anchor_xr = NormalizeQuat(anchor_xr_quat_);
  const std::array<double, 4> q_delta_xr = NormalizeQuat(QuatMultiply(q_xr, QuatConjugate(q_anchor_xr)));

  const std::array<double, 3> rotvec_xr = QuatToRotVec(q_delta_xr);
  const std::array<double, 3> rotvec_robot = RotateXrVectorToRobot(rotvec_xr);
  const std::array<double, 4> q_delta_robot = NormalizeQuat(RotVecToQuat(rotvec_robot));

  mapped_target_pose->p = {
      anchor_robot_pose_.p[0] + robot_delta[0],
      anchor_robot_pose_.p[1] + robot_delta[1],
      anchor_robot_pose_.p[2] + robot_delta[2],
  };
  mapped_target_pose->q = NormalizeQuat(QuatMultiply(q_delta_robot, anchor_robot_pose_.q));

  raw_action->delta_translation_m = robot_delta;
  raw_action->delta_rotation_rad = rotvec_robot;
  raw_action->gripper_command = std::clamp(xr_cmd.gripper_command, 0.0, 1.0);
  return true;
}

}  // namespace teleop
