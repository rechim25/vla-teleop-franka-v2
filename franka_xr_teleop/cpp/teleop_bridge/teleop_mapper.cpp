#include "teleop_mapper.h"

#include <algorithm>

#include <Eigen/Geometry>

#include "math_utils.h"

namespace teleop {

TeleopMapper::TeleopMapper(const TeleopBridgeConfig& config) : config_(config) {}

void TeleopMapper::Reset() {
  anchor_initialized_ = false;
  xr_filter_initialized_ = false;
}

std::array<double, 3> TeleopMapper::ApplyVectorDeadband(const std::array<double, 3>& value,
                                                        double deadband) {
  const Eigen::Vector3d v = ToEigen(value);
  const double norm = v.norm();
  if (norm <= deadband || norm <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }
  const double scaled_norm = norm - deadband;
  return ToArray3(v * (scaled_norm / norm));
}

Pose TeleopMapper::FilterXrPose(const Pose& raw_pose) {
  const double alpha = std::clamp(config_.teleop.xr_pose_lowpass_alpha, 0.0, 1.0);
  if (!xr_filter_initialized_) {
    filtered_xr_pose_ = raw_pose;
    xr_filter_initialized_ = true;
    return filtered_xr_pose_;
  }

  for (size_t i = 0; i < 3; ++i) {
    filtered_xr_pose_.p[i] =
        (1.0 - alpha) * filtered_xr_pose_.p[i] + alpha * raw_pose.p[i];
  }

  const Eigen::Quaterniond q_prev = ToEigenQuat(filtered_xr_pose_.q);
  const Eigen::Quaterniond q_raw = ToEigenQuat(raw_pose.q);
  filtered_xr_pose_.q = ToArrayQuat(q_prev.slerp(alpha, q_raw));
  return filtered_xr_pose_;
}

std::array<double, 3> TeleopMapper::RotateXrVectorToRobot(const std::array<double, 3>& value) const {
  std::array<double, 3> out{};
  for (size_t row = 0; row < 3; ++row) {
    out[row] = config_.xr_to_robot_rotation[row][0] * value[0] +
               config_.xr_to_robot_rotation[row][1] * value[1] +
               config_.xr_to_robot_rotation[row][2] * value[2];
  }
  return out;
}

bool TeleopMapper::ComputeTargetPose(const Pose& current_robot_pose,
                                     const XRCommand& xr_cmd,
                                     bool teleop_active,
                                     ControlMode control_mode,
                                     Pose* mapped_target_pose,
                                     TeleopAction* requested_action) {
  if (!teleop_active || control_mode == ControlMode::kHold) {
    Reset();
    return false;
  }

  const Pose xr_filtered_pose = FilterXrPose(xr_cmd.right_controller_pose);

  if (!anchor_initialized_) {
    anchor_robot_pose_ = current_robot_pose;
    anchor_xr_pose_ = xr_filtered_pose;
    anchor_initialized_ = true;
    return false;
  }

  const std::array<double, 3> xr_delta_raw = {
      xr_filtered_pose.p[0] - anchor_xr_pose_.p[0],
      xr_filtered_pose.p[1] - anchor_xr_pose_.p[1],
      xr_filtered_pose.p[2] - anchor_xr_pose_.p[2],
  };
  const std::array<double, 3> xr_delta =
      ApplyVectorDeadband(xr_delta_raw, config_.teleop.xr_translation_deadband_m);
  std::array<double, 3> robot_delta = RotateXrVectorToRobot(xr_delta);
  for (double& value : robot_delta) {
    value *= config_.teleop.scale_factor;
  }

  mapped_target_pose->p = {
      anchor_robot_pose_.p[0] + robot_delta[0],
      anchor_robot_pose_.p[1] + robot_delta[1],
      anchor_robot_pose_.p[2] + robot_delta[2],
  };

  requested_action->delta_translation_m = robot_delta;
  requested_action->gripper_command = Clamp01(xr_cmd.gripper_trigger_value);

  if (control_mode == ControlMode::kPosition) {
    mapped_target_pose->q = anchor_robot_pose_.q;
    requested_action->delta_rotation_rad = {0.0, 0.0, 0.0};
    return true;
  }

  const Eigen::Quaterniond xr_current = ToEigenQuat(xr_filtered_pose.q);
  const Eigen::Quaterniond xr_anchor = ToEigenQuat(anchor_xr_pose_.q);
  const Eigen::Vector3d xr_rot_delta_raw = QuaternionErrorAngleAxis(xr_anchor, xr_current);
  const std::array<double, 3> xr_rot_array =
      ApplyVectorDeadband(ToArray3(xr_rot_delta_raw), config_.teleop.xr_rotation_deadband_rad);
  const std::array<double, 3> robot_rot_array = RotateXrVectorToRobot(xr_rot_array);
  const Eigen::Vector3d robot_rot_delta =
      Eigen::Vector3d(robot_rot_array[0], robot_rot_array[1], robot_rot_array[2]);

  const double angle = robot_rot_delta.norm();
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  if (angle > 1e-9) {
    delta_q = Eigen::AngleAxisd(angle, robot_rot_delta / angle);
  }

  mapped_target_pose->q = ToArrayQuat(delta_q * ToEigenQuat(anchor_robot_pose_.q));
  requested_action->delta_rotation_rad = robot_rot_array;
  return true;
}

}  // namespace teleop
