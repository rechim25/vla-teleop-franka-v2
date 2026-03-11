#include "teleop_mapper.h"

#include <Eigen/Geometry>

#include "math_utils.h"

namespace teleop {

TeleopMapper::TeleopMapper(const TeleopBridgeConfig& config) : config_(config) {}

void TeleopMapper::Reset() {
  anchor_initialized_ = false;
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
                                     bool clutch_pressed,
                                     ControlMode control_mode,
                                     Pose* mapped_target_pose,
                                     TeleopAction* requested_action) {
  if (!teleop_active || control_mode == ControlMode::kHold) {
    Reset();
    return false;
  }

  if (clutch_pressed) {
    anchor_robot_pose_ = current_robot_pose;
    anchor_xr_pose_ = xr_cmd.right_controller_pose;
    anchor_initialized_ = true;
    return false;
  }

  if (!anchor_initialized_) {
    anchor_robot_pose_ = current_robot_pose;
    anchor_xr_pose_ = xr_cmd.right_controller_pose;
    anchor_initialized_ = true;
    return false;
  }

  const std::array<double, 3> xr_delta = {
      xr_cmd.right_controller_pose.p[0] - anchor_xr_pose_.p[0],
      xr_cmd.right_controller_pose.p[1] - anchor_xr_pose_.p[1],
      xr_cmd.right_controller_pose.p[2] - anchor_xr_pose_.p[2],
  };
  std::array<double, 3> robot_delta = RotateXrVectorToRobot(xr_delta);
  for (double& value : robot_delta) {
    value *= config_.teleop.scale_factor;
  }
  const Eigen::Vector3d robot_delta_vec(robot_delta[0], robot_delta[1], robot_delta[2]);
  if (robot_delta_vec.norm() < config_.teleop.translation_deadband_m) {
    robot_delta = {0.0, 0.0, 0.0};
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

  const Eigen::Quaterniond xr_current = ToEigenQuat(xr_cmd.right_controller_pose.q);
  const Eigen::Quaterniond xr_anchor = ToEigenQuat(anchor_xr_pose_.q);
  const Eigen::Vector3d xr_rot_delta = QuaternionErrorAngleAxis(xr_anchor, xr_current);
  const std::array<double, 3> xr_rot_array = ToArray3(xr_rot_delta);
  const std::array<double, 3> robot_rot_array = RotateXrVectorToRobot(xr_rot_array);
  Eigen::Vector3d robot_rot_delta =
      Eigen::Vector3d(robot_rot_array[0], robot_rot_array[1], robot_rot_array[2]);
  if (robot_rot_delta.norm() < config_.teleop.rotation_deadband_rad) {
    robot_rot_delta = Eigen::Vector3d::Zero();
  }

  const double angle = robot_rot_delta.norm();
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  if (angle > 1e-9) {
    delta_q = Eigen::AngleAxisd(angle, robot_rot_delta / angle);
  }

  mapped_target_pose->q = ToArrayQuat(delta_q * ToEigenQuat(anchor_robot_pose_.q));
  requested_action->delta_rotation_rad = {
      robot_rot_delta.x(),
      robot_rot_delta.y(),
      robot_rot_delta.z(),
  };
  return true;
}

}  // namespace teleop
