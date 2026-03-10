#pragma once

#include "common_types.h"

namespace teleop {

class TeleopMapper {
 public:
  explicit TeleopMapper(const TeleopBridgeConfig& config);

  void Reset();

  bool ComputeTargetPose(const Pose& current_robot_pose,
                         const XRCommand& xr_cmd,
                         bool teleop_active,
                         ControlMode control_mode,
                         Pose* mapped_target_pose,
                         TeleopAction* requested_action);

 private:
  std::array<double, 3> RotateXrVectorToRobot(const std::array<double, 3>& value) const;

  TeleopBridgeConfig config_{};
  bool anchor_initialized_ = false;
  Pose anchor_robot_pose_{};
  Pose anchor_xr_pose_{};
};

}  // namespace teleop
