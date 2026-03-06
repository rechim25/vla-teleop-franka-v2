#pragma once

#include "common_types.h"

namespace teleop {

class TeleopMapper {
 public:
  explicit TeleopMapper(const TeleopBridgeConfig& config);

  void Reset(const Pose& robot_pose, const XRCommand& xr_cmd);

  // Produces a mapped absolute target pose and the raw operator delta command.
  // Returns false when teleop is not active (caller should hold).
  bool MapToTarget(const Pose& current_robot_pose,
                   const XRCommand& xr_cmd,
                   bool teleop_active,
                   bool clutch_pressed,
                   Pose* mapped_target_pose,
                   TeleopAction* raw_action);

 private:
  std::array<double, 3> RotateXrVectorToRobot(const std::array<double, 3>& v) const;

  TeleopBridgeConfig config_{};
  bool initialized_ = false;
  bool last_clutch_pressed_ = false;

  Pose anchor_robot_pose_{};
  std::array<double, 3> anchor_xr_pos_{};
  std::array<double, 4> anchor_xr_quat_{{0.0, 0.0, 0.0, 1.0}};
};

}  // namespace teleop
