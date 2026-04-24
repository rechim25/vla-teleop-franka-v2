#pragma once

#include "common_types.h"

namespace teleop {

class TeleopMapper {
 public:
  explicit TeleopMapper(const TeleopBridgeConfig& config);

  void Reset();
  void Reanchor(const Pose& target_robot_pose, const XRCommand& xr_cmd);

  bool ComputeTargetPose(const Pose& current_robot_pose,
                         const XRCommand& xr_cmd,
                         bool teleop_active,
                         ControlMode control_mode,
                         Pose* mapped_target_pose,
                         TeleopAction* requested_action);

 private:
  void ComputeMappedPoseFromAnchors(const Pose& current_robot_pose,
                                    const Pose& xr_filtered_pose,
                                    ControlMode control_mode,
                                    Pose* mapped_target_pose,
                                    TeleopAction* requested_action) const;
  void SetFrozenOutput(const XRCommand& xr_cmd,
                       Pose* mapped_target_pose,
                       TeleopAction* requested_action) const;
  bool HoldFreezeEnabled(ControlMode control_mode) const;
  Pose FilterXrPose(const Pose& raw_pose);
  static std::array<double, 3> ApplyVectorDeadband(const std::array<double, 3>& value,
                                                    double deadband);
  std::array<double, 3> RotateXrTranslationToRobot(const std::array<double, 3>& value) const;
  std::array<double, 3> RotateXrRotationToRobot(const std::array<double, 3>& value) const;

  TeleopBridgeConfig config_{};
  bool anchor_initialized_ = false;
  bool xr_filter_initialized_ = false;
  bool xr_motion_initialized_ = false;
  bool hold_frozen_ = false;
  uint64_t last_xr_sequence_id_ = 0;
  uint64_t still_since_ns_ = 0;
  Pose anchor_robot_pose_{};
  Pose anchor_xr_pose_{};
  Pose filtered_xr_pose_{};
  Pose last_motion_xr_pose_{};
  Pose frozen_target_pose_{};
};

}  // namespace teleop
