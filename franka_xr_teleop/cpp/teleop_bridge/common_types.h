#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>

namespace teleop {

enum class TeleopState : uint8_t {
  kDisconnected = 0,
  kConnectedIdle = 1,
  kTeleopArmed = 2,
  kTeleopActive = 3,
  kFault = 4,
};

enum class ControlMode : uint8_t {
  kHold = 0,
  kPosition = 1,
  kPose = 2,
};

inline const char* ToString(ControlMode mode) {
  switch (mode) {
    case ControlMode::kHold:
      return "HOLD";
    case ControlMode::kPosition:
      return "POSITION";
    case ControlMode::kPose:
      return "POSE";
  }
  return "UNKNOWN";
}

inline bool ParseControlMode(std::string_view value, ControlMode* mode) {
  if (value == "position") {
    *mode = ControlMode::kPosition;
    return true;
  }
  if (value == "pose") {
    *mode = ControlMode::kPose;
    return true;
  }
  if (value == "hold") {
    *mode = ControlMode::kHold;
    return true;
  }
  return false;
}

struct Pose {
  std::array<double, 3> p{};
  std::array<double, 4> q{{0.0, 0.0, 0.0, 1.0}};  // xyzw
};

struct XRCommand {
  uint64_t timestamp_ns = 0;
  uint64_t sequence_id = 0;
  Pose right_controller_pose{};
  double control_trigger_value = 0.0;
  double gripper_trigger_value = 0.0;
  bool button_a = false;
  bool button_b = false;
  bool right_axis_click = false;
};

struct TeleopAction {
  std::array<double, 3> delta_translation_m{};
  std::array<double, 3> delta_rotation_rad{};  // angle-axis
  double gripper_command = 1.0;                // normalized trigger in [0, 1]
};

struct FaultFlags {
  bool packet_timeout = false;
  bool jump_rejected = false;
  bool workspace_clamped = false;
  bool robot_not_ready = false;
  bool control_exception = false;
  bool ik_rejected = false;
};

struct RobotSnapshot {
  uint64_t timestamp_ns = 0;
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  Pose tcp_pose{};
  std::array<double, 16> F_T_EE{};
  std::array<double, 16> EE_T_K{};
  bool robot_ok = false;
  bool in_reflex = false;
};

struct PlannedTarget {
  uint64_t target_timestamp_ns = 0;
  std::array<double, 7> target_q{};
  Pose desired_tcp_pose{};
  double target_gripper_width_m = 0.0;
  ControlMode control_mode = ControlMode::kHold;
  bool teleop_active = false;
  bool target_fresh = false;
  double manipulability = 0.0;
  TeleopAction requested_action{};
  TeleopState teleop_state = TeleopState::kDisconnected;
  uint64_t packet_age_ns = 0;
  FaultFlags faults{};
};

struct RobotObservation {
  uint64_t timestamp_ns = 0;
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  Pose tcp_pose{};
  double gripper_width = 0.0;
  TeleopAction executed_action{};
  ControlMode control_mode = ControlMode::kHold;
  TeleopState teleop_state = TeleopState::kDisconnected;
  uint64_t packet_age_ns = 0;
  uint64_t target_age_ns = 0;
  bool target_fresh = false;
  bool teleop_active = false;
  double target_manipulability = 0.0;
  FaultFlags faults{};
};

struct SafetyLimits {
  double max_translation_speed_mps = 0.20;
  double max_rotation_speed_rps = 0.80;
  double max_step_translation_m = 0.0015;
  double max_step_rotation_rad = 0.010;
  double packet_timeout_s = 0.120;
  double jump_reject_translation_m = 0.08;
  double jump_reject_rotation_rad = 0.80;
  std::array<double, 3> workspace_min{{0.20, -0.45, 0.05}};
  std::array<double, 3> workspace_max{{0.80, 0.45, 0.85}};
};

struct TeleopRuntimeConfig {
  ControlMode control_mode = ControlMode::kPose;
  double scale_factor = 0.8;
  double control_trigger_threshold = 0.9;
  double control_trigger_release_threshold = 0.8;
  double xr_pose_lowpass_alpha = 0.2;
  double xr_translation_deadband_m = 0.0015;
  double xr_rotation_deadband_rad = 0.015;
  double planner_rate_hz = 100.0;
  std::array<double, 7> start_joint_positions_rad{
      {0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483}};
};

struct GripperConfig {
  bool enabled = true;
  double max_width_m = 0.08;
  double min_width_m = 0.0;
  double speed_mps = 0.05;
  double min_command_delta_m = 0.002;
  double max_command_rate_hz = 20.0;
};

struct IkConfig {
  double damping = 0.05;
  double nullspace_gain = 0.15;
  double max_joint_velocity_radps = 0.35;
  double max_joint_acceleration_radps2 = 1.5;
  double max_joint_step_rad = 0.008;
  double target_smoothing_alpha = 0.25;
  double realtime_target_smoothing_alpha = 0.08;
  double position_gain = 3.0;
  double orientation_gain = 2.0;
  double task_translation_deadband_m = 0.001;
  double task_rotation_deadband_rad = 0.015;
  double manipulability_threshold = 0.05;
  double singularity_damping_gain = 1.0;
};

struct TeleopBridgeConfig {
  SafetyLimits safety{};
  TeleopRuntimeConfig teleop{};
  GripperConfig gripper{};
  IkConfig ik{};
  std::array<std::array<double, 3>, 3> xr_to_robot_rotation{{
      {{0.0, 0.0, -1.0}},
      {{-1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
  }};
  bool allow_motion = true;
  std::string robot_ip;
};

struct AppConfig {
  TeleopBridgeConfig bridge{};
  std::string observation_ip = "127.0.0.1";
  uint16_t observation_port = 28081;
  bool dry_run = false;
};

template <typename T>
class LatestValueBuffer {
 public:
  void Publish(const T& value) {
    const uint32_t next = 1u - published_index_.load(std::memory_order_relaxed);
    slots_[next] = value;
    published_index_.store(next, std::memory_order_release);
  }

  T ReadLatest() const {
    const uint32_t idx = published_index_.load(std::memory_order_acquire);
    return slots_[idx];
  }

 private:
  T slots_[2]{};
  std::atomic<uint32_t> published_index_{0};
};

using LatestCommandBuffer = LatestValueBuffer<XRCommand>;
using LatestObservationBuffer = LatestValueBuffer<RobotObservation>;
using LatestRobotStateBuffer = LatestValueBuffer<RobotSnapshot>;
using LatestPlannedTargetBuffer = LatestValueBuffer<PlannedTarget>;

}  // namespace teleop
