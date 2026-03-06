#pragma once

#include <array>
#include <atomic>
#include <cstdint>

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
  kCartesianDelta = 1,
};

struct Pose {
  std::array<double, 3> p{};
  std::array<double, 4> q{{0.0, 0.0, 0.0, 1.0}};  // xyzw
};

struct XRCommand {
  uint64_t timestamp_ns = 0;
  uint64_t sequence_id = 0;
  bool teleop_enabled = false;
  bool clutch_pressed = false;
  std::array<double, 3> target_position_xyz{};
  std::array<double, 4> target_orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
  double gripper_command = 1.0;  // [0, 1]
};

struct RawOperatorCommand {
  uint64_t timestamp_ns = 0;
  uint64_t sequence_id = 0;
  bool teleop_enabled = false;
  bool clutch_pressed = false;
  std::array<double, 3> xr_pos{};
  std::array<double, 4> xr_quat{{0.0, 0.0, 0.0, 1.0}};
  double gripper_command = 1.0;
};

struct TeleopAction {
  std::array<double, 3> delta_translation_m{};
  std::array<double, 3> delta_rotation_rad{};  // axis-angle increments
  double gripper_command = 1.0;
};

struct FaultFlags {
  bool packet_timeout = false;
  bool jump_rejected = false;
  bool workspace_clamped = false;
  bool robot_not_ready = false;
  bool control_exception = false;
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

struct TeleopBridgeConfig {
  SafetyLimits safety{};
  std::array<std::array<double, 3>, 3> xr_to_robot_rotation{{
      {{1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
      {{0.0, 0.0, 1.0}},
  }};
  bool allow_motion = true;
};

struct alignas(64) CommandSlot {
  XRCommand cmd{};
};

struct alignas(64) ObservationSlot {
  RobotObservation obs{};
};

// Lock-free double-buffer for latest command handoff into the servo loop.
class LatestCommandBuffer {
 public:
  void Publish(const XRCommand& cmd) {
    const uint32_t next = 1u - published_index_.load(std::memory_order_relaxed);
    slots_[next].cmd = cmd;
    published_index_.store(next, std::memory_order_release);
    published_seq_.store(cmd.sequence_id, std::memory_order_release);
  }

  XRCommand ReadLatest() const {
    const uint32_t idx = published_index_.load(std::memory_order_acquire);
    return slots_[idx].cmd;
  }

  uint64_t PublishedSeq() const { return published_seq_.load(std::memory_order_acquire); }

 private:
  CommandSlot slots_[2]{};
  std::atomic<uint32_t> published_index_{0};
  std::atomic<uint64_t> published_seq_{0};
};

class LatestObservationBuffer {
 public:
  void Publish(const RobotObservation& obs) {
    const uint32_t next = 1u - published_index_.load(std::memory_order_relaxed);
    slots_[next].obs = obs;
    published_index_.store(next, std::memory_order_release);
  }

  RobotObservation ReadLatest() const {
    const uint32_t idx = published_index_.load(std::memory_order_acquire);
    return slots_[idx].obs;
  }

 private:
  ObservationSlot slots_[2]{};
  std::atomic<uint32_t> published_index_{0};
};

}  // namespace teleop
