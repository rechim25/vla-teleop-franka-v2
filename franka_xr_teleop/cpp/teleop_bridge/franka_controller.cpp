#include "franka_controller.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "gripper.h"
#include "safety.h"
#include "teleop_mapper.h"
#include "teleop_state_machine.h"

namespace teleop {
namespace {

uint64_t MonotonicNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

Pose MatrixToPose(const std::array<double, 16>& t) {
  Pose p;
  p.p = {t[12], t[13], t[14]};

  const double m00 = t[0];
  const double m01 = t[4];
  const double m02 = t[8];
  const double m10 = t[1];
  const double m11 = t[5];
  const double m12 = t[9];
  const double m20 = t[2];
  const double m21 = t[6];
  const double m22 = t[10];

  const double trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    p.q[3] = 0.25 * s;
    p.q[0] = (m21 - m12) / s;
    p.q[1] = (m02 - m20) / s;
    p.q[2] = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    p.q[3] = (m21 - m12) / s;
    p.q[0] = 0.25 * s;
    p.q[1] = (m01 + m10) / s;
    p.q[2] = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    p.q[3] = (m02 - m20) / s;
    p.q[0] = (m01 + m10) / s;
    p.q[1] = 0.25 * s;
    p.q[2] = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    p.q[3] = (m10 - m01) / s;
    p.q[0] = (m02 + m20) / s;
    p.q[1] = (m12 + m21) / s;
    p.q[2] = 0.25 * s;
  }
  return p;
}

std::array<double, 4> NormalizeQuat(const std::array<double, 4>& q) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

std::array<double, 4> QuatMultiply(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  return {
      a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
      a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
      a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
      a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  };
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

void PoseToMatrix(const Pose& p, std::array<double, 16>* out) {
  const auto q = NormalizeQuat(p.q);
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;

  // Column-major homogeneous transform.
  (*out)[0] = 1.0 - 2.0 * (yy + zz);
  (*out)[1] = 2.0 * (xy + wz);
  (*out)[2] = 2.0 * (xz - wy);
  (*out)[3] = 0.0;

  (*out)[4] = 2.0 * (xy - wz);
  (*out)[5] = 1.0 - 2.0 * (xx + zz);
  (*out)[6] = 2.0 * (yz + wx);
  (*out)[7] = 0.0;

  (*out)[8] = 2.0 * (xz + wy);
  (*out)[9] = 2.0 * (yz - wx);
  (*out)[10] = 1.0 - 2.0 * (xx + yy);
  (*out)[11] = 0.0;

  (*out)[12] = p.p[0];
  (*out)[13] = p.p[1];
  (*out)[14] = p.p[2];
  (*out)[15] = 1.0;
}

void ConfigureConservativeBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  robot.setJointImpedance({{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0}});
  robot.setCartesianImpedance({{3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0}});
}

}  // namespace

FrankaTeleopController::FrankaTeleopController(const FrankaControllerOptions& options,
                                               const TeleopBridgeConfig& config,
                                               const LatestCommandBuffer* command_buffer,
                                               LatestObservationBuffer* observation_buffer)
    : options_(options),
      config_(config),
      command_buffer_(command_buffer),
      observation_buffer_(observation_buffer) {}

int FrankaTeleopController::Run(std::atomic<bool>* stop_requested) {
  try {
    franka::Robot robot(options_.robot_ip);
    ConfigureConservativeBehavior(robot);

    SafetyFilter safety(config_.safety);
    TeleopMapper mapper(config_);
    TeleopStateMachine state_machine;
    GripperController gripper;

    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
      const double dt_s = period.toSec();
      const uint64_t now_ns = MonotonicNowNs();

      const XRCommand xr_cmd = command_buffer_->ReadLatest();
      const uint64_t packet_age_ns = now_ns > xr_cmd.timestamp_ns ? (now_ns - xr_cmd.timestamp_ns) : 0;
      const double packet_age_s = static_cast<double>(packet_age_ns) * 1e-9;

      StateInputs inputs{};
      inputs.xr_stream_healthy = safety.IsStreamHealthy(packet_age_s);
      inputs.deadman_pressed = xr_cmd.teleop_enabled;
      inputs.robot_ok = (state.robot_mode != franka::RobotMode::kReflex);
      inputs.fault_requested = false;
      inputs.clear_fault_requested = true;
      const TeleopState teleop_state = state_machine.Update(inputs);

      const bool teleop_active = teleop_state == TeleopState::kTeleopActive;

      const Pose current_pose_cmd = MatrixToPose(state.O_T_EE_d);

      Pose target_pose = current_pose_cmd;
      TeleopAction raw_action{};
      const bool has_target = mapper.MapToTarget(current_pose_cmd,
                                                 xr_cmd,
                                                 teleop_active,
                                                 xr_cmd.clutch_pressed,
                                                 &target_pose,
                                                 &raw_action);

      FaultFlags faults{};
      faults.robot_not_ready = !inputs.robot_ok;

      TeleopAction safe_action{};
      if (has_target && config_.allow_motion) {
        safe_action = safety.ComputeSafeAction(current_pose_cmd,
                                               target_pose,
                                               xr_cmd.gripper_command,
                                               dt_s,
                                               packet_age_s,
                                               &faults);
      } else {
        safety.Reset();
      }

      Pose next_pose = current_pose_cmd;
      for (size_t i = 0; i < 3; ++i) {
        next_pose.p[i] += safe_action.delta_translation_m[i];
      }
      const std::array<double, 4> dq = RotVecToQuat(safe_action.delta_rotation_rad);
      next_pose.q = NormalizeQuat(QuatMultiply(dq, next_pose.q));

      const double gripper_cmd = gripper.Update(safe_action.gripper_command, now_ns, dt_s);

      std::array<double, 16> next_t{};
      PoseToMatrix(next_pose, &next_t);

      RobotObservation obs{};
      obs.timestamp_ns = now_ns;
      obs.q = state.q;
      obs.dq = state.dq;
      obs.tcp_pose = MatrixToPose(state.O_T_EE);
      obs.gripper_width = gripper_cmd;
      obs.executed_action = safe_action;
      obs.control_mode = (std::abs(safe_action.delta_translation_m[0]) +
                                  std::abs(safe_action.delta_translation_m[1]) +
                                  std::abs(safe_action.delta_translation_m[2]) >
                          1e-9)
                             ? ControlMode::kCartesianDelta
                             : ControlMode::kHold;
      obs.teleop_state = teleop_state;
      obs.packet_age_ns = packet_age_ns;
      obs.faults = faults;
      observation_buffer_->Publish(obs);

      franka::CartesianPose out(next_t, state.elbow_d);
      if (stop_requested->load(std::memory_order_acquire)) {
        return franka::MotionFinished(out);
      }
      return out;
    });

    return 0;
  } catch (const franka::ControlException& e) {
    std::cerr << "Franka control exception: " << e.what() << "\n";
    return 2;
  } catch (const franka::Exception& e) {
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 3;
  } catch (const std::exception& e) {
    std::cerr << "Std exception: " << e.what() << "\n";
    return 4;
  }
}

}  // namespace teleop
