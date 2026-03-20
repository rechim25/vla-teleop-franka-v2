#include "franka_controller.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "gripper.h"
#include "math_utils.h"
#include "safety.h"
#include "teleop_mapper.h"
#include "teleop_state_machine.h"
#include "trace_logger.h"

namespace teleop {
namespace {

constexpr double kStartupHomeSpeedRadPerS = 0.12;
constexpr double kStartupHomeAccelerationRadPerS2 = 0.6;
constexpr double kStartupHomeJerkRadPerS3 = 4.0;
constexpr double kStartupHomeServoKp = 6.0;
constexpr double kStartupHomeServoKd = 1.5;
constexpr double kStartupHomeArrivalToleranceRad = 5e-3;
constexpr double kStartupHomeVelocityToleranceRadPerS = 2e-2;
constexpr uint32_t kStartupHomeSettledCycles = 100;

void ConfigureConservativeBehavior(franka::Robot* robot) {
  robot->setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  // Keep the internal joint impedance less aggressive to reduce audible chatter
  // when teleop commands dither around a steady target.
  robot->setJointImpedance({{1500.0, 1500.0, 1500.0, 1250.0, 1250.0, 1000.0, 1000.0}});
}

RobotSnapshot ToSnapshot(const franka::RobotState& state) {
  RobotSnapshot snapshot{};
  snapshot.timestamp_ns = MonotonicNowNs();
  snapshot.q = state.q;
  snapshot.q_d = state.q_d;
  snapshot.dq = state.dq;
  snapshot.tcp_pose = MatrixToPose(state.O_T_EE);
  snapshot.tcp_pose_d = MatrixToPose(state.O_T_EE_d);
  snapshot.F_T_EE = state.F_T_EE;
  snapshot.EE_T_K = state.EE_T_K;
  snapshot.in_reflex = (state.robot_mode == franka::RobotMode::kReflex);
  snapshot.robot_ok = !snapshot.in_reflex;
  return snapshot;
}

bool RecoverRobotIfNeeded(franka::Robot* robot) {
  const franka::RobotState state = robot->readOnce();
  if (state.robot_mode != franka::RobotMode::kReflex) {
    return true;
  }
  robot->automaticErrorRecovery();
  return true;
}

bool MoveToHomePose(franka::Robot* robot,
                    const std::array<double, 7>& q_goal,
                    std::atomic<bool>* stop_requested) {
  const franka::RobotState state0 = robot->readOnce();
  const std::array<double, 7> q_start = state0.q_d;
  double max_error = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    max_error = std::max(max_error, std::abs(q_goal[i] - q_start[i]));
  }
  if (max_error < kStartupHomeArrivalToleranceRad) {
    return true;
  }

  std::array<double, 7> q_ref = q_start;
  std::array<double, 7> dq_ref{};
  std::array<double, 7> ddq_ref{};

  uint32_t settled_cycles = 0;
  robot->control(
      [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
        const double dt = std::max(period.toSec(), 1e-6);
        const double max_delta_acceleration = kStartupHomeJerkRadPerS3 * dt;
        bool internal_finished = true;

        for (size_t i = 0; i < 7; ++i) {
          const double error = q_goal[i] - q_ref[i];
          const double desired_acceleration =
              std::clamp(kStartupHomeServoKp * error - kStartupHomeServoKd * dq_ref[i],
                         -kStartupHomeAccelerationRadPerS2,
                         kStartupHomeAccelerationRadPerS2);
          ddq_ref[i] += std::clamp(desired_acceleration - ddq_ref[i],
                                   -max_delta_acceleration,
                                   max_delta_acceleration);
          ddq_ref[i] = std::clamp(ddq_ref[i],
                                  -kStartupHomeAccelerationRadPerS2,
                                  kStartupHomeAccelerationRadPerS2);
          dq_ref[i] += ddq_ref[i] * dt;
          dq_ref[i] = std::clamp(dq_ref[i], -kStartupHomeSpeedRadPerS, kStartupHomeSpeedRadPerS);
          q_ref[i] += dq_ref[i] * dt;

          const double new_error = q_goal[i] - q_ref[i];
          if ((error > 0.0 && new_error < 0.0) || (error < 0.0 && new_error > 0.0)) {
            q_ref[i] = q_goal[i];
            dq_ref[i] = 0.0;
            ddq_ref[i] = 0.0;
          }

          if (std::abs(q_goal[i] - q_ref[i]) > kStartupHomeArrivalToleranceRad ||
              std::abs(dq_ref[i]) > kStartupHomeVelocityToleranceRadPerS) {
            internal_finished = false;
          }
        }

        franka::JointPositions out(q_ref);
        bool robot_settled = internal_finished;
        for (size_t i = 0; i < 7 && robot_settled; ++i) {
          robot_settled = robot_settled &&
                          std::abs(q_goal[i] - state.q_d[i]) <= kStartupHomeArrivalToleranceRad &&
                          std::abs(q_goal[i] - state.q[i]) <= 2.0 * kStartupHomeArrivalToleranceRad &&
                          std::abs(state.dq_d[i]) <= kStartupHomeVelocityToleranceRadPerS &&
                          std::abs(state.dq[i]) <= 2.0 * kStartupHomeVelocityToleranceRadPerS;
        }

        if (robot_settled) {
          ++settled_cycles;
        } else {
          settled_cycles = 0;
        }

        if (settled_cycles >= kStartupHomeSettledCycles ||
            stop_requested->load(std::memory_order_acquire)) {
          return franka::MotionFinished(out);
        }
        return out;
      },
      franka::ControllerMode::kJointImpedance,
      true,
      100.0);
  return !stop_requested->load(std::memory_order_acquire);
}

double MapTriggerToWidth(const GripperConfig& config, double trigger) {
  const double clamped = Clamp01(trigger);
  const double span = config.max_width_m - config.min_width_m;
  return config.max_width_m - clamped * span;
}

double ClampWidth(const GripperConfig& config, double width) {
  return std::clamp(width, config.min_width_m, config.max_width_m);
}

double MapStateToWidth(const GripperConfig& config, GripperState state) {
  return state == GripperState::kClose ? config.min_width_m : config.max_width_m;
}

bool GripperStateSatisfiesDesired(GripperState actual, GripperState desired) {
  if (desired == GripperState::kOpen) {
    return actual == GripperState::kOpen;
  }
  if (desired == GripperState::kClose) {
    return actual == GripperState::kClose || actual == GripperState::kHold;
  }
  return actual == desired;
}

Eigen::Matrix<double, 6, 7> JacobianToEigen(const std::array<double, 42>& jacobian_col_major) {
  return Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::ColMajor>>(jacobian_col_major.data());
}

double ComputeManipulability(const Eigen::Matrix<double, 6, 7>& jacobian) {
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  double product = 1.0;
  for (int i = 0; i < singular_values.size(); ++i) {
    product *= std::max(singular_values[i], 0.0);
  }
  return product;
}

Eigen::Vector3d ApplyVectorDeadband(const Eigen::Vector3d& value, double deadband) {
  const double norm = value.norm();
  if (norm <= deadband || norm <= 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return value * ((norm - deadband) / norm);
}

bool SolveIkStep(const franka::Model& model,
                 const RobotSnapshot& snapshot,
                 const Pose& desired_pose,
                 const TeleopBridgeConfig& config,
                 ControlMode control_mode,
                 std::array<double, 7>* target_q,
                 double* manipulability) {
  if (control_mode == ControlMode::kHold) {
    *target_q = snapshot.q_d;
    *manipulability = 0.0;
    return true;
  }

  // Build the next reference from the commanded state that libfranka is already
  // tracking, instead of re-seeding from measured joint motion every planner tick.
  const Eigen::Matrix<double, 6, 7> jacobian = JacobianToEigen(
      model.zeroJacobian(franka::Frame::kEndEffector, snapshot.q_d, snapshot.F_T_EE, snapshot.EE_T_K));
  Eigen::Matrix<double, 6, 7> jacobian_task = jacobian;

  Eigen::Matrix<double, 6, 1> task_error = Eigen::Matrix<double, 6, 1>::Zero();
  const Eigen::Vector3d raw_position_error = ToEigen(desired_pose.p) - ToEigen(snapshot.tcp_pose_d.p);
  const Eigen::Vector3d position_error =
      ApplyVectorDeadband(raw_position_error, config.ik.task_translation_deadband_m);
  task_error.head<3>() = config.ik.position_gain * position_error;

  if (control_mode == ControlMode::kPose) {
    const Eigen::Vector3d raw_rotation_error =
        QuaternionErrorAngleAxis(ToEigenQuat(snapshot.tcp_pose_d.q), ToEigenQuat(desired_pose.q));
    const Eigen::Vector3d rotation_error =
        ApplyVectorDeadband(raw_rotation_error, config.ik.task_rotation_deadband_rad);
    task_error.tail<3>() = config.ik.orientation_gain * rotation_error;
  } else {
    jacobian_task.bottomRows<3>().setZero();
  }

  *manipulability = ComputeManipulability(jacobian_task);

  double lambda = config.ik.damping;
  if (*manipulability < config.ik.manipulability_threshold) {
    lambda +=
        (config.ik.manipulability_threshold - *manipulability) * config.ik.singularity_damping_gain;
  }

  const Eigen::Matrix<double, 6, 6> a_matrix =
      jacobian_task * jacobian_task.transpose() +
      (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> a_ldlt(a_matrix);
  if (a_ldlt.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Matrix<double, 7, 1> q_current =
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>(snapshot.q_d.data());
  const Eigen::Matrix<double, 7, 1> q_home =
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>(config.teleop.start_joint_positions_rad.data());
  const Eigen::Matrix<double, 7, 1> dq_primary =
      jacobian_task.transpose() * a_ldlt.solve(task_error);
  const Eigen::Matrix<double, 7, 7> nullspace_projector =
      Eigen::Matrix<double, 7, 7>::Identity() -
      jacobian_task.transpose() * a_ldlt.solve(jacobian_task);
  Eigen::Matrix<double, 7, 1> dq =
      dq_primary + config.ik.nullspace_gain * nullspace_projector * (q_home - q_current);

  const double dt = 1.0 / config.teleop.planner_rate_hz;
  std::array<double, 7> q_next = snapshot.q_d;
  const double max_step_by_velocity = config.ik.max_joint_velocity_radps * dt;
  const double max_step = std::min(config.ik.max_joint_step_rad, max_step_by_velocity);

  for (size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(dq[i])) {
      return false;
    }
    const double step = std::clamp(dq[i] * dt, -max_step, max_step);
    q_next[i] += step;
  }

  *target_q = q_next;
  return true;
}

class JointPositionTrajectoryGenerator {
 public:
  explicit JointPositionTrajectoryGenerator(const TeleopBridgeConfig& config)
      : max_velocity_(std::max(config.ik.max_joint_velocity_radps, 1e-6)),
        max_acceleration_(std::max(config.ik.max_joint_acceleration_radps2, 1e-6)),
        max_jerk_(std::max(config.ik.max_joint_jerk_radps3, 1e-6)),
        max_step_(std::max(0.0, config.ik.max_joint_step_rad)),
        joint_deadzone_(std::max(0.0, config.ik.realtime_joint_deadzone_rad)),
        servo_kp_(std::max(config.ik.realtime_servo_kp, 1e-6)),
        servo_kd_(std::max(0.0, config.ik.realtime_servo_kd)),
        hold_position_threshold_(std::max(0.0, config.ik.realtime_hold_position_threshold_rad)),
        hold_velocity_threshold_(
            std::max(0.0, config.ik.realtime_hold_velocity_threshold_radps)),
        hold_release_threshold_(
            std::max(config.ik.realtime_hold_position_threshold_rad,
                     config.ik.realtime_hold_release_threshold_rad)) {}

  void Reset() {
    initialized_ = false;
    hold_active_ = false;
    q_ref_.fill(0.0);
    dq_ref_.fill(0.0);
    ddq_ref_.fill(0.0);
  }

  std::array<double, 7> Update(const std::array<double, 7>& target_q,
                               const std::array<double, 7>& reference_q,
                               double dt,
                               bool apply_motion,
                               std::array<double, 7>* target_delta,
                               std::array<double, 7>* filtered_delta,
                               std::array<double, 7>* command_delta,
                               std::array<uint8_t, 7>* clamp_saturated) {
    if (!apply_motion) {
      Reset();
      target_delta->fill(0.0);
      filtered_delta->fill(0.0);
      command_delta->fill(0.0);
      clamp_saturated->fill(0);
      return reference_q;
    }

    if (!initialized_) {
      q_ref_ = reference_q;
      dq_ref_.fill(0.0);
      ddq_ref_.fill(0.0);
      initialized_ = true;
    }

    dt = std::max(dt, 1e-6);
    const double max_step_from_velocity = max_velocity_ * dt;
    const double max_step = max_step_ > 0.0 ? std::min(max_step_, max_step_from_velocity)
                                            : max_step_from_velocity;
    const double max_delta_acceleration = max_jerk_ * dt;

    for (size_t i = 0; i < 7; ++i) {
      (*target_delta)[i] = target_q[i] - reference_q[i];
      (*filtered_delta)[i] = q_ref_[i] - reference_q[i];
    }

    double max_abs_hold_error = 0.0;
    bool within_hold_band = true;
    for (size_t i = 0; i < 7; ++i) {
      const double error_to_ref = target_q[i] - q_ref_[i];
      max_abs_hold_error = std::max(max_abs_hold_error, std::abs(error_to_ref));
      if (std::abs(error_to_ref) > hold_position_threshold_ ||
          std::abs(dq_ref_[i]) > hold_velocity_threshold_) {
        within_hold_band = false;
      }
    }

    if (hold_active_ && max_abs_hold_error <= hold_release_threshold_) {
      dq_ref_.fill(0.0);
      ddq_ref_.fill(0.0);
      command_delta->fill(0.0);
      clamp_saturated->fill(0);
      return q_ref_;
    }

    if (within_hold_band) {
      hold_active_ = true;
      dq_ref_.fill(0.0);
      ddq_ref_.fill(0.0);
      command_delta->fill(0.0);
      clamp_saturated->fill(0);
      return q_ref_;
    }

    hold_active_ = false;
    std::array<double, 7> q_out = q_ref_;
    std::array<double, 7> dq_out = dq_ref_;
    std::array<double, 7> ddq_out = ddq_ref_;
    command_delta->fill(0.0);
    clamp_saturated->fill(0);

    for (size_t i = 0; i < 7; ++i) {
      double error = target_q[i] - q_ref_[i];
      if (std::abs(error) < joint_deadzone_) {
        error = 0.0;
      }

      const double desired_acceleration =
          std::clamp(servo_kp_ * error - servo_kd_ * dq_ref_[i],
                     -max_acceleration_,
                     max_acceleration_);
      ddq_out[i] = ddq_ref_[i] +
                   std::clamp(desired_acceleration - ddq_ref_[i],
                              -max_delta_acceleration,
                              max_delta_acceleration);
      ddq_out[i] = std::clamp(ddq_out[i], -max_acceleration_, max_acceleration_);

      const double velocity_unclamped = dq_ref_[i] + ddq_out[i] * dt;
      dq_out[i] = std::clamp(velocity_unclamped, -max_velocity_, max_velocity_);

      const double step_unclamped = dq_out[i] * dt;
      const double step = std::clamp(step_unclamped, -max_step, max_step);
      q_out[i] = q_ref_[i] + step;

      if (error != 0.0) {
        const double new_error = target_q[i] - q_out[i];
        if ((error > 0.0 && new_error < 0.0) || (error < 0.0 && new_error > 0.0)) {
          q_out[i] = target_q[i];
          dq_out[i] = 0.0;
          ddq_out[i] = 0.0;
        }
      }

      (*command_delta)[i] = q_out[i] - reference_q[i];
      if (std::abs(velocity_unclamped - dq_out[i]) > 1e-12 || std::abs(step_unclamped - step) > 1e-12) {
        (*clamp_saturated)[i] = 1;
      }
    }

    q_ref_ = q_out;
    dq_ref_ = dq_out;
    ddq_ref_ = ddq_out;
    for (size_t i = 0; i < 7; ++i) {
      (*filtered_delta)[i] = q_ref_[i] - reference_q[i];
    }
    return q_out;
  }

 private:
  double max_velocity_ = 0.35;
  double max_acceleration_ = 1.5;
  double max_jerk_ = 12.0;
  double max_step_ = 0.008;
  double joint_deadzone_ = 0.001;
  double servo_kp_ = 60.0;
  double servo_kd_ = 10.0;
  double hold_position_threshold_ = 0.0008;
  double hold_velocity_threshold_ = 0.01;
  double hold_release_threshold_ = 0.0016;
  bool initialized_ = false;
  bool hold_active_ = false;
  std::array<double, 7> q_ref_{};
  std::array<double, 7> dq_ref_{};
  std::array<double, 7> ddq_ref_{};
};

void PlannerLoop(const TeleopBridgeConfig& config,
                 const franka::Model& model,
                 const LatestCommandBuffer* command_buffer,
                 const LatestRobotStateBuffer* robot_state_buffer,
                 LatestPlannedTargetBuffer* planned_target_buffer,
                 TraceRecorder* trace_recorder,
                 uint32_t trace_decimation,
                 std::atomic<GripperState>* desired_gripper_state,
                 std::atomic<bool>* stop_requested) {
  try {
    const auto sleep_period =
        std::chrono::microseconds(static_cast<int64_t>(1e6 / config.teleop.planner_rate_hz));

    SafetyFilter safety(config.safety, config.teleop.planner_rate_hz);
    GripperController gripper_controller;
    gripper_controller.Reset(GripperState::kOpen);
    TeleopMapper mapper(config);
    TeleopStateMachine state_machine;
    bool deadman_latched = false;
    uint64_t planner_last_ns = 0;
    uint64_t planner_trace_counter = 0;
    const uint32_t planner_trace_decimation = std::max<uint32_t>(1, trace_decimation);

    while (!stop_requested->load(std::memory_order_acquire)) {
      const uint64_t now_ns = MonotonicNowNs();
      const uint64_t loop_dt_ns = (planner_last_ns == 0) ? 0 : (now_ns - planner_last_ns);
      planner_last_ns = now_ns;
      const XRCommand xr_cmd = command_buffer->ReadLatest();
      const RobotSnapshot robot = robot_state_buffer->ReadLatest();

      PlannedTarget planned{};
      planned.target_timestamp_ns = now_ns;
      planned.target_q = robot.q;
      planned.desired_tcp_pose = robot.tcp_pose;
      planned.control_mode = ControlMode::kHold;
      const GripperState desired_state =
          gripper_controller.UpdateDesiredState(config.gripper, xr_cmd.gripper_trigger_value, now_ns);
      const double gripper_command = desired_state == GripperState::kClose ? 1.0 : 0.0;
      planned.target_gripper_width_m = MapStateToWidth(config.gripper, desired_state);
      planned.requested_action.gripper_command = gripper_command;
      desired_gripper_state->store(desired_state, std::memory_order_release);
      const double control_value = Clamp01(xr_cmd.control_trigger_value);

      bool has_target = false;
      bool safe_target = false;
      bool ik_ok = false;
      Pose desired_pose = robot.tcp_pose;
      Pose safe_pose = robot.tcp_pose;
      std::array<double, 7> q_target = robot.q;

      auto publish_trace = [&]() {
        if (trace_recorder == nullptr) {
          return;
        }
        if ((planner_trace_counter++ % planner_trace_decimation) != 0) {
          return;
        }

        PlannerTraceSample trace{};
        trace.timestamp_ns = now_ns;
        trace.loop_dt_ns = loop_dt_ns;
        trace.xr_timestamp_ns = xr_cmd.timestamp_ns;
        trace.xr_sequence_id = xr_cmd.sequence_id;
        trace.packet_age_ns = planned.packet_age_ns;
        trace.teleop_state = static_cast<int>(planned.teleop_state);
        trace.control_mode = static_cast<int>(planned.control_mode);
        trace.teleop_active = planned.teleop_active;
        trace.target_fresh = planned.target_fresh;
        trace.deadman_latched = deadman_latched;
        trace.has_target = has_target;
        trace.safe_target = safe_target;
        trace.ik_ok = ik_ok;
        trace.faults = planned.faults;
        trace.control_trigger_value = control_value;
        trace.xr_position = xr_cmd.right_controller_pose.p;
        trace.desired_position = desired_pose.p;
        trace.safe_position = safe_pose.p;
        trace.robot_position = robot.tcp_pose.p;
        trace.requested_delta_translation = planned.requested_action.delta_translation_m;
        trace.requested_delta_rotation = planned.requested_action.delta_rotation_rad;
        trace.safe_delta_translation = {
            safe_pose.p[0] - robot.tcp_pose.p[0],
            safe_pose.p[1] - robot.tcp_pose.p[1],
            safe_pose.p[2] - robot.tcp_pose.p[2]};
        trace.safe_delta_rotation =
            ToArray3(QuaternionErrorAngleAxis(ToEigenQuat(robot.tcp_pose.q), ToEigenQuat(safe_pose.q)));
        trace.q_robot = robot.q;
        trace.q_raw_target = q_target;
        trace.q_planned = planned.target_q;
        trace.manipulability = planned.manipulability;
        trace_recorder->PushPlanner(trace);
      };

      if (robot.timestamp_ns == 0) {
        planned_target_buffer->Publish(planned);
        publish_trace();
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      const uint64_t packet_age_ns = now_ns > xr_cmd.timestamp_ns ? (now_ns - xr_cmd.timestamp_ns) : 0;
      planned.packet_age_ns = packet_age_ns;
      const double packet_age_s = static_cast<double>(packet_age_ns) * 1e-9;

      StateInputs inputs{};
      inputs.xr_stream_healthy = safety.IsStreamHealthy(packet_age_s);
      if (!inputs.xr_stream_healthy) {
        deadman_latched = false;
      } else if (deadman_latched) {
        if (control_value <= config.teleop.control_trigger_release_threshold) {
          deadman_latched = false;
        }
      } else if (control_value >= config.teleop.control_trigger_threshold) {
        deadman_latched = true;
      }
      inputs.deadman_pressed = deadman_latched;
      inputs.robot_ok = robot.robot_ok;
      inputs.fault_requested = false;
      inputs.clear_fault_requested = true;
      planned.teleop_state = state_machine.Update(inputs);
      planned.teleop_active = (planned.teleop_state == TeleopState::kTeleopActive);

      planned.faults.robot_not_ready = !inputs.robot_ok;
      if (!inputs.xr_stream_healthy) {
        planned.faults.packet_timeout = true;
      }

      if (!planned.teleop_active) {
        mapper.Reset();
        planned_target_buffer->Publish(planned);
        publish_trace();
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      planned.control_mode = config.teleop.control_mode;
      TeleopAction requested_action{};
      has_target = mapper.ComputeTargetPose(robot.tcp_pose,
                                            xr_cmd,
                                            true,
                                            planned.control_mode,
                                            &desired_pose,
                                            &requested_action);
      planned.requested_action = requested_action;
      planned.requested_action.gripper_command = gripper_command;
      if (!has_target) {
        planned.control_mode = ControlMode::kHold;
        planned_target_buffer->Publish(planned);
        publish_trace();
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      // Safety target shaping is intentionally bypassed in this simplified mode:
      // planner uses mapper output directly for IK.
      safe_target = true;
      safe_pose = desired_pose;
      planned.desired_tcp_pose = safe_pose;

      double manipulability = 0.0;
      ik_ok = SolveIkStep(model,
                          robot,
                          safe_pose,
                          config,
                          planned.control_mode,
                          &q_target,
                          &manipulability);
      if (!ik_ok) {
        planned.faults.ik_rejected = true;
        planned.control_mode = ControlMode::kHold;
        planned_target_buffer->Publish(planned);
        publish_trace();
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      planned.target_q = q_target;
      planned.manipulability = manipulability;
      planned.target_fresh = true;
      planned_target_buffer->Publish(planned);
      publish_trace();
      std::this_thread::sleep_for(sleep_period);
    }
  } catch (const std::exception& e) {
    std::cerr << "Planner thread exception: " << e.what() << "\n";
    stop_requested->store(true, std::memory_order_release);
  } catch (...) {
    std::cerr << "Planner thread unknown exception\n";
    stop_requested->store(true, std::memory_order_release);
  }
}

void GripperLoop(franka::Gripper* gripper,
                 const GripperConfig& config,
                 const std::atomic<GripperState>* desired_gripper_state,
                 std::atomic<GripperState>* active_gripper_state,
                 std::atomic<double>* measured_gripper_width_m,
                 std::atomic<bool>* stop_requested) {
  try {
    franka::GripperState measured_state = gripper->readOnce();
    measured_gripper_width_m->store(measured_state.width, std::memory_order_release);

    GripperState current_state =
        measured_state.width >= (config.max_width_m - config.width_tolerance_m) ? GripperState::kOpen
                                                                                 : GripperState::kHold;
    active_gripper_state->store(current_state, std::memory_order_release);

    bool action_in_progress = false;
    GripperState action_target_state = current_state;
    std::future<bool> action_future;
    bool action_abort_expected = false;
    const char* action_abort_reason = "";
    double stall_reference_width = measured_state.width;
    uint64_t last_width_progress_ns = MonotonicNowNs();
    uint64_t last_successful_read_ns = last_width_progress_ns;

    auto set_state = [&](GripperState next_state, const char* reason) {
      if (current_state != next_state) {
        std::cout << "Gripper state " << ToString(current_state) << " -> " << ToString(next_state)
                  << " reason=" << reason << "\n";
      }
      current_state = next_state;
      active_gripper_state->store(current_state, std::memory_order_release);
    };

    auto wait_for_action_completion = [&]() -> bool {
      if (!action_in_progress) {
        return true;
      }
      try {
        (void)action_future.get();
      } catch (const std::exception& e) {
        if (action_abort_expected) {
          std::cout << "Gripper action preempted: reason=" << action_abort_reason << "\n";
          action_in_progress = false;
          action_abort_expected = false;
          action_abort_reason = "";
          return true;
        }
        std::cerr << "Gripper action completion failed: " << e.what() << "\n";
        set_state(GripperState::kFault, "action_completion_exception");
        action_in_progress = false;
        return false;
      }
      action_in_progress = false;
      action_abort_expected = false;
      action_abort_reason = "";
      return true;
    };

    auto preempt_action = [&](const char* reason) -> bool {
      if (!action_in_progress) {
        return true;
      }
      action_abort_expected = true;
      action_abort_reason = reason;
      try {
        (void)gripper->stop();
      } catch (const std::exception& e) {
        std::cerr << "Gripper stop failed during preemption: " << e.what() << "\n";
        set_state(GripperState::kFault, "preempt_stop_failed");
        action_in_progress = false;
        action_abort_expected = false;
        action_abort_reason = "";
        return false;
      }
      return wait_for_action_completion();
    };

    auto start_action = [&](GripperState target_state, uint64_t now_ns) {
      const double target_width = MapStateToWidth(config, target_state);
      action_target_state = target_state;
      action_abort_expected = false;
      action_abort_reason = "";
      stall_reference_width = measured_state.width;
      last_width_progress_ns = now_ns;
      action_future = std::async(std::launch::async, [gripper, target_width, speed = config.speed_mps]() {
        return gripper->move(target_width, speed);
      });
      action_in_progress = true;
      set_state(target_state, target_state == GripperState::kOpen ? "command_open" : "command_close");
    };

    while (!stop_requested->load(std::memory_order_acquire)) {
      const uint64_t now_ns = MonotonicNowNs();
      const GripperState desired_state = desired_gripper_state->load(std::memory_order_acquire);

      try {
        measured_state = gripper->readOnce();
        measured_gripper_width_m->store(measured_state.width, std::memory_order_release);
        last_successful_read_ns = now_ns;
      } catch (const std::exception& e) {
        if (static_cast<double>(now_ns - last_successful_read_ns) * 1e-9 >=
            config.read_failure_timeout_s) {
          std::cerr << "Gripper read failed persistently: " << e.what() << "\n";
          set_state(GripperState::kFault, "read_failure_timeout");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      if (action_in_progress && action_future.valid() &&
          action_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          const bool success = action_future.get();
          action_in_progress = false;
          action_abort_expected = false;
          action_abort_reason = "";
          if (current_state != GripperState::kFault) {
            if (action_target_state == GripperState::kOpen) {
              if (success || measured_state.width >= (config.max_width_m - config.width_tolerance_m)) {
                set_state(GripperState::kOpen, "open_complete");
              } else {
                std::cerr << "Gripper open command returned false.\n";
                set_state(GripperState::kFault, "open_failed");
              }
            } else if (action_target_state == GripperState::kClose) {
              if (success || measured_state.width <= (config.min_width_m + config.width_tolerance_m)) {
                set_state(GripperState::kClose, "close_complete");
              } else {
                set_state(GripperState::kHold, "close_contact");
              }
            }
          }
        } catch (const std::exception& e) {
          if (action_abort_expected) {
            std::cout << "Gripper action preempted: reason=" << action_abort_reason << "\n";
            action_in_progress = false;
            action_abort_expected = false;
            action_abort_reason = "";
          } else {
            std::cerr << "Gripper command failed: " << e.what() << "\n";
            set_state(GripperState::kFault, "command_exception");
          }
        }
      }

      if (current_state == GripperState::kFault) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      if (action_in_progress) {
        if (desired_state == GripperState::kOpen && action_target_state != GripperState::kOpen) {
          if (!preempt_action("latest_open")) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
          }
        } else if (desired_state == GripperState::kClose && action_target_state != GripperState::kClose) {
          if (!preempt_action("latest_close")) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
          }
        } else if (action_target_state == GripperState::kClose) {
          const double width_delta = std::abs(measured_state.width - stall_reference_width);
          if (width_delta >= config.stall_width_delta_m) {
            stall_reference_width = measured_state.width;
            last_width_progress_ns = now_ns;
          } else if (measured_state.width > (config.min_width_m + config.width_tolerance_m) &&
                     static_cast<double>(now_ns - last_width_progress_ns) * 1e-9 >=
                         config.stall_timeout_s) {
            if (!preempt_action("close_stall")) {
              std::this_thread::sleep_for(std::chrono::milliseconds(20));
              continue;
            }
            if (current_state != GripperState::kFault) {
              set_state(GripperState::kHold, "close_stall");
            }
          }
        }
      }

      if (!action_in_progress && current_state != GripperState::kFault &&
          !GripperStateSatisfiesDesired(current_state, desired_state)) {
        start_action(desired_state, now_ns);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (action_in_progress) {
      (void)preempt_action("shutdown");
    }
  } catch (const std::exception& e) {
    std::cerr << "Gripper thread exception: " << e.what() << "\n";
    stop_requested->store(true, std::memory_order_release);
  } catch (...) {
    std::cerr << "Gripper thread unknown exception\n";
    stop_requested->store(true, std::memory_order_release);
  }
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
  std::thread planner_thread;
  std::thread gripper_thread;
  std::unique_ptr<TraceRecorder> trace_recorder;
  auto join_threads = [&]() {
    stop_requested->store(true, std::memory_order_release);
    if (planner_thread.joinable()) {
      planner_thread.join();
    }
    if (gripper_thread.joinable()) {
      gripper_thread.join();
    }
  };
  auto stop_trace = [&]() {
    if (trace_recorder == nullptr) {
      return;
    }
    trace_recorder->Stop();
    std::cout << "Trace capture stopped. output_dir=" << trace_recorder->output_dir()
              << " dropped_planner=" << trace_recorder->dropped_planner()
              << " dropped_rt=" << trace_recorder->dropped_rt() << "\n";
    trace_recorder.reset();
  };

  try {
    if (options_.trace.enabled) {
      TraceConfig trace_config{};
      trace_config.enabled = true;
      trace_config.output_dir = options_.trace.output_dir;
      trace_config.planner_decimation = std::max<uint32_t>(1, options_.trace.planner_decimation);
      trace_config.rt_decimation = std::max<uint32_t>(1, options_.trace.rt_decimation);

      trace_recorder = std::make_unique<TraceRecorder>(trace_config);
      std::string trace_error;
      if (!trace_recorder->Start(&trace_error)) {
        std::cerr << "Failed to start trace recorder: " << trace_error << "\n";
        return 8;
      }
      std::cout << "Trace capture enabled: output_dir=" << trace_recorder->output_dir()
                << " planner_decimation=" << trace_config.planner_decimation
                << " rt_decimation=" << trace_config.rt_decimation << "\n";
    }

    franka::Robot robot(options_.robot_ip);
    ConfigureConservativeBehavior(&robot);
    robot.setLoad(config_.load.mass_kg, config_.load.center_of_mass_m, config_.load.inertia_kgm2);

    std::unique_ptr<franka::Gripper> gripper;
    std::atomic<double> measured_gripper_width_m{0.0};
    if (config_.gripper.enabled) {
      try {
        gripper = std::make_unique<franka::Gripper>(options_.robot_ip);
        franka::GripperState state = gripper->readOnce();
        if (state.max_width <= 1e-4) {
          std::cout << "Gripper max_width unavailable, running homing...\n";
          if (!gripper->homing()) {
            std::cerr << "Gripper homing returned false.\n";
          }
          state = gripper->readOnce();
        }
        if (state.max_width > 1e-4) {
          config_.gripper.max_width_m = std::min(config_.gripper.max_width_m, state.max_width);
        }
        measured_gripper_width_m.store(state.width, std::memory_order_release);
        std::cout << "Gripper ready: width=" << state.width
                  << " max_width=" << state.max_width
                  << " command_mode="
                  << (config_.gripper.command_mode == GripperCommandMode::kBinary ? "binary"
                                                                                  : "analog")
                  << "\n";
      } catch (const std::exception& e) {
        std::cerr << "Gripper unavailable, continuing without gripper control: " << e.what() << "\n";
        gripper.reset();
      }
    }

    if (!RecoverRobotIfNeeded(&robot)) {
      std::cerr << "Failed to recover robot from reflex mode.\n";
      return 5;
    }
    if (!MoveToHomePose(&robot, config_.teleop.start_joint_positions_rad, stop_requested)) {
      std::cerr << "Home move interrupted.\n";
      return 6;
    }

    franka::Model model = robot.loadModel();

    LatestRobotStateBuffer robot_state_buffer;
    LatestPlannedTargetBuffer planned_target_buffer;

    const RobotSnapshot initial_robot_snapshot = ToSnapshot(robot.readOnce());
    robot_state_buffer.Publish(initial_robot_snapshot);

    PlannedTarget initial_plan{};
    initial_plan.target_timestamp_ns = MonotonicNowNs();
    initial_plan.target_q = initial_robot_snapshot.q;
    initial_plan.desired_tcp_pose = initial_robot_snapshot.tcp_pose;
    initial_plan.target_gripper_width_m = config_.gripper.max_width_m;
    initial_plan.control_mode = ControlMode::kHold;
    initial_plan.teleop_state = TeleopState::kDisconnected;
    planned_target_buffer.Publish(initial_plan);

    std::atomic<GripperState> desired_gripper_state{GripperState::kOpen};
    const GripperState initial_gripper_state = !config_.gripper.enabled
                                                   ? GripperState::kOpen
                                                   : (gripper == nullptr
                                                          ? GripperState::kFault
                                                          : (measured_gripper_width_m.load(
                                                                     std::memory_order_acquire) >=
                                                                     (config_.gripper.max_width_m -
                                                                      config_.gripper.width_tolerance_m)
                                                                 ? GripperState::kOpen
                                                                 : GripperState::kHold));
    std::atomic<GripperState> active_gripper_state{initial_gripper_state};
    planner_thread = std::thread(PlannerLoop,
                                 std::cref(config_),
                                 std::cref(model),
                                 command_buffer_,
                                 &robot_state_buffer,
                                 &planned_target_buffer,
                                 trace_recorder.get(),
                                 std::max<uint32_t>(1, options_.trace.planner_decimation),
                                 &desired_gripper_state,
                                 stop_requested);

    if (gripper != nullptr && config_.allow_motion) {
      std::cout << "Gripper control thread started.\n";
      gripper_thread = std::thread(GripperLoop,
                                   gripper.get(),
                                   std::cref(config_.gripper),
                                   &desired_gripper_state,
                                   &active_gripper_state,
                                   &measured_gripper_width_m,
                                   stop_requested);
    } else if (gripper != nullptr) {
      std::cout << "Gripper control disabled because allow_motion=false.\n";
    }

    uint64_t rt_last_ns = 0;
    uint64_t rt_trace_counter = 0;
    uint64_t last_success_rate_log_ns = 0;
    JointPositionTrajectoryGenerator trajectory_generator(config_);
    const uint32_t rt_trace_decimation = std::max<uint32_t>(1, options_.trace.rt_decimation);
    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
      const uint64_t now_ns = MonotonicNowNs();
      const uint64_t rt_loop_dt_ns = (rt_last_ns == 0) ? 0 : (now_ns - rt_last_ns);
      rt_last_ns = now_ns;
      if (last_success_rate_log_ns == 0 || (now_ns - last_success_rate_log_ns) >= 5000000000ull) {
        last_success_rate_log_ns = now_ns;
        std::cerr << "libfranka control_command_success_rate=" << state.control_command_success_rate
                  << " q_err_max=" << [&state]() {
                       double max_err = 0.0;
                       for (size_t i = 0; i < 7; ++i) {
                         max_err = std::max(max_err, std::abs(state.q[i] - state.q_d[i]));
                       }
                       return max_err;
                     }()
                  << "\n";
      }
      const RobotSnapshot robot_snapshot = ToSnapshot(state);
      robot_state_buffer.Publish(robot_snapshot);

      const PlannedTarget planned = planned_target_buffer.ReadLatest();
      const bool apply_motion = config_.allow_motion && planned.teleop_active && planned.target_fresh &&
                                planned.control_mode != ControlMode::kHold;

      const double dt = std::max(period.toSec(), 1e-6);
      const double max_step = std::min(config_.ik.max_joint_step_rad,
                                       config_.ik.max_joint_velocity_radps * dt);
      const double rt_alpha = 0.0;
      std::array<double, 7> target_delta{};
      std::array<double, 7> filtered_delta{};
      std::array<double, 7> command_delta{};
      std::array<uint8_t, 7> clamp_saturated{};
      const std::array<double, 7> q_cmd = trajectory_generator.Update(planned.target_q,
                                                                      state.q_d,
                                                                      dt,
                                                                      apply_motion,
                                                                      &target_delta,
                                                                      &filtered_delta,
                                                                      &command_delta,
                                                                      &clamp_saturated);
      double max_abs_target_delta = 0.0;
      double max_abs_filtered_delta = 0.0;
      double max_abs_command_delta = 0.0;
      for (size_t i = 0; i < 7; ++i) {
        max_abs_target_delta = std::max(max_abs_target_delta, std::abs(target_delta[i]));
        max_abs_filtered_delta = std::max(max_abs_filtered_delta, std::abs(filtered_delta[i]));
        max_abs_command_delta = std::max(max_abs_command_delta, std::abs(command_delta[i]));
      }

      RobotObservation obs{};
      obs.timestamp_ns = now_ns;
      obs.q = state.q;
      obs.dq = state.dq;
      obs.tcp_pose = MatrixToPose(state.O_T_EE);
      obs.gripper_width = measured_gripper_width_m.load(std::memory_order_acquire);
      obs.gripper_state = active_gripper_state.load(std::memory_order_acquire);
      obs.executed_action = planned.requested_action;
      obs.control_mode = planned.teleop_active ? planned.control_mode : ControlMode::kHold;
      obs.teleop_state = planned.teleop_state;
      obs.packet_age_ns = planned.packet_age_ns;
      obs.target_age_ns =
          now_ns > planned.target_timestamp_ns ? (now_ns - planned.target_timestamp_ns) : 0;
      obs.target_fresh = planned.target_fresh;
      obs.teleop_active = planned.teleop_active;
      obs.target_manipulability = planned.manipulability;
      obs.faults = planned.faults;
      obs.faults.robot_not_ready = obs.faults.robot_not_ready || !robot_snapshot.robot_ok;
      obs.faults.gripper_fault = obs.faults.gripper_fault || obs.gripper_state == GripperState::kFault;
      observation_buffer_->Publish(obs);

      if (trace_recorder != nullptr && ((rt_trace_counter++ % rt_trace_decimation) == 0)) {
        RtTraceSample trace{};
        trace.timestamp_ns = now_ns;
        trace.loop_dt_ns = rt_loop_dt_ns;
        trace.callback_period_ns = static_cast<uint64_t>(dt * 1e9);
        trace.target_age_ns = obs.target_age_ns;
        trace.teleop_state = static_cast<int>(obs.teleop_state);
        trace.control_mode = static_cast<int>(obs.control_mode);
        trace.teleop_active = obs.teleop_active;
        trace.target_fresh = obs.target_fresh;
        trace.apply_motion = apply_motion;
        trace.faults = obs.faults;
        trace.max_step = max_step;
        trace.rt_alpha = rt_alpha;
        trace.q = state.q;
        trace.dq = state.dq;
        trace.q_d = state.q_d;
        trace.q_planned = planned.target_q;
        trace.q_cmd = q_cmd;
        trace.target_delta = target_delta;
        trace.filtered_delta = filtered_delta;
        trace.command_delta = command_delta;
        trace.clamp_saturated = clamp_saturated;
        trace.max_abs_target_delta = max_abs_target_delta;
        trace.max_abs_filtered_delta = max_abs_filtered_delta;
        trace.max_abs_command_delta = max_abs_command_delta;
        trace_recorder->PushRt(trace);
      }

      franka::JointPositions out(q_cmd);
      if (stop_requested->load(std::memory_order_acquire)) {
        return franka::MotionFinished(out);
      }
      return out;
    },
                  franka::ControllerMode::kJointImpedance,
                  config_.limit_rate,
                  config_.lpf_cutoff_frequency);

    join_threads();
    stop_trace();
    return 0;
  } catch (const franka::ControlException& e) {
    join_threads();
    stop_trace();
    std::cerr << "Franka control exception: " << e.what() << "\n";
    return 2;
  } catch (const franka::Exception& e) {
    join_threads();
    stop_trace();
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 3;
  } catch (const std::exception& e) {
    join_threads();
    stop_trace();
    std::cerr << "Std exception: " << e.what() << "\n";
    return 4;
  } catch (...) {
    join_threads();
    stop_trace();
    std::cerr << "Unknown controller exception\n";
    return 7;
  }
}

}  // namespace teleop
