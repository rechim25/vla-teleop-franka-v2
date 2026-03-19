#include "franka_controller.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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

#include "math_utils.h"
#include "safety.h"
#include "teleop_mapper.h"
#include "teleop_state_machine.h"
#include "trace_logger.h"

namespace teleop {
namespace {

constexpr double kDefaultHomeDurationS = 6.0;

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
  snapshot.dq = state.dq;
  snapshot.tcp_pose = MatrixToPose(state.O_T_EE);
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
  const std::array<double, 7> q_start = state0.q;

  double max_error = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    max_error = std::max(max_error, std::abs(q_goal[i] - q_start[i]));
  }
  if (max_error < 1e-3) {
    return true;
  }

  const double duration_s = std::max(kDefaultHomeDurationS, max_error / 0.2);
  double elapsed_s = 0.0;

  robot->control(
      [&](const franka::RobotState&, franka::Duration period) -> franka::JointPositions {
        elapsed_s += period.toSec();
        const double tau = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
        const double s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));

        std::array<double, 7> q_cmd{};
        for (size_t i = 0; i < 7; ++i) {
          q_cmd[i] = q_start[i] + s * (q_goal[i] - q_start[i]);
        }

        franka::JointPositions out(q_cmd);
        if (tau >= 1.0 || stop_requested->load(std::memory_order_acquire)) {
          return franka::MotionFinished(out);
        }
        return out;
      });
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
                 const std::array<double, 7>& reference_q,
                 const Pose& desired_pose,
                 const TeleopBridgeConfig& config,
                 ControlMode control_mode,
                 std::array<double, 7>* target_q,
                 double* manipulability) {
  if (control_mode == ControlMode::kHold) {
    *target_q = reference_q;
    *manipulability = 0.0;
    return true;
  }

  const Eigen::Matrix<double, 6, 7> jacobian = JacobianToEigen(
      model.zeroJacobian(franka::Frame::kEndEffector, snapshot.q, snapshot.F_T_EE, snapshot.EE_T_K));
  Eigen::Matrix<double, 6, 7> jacobian_task = jacobian;

  Eigen::Matrix<double, 6, 1> task_error = Eigen::Matrix<double, 6, 1>::Zero();
  const Eigen::Vector3d raw_position_error = ToEigen(desired_pose.p) - ToEigen(snapshot.tcp_pose.p);
  const Eigen::Vector3d position_error =
      ApplyVectorDeadband(raw_position_error, config.ik.task_translation_deadband_m);
  task_error.head<3>() = config.ik.position_gain * position_error;

  if (control_mode == ControlMode::kPose) {
    const Eigen::Vector3d raw_rotation_error =
        QuaternionErrorAngleAxis(ToEigenQuat(snapshot.tcp_pose.q), ToEigenQuat(desired_pose.q));
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
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>(reference_q.data());
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
  std::array<double, 7> q_next = reference_q;
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

class JointTargetSmoother {
 public:
  explicit JointTargetSmoother(const TeleopBridgeConfig& config)
      : dt_(1.0 / std::max(config.teleop.planner_rate_hz, 1.0)),
        max_velocity_(std::max(config.ik.max_joint_velocity_radps, 1e-6)),
        max_acceleration_(std::max(config.ik.max_joint_acceleration_radps2, 1e-6)),
        max_step_(std::max(0.0, config.ik.max_joint_step_rad)),
        alpha_(std::clamp(config.ik.target_smoothing_alpha, 0.0, 1.0)) {}

  void Reset() {
    initialized_ = false;
    prev_q_.fill(0.0);
    prev_dq_.fill(0.0);
  }

  std::array<double, 7> Update(const std::array<double, 7>& raw_target_q,
                               const std::array<double, 7>& reference_q) {
    if (!initialized_) {
      prev_q_ = reference_q;
      prev_dq_.fill(0.0);
      initialized_ = true;
    }

    std::array<double, 7> q_out = prev_q_;
    std::array<double, 7> dq_out = prev_dq_;

    const double max_step_from_velocity = max_velocity_ * dt_;
    const double max_step = max_step_ > 0.0 ? std::min(max_step_, max_step_from_velocity)
                                            : max_step_from_velocity;
    const double max_delta_velocity = max_acceleration_ * dt_;

    for (size_t i = 0; i < 7; ++i) {
      const double raw_vel = (raw_target_q[i] - prev_q_[i]) / dt_;
      const double blended_vel = alpha_ * raw_vel + (1.0 - alpha_) * prev_dq_[i];
      const double velocity_limited = std::clamp(blended_vel, -max_velocity_, max_velocity_);
      const double accel_limited = prev_dq_[i] +
                                   std::clamp(velocity_limited - prev_dq_[i],
                                              -max_delta_velocity,
                                              max_delta_velocity);
      const double step = std::clamp(accel_limited * dt_, -max_step, max_step);
      q_out[i] = prev_q_[i] + step;
      dq_out[i] = step / dt_;
    }

    prev_q_ = q_out;
    prev_dq_ = dq_out;
    return q_out;
  }

 private:
  double dt_ = 0.01;
  double max_velocity_ = 0.35;
  double max_acceleration_ = 1.5;
  double max_step_ = 0.008;
  double alpha_ = 0.25;
  bool initialized_ = false;
  std::array<double, 7> prev_q_{};
  std::array<double, 7> prev_dq_{};
};

void PlannerLoop(const TeleopBridgeConfig& config,
                 const franka::Model& model,
                 const LatestCommandBuffer* command_buffer,
                 const LatestRobotStateBuffer* robot_state_buffer,
                 LatestPlannedTargetBuffer* planned_target_buffer,
                 TraceRecorder* trace_recorder,
                 uint32_t trace_decimation,
                 std::atomic<double>* desired_gripper_width_m,
                 std::atomic<bool>* stop_requested) {
  try {
    const auto sleep_period =
        std::chrono::microseconds(static_cast<int64_t>(1e6 / config.teleop.planner_rate_hz));

    SafetyFilter safety(config.safety, config.teleop.planner_rate_hz);
    TeleopMapper mapper(config);
    TeleopStateMachine state_machine;
    bool deadman_latched = false;
    bool planner_q_ref_initialized = false;
    std::array<double, 7> planner_q_ref{};
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
      planned.target_gripper_width_m = MapTriggerToWidth(config.gripper, xr_cmd.gripper_trigger_value);
      planned.requested_action.gripper_command = Clamp01(xr_cmd.gripper_trigger_value);
      desired_gripper_width_m->store(planned.target_gripper_width_m, std::memory_order_release);
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
        planner_q_ref_initialized = false;
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
        planner_q_ref_initialized = false;
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
      if (!planner_q_ref_initialized) {
        planner_q_ref = robot.q;
        planner_q_ref_initialized = true;
      }

      double manipulability = 0.0;
      ik_ok = SolveIkStep(model,
                          robot,
                          planner_q_ref,
                          safe_pose,
                          config,
                          planned.control_mode,
                          &q_target,
                          &manipulability);
      if (!ik_ok) {
        planned.faults.ik_rejected = true;
        planned.control_mode = ControlMode::kHold;
        planner_q_ref_initialized = false;
        planned_target_buffer->Publish(planned);
        publish_trace();
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      planned.target_q = q_target;
      planner_q_ref = q_target;
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
                 const std::atomic<double>* desired_gripper_width_m,
                 std::atomic<double>* measured_gripper_width_m,
                 std::atomic<bool>* stop_requested) {
  try {
    const double min_period_s = 1.0 / std::max(config.max_command_rate_hz, 1.0);
    double last_sent_width = ClampWidth(config, desired_gripper_width_m->load(std::memory_order_acquire));
    uint64_t last_command_time_ns = 0;

    while (!stop_requested->load(std::memory_order_acquire)) {
      const uint64_t now_ns = MonotonicNowNs();
      const double desired_width = ClampWidth(
          config, desired_gripper_width_m->load(std::memory_order_acquire));
      const double elapsed_s = last_command_time_ns == 0
                                   ? std::numeric_limits<double>::infinity()
                                   : static_cast<double>(now_ns - last_command_time_ns) * 1e-9;

      if (std::abs(desired_width - last_sent_width) >= config.min_command_delta_m &&
          elapsed_s >= min_period_s) {
        try {
          (void)gripper->move(desired_width, config.speed_mps);
          last_sent_width = desired_width;
          last_command_time_ns = now_ns;
        } catch (const std::exception& e) {
          std::cerr << "Gripper command failed: " << e.what() << "\n";
        }
      }

      try {
        const franka::GripperState state = gripper->readOnce();
        measured_gripper_width_m->store(state.width, std::memory_order_release);
      } catch (const std::exception&) {
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
        const franka::GripperState state = gripper->readOnce();
        measured_gripper_width_m.store(state.width, std::memory_order_release);
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

    std::atomic<double> desired_gripper_width_m{config_.gripper.max_width_m};
    planner_thread = std::thread(PlannerLoop,
                                 std::cref(config_),
                                 std::cref(model),
                                 command_buffer_,
                                 &robot_state_buffer,
                                 &planned_target_buffer,
                                 trace_recorder.get(),
                                 std::max<uint32_t>(1, options_.trace.planner_decimation),
                                 &desired_gripper_width_m,
                                 stop_requested);

    if (gripper != nullptr && config_.allow_motion) {
      gripper_thread = std::thread(GripperLoop,
                                   gripper.get(),
                                   std::cref(config_.gripper),
                                   &desired_gripper_width_m,
                                   &measured_gripper_width_m,
                                   stop_requested);
    }

    uint64_t rt_last_ns = 0;
    uint64_t rt_trace_counter = 0;
    uint64_t last_success_rate_log_ns = 0;
    std::array<double, 7> prev_command_velocity{};
    bool prev_command_velocity_initialized = false;
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
      const double max_velocity = std::max(config_.ik.max_joint_velocity_radps, 1e-6);
      const double max_delta_velocity =
          std::max(config_.ik.max_joint_acceleration_radps2, 1e-6) * dt;
      const double rt_alpha = std::clamp(config_.ik.realtime_target_smoothing_alpha, 0.0, 1.0);
      const double joint_deadzone = std::max(0.0, config_.ik.realtime_joint_deadzone_rad);
      if (!apply_motion) {
        prev_command_velocity.fill(0.0);
        prev_command_velocity_initialized = false;
      } else if (!prev_command_velocity_initialized) {
        prev_command_velocity.fill(0.0);
        prev_command_velocity_initialized = true;
      }
      std::array<double, 7> q_cmd = state.q_d;
      std::array<double, 7> target_delta{};
      std::array<double, 7> filtered_delta{};
      std::array<double, 7> command_delta{};
      std::array<uint8_t, 7> clamp_saturated{};
      double max_abs_target_delta = 0.0;
      double max_abs_filtered_delta = 0.0;
      double max_abs_command_delta = 0.0;
      for (size_t i = 0; i < 7; ++i) {
        target_delta[i] = planned.target_q[i] - state.q_d[i];
        if (std::abs(target_delta[i]) < joint_deadzone) {
          target_delta[i] = 0.0;
        }
        if (apply_motion) {
          filtered_delta[i] = rt_alpha * target_delta[i];
          const double desired_velocity = filtered_delta[i] / dt;
          const double velocity_limited = std::clamp(desired_velocity, -max_velocity, max_velocity);
          const double accel_limited_velocity =
              prev_command_velocity[i] +
              std::clamp(velocity_limited - prev_command_velocity[i],
                         -max_delta_velocity,
                         max_delta_velocity);
          const double unclamped_delta = accel_limited_velocity * dt;
          const double clamped_delta = std::clamp(unclamped_delta, -max_step, max_step);
          q_cmd[i] = state.q_d[i] + clamped_delta;
          command_delta[i] = clamped_delta;
          prev_command_velocity[i] = clamped_delta / dt;
          if (std::abs(unclamped_delta) > max_step + 1e-12) {
            clamp_saturated[i] = 1;
          }
        }
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
