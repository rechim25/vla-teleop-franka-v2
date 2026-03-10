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
  robot->setJointImpedance({{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0}});
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

bool SolveIkStep(const franka::Model& model,
                 const RobotSnapshot& snapshot,
                 const Pose& desired_pose,
                 const TeleopBridgeConfig& config,
                 ControlMode control_mode,
                 std::array<double, 7>* target_q,
                 double* manipulability) {
  if (control_mode == ControlMode::kHold) {
    *target_q = snapshot.q;
    *manipulability = 0.0;
    return true;
  }

  const Eigen::Matrix<double, 6, 7> jacobian = JacobianToEigen(
      model.zeroJacobian(franka::Frame::kEndEffector, snapshot.q, snapshot.F_T_EE, snapshot.EE_T_K));
  Eigen::Matrix<double, 6, 7> jacobian_task = jacobian;

  Eigen::Matrix<double, 6, 1> task_error = Eigen::Matrix<double, 6, 1>::Zero();
  const Eigen::Vector3d position_error = ToEigen(desired_pose.p) - ToEigen(snapshot.tcp_pose.p);
  task_error.head<3>() = config.ik.position_gain * position_error;

  if (control_mode == ControlMode::kPose) {
    const Eigen::Vector3d rotation_error =
        QuaternionErrorAngleAxis(ToEigenQuat(snapshot.tcp_pose.q), ToEigenQuat(desired_pose.q));
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
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>(snapshot.q.data());
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
  std::array<double, 7> q_next = snapshot.q;
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

void PlannerLoop(const TeleopBridgeConfig& config,
                 const franka::Model& model,
                 const LatestCommandBuffer* command_buffer,
                 const LatestRobotStateBuffer* robot_state_buffer,
                 LatestPlannedTargetBuffer* planned_target_buffer,
                 std::atomic<double>* desired_gripper_width_m,
                 std::atomic<bool>* stop_requested) {
  try {
    const auto sleep_period =
        std::chrono::microseconds(static_cast<int64_t>(1e6 / config.teleop.planner_rate_hz));

    SafetyFilter safety(config.safety);
    TeleopMapper mapper(config);
    TeleopStateMachine state_machine;

    while (!stop_requested->load(std::memory_order_acquire)) {
      const uint64_t now_ns = MonotonicNowNs();
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

      if (robot.timestamp_ns == 0) {
        planned_target_buffer->Publish(planned);
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      const uint64_t packet_age_ns = now_ns > xr_cmd.timestamp_ns ? (now_ns - xr_cmd.timestamp_ns) : 0;
      planned.packet_age_ns = packet_age_ns;
      const double packet_age_s = static_cast<double>(packet_age_ns) * 1e-9;

      StateInputs inputs{};
      inputs.xr_stream_healthy = safety.IsStreamHealthy(packet_age_s);
      inputs.deadman_pressed =
          (xr_cmd.control_trigger_value >= config.teleop.control_trigger_threshold);
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
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      planned.control_mode = config.teleop.control_mode;
      Pose desired_pose = robot.tcp_pose;
      TeleopAction requested_action{};
      const bool has_target =
          mapper.ComputeTargetPose(robot.tcp_pose,
                                   xr_cmd,
                                   true,
                                   planned.control_mode,
                                   &desired_pose,
                                   &requested_action);
      planned.requested_action = requested_action;
      if (!has_target) {
        planned.control_mode = ControlMode::kHold;
        planned_target_buffer->Publish(planned);
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      Pose safe_pose{};
      const bool safe_target = safety.FilterTargetPose(
          robot.tcp_pose, desired_pose, packet_age_s, &planned.faults, &safe_pose);
      planned.desired_tcp_pose = safe_pose;
      if (!safe_target) {
        if (planned.faults.jump_rejected) {
          // Re-anchor immediately so long motions recover without deadman release/re-press.
          mapper.Reset();
        }
        planned.control_mode = ControlMode::kHold;
        planned_target_buffer->Publish(planned);
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      double manipulability = 0.0;
      std::array<double, 7> q_target = robot.q;
      if (!SolveIkStep(model,
                       robot,
                       safe_pose,
                       config,
                       planned.control_mode,
                       &q_target,
                       &manipulability)) {
        planned.faults.ik_rejected = true;
        planned.control_mode = ControlMode::kHold;
        planned_target_buffer->Publish(planned);
        std::this_thread::sleep_for(sleep_period);
        continue;
      }

      planned.target_q = q_target;
      planned.manipulability = manipulability;
      planned.target_fresh = true;
      planned_target_buffer->Publish(planned);
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
  auto join_threads = [&]() {
    stop_requested->store(true, std::memory_order_release);
    if (planner_thread.joinable()) {
      planner_thread.join();
    }
    if (gripper_thread.joinable()) {
      gripper_thread.join();
    }
  };

  try {
    franka::Robot robot(options_.robot_ip);
    ConfigureConservativeBehavior(&robot);

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

    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
      const uint64_t now_ns = MonotonicNowNs();
      const RobotSnapshot robot_snapshot = ToSnapshot(state);
      robot_state_buffer.Publish(robot_snapshot);

      const PlannedTarget planned = planned_target_buffer.ReadLatest();
      const bool apply_motion = config_.allow_motion && planned.teleop_active && planned.target_fresh &&
                                planned.control_mode != ControlMode::kHold;

      std::array<double, 7> q_cmd = state.q_d;
      if (apply_motion) {
        const double dt = std::max(period.toSec(), 1e-6);
        const double max_step = std::min(config_.ik.max_joint_step_rad,
                                         config_.ik.max_joint_velocity_radps * dt);
        for (size_t i = 0; i < 7; ++i) {
          const double raw_delta = planned.target_q[i] - state.q_d[i];
          q_cmd[i] = state.q_d[i] + std::clamp(raw_delta, -max_step, max_step);
        }
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

      franka::JointPositions out(q_cmd);
      if (stop_requested->load(std::memory_order_acquire)) {
        return franka::MotionFinished(out);
      }
      return out;
    });

    join_threads();
    return 0;
  } catch (const franka::ControlException& e) {
    join_threads();
    std::cerr << "Franka control exception: " << e.what() << "\n";
    return 2;
  } catch (const franka::Exception& e) {
    join_threads();
    std::cerr << "Franka exception: " << e.what() << "\n";
    return 3;
  } catch (const std::exception& e) {
    join_threads();
    std::cerr << "Std exception: " << e.what() << "\n";
    return 4;
  } catch (...) {
    join_threads();
    std::cerr << "Unknown controller exception\n";
    return 7;
  }
}

}  // namespace teleop
