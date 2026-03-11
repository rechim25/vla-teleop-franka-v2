#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

#include "common_types.h"
#include "math_utils.h"
#include "safety.h"
#include "teleop_mapper.h"

namespace {

bool NearlyEqual(double a, double b, double eps = 1e-6) {
  return std::abs(a - b) <= eps;
}

void TestSafetyTranslationShaping() {
  teleop::SafetyLimits limits;
  limits.max_translation_speed_mps = 0.10;
  limits.max_step_translation_m = 0.001;
  limits.max_rotation_speed_rps = 1.0;
  limits.max_step_rotation_rad = 0.1;

  teleop::SafetyFilter filter(limits);
  teleop::Pose current{};
  teleop::Pose desired = current;
  desired.p = {0.10, 0.0, 0.0};
  teleop::FaultFlags faults{};
  teleop::Pose safe{};

  const bool ok = filter.FilterTargetPose(current, desired, 0.001, 0.01, &faults, &safe);
  if (!ok) {
    throw std::runtime_error("TestSafetyTranslationShaping: target unexpectedly rejected");
  }
  if (!(safe.p[0] > 0.0 && safe.p[0] <= (0.001 + 1e-6))) {
    throw std::runtime_error("TestSafetyTranslationShaping: translation exceeded step/speed limits");
  }
}

void TestSafetyRotationShaping() {
  teleop::SafetyLimits limits;
  limits.max_translation_speed_mps = 1.0;
  limits.max_step_translation_m = 0.1;
  limits.max_rotation_speed_rps = 1.0;
  limits.max_step_rotation_rad = 0.01;

  teleop::SafetyFilter filter(limits);
  teleop::Pose current{};
  teleop::Pose desired = current;
  const Eigen::Quaterniond q_target(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  desired.q = teleop::ToArrayQuat(q_target);

  teleop::FaultFlags faults{};
  teleop::Pose safe{};
  const bool ok = filter.FilterTargetPose(current, desired, 0.001, 0.01, &faults, &safe);
  if (!ok) {
    throw std::runtime_error("TestSafetyRotationShaping: target unexpectedly rejected");
  }

  const Eigen::Vector3d rot_err =
      teleop::QuaternionErrorAngleAxis(teleop::ToEigenQuat(current.q), teleop::ToEigenQuat(safe.q));
  if (!(rot_err.norm() > 0.0 && rot_err.norm() <= (0.01 + 1e-4))) {
    throw std::runtime_error("TestSafetyRotationShaping: rotation exceeded step/speed limits");
  }
}

void TestClutchRecenter() {
  teleop::TeleopBridgeConfig config;
  config.teleop.scale_factor = 1.0;
  teleop::TeleopMapper mapper(config);

  teleop::Pose robot_pose{};
  teleop::XRCommand cmd{};
  teleop::Pose target{};
  teleop::TeleopAction action{};

  const bool first = mapper.ComputeTargetPose(
      robot_pose, cmd, true, false, teleop::ControlMode::kPosition, &target, &action);
  if (first) {
    throw std::runtime_error("TestClutchRecenter: first frame should only initialize anchor");
  }

  cmd.right_controller_pose.p = {1.0, 0.0, 0.0};
  robot_pose.p = {0.2, 0.0, 0.0};
  const bool clutch_hold = mapper.ComputeTargetPose(
      robot_pose, cmd, true, true, teleop::ControlMode::kPosition, &target, &action);
  if (clutch_hold) {
    throw std::runtime_error("TestClutchRecenter: clutch hold should freeze motion");
  }

  const bool after_release = mapper.ComputeTargetPose(
      robot_pose, cmd, true, false, teleop::ControlMode::kPosition, &target, &action);
  if (!after_release) {
    throw std::runtime_error("TestClutchRecenter: release should produce a target");
  }
  if (!NearlyEqual(target.p[0], robot_pose.p[0], 1e-6)) {
    throw std::runtime_error("TestClutchRecenter: release caused a pose jump");
  }
}

}  // namespace

int main() {
  try {
    TestSafetyTranslationShaping();
    TestSafetyRotationShaping();
    TestClutchRecenter();
    std::cout << "teleop_bridge_safety_mapper_test: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "teleop_bridge_safety_mapper_test: FAIL: " << e.what() << "\n";
    return 1;
  }
}
