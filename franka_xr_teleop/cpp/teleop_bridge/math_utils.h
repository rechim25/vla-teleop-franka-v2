#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common_types.h"

namespace teleop {

inline uint64_t MonotonicNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

inline Eigen::Vector3d ToEigen(const std::array<double, 3>& value) {
  return Eigen::Vector3d(value[0], value[1], value[2]);
}

inline std::array<double, 3> ToArray3(const Eigen::Vector3d& value) {
  return {value.x(), value.y(), value.z()};
}

inline Eigen::Quaterniond ToEigenQuat(const std::array<double, 4>& q_xyzw) {
  return Eigen::Quaterniond(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]).normalized();
}

inline std::array<double, 4> ToArrayQuat(const Eigen::Quaterniond& q_wxyz) {
  Eigen::Quaterniond normalized = q_wxyz.normalized();
  return {normalized.x(), normalized.y(), normalized.z(), normalized.w()};
}

inline Pose MatrixToPose(const std::array<double, 16>& t) {
  Pose p;
  p.p = {t[12], t[13], t[14]};

  Eigen::Matrix3d rotation;
  rotation << t[0], t[4], t[8], t[1], t[5], t[9], t[2], t[6], t[10];
  p.q = ToArrayQuat(Eigen::Quaterniond(rotation));
  return p;
}

inline std::array<double, 16> PoseToMatrix(const Pose& pose) {
  const Eigen::Quaterniond q = ToEigenQuat(pose.q);
  const Eigen::Matrix3d rotation = q.toRotationMatrix();

  std::array<double, 16> out{};
  out[0] = rotation(0, 0);
  out[1] = rotation(1, 0);
  out[2] = rotation(2, 0);
  out[3] = 0.0;

  out[4] = rotation(0, 1);
  out[5] = rotation(1, 1);
  out[6] = rotation(2, 1);
  out[7] = 0.0;

  out[8] = rotation(0, 2);
  out[9] = rotation(1, 2);
  out[10] = rotation(2, 2);
  out[11] = 0.0;

  out[12] = pose.p[0];
  out[13] = pose.p[1];
  out[14] = pose.p[2];
  out[15] = 1.0;
  return out;
}

inline Eigen::Vector3d QuaternionErrorAngleAxis(const Eigen::Quaterniond& source,
                                                const Eigen::Quaterniond& target) {
  Eigen::Quaterniond delta = target * source.conjugate();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }
  Eigen::AngleAxisd angle_axis(delta.normalized());
  if (!std::isfinite(angle_axis.angle())) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

inline TeleopAction DescribeAction(const Pose& from_pose,
                                   const Pose& to_pose,
                                   double gripper_command) {
  TeleopAction action;
  action.delta_translation_m = {
      to_pose.p[0] - from_pose.p[0],
      to_pose.p[1] - from_pose.p[1],
      to_pose.p[2] - from_pose.p[2],
  };
  action.delta_rotation_rad =
      ToArray3(QuaternionErrorAngleAxis(ToEigenQuat(from_pose.q), ToEigenQuat(to_pose.q)));
  action.gripper_command = std::clamp(gripper_command, 0.0, 1.0);
  return action;
}

inline double Clamp01(double value) {
  return std::clamp(value, 0.0, 1.0);
}

}  // namespace teleop
