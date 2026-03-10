#include "config_loader.h"

#include <filesystem>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace teleop {
namespace {

template <typename T>
bool ReadScalar(const YAML::Node& node, const char* key, T* out) {
  if (!node[key]) {
    return false;
  }
  *out = node[key].as<T>();
  return true;
}

template <size_t N>
bool ReadArray(const YAML::Node& node, const char* key, std::array<double, N>* out) {
  if (!node[key] || !node[key].IsSequence() || node[key].size() != N) {
    return false;
  }
  for (size_t i = 0; i < N; ++i) {
    (*out)[i] = node[key][i].as<double>();
  }
  return true;
}

bool ReadMatrix3(const YAML::Node& node,
                 const char* key,
                 std::array<std::array<double, 3>, 3>* out) {
  if (!node[key] || !node[key].IsSequence() || node[key].size() != 3) {
    return false;
  }
  for (size_t row = 0; row < 3; ++row) {
    if (!node[key][row].IsSequence() || node[key][row].size() != 3) {
      return false;
    }
    for (size_t col = 0; col < 3; ++col) {
      (*out)[row][col] = node[key][row][col].as<double>();
    }
  }
  return true;
}

std::string JoinPath(const std::string& root, const std::string& leaf) {
  return (std::filesystem::path(root) / leaf).string();
}

bool LoadRobotConfig(const std::string& path, AppConfig* config, std::string* error) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node robot = root["robot"];
  if (!robot || !robot.IsMap()) {
    *error = "Missing 'robot' map in " + path;
    return false;
  }
  ReadScalar(robot, "ip", &config->bridge.robot_ip);
  return true;
}

bool LoadSafetyConfig(const std::string& path, AppConfig* config, std::string* error) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node safety = root["safety"];
  if (!safety || !safety.IsMap()) {
    *error = "Missing 'safety' map in " + path;
    return false;
  }
  ReadScalar(safety, "packet_timeout_s", &config->bridge.safety.packet_timeout_s);
  ReadScalar(safety, "jump_reject_translation_m", &config->bridge.safety.jump_reject_translation_m);
  ReadScalar(safety, "jump_reject_rotation_rad", &config->bridge.safety.jump_reject_rotation_rad);
  ReadArray(safety, "workspace_min_xyz", &config->bridge.safety.workspace_min);
  ReadArray(safety, "workspace_max_xyz", &config->bridge.safety.workspace_max);
  return true;
}

bool LoadTeleopConfig(const std::string& path, AppConfig* config, std::string* error) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node teleop = root["teleop"];
  if (!teleop || !teleop.IsMap()) {
    *error = "Missing 'teleop' map in " + path;
    return false;
  }

  ReadScalar(teleop, "observation_ip", &config->observation_ip);
  int observation_port = static_cast<int>(config->observation_port);
  if (ReadScalar(teleop, "observation_port", &observation_port)) {
    config->observation_port = static_cast<uint16_t>(observation_port);
  }
  ReadScalar(teleop, "allow_motion", &config->bridge.allow_motion);
  ReadScalar(teleop, "dry_run", &config->dry_run);
  ReadScalar(teleop, "scale_factor", &config->bridge.teleop.scale_factor);
  ReadScalar(teleop, "control_trigger_threshold", &config->bridge.teleop.control_trigger_threshold);
  ReadScalar(teleop, "planner_rate_hz", &config->bridge.teleop.planner_rate_hz);
  ReadArray(teleop, "start_joint_positions_rad", &config->bridge.teleop.start_joint_positions_rad);

  std::string control_mode;
  if (ReadScalar(teleop, "control_mode", &control_mode)) {
    if (!ParseControlMode(control_mode, &config->bridge.teleop.control_mode) ||
        config->bridge.teleop.control_mode == ControlMode::kHold) {
      *error = "Unsupported teleop.control_mode '" + control_mode + "' in " + path;
      return false;
    }
  }

  if (const YAML::Node ik = teleop["ik"]; ik && ik.IsMap()) {
    ReadScalar(ik, "damping", &config->bridge.ik.damping);
    ReadScalar(ik, "nullspace_gain", &config->bridge.ik.nullspace_gain);
    ReadScalar(ik, "max_joint_velocity_radps", &config->bridge.ik.max_joint_velocity_radps);
    ReadScalar(ik, "max_joint_step_rad", &config->bridge.ik.max_joint_step_rad);
    ReadScalar(ik, "position_gain", &config->bridge.ik.position_gain);
    ReadScalar(ik, "orientation_gain", &config->bridge.ik.orientation_gain);
    ReadScalar(ik, "manipulability_threshold", &config->bridge.ik.manipulability_threshold);
    ReadScalar(ik, "singularity_damping_gain", &config->bridge.ik.singularity_damping_gain);
  }

  if (const YAML::Node gripper = teleop["gripper"]; gripper && gripper.IsMap()) {
    ReadScalar(gripper, "enabled", &config->bridge.gripper.enabled);
    ReadScalar(gripper, "max_width_m", &config->bridge.gripper.max_width_m);
    ReadScalar(gripper, "min_width_m", &config->bridge.gripper.min_width_m);
    ReadScalar(gripper, "speed_mps", &config->bridge.gripper.speed_mps);
    ReadScalar(gripper, "min_command_delta_m", &config->bridge.gripper.min_command_delta_m);
    ReadScalar(gripper, "max_command_rate_hz", &config->bridge.gripper.max_command_rate_hz);
  }

  return true;
}

bool LoadXrFrameConfig(const std::string& path, AppConfig* config, std::string* error) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node xr_frame = root["xr_frame"];
  if (!xr_frame || !xr_frame.IsMap()) {
    *error = "Missing 'xr_frame' map in " + path;
    return false;
  }
  if (!ReadMatrix3(xr_frame, "rotation_matrix_row_major", &config->bridge.xr_to_robot_rotation)) {
    *error = "Missing/invalid xr_frame.rotation_matrix_row_major in " + path;
    return false;
  }
  return true;
}

}  // namespace

bool LoadAppConfig(const std::string& config_dir, AppConfig* config, std::string* error) {
  try {
    if (!LoadRobotConfig(JoinPath(config_dir, "robot.yaml"), config, error)) {
      return false;
    }
    if (!LoadSafetyConfig(JoinPath(config_dir, "safety.yaml"), config, error)) {
      return false;
    }
    if (!LoadTeleopConfig(JoinPath(config_dir, "teleop.yaml"), config, error)) {
      return false;
    }
    if (!LoadXrFrameConfig(JoinPath(config_dir, "xr_frame.yaml"), config, error)) {
      return false;
    }
  } catch (const std::exception& e) {
    *error = std::string("Failed to load config: ") + e.what();
    return false;
  }

  if (config->bridge.robot_ip.empty()) {
    *error = "robot.ip is required";
    return false;
  }
  if (config->bridge.teleop.planner_rate_hz <= 1.0) {
    *error = "teleop.planner_rate_hz must be > 1.0";
    return false;
  }
  if (config->bridge.teleop.control_trigger_threshold < 0.0 ||
      config->bridge.teleop.control_trigger_threshold > 1.0) {
    *error = "teleop.control_trigger_threshold must be in [0, 1]";
    return false;
  }
  if (config->bridge.gripper.min_width_m < 0.0 ||
      config->bridge.gripper.max_width_m < config->bridge.gripper.min_width_m) {
    *error = "Invalid gripper width range";
    return false;
  }
  return true;
}

}  // namespace teleop
