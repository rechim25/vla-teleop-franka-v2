#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "config_loader.h"

namespace {

void WriteFile(const std::filesystem::path& path, const std::string& body) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to write test file: " + path.string());
  }
  out << body;
}

std::filesystem::path CreateBaseConfigDir(const std::string& name) {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / ("teleop_bridge_config_test_" + name);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  WriteFile(dir / "robot.yaml", "robot:\n  ip: 192.168.2.200\n");
  WriteFile(dir / "safety.yaml",
            "safety:\n"
            "  max_translation_speed_mps: 0.20\n"
            "  max_rotation_speed_rps: 0.80\n"
            "  max_step_translation_m: 0.0015\n"
            "  max_step_rotation_rad: 0.010\n"
            "  jump_reject_translation_m: 0.08\n"
            "  jump_reject_rotation_rad: 0.80\n"
            "  workspace_min_xyz: [0.20, -0.45, 0.05]\n"
            "  workspace_max_xyz: [0.80, 0.45, 0.85]\n"
            "  packet_timeout_s: 0.120\n");
  WriteFile(dir / "teleop.yaml",
            "teleop:\n"
            "  observation_ip: 127.0.0.1\n"
            "  observation_port: 28081\n"
            "  allow_motion: true\n"
            "  dry_run: false\n"
            "  control_mode: position\n"
            "  scale_factor: 0.8\n"
            "  control_trigger_threshold: 0.9\n"
            "  planner_rate_hz: 100.0\n"
            "  target_timeout_s: 0.12\n"
            "  start_joint_positions_rad: [0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816]\n"
            "recording:\n"
            "  enabled: true\n"
            "  directory: logs/test\n"
            "  flush_hz: 25.0\n"
            "  max_buffer_entries: 1024\n");
  WriteFile(dir / "xr_frame.yaml",
            "xr_frame:\n"
            "  rotation_matrix_row_major:\n"
            "    - [0.0, 0.0, -1.0]\n"
            "    - [-1.0, 0.0, 0.0]\n"
            "    - [0.0, 1.0, 0.0]\n");
  return dir;
}

void TestValidConfig() {
  const std::filesystem::path dir = CreateBaseConfigDir("valid");
  teleop::AppConfig config;
  std::string error;
  const bool ok = teleop::LoadAppConfig(dir.string(), &config, &error);
  std::filesystem::remove_all(dir);
  if (!ok) {
    throw std::runtime_error("TestValidConfig: expected success, got: " + error);
  }
  if (config.bridge.teleop.control_mode != teleop::ControlMode::kPosition) {
    throw std::runtime_error("TestValidConfig: expected position control mode");
  }
  if (!config.recording.enabled || config.recording.flush_hz != 25.0 ||
      config.recording.max_buffer_entries != 1024) {
    throw std::runtime_error("TestValidConfig: recording config not loaded");
  }
}

void TestInvalidSafetyConfig() {
  const std::filesystem::path dir = CreateBaseConfigDir("invalid_safety");
  WriteFile(dir / "safety.yaml",
            "safety:\n"
            "  max_translation_speed_mps: 0.20\n"
            "  max_rotation_speed_rps: 0.80\n"
            "  max_step_translation_m: 0.0\n"
            "  max_step_rotation_rad: 0.010\n"
            "  jump_reject_translation_m: 0.08\n"
            "  jump_reject_rotation_rad: 0.80\n"
            "  workspace_min_xyz: [0.20, -0.45, 0.05]\n"
            "  workspace_max_xyz: [0.80, 0.45, 0.85]\n"
            "  packet_timeout_s: 0.120\n");

  teleop::AppConfig config;
  std::string error;
  const bool ok = teleop::LoadAppConfig(dir.string(), &config, &error);
  std::filesystem::remove_all(dir);
  if (ok) {
    throw std::runtime_error("TestInvalidSafetyConfig: expected validation failure");
  }
  if (error.find("Safety max speed/step values must be > 0") == std::string::npos) {
    throw std::runtime_error("TestInvalidSafetyConfig: unexpected error message: " + error);
  }
}

}  // namespace

int main() {
  try {
    TestValidConfig();
    TestInvalidSafetyConfig();
    std::cout << "teleop_bridge_config_loader_test: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "teleop_bridge_config_loader_test: FAIL: " << e.what() << "\n";
    return 1;
  }
}
