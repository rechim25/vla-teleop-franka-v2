#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "common_types.h"
#include "config_loader.h"
#include "franka_controller.h"
#include "observation_pub.h"
#include "teleop_state_machine.h"
#include "xrobotics_source.h"

namespace {

using json = nlohmann::json;

std::atomic<bool> g_stop_requested{false};

void HandleSignal(int) {
  g_stop_requested.store(true, std::memory_order_release);
}

struct Options {
  std::string config_dir = "configs";
  bool dry_run_override = false;
  bool dry_run = false;
  bool allow_motion_override = false;
  bool allow_motion = true;
  bool move_to_home_override = false;
  bool move_to_home = false;
  bool robot_ip_override = false;
  std::string robot_ip;
  bool obs_ip_override = false;
  std::string obs_ip;
  bool obs_port_override = false;
  uint16_t obs_port = 28081;
  bool control_mode_override = false;
  teleop::ControlMode control_mode = teleop::ControlMode::kPosition;
};

void PrintUsage(const char* prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " [--config-dir configs] [--dry-run] [--no-motion]\n"
            << "             [--move-to-home|--skip-home]\n"
            << "             [--robot-ip <ip>] [--obs-ip <ip>] [--obs-port <port>]\n"
            << "             [--control-mode <pose|position>]\n\n"
            << "Examples:\n"
            << "  " << prog << " --dry-run\n"
            << "  " << prog << " --robot-ip 192.168.2.200 --control-mode position --skip-home\n";
}

bool ParseArgs(int argc, char** argv, Options* out) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--config-dir") {
      if (i + 1 >= argc) {
        return false;
      }
      out->config_dir = argv[++i];
      continue;
    }
    if (arg == "--dry-run") {
      out->dry_run_override = true;
      out->dry_run = true;
      continue;
    }
    if (arg == "--no-motion") {
      out->allow_motion_override = true;
      out->allow_motion = false;
      continue;
    }
    if (arg == "--move-to-home") {
      out->move_to_home_override = true;
      out->move_to_home = true;
      continue;
    }
    if (arg == "--skip-home") {
      out->move_to_home_override = true;
      out->move_to_home = false;
      continue;
    }
    if (arg == "--robot-ip") {
      if (i + 1 >= argc) {
        return false;
      }
      out->robot_ip_override = true;
      out->robot_ip = argv[++i];
      continue;
    }
    if (arg == "--obs-ip") {
      if (i + 1 >= argc) {
        return false;
      }
      out->obs_ip_override = true;
      out->obs_ip = argv[++i];
      continue;
    }
    if (arg == "--obs-port") {
      if (i + 1 >= argc) {
        return false;
      }
      out->obs_port_override = true;
      out->obs_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      continue;
    }
    if (arg == "--control-mode") {
      if (i + 1 >= argc) {
        return false;
      }
      if (!teleop::ParseControlMode(argv[++i], &out->control_mode) ||
          out->control_mode == teleop::ControlMode::kHold) {
        return false;
      }
      out->control_mode_override = true;
      continue;
    }
    if (arg == "-h" || arg == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    }
    return false;
  }
  return true;
}

uint64_t MonotonicNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

template <size_t N>
json ToJsonArray(const std::array<double, N>& value) {
  json out = json::array();
  for (double v : value) {
    out.push_back(v);
  }
  return out;
}

json ObservationToJson(const teleop::RobotObservation& obs) {
  json payload;
  payload["timestamp_ns"] = obs.timestamp_ns;
  payload["robot_state"] = {
      {"q", ToJsonArray(obs.q)},
      {"dq", ToJsonArray(obs.dq)},
      {"tcp_position_xyz", ToJsonArray(obs.tcp_pose.p)},
      {"tcp_orientation_xyzw", ToJsonArray(obs.tcp_pose.q)},
      {"gripper_width", obs.gripper_width},
  };
  payload["executed_action"] = {
      {"cartesian_delta_translation", ToJsonArray(obs.executed_action.delta_translation_m)},
      {"cartesian_delta_rotation", ToJsonArray(obs.executed_action.delta_rotation_rad)},
      {"gripper_command", obs.executed_action.gripper_command},
  };
  json fault_flags;
  fault_flags["packet_timeout"] = obs.faults.packet_timeout;
  fault_flags["jump_rejected"] = obs.faults.jump_rejected;
  fault_flags["workspace_clamped"] = obs.faults.workspace_clamped;
  fault_flags["robot_not_ready"] = obs.faults.robot_not_ready;
  fault_flags["control_exception"] = obs.faults.control_exception;
  fault_flags["ik_rejected"] = obs.faults.ik_rejected;

  json status;
  status["control_mode"] = teleop::ToString(obs.control_mode);
  status["teleop_state"] = teleop::ToString(obs.teleop_state);
  status["packet_age_ns"] = obs.packet_age_ns;
  status["target_age_ns"] = obs.target_age_ns;
  status["target_fresh"] = obs.target_fresh;
  status["teleop_active"] = obs.teleop_active;
  status["target_manipulability"] = obs.target_manipulability;
  status["fault_flags"] = fault_flags;
  payload["status"] = status;
  return payload;
}

json XrInputToJson(const teleop::XRCommand& cmd) {
  json payload;
  payload["timestamp_ns"] = cmd.timestamp_ns;
  payload["sequence_id"] = cmd.sequence_id;
  payload["right_controller_pose"] = {
      {"position_xyz", ToJsonArray(cmd.right_controller_pose.p)},
      {"orientation_xyzw", ToJsonArray(cmd.right_controller_pose.q)},
  };
  payload["right_grip"] = cmd.control_trigger_value;
  payload["right_trigger"] = cmd.gripper_trigger_value;
  payload["button_a"] = cmd.button_a;
  payload["button_b"] = cmd.button_b;
  payload["right_axis_click"] = cmd.right_axis_click;
  return payload;
}

class JsonlSessionRecorder {
 public:
  explicit JsonlSessionRecorder(const teleop::RecordingConfig& config)
      : config_(config),
        flush_period_ns_(static_cast<uint64_t>(1e9 / config.flush_hz)) {}

  bool is_recording() const { return recording_; }
  bool enabled() const { return config_.enabled; }

  void ToggleRecording() {
    if (!enabled()) {
      return;
    }
    if (recording_) {
      StopAndSave();
    } else {
      Start();
    }
  }

  void DiscardActiveSession() {
    if (!recording_) {
      return;
    }
    pending_lines_.clear();
    if (out_.is_open()) {
      out_.close();
    }
    std::error_code ec;
    std::filesystem::remove(active_session_path_, ec);
    if (ec) {
      std::cerr << "Recorder warning: failed to delete discarded session: " << ec.message() << "\n";
    }
    recording_ = false;
    std::cout << "Recording stopped. Session discarded.\n";
  }

  void Append(const teleop::RobotObservation& obs,
              const teleop::XRCommand& cmd,
              uint64_t timestamp_ns) {
    if (!recording_) {
      return;
    }

    json entry;
    entry["timestamp_ns"] = timestamp_ns;
    entry["session_file"] = active_session_path_;
    entry["observation"] = ObservationToJson(obs);
    entry["xr_input"] = XrInputToJson(cmd);
    pending_lines_.push_back(entry.dump());

    const bool flush_due_to_size = pending_lines_.size() >= config_.max_buffer_entries;
    const bool flush_due_to_time = timestamp_ns > last_flush_ns_ &&
                                   (timestamp_ns - last_flush_ns_) >= flush_period_ns_;
    if (flush_due_to_size || flush_due_to_time) {
      Flush();
    }
  }

  void Shutdown() {
    if (recording_) {
      StopAndSave();
    } else if (out_.is_open()) {
      out_.close();
    }
  }

 private:
  void Start() {
    pending_lines_.clear();
    const auto wall_now = std::chrono::system_clock::now();
    const std::time_t wall_time = std::chrono::system_clock::to_time_t(wall_now);
    std::tm tm_local{};
    localtime_r(&wall_time, &tm_local);

    std::ostringstream filename;
    filename << "teleop_session_" << std::put_time(&tm_local, "%Y%m%d_%H%M%S")
             << "_" << (++session_index_) << ".jsonl";

    std::filesystem::path out_path(config_.directory);
    std::error_code ec;
    std::filesystem::create_directories(out_path, ec);
    if (ec) {
      std::cerr << "Recorder error: failed to create log directory '" << config_.directory
                << "': " << ec.message() << "\n";
      return;
    }

    active_session_path_ = (out_path / filename.str()).string();
    out_.open(active_session_path_, std::ios::out | std::ios::trunc);
    if (!out_.is_open()) {
      std::cerr << "Recorder error: failed to open '" << active_session_path_ << "'\n";
      return;
    }
    recording_ = true;
    last_flush_ns_ = MonotonicNowNs();
    std::cout << "Recording started: " << active_session_path_ << "\n";
  }

  void StopAndSave() {
    Flush();
    if (out_.is_open()) {
      out_.close();
    }
    recording_ = false;
    std::cout << "Recording stopped. Session saved: " << active_session_path_ << "\n";
  }

  void Flush() {
    if (!out_.is_open() || pending_lines_.empty()) {
      return;
    }
    for (const std::string& line : pending_lines_) {
      out_ << line << '\n';
    }
    out_.flush();
    pending_lines_.clear();
    last_flush_ns_ = MonotonicNowNs();
  }

  teleop::RecordingConfig config_{};
  uint64_t flush_period_ns_ = 0;
  uint64_t last_flush_ns_ = 0;
  uint64_t session_index_ = 0;
  bool recording_ = false;
  std::string active_session_path_;
  std::ofstream out_;
  std::vector<std::string> pending_lines_;
};

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  teleop::AppConfig config;
  std::string config_error;
  if (!teleop::LoadAppConfig(options.config_dir, &config, &config_error)) {
    if (options.config_dir == "configs") {
      const std::string fallback_config_dir = "franka_xr_teleop/configs";
      if (!teleop::LoadAppConfig(fallback_config_dir, &config, &config_error)) {
        std::cerr << "Config error: " << config_error << "\n";
        return 1;
      }
    } else {
      std::cerr << "Config error: " << config_error << "\n";
      return 1;
    }
  }

  if (options.dry_run_override) {
    config.dry_run = options.dry_run;
  }
  if (options.allow_motion_override) {
    config.bridge.allow_motion = options.allow_motion;
  }
  if (options.move_to_home_override) {
    config.bridge.teleop.move_to_home_on_startup = options.move_to_home;
  }
  if (options.robot_ip_override) {
    config.bridge.robot_ip = options.robot_ip;
  }
  if (options.obs_ip_override) {
    config.observation_ip = options.obs_ip;
  }
  if (options.obs_port_override) {
    config.observation_port = options.obs_port;
  }
  if (options.control_mode_override) {
    config.bridge.teleop.control_mode = options.control_mode;
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  teleop::LatestCommandBuffer command_buffer;
  teleop::LatestObservationBuffer observation_buffer;

  teleop::XrRoboticsSource xr_source(&command_buffer, &g_stop_requested);
  if (!xr_source.Start()) {
    std::cerr << "Failed to initialize XRoboToolkit SDK source.\n"
              << "Ensure XRoboToolkit PC Service is installed and running "
              << "(for example /opt/apps/roboticsservice/runService.sh).\n";
    return 2;
  }

  teleop::ObservationPublisher observation_pub(config.observation_ip, config.observation_port);
  observation_pub.Start();
  JsonlSessionRecorder recorder(config.recording);

  std::thread observation_thread([&]() {
    bool prev_b_button = false;
    while (!g_stop_requested.load(std::memory_order_acquire)) {
      const teleop::RobotObservation obs = observation_buffer.ReadLatest();
      observation_pub.Publish(obs);
      if (recorder.enabled()) {
        const teleop::XRCommand cmd = command_buffer.ReadLatest();
        const bool deadman_pressed =
            cmd.control_trigger_value >= config.bridge.teleop.control_trigger_threshold;
        if (cmd.button_b && !prev_b_button && !deadman_pressed) {
          recorder.ToggleRecording();
        }
        prev_b_button = cmd.button_b;
        if (cmd.right_axis_click && recorder.is_recording()) {
          recorder.DiscardActiveSession();
        }
        recorder.Append(obs, cmd, MonotonicNowNs());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    recorder.Shutdown();
  });

  if (config.dry_run) {
    std::cout << "Dry-run mode: receiving XR state via XRoboToolkit PC Service callbacks\n";
    uint64_t last_print_ns = 0;
    while (!g_stop_requested.load(std::memory_order_acquire)) {
      const uint64_t now_ns = MonotonicNowNs();
      if (now_ns - last_print_ns > 500000000ULL) {
        const teleop::XRCommand cmd = command_buffer.ReadLatest();
        const uint64_t age_ns = now_ns > cmd.timestamp_ns ? (now_ns - cmd.timestamp_ns) : 0;
        std::cout << "server_connected=" << (xr_source.server_connected() ? 1 : 0)
                  << " device_connected=" << (xr_source.device_connected() ? 1 : 0)
                  << " rx_count=" << xr_source.received_count()
                  << " dropped=" << xr_source.dropped_count()
                  << " seq=" << cmd.sequence_id
                  << " age_ms=" << (age_ns * 1e-6)
                  << " right_grip=" << cmd.control_trigger_value
                  << " right_trigger=" << cmd.gripper_trigger_value
                  << " A=" << (cmd.button_a ? 1 : 0)
                  << " B=" << (cmd.button_b ? 1 : 0)
                  << " right_axis_click=" << (cmd.right_axis_click ? 1 : 0)
                  << "\n";
        last_print_ns = now_ns;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    teleop::FrankaControllerOptions controller_options;
    controller_options.robot_ip = config.bridge.robot_ip;
    teleop::FrankaTeleopController controller(
        controller_options, config.bridge, &command_buffer, &observation_buffer);

    const int rc = controller.Run(&g_stop_requested);
    g_stop_requested.store(true, std::memory_order_release);
    xr_source.Stop();
    if (observation_thread.joinable()) {
      observation_thread.join();
    }
    observation_pub.Stop();
    return rc;
  }

  g_stop_requested.store(true, std::memory_order_release);
  xr_source.Stop();
  if (observation_thread.joinable()) {
    observation_thread.join();
  }
  observation_pub.Stop();
  return 0;
}
