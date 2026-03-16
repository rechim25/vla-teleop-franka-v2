#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "common_types.h"
#include "config_loader.h"
#include "franka_controller.h"
#include "observation_pub.h"
#include "xrobotics_source.h"

namespace {

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
  bool robot_ip_override = false;
  std::string robot_ip;
  bool obs_ip_override = false;
  std::string obs_ip;
  bool obs_port_override = false;
  uint16_t obs_port = 28081;
  bool control_mode_override = false;
  teleop::ControlMode control_mode = teleop::ControlMode::kPose;
  bool trace_enabled = false;
  std::string trace_dir = "teleop_trace";
  uint32_t trace_planner_decimation = 1;
  uint32_t trace_rt_decimation = 1;
};

void PrintUsage(const char* prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " [--config-dir configs] [--dry-run] [--no-motion]\n"
            << "             [--robot-ip <ip>] [--obs-ip <ip>] [--obs-port <port>]\n"
            << "             [--control-mode <pose|position>]\n"
            << "             [--trace-dir <dir>] [--trace-planner-decimation <N>] "
               "[--trace-rt-decimation <N>]\n\n"
            << "Examples:\n"
            << "  " << prog << " --dry-run\n"
            << "  " << prog << " --robot-ip 192.168.2.200 --control-mode position\n"
            << "  " << prog << " --robot-ip 192.168.2.200 --trace-dir trace_run_01\n";
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
    if (arg == "--trace-dir") {
      if (i + 1 >= argc) {
        return false;
      }
      out->trace_enabled = true;
      out->trace_dir = argv[++i];
      continue;
    }
    if (arg == "--trace-planner-decimation") {
      if (i + 1 >= argc) {
        return false;
      }
      out->trace_enabled = true;
      out->trace_planner_decimation = static_cast<uint32_t>(std::stoul(argv[++i]));
      if (out->trace_planner_decimation == 0) {
        return false;
      }
      continue;
    }
    if (arg == "--trace-rt-decimation") {
      if (i + 1 >= argc) {
        return false;
      }
      out->trace_enabled = true;
      out->trace_rt_decimation = static_cast<uint32_t>(std::stoul(argv[++i]));
      if (out->trace_rt_decimation == 0) {
        return false;
      }
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

  std::thread observation_thread([&]() {
    while (!g_stop_requested.load(std::memory_order_acquire)) {
      const teleop::RobotObservation obs = observation_buffer.ReadLatest();
      observation_pub.Publish(obs);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
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
    controller_options.trace.enabled = options.trace_enabled;
    controller_options.trace.output_dir = options.trace_dir;
    controller_options.trace.planner_decimation = options.trace_planner_decimation;
    controller_options.trace.rt_decimation = options.trace_rt_decimation;
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
