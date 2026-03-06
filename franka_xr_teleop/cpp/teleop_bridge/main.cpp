#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "common_types.h"
#include "franka_controller.h"
#include "observation_pub.h"
#include "xr_receiver.h"

namespace {

std::atomic<bool> g_stop_requested{false};

void HandleSignal(int) {
  g_stop_requested.store(true, std::memory_order_release);
}

struct Options {
  bool dry_run = false;
  bool allow_motion = true;
  std::string robot_ip;
  std::string bind_ip = "0.0.0.0";
  uint16_t listen_port = 28080;
  std::string obs_ip = "127.0.0.1";
  uint16_t obs_port = 28081;
};

void PrintUsage(const char* prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " --robot-ip <ip> [--bind-ip 0.0.0.0] [--listen-port 28080]\n"
            << "             [--obs-ip 127.0.0.1] [--obs-port 28081] [--dry-run] [--no-motion]\n\n"
            << "Examples:\n"
            << "  " << prog << " --dry-run --listen-port 28080\n"
            << "  " << prog << " --robot-ip 192.168.2.200 --listen-port 28080 --obs-port 28081\n";
}

bool ParseArgs(int argc, char** argv, Options* out) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--dry-run") {
      out->dry_run = true;
      continue;
    }
    if (arg == "--no-motion") {
      out->allow_motion = false;
      continue;
    }
    if (arg == "--robot-ip") {
      if (i + 1 >= argc) {
        return false;
      }
      out->robot_ip = argv[++i];
      continue;
    }
    if (arg == "--bind-ip") {
      if (i + 1 >= argc) {
        return false;
      }
      out->bind_ip = argv[++i];
      continue;
    }
    if (arg == "--listen-port") {
      if (i + 1 >= argc) {
        return false;
      }
      out->listen_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      continue;
    }
    if (arg == "--obs-ip") {
      if (i + 1 >= argc) {
        return false;
      }
      out->obs_ip = argv[++i];
      continue;
    }
    if (arg == "--obs-port") {
      if (i + 1 >= argc) {
        return false;
      }
      out->obs_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      continue;
    }
    if (arg == "-h" || arg == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    }
    return false;
  }

  if (!out->dry_run && out->robot_ip.empty()) {
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

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  teleop::LatestCommandBuffer command_buffer;
  teleop::LatestObservationBuffer observation_buffer;

  teleop::XRReceiver receiver(options.bind_ip,
                              options.listen_port,
                              &command_buffer,
                              &g_stop_requested);
  receiver.Start();

  teleop::ObservationPublisher observation_pub(options.obs_ip, options.obs_port);
  observation_pub.Start();

  std::thread observation_thread([&]() {
    while (!g_stop_requested.load(std::memory_order_acquire)) {
      const teleop::RobotObservation obs = observation_buffer.ReadLatest();
      observation_pub.Publish(obs);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  teleop::TeleopBridgeConfig config;
  config.allow_motion = options.allow_motion;

  if (options.dry_run) {
    std::cout << "Dry-run mode: receiving XR packets on UDP " << options.bind_ip << ":"
              << options.listen_port << "\n";
    uint64_t last_print_ns = 0;
    while (!g_stop_requested.load(std::memory_order_acquire)) {
      const uint64_t now_ns = MonotonicNowNs();
      if (now_ns - last_print_ns > 500000000ULL) {
        const auto cmd = command_buffer.ReadLatest();
        const uint64_t age_ns = now_ns > cmd.timestamp_ns ? (now_ns - cmd.timestamp_ns) : 0;
        std::cout << "rx_count=" << receiver.received_count() << " dropped=" << receiver.dropped_count()
                  << " seq=" << cmd.sequence_id << " age_ms=" << (age_ns * 1e-6)
                  << " deadman=" << (cmd.teleop_enabled ? 1 : 0)
                  << " clutch=" << (cmd.clutch_pressed ? 1 : 0)
                  << " gripper=" << cmd.gripper_command << "\n";
        last_print_ns = now_ns;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    teleop::FrankaControllerOptions controller_options;
    controller_options.robot_ip = options.robot_ip;

    teleop::FrankaTeleopController controller(controller_options,
                                              config,
                                              &command_buffer,
                                              &observation_buffer);
    const int rc = controller.Run(&g_stop_requested);
    g_stop_requested.store(true, std::memory_order_release);
    receiver.Stop();
    if (observation_thread.joinable()) {
      observation_thread.join();
    }
    observation_pub.Stop();
    return rc;
  }

  g_stop_requested.store(true, std::memory_order_release);
  receiver.Stop();
  if (observation_thread.joinable()) {
    observation_thread.join();
  }
  observation_pub.Stop();
  return 0;
}
