#include "xrobotics_source.h"

#include <PXREARobotSDK.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

namespace teleop {
namespace {

using json = nlohmann::json;

uint64_t MonotonicNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool ParsePoseString(const std::string& pose_str, std::array<double, 7>* out) {
  std::stringstream ss(pose_str);
  std::string token;
  size_t i = 0;
  while (std::getline(ss, token, ',') && i < out->size()) {
    try {
      const double v = std::stod(token);
      if (!std::isfinite(v)) {
        return false;
      }
      (*out)[i++] = v;
    } catch (...) {
      return false;
    }
  }
  return i == out->size();
}

extern "C" void OnPxreaClientCallbackBridge(void* context,
                                              PXREAClientCallbackType type,
                                              int status,
                                              void* user_data) {
  if (context == nullptr) {
    return;
  }
  auto* self = static_cast<XrRoboticsSource*>(context);
  self->OnCallback(static_cast<int>(type), status, user_data);
}

}  // namespace

XrRoboticsSource::XrRoboticsSource(LatestCommandBuffer* cmd_buffer,
                                   std::atomic<bool>* stop_requested)
    : cmd_buffer_(cmd_buffer), stop_requested_(stop_requested) {}

XrRoboticsSource::~XrRoboticsSource() {
  Stop();
}

bool XrRoboticsSource::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return true;
  }

  const int rc = PXREAInit(this, OnPxreaClientCallbackBridge, PXREAFullMask);
  if (rc != 0) {
    running_.store(false, std::memory_order_release);
    std::cerr << "XrRoboticsSource: PXREAInit failed with code " << rc << "\n";
    return false;
  }
  return true;
}

void XrRoboticsSource::Stop() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  (void)PXREADeinit();
  server_connected_.store(false, std::memory_order_release);
  device_connected_.store(false, std::memory_order_release);
}

void XrRoboticsSource::OnCallback(int type, int status, void* user_data) {
  (void)status;
  if (!running_.load(std::memory_order_acquire) ||
      stop_requested_->load(std::memory_order_acquire)) {
    return;
  }

  const auto callback_type = static_cast<PXREAClientCallbackType>(type);
  switch (callback_type) {
    case PXREAServerConnect:
      server_connected_.store(true, std::memory_order_release);
      return;
    case PXREAServerDisconnect:
      server_connected_.store(false, std::memory_order_release);
      device_connected_.store(false, std::memory_order_release);
      return;
    case PXREADeviceConnect:
      device_connected_.store(true, std::memory_order_release);
      return;
    case PXREADeviceMissing:
      device_connected_.store(false, std::memory_order_release);
      return;
    case PXREADeviceStateJson:
      break;
    default:
      return;
  }

  if (user_data == nullptr) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  const auto& state_json = *reinterpret_cast<PXREADevStateJson*>(user_data);

  json root = json::parse(state_json.stateJson, nullptr, false);
  if (root.is_discarded() || !root.contains("value") || !root["value"].is_string()) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  json value = json::parse(root["value"].get<std::string>(), nullptr, false);
  if (value.is_discarded() || !value.contains("Controller") || !value["Controller"].is_object()) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  const auto& controller = value["Controller"];
  if (!controller.contains("right") || !controller["right"].is_object()) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  const auto& right = controller["right"];
  if (!right.contains("pose") || !right["pose"].is_string()) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  std::array<double, 7> pose{};
  if (!ParsePoseString(right["pose"].get<std::string>(), &pose)) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  // Use local monotonic receive time for timeout gating in the servo loop.
  const uint64_t receive_ns = MonotonicNowNs();
  XRCommand cmd{};
  cmd.timestamp_ns = receive_ns;
  cmd.sequence_id = sequence_id_.fetch_add(1, std::memory_order_acq_rel) + 1;
  cmd.teleop_enabled = right.value("primaryButton", false);   // A button on right controller.
  cmd.clutch_pressed = right.value("secondaryButton", false); // B button on right controller.
  cmd.target_position_xyz = {pose[0], pose[1], pose[2]};
  cmd.target_orientation_xyzw = {pose[3], pose[4], pose[5], pose[6]};
  cmd.gripper_command = std::clamp(right.value("trigger", 1.0), 0.0, 1.0);

  cmd_buffer_->Publish(cmd);
  received_count_.fetch_add(1, std::memory_order_acq_rel);
  last_packet_time_ns_.store(receive_ns, std::memory_order_release);
}

}  // namespace teleop
