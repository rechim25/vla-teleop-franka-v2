#include "xrobotics_source.h"

#include <PXREARobotSDK.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "math_utils.h"

namespace teleop {
namespace {

using json = nlohmann::json;

bool ParsePoseString(const std::string& pose_str, Pose* out) {
  std::stringstream ss(pose_str);
  std::string token;
  std::array<double, 7> values{};
  size_t i = 0;
  while (std::getline(ss, token, ',') && i < values.size()) {
    try {
      const double value = std::stod(token);
      if (!std::isfinite(value)) {
        return false;
      }
      values[i++] = value;
    } catch (...) {
      return false;
    }
  }
  if (i != values.size()) {
    return false;
  }

  out->p = {values[0], values[1], values[2]};
  out->q = {values[3], values[4], values[5], values[6]};
  return true;
}

double GetDoubleWithFallback(const json& object,
                             std::initializer_list<const char*> keys,
                             double fallback) {
  for (const char* key : keys) {
    if (object.contains(key) && object[key].is_number()) {
      return object[key].get<double>();
    }
  }
  return fallback;
}

bool GetBoolWithFallback(const json& object,
                         std::initializer_list<const char*> keys,
                         bool fallback) {
  for (const char* key : keys) {
    if (object.contains(key) && object[key].is_boolean()) {
      return object[key].get<bool>();
    }
  }
  return fallback;
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

  XRCommand cmd{};
  if (!ParsePoseString(right["pose"].get<std::string>(), &cmd.right_controller_pose)) {
    dropped_count_.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  const uint64_t receive_ns = MonotonicNowNs();
  cmd.timestamp_ns = receive_ns;
  cmd.sequence_id = sequence_id_.fetch_add(1, std::memory_order_acq_rel) + 1;
  cmd.control_trigger_value = Clamp01(
      GetDoubleWithFallback(right, {"grip", "squeeze"}, right.value("primaryButton", false) ? 1.0 : 0.0));
  cmd.gripper_trigger_value = Clamp01(GetDoubleWithFallback(right, {"trigger"}, 0.0));
  cmd.button_a = right.value("primaryButton", false);
  cmd.button_b = right.value("secondaryButton", false);
  cmd.right_axis_click =
      GetBoolWithFallback(right, {"axisClick", "primary2DAxisClick", "rightAxisClick"}, false);

  cmd_buffer_->Publish(cmd);
  received_count_.fetch_add(1, std::memory_order_acq_rel);
  last_packet_time_ns_.store(receive_ns, std::memory_order_release);
}

}  // namespace teleop
