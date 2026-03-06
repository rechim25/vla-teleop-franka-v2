#pragma once

#include <atomic>
#include <cstdint>

#include "common_types.h"

namespace teleop {

class XrRoboticsSource {
 public:
  XrRoboticsSource(LatestCommandBuffer* cmd_buffer, std::atomic<bool>* stop_requested);
  ~XrRoboticsSource();

  bool Start();
  void Stop();

  uint64_t last_packet_time_ns() const { return last_packet_time_ns_.load(std::memory_order_acquire); }
  uint64_t received_count() const { return received_count_.load(std::memory_order_acquire); }
  uint64_t dropped_count() const { return dropped_count_.load(std::memory_order_acquire); }
  bool server_connected() const { return server_connected_.load(std::memory_order_acquire); }
  bool device_connected() const { return device_connected_.load(std::memory_order_acquire); }

  // Internal callback entrypoint invoked by the C SDK bridge callback.
  void OnCallback(int type, int status, void* user_data);

 private:
  LatestCommandBuffer* cmd_buffer_;
  std::atomic<bool>* stop_requested_;

  std::atomic<bool> running_{false};
  std::atomic<bool> server_connected_{false};
  std::atomic<bool> device_connected_{false};
  std::atomic<uint64_t> sequence_id_{0};

  std::atomic<uint64_t> last_packet_time_ns_{0};
  std::atomic<uint64_t> received_count_{0};
  std::atomic<uint64_t> dropped_count_{0};
};

}  // namespace teleop
