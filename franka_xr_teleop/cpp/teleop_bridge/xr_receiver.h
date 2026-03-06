#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "common_types.h"

namespace teleop {

class XRReceiver {
 public:
  XRReceiver(std::string bind_ip,
             uint16_t bind_port,
             LatestCommandBuffer* cmd_buffer,
             std::atomic<bool>* stop_requested);
  ~XRReceiver();

  bool Start();
  void Stop();

  uint64_t last_packet_time_ns() const { return last_packet_time_ns_.load(std::memory_order_acquire); }
  uint64_t received_count() const { return received_count_.load(std::memory_order_acquire); }
  uint64_t dropped_count() const { return dropped_count_.load(std::memory_order_acquire); }

 private:
  void ReceiveLoop();

  std::string bind_ip_;
  uint16_t bind_port_;
  LatestCommandBuffer* cmd_buffer_;
  std::atomic<bool>* stop_requested_;

  std::atomic<bool> running_{false};
  std::thread thread_{};

  std::atomic<uint64_t> last_packet_time_ns_{0};
  std::atomic<uint64_t> received_count_{0};
  std::atomic<uint64_t> dropped_count_{0};
};

}  // namespace teleop
