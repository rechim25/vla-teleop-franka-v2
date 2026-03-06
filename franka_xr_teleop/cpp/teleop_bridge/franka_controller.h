#pragma once

#include <atomic>
#include <string>

#include "common_types.h"

namespace teleop {

struct FrankaControllerOptions {
  std::string robot_ip;
};

class FrankaTeleopController {
 public:
  FrankaTeleopController(const FrankaControllerOptions& options,
                         const TeleopBridgeConfig& config,
                         const LatestCommandBuffer* command_buffer,
                         LatestObservationBuffer* observation_buffer);

  // Blocking call. Returns 0 on clean stop, non-zero on error.
  int Run(std::atomic<bool>* stop_requested);

 private:
  FrankaControllerOptions options_{};
  TeleopBridgeConfig config_{};
  const LatestCommandBuffer* command_buffer_;
  LatestObservationBuffer* observation_buffer_;
};

}  // namespace teleop
