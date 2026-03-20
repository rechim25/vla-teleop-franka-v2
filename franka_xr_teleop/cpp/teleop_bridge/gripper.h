#pragma once

#include <cstdint>

#include "common_types.h"

namespace teleop {

class GripperController {
 public:
  GripperController() = default;

  // Returns the desired high-level gripper state from the XR input.
  GripperState UpdateDesiredState(const GripperConfig& config, double desired_command, uint64_t now_ns);
  void Reset(GripperState initial = GripperState::kOpen);

 private:
  GripperState current_ = GripperState::kOpen;
  uint64_t last_toggle_time_ns_ = 0;
};

}  // namespace teleop
