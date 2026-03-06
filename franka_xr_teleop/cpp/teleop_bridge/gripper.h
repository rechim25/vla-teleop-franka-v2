#pragma once

#include <cstdint>

namespace teleop {

class GripperController {
 public:
  GripperController() = default;

  // Returns filtered command in [0, 1].
  double Update(double desired_command, uint64_t now_ns, double dt_s);
  void Reset(double initial = 1.0);

 private:
  double current_ = 1.0;
  uint64_t last_toggle_time_ns_ = 0;

  double max_rate_per_s_ = 2.0;
  double debounce_s_ = 0.08;
};

}  // namespace teleop
