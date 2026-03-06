#include "gripper.h"

#include <algorithm>
#include <cmath>

namespace teleop {

void GripperController::Reset(double initial) {
  current_ = std::clamp(initial, 0.0, 1.0);
  last_toggle_time_ns_ = 0;
}

double GripperController::Update(double desired_command, uint64_t now_ns, double dt_s) {
  desired_command = std::clamp(desired_command, 0.0, 1.0);

  // Binary-ish debounce: if crossing midpoint, require a short hold-off.
  const bool curr_closed = current_ < 0.5;
  const bool desired_closed = desired_command < 0.5;
  if (curr_closed != desired_closed) {
    const double elapsed_s = static_cast<double>(now_ns - last_toggle_time_ns_) * 1e-9;
    if (last_toggle_time_ns_ != 0 && elapsed_s < debounce_s_) {
      desired_command = current_;
    } else {
      last_toggle_time_ns_ = now_ns;
    }
  }

  const double max_step = std::max(0.0, max_rate_per_s_ * dt_s);
  const double delta = std::clamp(desired_command - current_, -max_step, max_step);
  current_ += delta;
  current_ = std::clamp(current_, 0.0, 1.0);
  return current_;
}

}  // namespace teleop
