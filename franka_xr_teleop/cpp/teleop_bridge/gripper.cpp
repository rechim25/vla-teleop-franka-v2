#include "gripper.h"

#include <algorithm>
#include <cmath>

namespace teleop {

void GripperController::Reset(GripperState initial) {
  current_ = initial;
  last_toggle_time_ns_ = 0;
}

GripperState GripperController::UpdateDesiredState(const GripperConfig& config,
                                                   double desired_command,
                                                   uint64_t now_ns) {
  desired_command = std::clamp(desired_command, 0.0, 1.0);
  if (config.command_mode == GripperCommandMode::kAnalog) {
    current_ = desired_command >= 0.5 ? GripperState::kClose : GripperState::kOpen;
    return current_;
  }

  GripperState resolved_state = current_;
  if (desired_command >= config.close_threshold) {
    resolved_state = GripperState::kClose;
  } else if (desired_command <= config.open_threshold) {
    resolved_state = GripperState::kOpen;
  }

  const bool curr_closed = current_ == GripperState::kClose;
  const bool desired_closed = resolved_state == GripperState::kClose;
  if (curr_closed != desired_closed) {
    const double elapsed_s = static_cast<double>(now_ns - last_toggle_time_ns_) * 1e-9;
    if (last_toggle_time_ns_ != 0 && elapsed_s < config.toggle_debounce_s) {
      return current_;
    }
    last_toggle_time_ns_ = now_ns;
  }

  current_ = resolved_state;
  return current_;
}

}  // namespace teleop
