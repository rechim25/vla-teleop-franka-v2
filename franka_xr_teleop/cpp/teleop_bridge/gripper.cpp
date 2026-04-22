#include "gripper.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace teleop {

namespace {

bool IsClosedLike(GripperState state) {
  return state == GripperState::kClose || state == GripperState::kHold;
}

}  // namespace

void GripperController::Reset(GripperState initial) {
  current_ = initial;
  last_toggle_time_ns_ = 0;
  trigger_pressed_ = false;
}

void GripperController::SyncCurrentState(GripperState measured_state) {
  if (measured_state == GripperState::kFault) {
    return;
  }
  current_ = IsClosedLike(measured_state) ? GripperState::kClose : GripperState::kOpen;
}

GripperState GripperController::UpdateDesiredState(const GripperConfig& config,
                                                   double desired_command,
                                                   uint64_t now_ns) {
  desired_command = std::clamp(desired_command, 0.0, 1.0);
  if (config.command_mode == GripperCommandMode::kAnalog) {
    current_ = desired_command >= 0.5 ? GripperState::kClose : GripperState::kOpen;
    return current_;
  }

  const double rearm_threshold = std::clamp(0.5 * (config.open_threshold + config.close_threshold),
                                            0.0,
                                            config.close_threshold);

  if (desired_command <= rearm_threshold) {
    trigger_pressed_ = false;
  }

  const bool pressed_now = desired_command >= config.close_threshold;
  if (pressed_now && !trigger_pressed_) {
    const double elapsed_s =
        last_toggle_time_ns_ == 0 ? std::numeric_limits<double>::infinity()
                                  : static_cast<double>(now_ns - last_toggle_time_ns_) * 1e-9;
    if (elapsed_s >= config.toggle_debounce_s) {
      current_ = IsClosedLike(current_) ? GripperState::kOpen : GripperState::kClose;
      last_toggle_time_ns_ = now_ns;
    }
    trigger_pressed_ = true;
  }

  return current_;
}

}  // namespace teleop
