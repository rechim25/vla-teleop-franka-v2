#include "gripper.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace teleop {

void GripperController::Reset(GripperState initial) {
  // Normalize to the only two logical states the planner publishes.
  current_ = initial == GripperState::kOpen ? GripperState::kOpen : GripperState::kClose;
  last_toggle_time_ns_ = 0;
  trigger_pressed_ = false;
}

void GripperController::SyncCurrentState(GripperState measured_state) {
  if (measured_state == GripperState::kFault) return;
  current_ = measured_state == GripperState::kOpen ? GripperState::kOpen : GripperState::kClose;
}

GripperState GripperController::UpdateDesiredState(const GripperConfig& config,
                                                   double desired_command,
                                                   uint64_t now_ns) {
  desired_command = std::clamp(desired_command, 0.0, 1.0);
  if (config.command_mode == GripperCommandMode::kAnalog) {
    current_ = desired_command >= 0.5 ? GripperState::kClose : GripperState::kOpen;
    return current_;
  }

  // Binary press-to-toggle with Schmitt-trigger hysteresis.
  //   OPEN  + press -> publish CLOSE (gripper thread will close; stall -> stay CLOSE)
  //   CLOSE + press -> publish OPEN  (gripper thread will open fully -> OPEN)
  // A "press" is the rising edge above close_threshold after falling below
  // open_threshold. Debounce rejects edges inside a short window only.
  const bool release_low = desired_command < config.open_threshold;
  const bool press_high = desired_command > config.close_threshold;

  if (release_low) {
    if (trigger_pressed_) {
      std::cout << "Gripper trigger released t_ms=" << (static_cast<double>(now_ns) * 1e-6)
                << " trigger=" << desired_command << "\n";
    }
    trigger_pressed_ = false;
  }

  if (press_high && !trigger_pressed_) {
    const double elapsed_s =
        last_toggle_time_ns_ == 0 ? std::numeric_limits<double>::infinity()
                                  : static_cast<double>(now_ns - last_toggle_time_ns_) * 1e-9;
    if (elapsed_s >= config.toggle_debounce_s) {
      const GripperState previous = current_;
      current_ = (current_ == GripperState::kOpen) ? GripperState::kClose : GripperState::kOpen;
      last_toggle_time_ns_ = now_ns;
      std::cout << "Gripper toggle " << ToString(previous) << " -> " << ToString(current_)
                << " t_ms=" << (static_cast<double>(now_ns) * 1e-6)
                << " trigger=" << desired_command << "\n";
    } else {
      std::cout << "Gripper press ignored (debounce) t_ms="
                << (static_cast<double>(now_ns) * 1e-6)
                << " trigger=" << desired_command
                << " since_last_toggle_s=" << elapsed_s << "\n";
    }
    trigger_pressed_ = true;
  }

  return current_;
}

}  // namespace teleop
