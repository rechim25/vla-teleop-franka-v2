#include "gripper.h"

#include <algorithm>
#include <cmath>

namespace teleop {

namespace {

bool IsClosedLike(GripperState state) {
  return state == GripperState::kClose || state == GripperState::kHold;
}

}  // namespace

void GripperController::Reset(GripperState initial) {
  current_ = initial;
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
  (void)now_ns;
  desired_command = std::clamp(desired_command, 0.0, 1.0);
  if (config.command_mode == GripperCommandMode::kAnalog) {
    current_ = desired_command >= 0.5 ? GripperState::kClose : GripperState::kOpen;
    return current_;
  }

  // Binary mode: hold-to-close, release-to-open.
  // Close requires a deliberate squeeze above close_threshold.
  if (desired_command >= config.close_threshold) {
    current_ = GripperState::kClose;
  } else {
    current_ = GripperState::kOpen;
  }

  return current_;
}

}  // namespace teleop
