#pragma once

#include "common_types.h"

namespace teleop {

struct StateInputs {
  bool xr_stream_healthy = false;
  bool deadman_pressed = false;
  bool robot_ok = false;
  bool fault_requested = false;
  bool clear_fault_requested = false;
};

class TeleopStateMachine {
 public:
  TeleopStateMachine() = default;

  TeleopState state() const { return state_; }
  void Reset();
  TeleopState Update(const StateInputs& inputs);

 private:
  TeleopState state_ = TeleopState::kDisconnected;
};

const char* ToString(TeleopState state);

}  // namespace teleop
