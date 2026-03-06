#include "teleop_state_machine.h"

namespace teleop {

void TeleopStateMachine::Reset() {
  state_ = TeleopState::kDisconnected;
}

TeleopState TeleopStateMachine::Update(const StateInputs& inputs) {
  if (state_ == TeleopState::kFault) {
    if (inputs.clear_fault_requested && inputs.xr_stream_healthy && inputs.robot_ok) {
      state_ = TeleopState::kConnectedIdle;
    }
    return state_;
  }

  if (inputs.fault_requested || !inputs.robot_ok) {
    state_ = TeleopState::kFault;
    return state_;
  }

  if (!inputs.xr_stream_healthy) {
    state_ = TeleopState::kDisconnected;
    return state_;
  }

  if (!inputs.deadman_pressed) {
    state_ = TeleopState::kConnectedIdle;
    return state_;
  }

  // Deadman held, stream healthy, robot OK.
  if (state_ == TeleopState::kConnectedIdle || state_ == TeleopState::kDisconnected) {
    state_ = TeleopState::kTeleopArmed;
    return state_;
  }

  state_ = TeleopState::kTeleopActive;
  return state_;
}

const char* ToString(TeleopState state) {
  switch (state) {
    case TeleopState::kDisconnected:
      return "DISCONNECTED";
    case TeleopState::kConnectedIdle:
      return "CONNECTED_IDLE";
    case TeleopState::kTeleopArmed:
      return "TELEOP_ARMED";
    case TeleopState::kTeleopActive:
      return "TELEOP_ACTIVE";
    case TeleopState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace teleop
