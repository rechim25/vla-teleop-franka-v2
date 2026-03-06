#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace {

enum class Mode { kReadOnly, kTinyMotion, kTinyCartesian, kRecoverOnly };
enum class CartesianAxis { kX, kY, kZ };

struct Options {
  std::string robot_ip;
  Mode mode = Mode::kReadOnly;
  size_t read_samples = 200;
  size_t joint_index = 3;
  double delta_rad = 0.01;
  CartesianAxis cart_axis = CartesianAxis::kX;
  double delta_m = 0.01;
  double motion_duration_s = 3.0;
  bool auto_recover = false;
};

void PrintUsage(const char* program) {
  std::cout << "Usage:\n"
            << "  " << program
            << " --robot-ip <IP> [--mode read-only|tiny-motion|tiny-cartesian|recover-only]\n"
            << "             [--read-samples <N>] [--joint-index 0..6]\n"
            << "             [--delta-rad <rad>] [--cart-axis x|y|z] [--delta-m <m>]\n"
            << "             [--duration-s <sec>] [--auto-recover]\n\n"
            << "Examples:\n"
            << "  " << program << " --robot-ip 192.168.2.200 --mode read-only\n"
            << "  " << program
            << " --robot-ip 192.168.2.200 --mode tiny-motion --delta-rad 0.01 --duration-s 3.0\n"
            << "  " << program
            << " --robot-ip 192.168.2.200 --mode tiny-cartesian --cart-axis z --delta-m 0.01 --duration-s 8.0\n"
            << "  " << program << " --robot-ip 192.168.2.200 --mode recover-only\n";
}

bool ParseSize(const std::string& value, size_t* out) {
  try {
    size_t pos = 0;
    unsigned long parsed = std::stoul(value, &pos);
    if (pos != value.size()) {
      return false;
    }
    *out = static_cast<size_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseDouble(const std::string& value, double* out) {
  try {
    size_t pos = 0;
    double parsed = std::stod(value, &pos);
    if (pos != value.size() || !std::isfinite(parsed)) {
      return false;
    }
    *out = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--robot-ip") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --robot-ip\n";
        return false;
      }
      options->robot_ip = argv[++i];
      continue;
    }
    if (arg == "--mode") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --mode\n";
        return false;
      }
      const std::string value(argv[++i]);
      if (value == "read-only") {
        options->mode = Mode::kReadOnly;
      } else if (value == "tiny-motion") {
        options->mode = Mode::kTinyMotion;
      } else if (value == "tiny-cartesian") {
        options->mode = Mode::kTinyCartesian;
      } else if (value == "recover-only") {
        options->mode = Mode::kRecoverOnly;
      } else {
        std::cerr << "Invalid mode: " << value << "\n";
        return false;
      }
      continue;
    }
    if (arg == "--read-samples") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --read-samples\n";
        return false;
      }
      if (!ParseSize(argv[++i], &options->read_samples)) {
        std::cerr << "Invalid --read-samples value\n";
        return false;
      }
      continue;
    }
    if (arg == "--joint-index") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --joint-index\n";
        return false;
      }
      if (!ParseSize(argv[++i], &options->joint_index) || options->joint_index > 6) {
        std::cerr << "Invalid --joint-index (must be 0..6)\n";
        return false;
      }
      continue;
    }
    if (arg == "--delta-rad") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --delta-rad\n";
        return false;
      }
      if (!ParseDouble(argv[++i], &options->delta_rad) || std::abs(options->delta_rad) > 0.05) {
        std::cerr << "Invalid --delta-rad (recommended |delta| <= 0.05)\n";
        return false;
      }
      continue;
    }
    if (arg == "--cart-axis") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --cart-axis\n";
        return false;
      }
      const std::string value(argv[++i]);
      if (value == "x") {
        options->cart_axis = CartesianAxis::kX;
      } else if (value == "y") {
        options->cart_axis = CartesianAxis::kY;
      } else if (value == "z") {
        options->cart_axis = CartesianAxis::kZ;
      } else {
        std::cerr << "Invalid --cart-axis (must be x|y|z)\n";
        return false;
      }
      continue;
    }
    if (arg == "--delta-m") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --delta-m\n";
        return false;
      }
      if (!ParseDouble(argv[++i], &options->delta_m)) {
        std::cerr << "Invalid --delta-m\n";
        return false;
      }
      continue;
    }
    if (arg == "--duration-s") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --duration-s\n";
        return false;
      }
      if (!ParseDouble(argv[++i], &options->motion_duration_s) || options->motion_duration_s <= 0.0) {
        std::cerr << "Invalid --duration-s (> 0 required)\n";
        return false;
      }
      continue;
    }
    if (arg == "--auto-recover") {
      options->auto_recover = true;
      continue;
    }

    if (!arg.empty() && arg[0] != '-' && options->robot_ip.empty()) {
      options->robot_ip = arg;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    return false;
  }

  if (options->robot_ip.empty()) {
    std::cerr << "robot IP is required.\n";
    return false;
  }
  return true;
}

template <size_t N>
void PrintArray(const std::string& name, const std::array<double, N>& values) {
  std::cout << name << " = [";
  for (size_t i = 0; i < values.size(); ++i) {
    std::cout << std::fixed << std::setprecision(6) << values[i];
    if (i + 1 < values.size()) {
      std::cout << ", ";
    }
  }
  std::cout << "]\n";
}

void PrintStateSnapshot(const franka::RobotState& state) {
  std::cout << "RobotState snapshot:\n" << state << "\n";
  PrintArray("q", state.q);
  PrintArray("O_T_EE", state.O_T_EE);
  std::cout << "current_errors: " << state.current_errors << "\n";
  std::cout << "last_motion_errors: " << state.last_motion_errors << "\n";
}

void ConfigureConservativeBehavior(franka::Robot& robot) {
  // Conservative thresholds and moderate impedance taken from official libfranka examples.
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  robot.setJointImpedance({{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0}});
  robot.setCartesianImpedance({{3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0}});
}

void PrintControlExceptionDetails(const franka::ControlException& e) {
  std::cerr << "ControlException: " << e.what() << "\n";
  if (e.log.empty()) {
    std::cerr << "ControlException log was empty; no RobotState records available.\n";
    return;
  }

  const franka::RobotState& last_state = e.log.back().state;
  std::cerr << "Triggered current_errors: " << last_state.current_errors << "\n";
  std::cerr << "Triggered last_motion_errors: " << last_state.last_motion_errors << "\n";
}

bool IsReflexMode(const franka::RobotState& state) {
  return state.robot_mode == franka::RobotMode::kReflex;
}

size_t CartesianTranslationIndex(CartesianAxis axis) {
  if (axis == CartesianAxis::kX) {
    return 12;
  }
  if (axis == CartesianAxis::kY) {
    return 13;
  }
  return 14;
}

const char* CartesianAxisName(CartesianAxis axis) {
  if (axis == CartesianAxis::kX) {
    return "x";
  }
  if (axis == CartesianAxis::kY) {
    return "y";
  }
  return "z";
}

bool ValidateMotionOptions(const Options& options) {
  if (options.mode == Mode::kTinyCartesian) {
    const double abs_delta_m = std::abs(options.delta_m);
    if (abs_delta_m <= 0.0 || abs_delta_m > 0.05) {
      std::cerr << "Invalid --delta-m for tiny-cartesian. Safety limit is 0 < |delta-m| <= 0.05.\n";
      return false;
    }
    if (options.motion_duration_s < 8.0) {
      std::cerr << "Invalid --duration-s for tiny-cartesian. Safety minimum is 8.0 seconds.\n";
      return false;
    }
    constexpr double kMaxAverageSpeedMps = 0.01;
    const double average_speed_mps = abs_delta_m / options.motion_duration_s;
    if (average_speed_mps > kMaxAverageSpeedMps) {
      std::cerr << "Invalid tiny-cartesian speed. |delta-m| / duration-s must be <= 0.01 m/s.\n";
      return false;
    }
  }
  return true;
}

void RecoverFromReflex(franka::Robot& robot) {
  const franka::RobotState before = robot.readOnce();
  if (!IsReflexMode(before)) {
    std::cout << "Robot is not in Reflex mode. No recovery needed.\n";
    return;
  }

  std::cerr << "Robot is in Reflex mode. Calling automaticErrorRecovery() once.\n";
  robot.automaticErrorRecovery();

  const franka::RobotState after = robot.readOnce();
  std::cout << "Post-recovery robot_mode: " << after.robot_mode << "\n";
  std::cout << "Post-recovery current_errors: " << after.current_errors << "\n";
  std::cout << "Post-recovery last_motion_errors: " << after.last_motion_errors << "\n";
}

void RunRecoverOnly(franka::Robot& robot) {
  RecoverFromReflex(robot);
  std::cout << "Recover-only mode completed.\n";
}

void RunReadOnly(franka::Robot& robot, const Options& options) {
  std::cout << "Entering read-only mode with " << options.read_samples << " read() samples.\n";
  if (options.read_samples == 0) {
    std::cout << "read_samples is 0, skipping streaming read().\n";
    return;
  }

  size_t count = 0;
  robot.read([&](const franka::RobotState& state) {
    ++count;
    if (count == 1 || count == options.read_samples) {
      std::cout << "read() sample " << count << ":\n";
      PrintArray("q", state.q);
      PrintArray("O_T_EE", state.O_T_EE);
      std::cout << "current_errors: " << state.current_errors << "\n";
      std::cout << "last_motion_errors: " << state.last_motion_errors << "\n";
    }
    return count < options.read_samples;
  });
  std::cout << "Read-only mode completed successfully.\n";
}

void RunTinyMotion(franka::Robot& robot, const Options& options) {
  ConfigureConservativeBehavior(robot);

  const franka::RobotState initial_state = robot.readOnce();
  const std::array<double, 7> q_start = initial_state.q;
  std::array<double, 7> q_goal = q_start;
  q_goal[options.joint_index] += options.delta_rad;

  std::cout << "Running one tiny smooth joint-space motion:\n";
  std::cout << "  joint index: " << options.joint_index << "\n";
  std::cout << "  delta [rad]: " << options.delta_rad << "\n";
  std::cout << "  duration [s]: " << options.motion_duration_s << "\n";
  PrintArray("q_start", q_start);
  PrintArray("q_goal", q_goal);

  double time_s = 0.0;
  robot.control([q_start, q_goal, &time_s, duration_s = options.motion_duration_s](
                    const franka::RobotState&, franka::Duration period) -> franka::JointPositions {
    time_s += period.toSec();
    const double tau = std::clamp(time_s / duration_s, 0.0, 1.0);
    const double smooth = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);

    std::array<double, 7> q_cmd{};
    for (size_t i = 0; i < q_cmd.size(); ++i) {
      q_cmd[i] = q_start[i] + smooth * (q_goal[i] - q_start[i]);
    }

    franka::JointPositions output(q_cmd);
    if (tau >= 1.0) {
      return franka::MotionFinished(output);
    }
    return output;
  });

  const franka::RobotState final_state = robot.readOnce();
  std::cout << "Tiny-motion mode completed successfully.\n";
  PrintArray("q_final", final_state.q);
  std::cout << "current_errors: " << final_state.current_errors << "\n";
  std::cout << "last_motion_errors: " << final_state.last_motion_errors << "\n";
}

void RunTinyCartesian(franka::Robot& robot, const Options& options) {
  ConfigureConservativeBehavior(robot);

  const franka::RobotState initial_state = robot.readOnce();
  // Start from commanded references to avoid a discontinuity at control start.
  const std::array<double, 16> pose_start = initial_state.O_T_EE_d;
  const std::array<double, 2> elbow_start = initial_state.elbow_d;
  std::array<double, 16> pose_goal = pose_start;
  const size_t translation_index = CartesianTranslationIndex(options.cart_axis);
  pose_goal[translation_index] += options.delta_m;

  std::cout << "Running one tiny smooth Cartesian translation:\n";
  std::cout << "  axis: " << CartesianAxisName(options.cart_axis) << "\n";
  std::cout << "  delta [m]: " << options.delta_m << "\n";
  std::cout << "  duration [s]: " << options.motion_duration_s << "\n";
  PrintArray("O_T_EE_start", pose_start);
  PrintArray("O_T_EE_goal", pose_goal);
  PrintArray("elbow_start", elbow_start);

  double time_s = 0.0;
  std::array<double, 16> last_pose_cmd = pose_start;
  constexpr double kMaxTransSpeedMps = 0.01;
  robot.control([pose_start, pose_goal, elbow_start, &time_s, &last_pose_cmd,
                 duration_s = options.motion_duration_s](
                    const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
    time_s += period.toSec();
    const double tau = std::clamp(time_s / duration_s, 0.0, 1.0);
    const double smooth = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);

    std::array<double, 16> pose_cmd = state.O_T_EE_d;
    const double max_step_m = kMaxTransSpeedMps * period.toSec();
    for (size_t i = 12; i <= 14; ++i) {
      const double desired = pose_start[i] + smooth * (pose_goal[i] - pose_start[i]);
      const double delta = desired - last_pose_cmd[i];
      const double limited_delta = std::clamp(delta, -max_step_m, max_step_m);
      pose_cmd[i] = last_pose_cmd[i] + limited_delta;
    }
    last_pose_cmd = pose_cmd;

    franka::CartesianPose output(pose_cmd, elbow_start);
    if (tau >= 1.0) {
      return franka::MotionFinished(output);
    }
    return output;
  });

  const franka::RobotState final_state = robot.readOnce();
  std::cout << "Tiny-cartesian mode completed successfully.\n";
  PrintArray("O_T_EE_final", final_state.O_T_EE);
  PrintArray("q_final", final_state.q);
  std::cout << "current_errors: " << final_state.current_errors << "\n";
  std::cout << "last_motion_errors: " << final_state.last_motion_errors << "\n";
}

int Execute(const Options& options) {
  franka::Robot robot(options.robot_ip);

  std::cout << "Connected to robot at " << options.robot_ip << "\n";
  std::cout << "Server version: " << robot.serverVersion() << "\n";

  const franka::RobotState state = robot.readOnce();
  PrintStateSnapshot(state);

  if (options.mode == Mode::kRecoverOnly) {
    RunRecoverOnly(robot);
    return EXIT_SUCCESS;
  }

  if (options.mode == Mode::kReadOnly) {
    RunReadOnly(robot, options);
    return EXIT_SUCCESS;
  }

  if (!ValidateMotionOptions(options)) {
    return EXIT_FAILURE;
  }

  const franka::RobotState pre_motion_state = robot.readOnce();
  if (IsReflexMode(pre_motion_state)) {
    if (options.auto_recover) {
      RecoverFromReflex(robot);
    } else {
      std::cerr << "Robot is already in Reflex mode before motion.\n"
                << "Run --mode recover-only first, or re-run with --auto-recover.\n";
      return EXIT_FAILURE;
    }
  }

  try {
    if (options.mode == Mode::kTinyMotion) {
      RunTinyMotion(robot, options);
    } else {
      RunTinyCartesian(robot, options);
    }
    return EXIT_SUCCESS;
  } catch (const franka::ControlException& e) {
    PrintControlExceptionDetails(e);
    if (options.auto_recover) {
      std::cerr << "Attempting one automaticErrorRecovery() call.\n";
      robot.automaticErrorRecovery();
      std::cerr << "automaticErrorRecovery() completed.\n";
    } else {
      std::cerr << "Not calling automaticErrorRecovery(). Re-run with --auto-recover if desired.\n";
    }
    return EXIT_FAILURE;
  } catch (const franka::CommandException& e) {
    std::cerr << "CommandException during motion setup/control: " << e.what() << "\n";
    if (options.auto_recover) {
      RecoverFromReflex(robot);
    }
    return EXIT_FAILURE;
  }
}

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return EXIT_SUCCESS;
    }
  }

  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return EXIT_FAILURE;
  }

  try {
    return Execute(options);
  } catch (const franka::IncompatibleVersionException& e) {
    std::cerr << "IncompatibleVersionException: " << e.what() << "\n";
    std::cerr << "  robot server version: " << e.server_version << "\n";
    std::cerr << "  libfranka version: " << e.library_version << "\n";
  } catch (const franka::RealtimeException& e) {
    std::cerr << "RealtimeException: " << e.what() << "\n";
  } catch (const franka::NetworkException& e) {
    std::cerr << "NetworkException: " << e.what() << "\n";
  } catch (const franka::CommandException& e) {
    std::cerr << "CommandException: " << e.what() << "\n";
  } catch (const franka::ControlException& e) {
    PrintControlExceptionDetails(e);
  } catch (const franka::Exception& e) {
    std::cerr << "franka::Exception: " << e.what() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "std::exception: " << e.what() << "\n";
  }

  return EXIT_FAILURE;
}
