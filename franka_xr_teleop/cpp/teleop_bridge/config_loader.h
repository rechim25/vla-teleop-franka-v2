#pragma once

#include <array>
#include <string>

#include "common_types.h"

namespace teleop {

bool LoadAppConfig(const std::string& config_dir, AppConfig* config, std::string* error);
bool SaveStartJointPositions(const std::string& config_dir,
                             const std::array<double, 7>& start_joint_positions_rad,
                             std::string* error);

}  // namespace teleop
