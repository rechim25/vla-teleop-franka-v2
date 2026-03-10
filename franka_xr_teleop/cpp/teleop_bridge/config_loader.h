#pragma once

#include <string>

#include "common_types.h"

namespace teleop {

bool LoadAppConfig(const std::string& config_dir, AppConfig* config, std::string* error);

}  // namespace teleop
