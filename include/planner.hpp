#pragma once

#include "types.hpp"

namespace qmini {

NavCommand plan(const LocalMap& map, const AppConfig& config, bool mapping_mode, float preferred_heading,
               bool has_preferred);

}  // namespace qmini
