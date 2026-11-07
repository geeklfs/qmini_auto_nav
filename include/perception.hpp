#pragma once

#include "types.hpp"

#include <sl/Camera.hpp>

namespace qmini {

void resetLocalMap(LocalMap* map);
void buildLocalMap(const sl::Mat& cloud, const FloorModel& floor, const AppConfig& config, LocalMap* map);
void updateGlobalMap(const Pose2D& pose, const LocalMap& local, GlobalMap* global_map);

}  // namespace qmini
