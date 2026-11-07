#include "perception.hpp"

#include <cmath>
#include <algorithm>

namespace qmini {
namespace {

int cellIndex(const LocalMap& map, int ix, int iy) { return iy * map.nx + ix; }

void mark(std::vector<uint8_t>* cells, int index, uint8_t value) {
    uint8_t& cell = (*cells)[index];
    if (value > cell) {
        cell = value;
    }
}

}  // namespace

void resetLocalMap(LocalMap* map) {
    map->nx = std::max(1, static_cast<int>(std::lround((map->x_max - map->x_min) / map->resolution)));
    map->ny = std::max(1, static_cast<int>(std::lround((map->y_max - map->y_min) / map->resolution)));
    map->cells.assign(static_cast<size_t>(map->nx * map->ny), 0);
    map->inflated.clear();
    map->obstacle_points = 0;
    map->valid_points = 0;
    map->center_depth = -1.f;
    map->corridor_clearance = map->x_max;
    map->near_invalid_ratio = 0.f;
    map->used_floor = false;
    map->obstacle_xyz.clear();
}

void buildLocalMap(const sl::Mat& cloud, const FloorModel& floor, const AppConfig& config, LocalMap* map) {
    resetLocalMap(map);
    const bool floor_ok = floor.valid && floor.age < 45;
    map->used_floor = floor_ok;

    const int width = static_cast<int>(cloud.getWidth());
    const int height = static_cast<int>(cloud.getHeight());
    if (width <= 0 || height <= 0) {
        return;
    }

    const unsigned char* base = reinterpret_cast<const unsigned char*>(cloud.getPtr<sl::uchar1>(sl::MEM::CPU));
    const size_t step = cloud.getStepBytes(sl::MEM::CPU);
    const int stride = std::max(2, width / 320);

    const int cx = width / 2;
    const int cy = height / 2;
    if (cy < height && cx < width) {
        const sl::float4* center_row =
            reinterpret_cast<const sl::float4*>(base + static_cast<size_t>(cy) * step);
        const float z = center_row[cx].x;  // X 是前方
        if (std::isfinite(z) && z > 0.f) {
            map->center_depth = z;
        }
    }

    float nearest_corridor = map->x_max;
    bool corridor_seen = false;
    int near_seen = 0;
    int near_invalid = 0;
    const int near_v0 = height * 3 / 5;
    const int near_u0 = width / 5;
    const int near_u1 = width * 4 / 5;

    for (int v = 0; v < height; v += stride) {
        const sl::float4* row = reinterpret_cast<const sl::float4*>(base + static_cast<size_t>(v) * step);
        for (int u = 0; u < width; u += stride) {
            const sl::float4& point = row[u];
            const float x = point.x;
            const float y = point.y;
            const float z = point.z;
            if (v >= near_v0 && u >= near_u0 && u < near_u1) {
                ++near_seen;
                if (!std::isfinite(x) || x < 0.35f) {
                    ++near_invalid;
                }
            }
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }
            if (x < 0.30f || x > config.max_range || y < map->y_min || y >= map->y_max) {
                continue;
            }
            ++map->valid_points;

            float height_above_ground = z + config.camera_height;
            if (floor_ok) {
                height_above_ground = floor.a * x + floor.b * y + floor.c * z - floor.d;
            }
            const bool obstacle =
                height_above_ground >= config.min_obstacle_height && height_above_ground <= config.max_obstacle_height;

            const int ix = static_cast<int>((x - map->x_min) / map->resolution);
            const int iy = static_cast<int>((y - map->y_min) / map->resolution);
            if (ix < 0 || iy < 0 || ix >= map->nx || iy >= map->ny) {
                continue;
            }
            const int index = cellIndex(*map, ix, iy);
            if (obstacle) {
                mark(&map->cells, index, 2);
                ++map->obstacle_points;
                if (map->obstacle_xyz.size() < 8000 * 3) {
                    map->obstacle_xyz.push_back(x);
                    map->obstacle_xyz.push_back(y);
                    map->obstacle_xyz.push_back(z);
                }
                if (std::fabs(y) < 0.25f && x < nearest_corridor) {
                    nearest_corridor = x;
                    corridor_seen = true;
                }
            } else if (height_above_ground < config.min_obstacle_height && height_above_ground > -0.15f) {
                mark(&map->cells, index, 1);
            }
        }
    }
    map->corridor_clearance = corridor_seen ? nearest_corridor : map->x_max;
    map->near_invalid_ratio = near_seen > 0 ? static_cast<float>(near_invalid) / static_cast<float>(near_seen) : 1.f;

    const int radius = std::max(1, static_cast<int>(std::ceil(config.robot_radius / map->resolution)));
    map->inflated = map->cells;
    for (int iy = 0; iy < map->ny; ++iy) {
        for (int ix = 0; ix < map->nx; ++ix) {
            if (map->cells[cellIndex(*map, ix, iy)] != 2) {
                continue;
            }
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (dx * dx + dy * dy > radius * radius) {
                        continue;
                    }
                    const int nx = ix + dx;
                    const int ny = iy + dy;
                    if (nx < 0 || ny < 0 || nx >= map->nx || ny >= map->ny) {
                        continue;
                    }
                    map->inflated[cellIndex(*map, nx, ny)] = 2;
                }
            }
        }
    }
}

void updateGlobalMap(const Pose2D& pose, const LocalMap& local, GlobalMap* global_map) {
    if (!pose.valid) {
        return;
    }
    if (global_map->last_seen.empty()) {
        global_map->last_seen.assign(static_cast<size_t>(global_map->n * global_map->n), 0);
    }
    ++global_map->frame;
    const float c = std::cos(pose.yaw);
    const float s = std::sin(pose.yaw);
    for (int iy = 0; iy < local.ny; ++iy) {
        for (int ix = 0; ix < local.nx; ++ix) {
            if (local.cells[iy * local.nx + ix] != 2) {
                continue;
            }
            const float lx = local.x_min + (static_cast<float>(ix) + 0.5f) * local.resolution;
            const float ly = local.y_min + (static_cast<float>(iy) + 0.5f) * local.resolution;
            const float wx = c * lx - s * ly + pose.x;
            const float wy = s * lx + c * ly + pose.y;
            const int gx = static_cast<int>((wx - global_map->origin_x) / global_map->resolution);
            const int gy = static_cast<int>((wy - global_map->origin_y) / global_map->resolution);
            if (gx < 0 || gy < 0 || gx >= global_map->n || gy >= global_map->n) {
                continue;
            }
            global_map->last_seen[gy * global_map->n + gx] = global_map->frame;
        }
    }
}

}  // namespace qmini
