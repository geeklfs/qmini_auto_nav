#include "planner.hpp"

#include <algorithm>
#include <cmath>

namespace qmini {
namespace {

bool cellOccupied(const LocalMap& map, float x, float y) {
    if (map.inflated.empty() || map.nx <= 0 || map.ny <= 0) {
        return false;
    }
    const int ix = static_cast<int>((x - map.x_min) / map.resolution);
    const int iy = static_cast<int>((y - map.y_min) / map.resolution);
    if (ix < 0 || iy < 0 || ix >= map.nx || iy >= map.ny) {
        return true;
    }
    return map.inflated[iy * map.nx + ix] == 2;
}

float rayClearance(const LocalMap& map, float angle, float max_range) {
    const float step = map.resolution;
    const float dx = std::cos(angle);
    const float dy = std::sin(angle);
    for (float dist = 0.30f; dist <= max_range; dist += step) {
        if (cellOccupied(map, dist * dx, dist * dy)) {
            return dist;
        }
    }
    return max_range;
}

}  // namespace

NavCommand plan(const LocalMap& map, const AppConfig& config, bool mapping_mode, float preferred_heading,
               bool has_preferred) {
    NavCommand command;
    const float max_range = std::min(config.max_range, 4.0f);
    const int beam_count = 9;
    const float max_angle = 0.70f;
    float best_score = -1.f;
    int best_index = beam_count / 2;

    command.beams.reserve(beam_count);
    for (int i = 0; i < beam_count; ++i) {
        Beam beam;
        const float ratio = beam_count == 1 ? 0.f : static_cast<float>(i) / static_cast<float>(beam_count - 1);
        beam.angle = -max_angle + ratio * (2.f * max_angle);
        beam.clearance = rayClearance(map, beam.angle, max_range);
        beam.blocked = beam.clearance < config.stop_distance;
        command.beams.push_back(beam);

        // 两边净空接近时，偏向正前方，避免左右来回摆。
        float score = beam.clearance;
        if (beam.clearance > 0.80f) {
            score += 0.25f * (1.f - std::fabs(beam.angle) / max_angle);
        }
        if (score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    if (has_preferred && !command.beams.empty()) {
        int preferred_index = 0;
        float preferred_gap = 1e9f;
        for (int i = 0; i < beam_count; ++i) {
            const float gap = std::fabs(command.beams[i].angle - preferred_heading);
            if (gap < preferred_gap) {
                preferred_gap = gap;
                preferred_index = i;
            }
        }
        // 两条路净空接近时保持上一次的方向，避免静止画面里左右来回打方向。
        if (command.beams[preferred_index].clearance + 0.35f >= command.beams[best_index].clearance) {
            best_index = preferred_index;
        }
    }

    const Beam& best = command.beams[best_index];
    command.clearance = best.clearance;
    command.heading = best.angle;

    if (mapping_mode) {
        command.reason = "建图模式，只记录环境，不输出行驶速度";
        return command;
    }
    if (map.valid_points < 30) {
        command.reason = "有效深度太少，停车等待纹理或光照";
        return command;
    }
    if (map.near_invalid_ratio > 0.65f) {
        command.reason = "近处超出立体匹配范围，先停车";
        return command;
    }
    if (best.clearance < config.stop_distance) {
        command.reason = "周围净空不足，停车";
        return command;
    }

    command.linear_x = std::min(config.max_linear, 0.18f * best.clearance);
    command.angular_z = std::max(-config.max_angular, std::min(config.max_angular, 1.1f * best.angle));
    if (std::fabs(best.angle) > 0.25f) {
        command.linear_x *= 0.55f;
    }
    if (std::fabs(best.angle) < 0.12f && best.clearance > 1.2f) {
        command.reason = "前方通畅，直行";
    } else if (best.angle > 0.f) {
        command.reason = "左侧更空，向左绕行";
    } else {
        command.reason = "右侧更空，向右绕行";
    }
    return command;
}

}  // namespace qmini
