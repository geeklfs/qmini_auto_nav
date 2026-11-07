#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace qmini {

enum class RunMode { Nav, Map, Probe };

struct AppConfig {
    RunMode mode = RunMode::Nav;
    std::string resolution = "vga";
    int fps = 15;
    int http_port = 8080;
    bool http = true;
    bool display = false;
    int seconds = 0;
    std::string output_dir = "output";
    bool area_memory = false;
    std::string load_area;
    float camera_height = 0.25f;
    float robot_radius = 0.18f;
    float min_obstacle_height = 0.10f;
    float max_obstacle_height = 1.10f;
    float max_range = 5.0f;
    float stop_distance = 0.40f;
    float max_linear = 0.25f;
    float max_angular = 0.70f;
};

struct Pose2D {
    double stamp = 0.0;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float roll = 0.f;
    float pitch = 0.f;
    float yaw = 0.f;
    int confidence = 0;
    bool valid = false;
    std::string tracking_state;
};

struct FloorModel {
    bool valid = false;
    float a = 0.f;
    float b = 0.f;
    float c = 1.f;
    float d = 0.f;
    int age = 1000;
};

// 机体坐标系：X 前、Y 左、Z 上。栅格按这个坐标系存储。
struct LocalMap {
    float x_min = -0.5f;
    float x_max = 5.0f;
    float y_min = -2.5f;
    float y_max = 2.5f;
    float resolution = 0.05f;
    int nx = 0;
    int ny = 0;
    std::vector<uint8_t> cells;       // 0 未知，1 空闲，2 障碍
    std::vector<uint8_t> inflated;    // 按机器人半径膨胀后的障碍
    int obstacle_points = 0;
    int valid_points = 0;
    float center_depth = -1.f;
    float corridor_clearance = -1.f;
    float near_invalid_ratio = 0.f;
    bool used_floor = false;
    std::vector<float> obstacle_xyz;  // 抽样后的障碍点，xyz 交错
};

struct GlobalMap {
    float resolution = 0.10f;
    float origin_x = -20.f;
    float origin_y = -20.f;
    int n = 400;
    std::vector<uint32_t> last_seen;
    uint32_t frame = 0;
};

struct Beam {
    float angle = 0.f;
    float clearance = 0.f;
    bool blocked = false;
};

struct NavCommand {
    double stamp = 0.0;
    float linear_x = 0.f;
    float angular_z = 0.f;
    float clearance = 0.f;
    float heading = 0.f;
    std::string reason;
    std::vector<Beam> beams;
};

struct FrameView {
    Pose2D pose;
    LocalMap local;
    NavCommand command;
    FloorModel floor;
    double grab_ms = 0.0;
    double process_ms = 0.0;
    float fps = 0.f;
    int frame_id = 0;
    std::string mode_name;
    std::string camera_line;
};

struct CameraReport {
    std::string model;
    unsigned serial = 0;
    unsigned firmware = 0;
    int width = 0;
    int height = 0;
    float fps = 0.f;
    bool imu = false;
    std::string summary;
};

}  // namespace qmini
