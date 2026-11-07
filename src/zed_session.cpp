#include "zed_session.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

namespace qmini {
namespace {

sl::RESOLUTION resolutionFromName(const std::string& name) {
    if (name == "hd720") {
        return sl::RESOLUTION::HD720;
    }
    return sl::RESOLUTION::VGA;
}

std::string modelName(sl::MODEL model) {
    if (model == sl::MODEL::ZED) {
        return "ZED";
    }
    if (model == sl::MODEL::ZED_M) {
        return "ZED Mini";
    }
    return "other";
}

}  // namespace

std::string trackingStateName(sl::POSITIONAL_TRACKING_STATE state) {
    std::ostringstream os;
    os << state;
    return os.str();
}

ZedSession::ZedSession(const AppConfig& config) : config_(config) {}

ZedSession::~ZedSession() { close(); }

bool ZedSession::open(std::string* error) {
    sl::InitParameters init;
    init.camera_resolution = resolutionFromName(config_.resolution);
    init.camera_fps = config_.fps;
    init.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init.coordinate_units = sl::UNIT::METER;
    // ROS REP-103：X 前、Y 左、Z 上。后面的避障和可视化都用这套坐标。
    init.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init.depth_minimum_distance = 0.40f;
    init.depth_maximum_distance = config_.mode == RunMode::Map ? 8.0f : config_.max_range;
    init.depth_stabilization = 1;
    // 初代 ZED 没有 IMU。这个开关设成 true 时，SDK 会直接拒绝打开。
    init.sensors_required = false;
    init.sdk_verbose = config_.mode == RunMode::Probe ? 1 : 0;

    const sl::ERROR_CODE opened = zed_.open(init);
    if (opened != sl::ERROR_CODE::SUCCESS) {
        if (error) {
            std::ostringstream os;
            os << sl::toString(opened) << " / " << sl::toVerbose(opened);
            *error = os.str();
        }
        return false;
    }
    opened_ = true;

    const sl::CameraInformation info = zed_.getCameraInformation();
    report_.model = modelName(info.camera_model);
    report_.serial = info.serial_number;
    report_.firmware = info.camera_configuration.firmware_version;
    report_.width = static_cast<int>(info.camera_configuration.resolution.width);
    report_.height = static_cast<int>(info.camera_configuration.resolution.height);
    report_.fps = info.camera_configuration.fps;
    sl::SensorsConfiguration sensors = info.sensors_configuration;
    report_.imu = sensors.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE);
    {
        std::ostringstream os;
        os << report_.model << "  sn=" << report_.serial << "  fw=" << report_.firmware << "  "
           << report_.width << "x" << report_.height << " @" << report_.fps << "fps"
           << "  imu=" << (report_.imu ? "yes" : "no");
        report_.summary = os.str();
    }

    sl::PositionalTrackingParameters tracking;
    tracking.enable_area_memory = config_.area_memory || config_.mode == RunMode::Map;
    tracking.enable_pose_smoothing = true;
    tracking.enable_imu_fusion = false;
    tracking.set_floor_as_origin = false;
    if (!config_.load_area.empty()) {
        tracking.area_file_path = sl::String(config_.load_area.c_str());
    }
    const sl::ERROR_CODE tracked = zed_.enablePositionalTracking(tracking);
    if (tracked != sl::ERROR_CODE::SUCCESS) {
        if (error) {
            std::ostringstream os;
            os << "定位模块启动失败: " << sl::toString(tracked);
            *error = os.str();
        }
        return false;
    }

    if (config_.mode == RunMode::Map) {
        // Nano 4GB 显存和内存共用。用低分辨率、短距离，并把网格内存压到 512MB。
        sl::SpatialMappingParameters mapping(sl::SpatialMappingParameters::MAPPING_RESOLUTION::LOW,
                                              sl::SpatialMappingParameters::MAPPING_RANGE::SHORT, 512, false,
                                              false, false, sl::SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH);
        const sl::ERROR_CODE mapped = zed_.enableSpatialMapping(mapping);
        if (mapped != sl::ERROR_CODE::SUCCESS) {
            if (error) {
                std::ostringstream os;
                os << "空间建图启动失败: " << sl::toString(mapped);
                *error = os.str();
            }
            return false;
        }
        mapping_ = true;
    }

    runtime_.sensing_mode = sl::SENSING_MODE::STANDARD;
    runtime_.confidence_threshold = 60;
    runtime_.texture_confidence_threshold = 100;
    runtime_.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    return true;
}

void ZedSession::close() {
    if (!opened_) {
        return;
    }
    if (mapping_) {
        zed_.disableSpatialMapping();
        mapping_ = false;
    }
    zed_.disablePositionalTracking();
    zed_.close();
    opened_ = false;
}

bool ZedSession::grab(Pose2D* pose, sl::Mat* left, sl::Mat* depth, sl::Mat* cloud, std::string* error) {
    const sl::ERROR_CODE grabbed = zed_.grab(runtime_);
    if (grabbed != sl::ERROR_CODE::SUCCESS) {
        if (error) {
            *error = sl::toString(grabbed).c_str();
        }
        return false;
    }

    zed_.retrieveImage(*left, sl::VIEW::LEFT);
    zed_.retrieveMeasure(*depth, sl::MEASURE::DEPTH);
    zed_.retrieveMeasure(*cloud, sl::MEASURE::XYZ);

    sl::Pose zed_pose;
    const sl::POSITIONAL_TRACKING_STATE state = zed_.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);
    const sl::Translation t = zed_pose.getTranslation();
    const sl::float3 euler = zed_pose.getEulerAngles(true);
    pose->stamp = static_cast<double>(zed_pose.timestamp.getMilliseconds()) / 1000.0;
    pose->x = t.x;
    pose->y = t.y;
    pose->z = t.z;
    pose->roll = euler.x;
    pose->pitch = euler.y;
    pose->yaw = euler.z;
    pose->confidence = zed_pose.pose_confidence;
    pose->tracking_state = trackingStateName(state);
    pose->valid = state == sl::POSITIONAL_TRACKING_STATE::OK && zed_pose.valid;
    return true;
}

bool ZedSession::detectFloor(FloorModel* floor) {
    sl::Plane plane;
    sl::Transform floor_frame;
    const sl::ERROR_CODE found = zed_.findFloorPlane(plane, floor_frame);
    if (found != sl::ERROR_CODE::SUCCESS) {
        return false;
    }
    const sl::float3 normal = plane.getNormal();
    const sl::float4 equation = plane.getPlaneEquation();
    // 初代 ZED 没有重力。相机往下看时，水平面的法线在相机坐标系里会倾斜，
    // 所以只要求 Z 仍是最大分量，用来丢掉墙面。
    if (std::fabs(normal.z) < 0.55f || std::fabs(normal.z) < std::fabs(normal.x) ||
        std::fabs(normal.z) < std::fabs(normal.y)) {
        return false;
    }
    const float sign = normal.z >= 0.f ? 1.f : -1.f;
    floor->valid = true;
    floor->a = equation.x * sign;
    floor->b = equation.y * sign;
    floor->c = equation.z * sign;
    floor->d = equation.w * sign;
    floor->age = 0;
    return true;
}

bool ZedSession::exportMesh(const std::string& obj_path, std::string* error) {
    if (!mapping_) {
        if (error) {
            *error = "空间建图没有打开";
        }
        return false;
    }
    sl::Mesh mesh;
    zed_.extractWholeSpatialMap(mesh);
    mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW);
    if (!mesh.save(obj_path.c_str())) {
        if (error) {
            *error = "网格写入失败: " + obj_path;
        }
        return false;
    }
    return true;
}

void ZedSession::saveArea(const std::string& area_path) {
    if (!opened_ || area_path.empty()) {
        return;
    }
    const sl::ERROR_CODE saved = zed_.saveAreaMap(sl::String(area_path.c_str()));
    if (saved != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "区域记忆导出失败: " << sl::toString(saved) << "\n";
        return;
    }
    for (int i = 0; i < 100; ++i) {
        const sl::AREA_EXPORTING_STATE state = zed_.getAreaExportState();
        if (state != sl::AREA_EXPORTING_STATE::RUNNING) {
            std::cout << "区域记忆状态: " << state << " -> " << area_path << "\n";
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

}  // namespace qmini
