#pragma once

#include "types.hpp"

#include <sl/Camera.hpp>

#include <string>

namespace qmini {

class ZedSession {
public:
    explicit ZedSession(const AppConfig& config);
    ~ZedSession();

    bool open(std::string* error);
    void close();

    // 成功时填充 pose。建图模式会在内部打开 spatial mapping。
    bool grab(Pose2D* pose, sl::Mat* left, sl::Mat* depth, sl::Mat* cloud, std::string* error);

    bool detectFloor(FloorModel* floor);
    bool exportMesh(const std::string& obj_path, std::string* error);
    void saveArea(const std::string& area_path);

    const CameraReport& report() const { return report_; }
    sl::Camera& camera() { return zed_; }

private:
    AppConfig config_;
    sl::Camera zed_;
    sl::RuntimeParameters runtime_;
    CameraReport report_;
    bool opened_ = false;
    bool mapping_ = false;
};

std::string trackingStateName(sl::POSITIONAL_TRACKING_STATE state);

}  // namespace qmini
