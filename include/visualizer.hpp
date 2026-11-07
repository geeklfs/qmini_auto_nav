#pragma once

#include "types.hpp"

#include <sl/Camera.hpp>

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace qmini {

class PreviewServer {
public:
    explicit PreviewServer(int port);
    ~PreviewServer();
    void publish(const std::vector<unsigned char>& jpeg, const std::string& status_json);
    bool ok() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void loop();
};

cv::Mat renderDashboard(const sl::Mat& left, const sl::Mat& depth, const FrameView& frame,
                        const GlobalMap& global_map);

std::string statusJson(const FrameView& frame);
std::vector<unsigned char> encodeJpeg(const cv::Mat& dashboard);
void writeObstaclePly(const std::string& path, const LocalMap& map);

}  // namespace qmini
