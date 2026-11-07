#include "visualizer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <thread>

namespace qmini {
namespace {

cv::Point localToPixel(const LocalMap& map, float x, float y, const cv::Rect& panel) {
    const float u = (map.y_max - y) / (map.y_max - map.y_min);
    const float v = (map.x_max - x) / (map.x_max - map.x_min);
    return cv::Point(panel.x + static_cast<int>(u * panel.width), panel.y + static_cast<int>(v * panel.height));
}

cv::Mat leftImage(const sl::Mat& left, const cv::Size& size) {
    cv::Mat panel(size, CV_8UC3, cv::Scalar(20, 20, 20));
    if (left.getWidth() == 0 || left.getHeight() == 0) {
        return panel;
    }
    cv::Mat bgra(static_cast<int>(left.getHeight()), static_cast<int>(left.getWidth()), CV_8UC4,
                 left.getPtr<sl::uchar1>(sl::MEM::CPU), left.getStepBytes(sl::MEM::CPU));
    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    cv::resize(bgr, panel, size);
    return panel;
}

cv::Mat depthImage(const sl::Mat& depth, const cv::Size& size) {
    cv::Mat gray(size, CV_8UC1, cv::Scalar(30));
    if (depth.getWidth() == 0 || depth.getHeight() == 0) {
        cv::Mat color;
        cv::applyColorMap(gray, color, cv::COLORMAP_JET);
        return color;
    }
    const unsigned char* base = reinterpret_cast<const unsigned char*>(depth.getPtr<sl::uchar1>(sl::MEM::CPU));
    const size_t step = depth.getStepBytes(sl::MEM::CPU);
    const int src_w = static_cast<int>(depth.getWidth());
    const int src_h = static_cast<int>(depth.getHeight());
    const float near_m = 0.4f;
    const float far_m = 6.0f;
    for (int y = 0; y < size.height; ++y) {
        const int sy = std::min(src_h - 1, y * src_h / size.height);
        const float* row = reinterpret_cast<const float*>(base + static_cast<size_t>(sy) * step);
        unsigned char* dst = gray.ptr<unsigned char>(y);
        for (int x = 0; x < size.width; ++x) {
            const int sx = std::min(src_w - 1, x * src_w / size.width);
            const float meters = row[sx];
            if (!std::isfinite(meters) || meters <= 0.f) {
                dst[x] = 0;
                continue;
            }
            const float clipped = std::max(near_m, std::min(far_m, meters));
            dst[x] = static_cast<unsigned char>(255.f * (far_m - clipped) / (far_m - near_m));
        }
    }
    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_JET);
    return color;
}

void drawLocal(cv::Mat* canvas, const cv::Rect& panel, const FrameView& frame) {
    const LocalMap& map = frame.local;
    cv::rectangle(*canvas, panel, cv::Scalar(18, 22, 28), cv::FILLED);
    if (map.nx <= 0 || map.ny <= 0 || map.cells.empty()) {
        return;
    }
    for (int iy = 0; iy < map.ny; ++iy) {
        for (int ix = 0; ix < map.nx; ++ix) {
            const uint8_t cell = map.cells[iy * map.nx + ix];
            if (cell == 0) {
                continue;
            }
            const float x = map.x_min + (static_cast<float>(ix) + 0.5f) * map.resolution;
            const float y = map.y_min + (static_cast<float>(iy) + 0.5f) * map.resolution;
            const cv::Point p = localToPixel(map, x, y, panel);
            if (cell == 2) {
                cv::circle(*canvas, p, 1, cv::Scalar(40, 40, 220), cv::FILLED);
            } else {
                cv::circle(*canvas, p, 1, cv::Scalar(70, 90, 40), cv::FILLED);
            }
        }
    }
    for (const Beam& beam : frame.command.beams) {
        const float reach = std::max(0.3f, beam.clearance);
        const cv::Point end = localToPixel(map, reach * std::cos(beam.angle), reach * std::sin(beam.angle), panel);
        const cv::Point start = localToPixel(map, 0.f, 0.f, panel);
        cv::line(*canvas, start, end, beam.blocked ? cv::Scalar(0, 80, 200) : cv::Scalar(0, 180, 80), 1);
    }
    const cv::Point chosen = localToPixel(map, frame.command.clearance * std::cos(frame.command.heading),
                                           frame.command.clearance * std::sin(frame.command.heading), panel);
    const cv::Point robot = localToPixel(map, 0.f, 0.f, panel);
    cv::line(*canvas, robot, chosen, cv::Scalar(0, 220, 255), 2);
    cv::circle(*canvas, robot, 5, cv::Scalar(255, 255, 255), cv::FILLED);
}

void drawGlobal(cv::Mat* canvas, const cv::Rect& panel, const FrameView& frame, const GlobalMap& global_map) {
    cv::rectangle(*canvas, panel, cv::Scalar(16, 16, 16), cv::FILLED);
    if (global_map.last_seen.empty()) {
        cv::putText(*canvas, "global map waits for tracking", cv::Point(panel.x + 16, panel.y + 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 180, 180), 1);
        return;
    }
    const float half = 6.0f;
    const float x0 = frame.pose.x - half;
    const float y0 = frame.pose.y - half;
    const float span = half * 2.f;
    const uint32_t fresh = 150;
    for (int py = 0; py < panel.height; py += 2) {
        for (int px = 0; px < panel.width; px += 2) {
            const float wx = x0 + span * (1.f - static_cast<float>(py) / panel.height);
            const float wy = y0 + span * (1.f - static_cast<float>(px) / panel.width);
            const int gx = static_cast<int>((wx - global_map.origin_x) / global_map.resolution);
            const int gy = static_cast<int>((wy - global_map.origin_y) / global_map.resolution);
            if (gx < 0 || gy < 0 || gx >= global_map.n || gy >= global_map.n) {
                continue;
            }
            const uint32_t seen = global_map.last_seen[gy * global_map.n + gx];
            if (seen == 0 || global_map.frame - seen > fresh) {
                continue;
            }
            cv::rectangle(*canvas, cv::Rect(panel.x + px, panel.y + py, 2, 2), cv::Scalar(30, 60, 220), cv::FILLED);
        }
    }
    const int rx = panel.x + panel.width / 2;
    const int ry = panel.y + panel.height / 2;
    const cv::Point tip(rx - static_cast<int>(18.f * std::sin(frame.pose.yaw)),
                        ry - static_cast<int>(18.f * std::cos(frame.pose.yaw)));
    cv::line(*canvas, cv::Point(rx, ry), tip, cv::Scalar(0, 220, 255), 2);
    cv::circle(*canvas, cv::Point(rx, ry), 4, cv::Scalar(255, 255, 255), cv::FILLED);
}

void putLabel(cv::Mat* canvas, const cv::Rect& panel, const std::string& text) {
    cv::putText(*canvas, text, cv::Point(panel.x + 8, panel.y + 22), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 0, 0), 3);
    cv::putText(*canvas, text, cv::Point(panel.x + 8, panel.y + 22), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(240, 240, 240), 1);
}

std::string httpResponse(const std::string& content_type, const std::string& body) {
    std::ostringstream os;
    os << "HTTP/1.1 200 OK\r\n"
       << "Content-Type: " << content_type << "\r\n"
       << "Content-Length: " << body.size() << "\r\n"
       << "Cache-Control: no-store\r\n"
       << "Connection: close\r\n\r\n"
       << body;
    return os.str();
}

const char* kPage =
    "<!doctype html><meta charset='utf-8'><title>qmini ZED</title>"
    "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
    "img{width:100%;max-width:1200px;display:block}pre{padding:12px;white-space:pre-wrap}</style>"
    "<img id='v'><pre id='s'>等待画面</pre><script>"
    "async function tick(){const t=Date.now();"
    "document.getElementById('v').src='/frame.jpg?t='+t;"
    "try{const r=await fetch('/status.json?t='+t);document.getElementById('s').textContent=await r.text();}"
    "catch(e){}}"
    "setInterval(tick,250);tick();</script>";

}  // namespace

struct PreviewServer::Impl {
    int port = 0;
    int listen_fd = -1;
    std::atomic<bool> stop{false};
    std::mutex mu;
    std::vector<unsigned char> jpeg;
    std::string json = "{}\n";
    std::thread thread;
};

PreviewServer::PreviewServer(int port) : impl_(new Impl) {
    impl_->port = port;
    impl_->listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (impl_->listen_fd < 0) {
        return;
    }
    int opt = 1;
    ::setsockopt(impl_->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(impl_->listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
        ::listen(impl_->listen_fd, 4) < 0) {
        ::close(impl_->listen_fd);
        impl_->listen_fd = -1;
        return;
    }
    impl_->thread = std::thread([this]() { this->loop(); });
}

PreviewServer::~PreviewServer() {
    if (!impl_) {
        return;
    }
    impl_->stop = true;
    if (impl_->listen_fd >= 0) {
        ::shutdown(impl_->listen_fd, SHUT_RDWR);
        ::close(impl_->listen_fd);
        impl_->listen_fd = -1;
    }
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}

bool PreviewServer::ok() const { return impl_ && impl_->listen_fd >= 0; }

void PreviewServer::publish(const std::vector<unsigned char>& jpeg, const std::string& status_json) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    impl_->jpeg = jpeg;
    impl_->json = status_json;
}

void PreviewServer::loop() {
    while (!impl_->stop) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(impl_->listen_fd, &fds);
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        const int ready = ::select(impl_->listen_fd + 1, &fds, nullptr, nullptr, &tv);
        if (ready <= 0 || impl_->stop) {
            continue;
        }
        const int client = ::accept(impl_->listen_fd, nullptr, nullptr);
        if (client < 0) {
            continue;
        }
        char request[1024];
        const ssize_t n = ::recv(client, request, sizeof(request) - 1, 0);
        std::string path = "/";
        if (n > 0) {
            request[n] = '\0';
            std::istringstream is(request);
            std::string method;
            is >> method >> path;
        }
        std::string response;
        if (path.find("/status.json") == 0) {
            std::string json;
            {
                std::lock_guard<std::mutex> lock(impl_->mu);
                json = impl_->json;
            }
            response = httpResponse("application/json; charset=utf-8", json);
        } else if (path.find("/frame.jpg") == 0) {
            std::string jpeg;
            {
                std::lock_guard<std::mutex> lock(impl_->mu);
                jpeg.assign(reinterpret_cast<const char*>(impl_->jpeg.data()), impl_->jpeg.size());
            }
            response = httpResponse("image/jpeg", jpeg);
        } else {
            response = httpResponse("text/html; charset=utf-8", kPage);
        }
        ::send(client, response.data(), response.size(), 0);
        ::close(client);
    }
}

cv::Mat renderDashboard(const sl::Mat& left, const sl::Mat& depth, const FrameView& frame, const GlobalMap& global_map) {
    cv::Mat canvas(700, 1120, CV_8UC3, cv::Scalar(12, 12, 12));
    const cv::Rect image_panel(10, 10, 540, 300);
    const cv::Rect depth_panel(570, 10, 540, 300);
    const cv::Rect local_panel(10, 340, 540, 340);
    const cv::Rect global_panel(570, 340, 540, 340);

    leftImage(left, image_panel.size()).copyTo(canvas(image_panel));
    depthImage(depth, depth_panel.size()).copyTo(canvas(depth_panel));
    drawLocal(&canvas, local_panel, frame);
    drawGlobal(&canvas, global_panel, frame, global_map);

    std::ostringstream image_label;
    image_label << "LEFT  " << frame.camera_line;
    putLabel(&canvas, image_panel, image_label.str());
    std::ostringstream depth_label;
    depth_label << "DEPTH  center " << (frame.local.center_depth > 0.f ? frame.local.center_depth : -1.f) << " m";
    putLabel(&canvas, depth_panel, depth_label.str());
    putLabel(&canvas, local_panel, "LOCAL  clearance and steering");
    std::ostringstream pose_label;
    pose_label.setf(std::ios::fixed);
    pose_label.precision(2);
    pose_label << "GLOBAL  " << frame.pose.tracking_state << " x=" << frame.pose.x << " y=" << frame.pose.y
               << " yaw=" << frame.pose.yaw * 57.3f;
    putLabel(&canvas, global_panel, pose_label.str());

    std::ostringstream bar;
    bar.setf(std::ios::fixed);
    bar.precision(2);
    bar << frame.mode_name << "  fps " << frame.fps << "  v " << frame.command.linear_x << "  w "
        << frame.command.angular_z << "  clear " << frame.command.clearance << " m  floor "
        << (frame.floor.valid ? "yes" : "no");
    cv::putText(canvas, bar.str(), cv::Point(16, 690), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1);
    return canvas;
}

std::string statusJson(const FrameView& frame) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << "{\n"
       << "  \"mode\": \"" << frame.mode_name << "\",\n"
       << "  \"camera\": \"" << frame.camera_line << "\",\n"
       << "  \"frame\": " << frame.frame_id << ",\n"
       << "  \"fps\": " << frame.fps << ",\n"
       << "  \"tracking\": \"" << frame.pose.tracking_state << "\",\n"
       << "  \"tracking_ok\": " << (frame.pose.valid ? "true" : "false") << ",\n"
       << "  \"confidence\": " << frame.pose.confidence << ",\n"
       << "  \"x\": " << frame.pose.x << ",\n"
       << "  \"y\": " << frame.pose.y << ",\n"
       << "  \"z\": " << frame.pose.z << ",\n"
       << "  \"yaw_deg\": " << frame.pose.yaw * 57.2958f << ",\n"
       << "  \"linear_x\": " << frame.command.linear_x << ",\n"
       << "  \"angular_z\": " << frame.command.angular_z << ",\n"
       << "  \"clearance_m\": " << frame.command.clearance << ",\n"
       << "  \"center_depth_m\": " << frame.local.center_depth << ",\n"
       << "  \"corridor_clearance_m\": " << frame.local.corridor_clearance << ",\n"
       << "  \"floor\": " << (frame.local.used_floor ? "true" : "false") << ",\n"
       << "  \"near_invalid_ratio\": " << frame.local.near_invalid_ratio << ",\n"
       << "  \"obstacle_points\": " << frame.local.obstacle_points << ",\n"
       << "  \"reason\": \"" << frame.command.reason << "\"\n"
       << "}\n";
    return os.str();
}

std::vector<unsigned char> encodeJpeg(const cv::Mat& dashboard) {
    std::vector<unsigned char> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", dashboard, jpeg, params);
    return jpeg;
}

void writeObstaclePly(const std::string& path, const LocalMap& map) {
    FILE* file = std::fopen(path.c_str(), "w");
    if (!file) {
        return;
    }
    const size_t count = map.obstacle_xyz.size() / 3;
    std::fprintf(file, "ply\nformat ascii 1.0\nelement vertex %zu\n", count);
    std::fprintf(file, "property float x\nproperty float y\nproperty float z\nend_header\n");
    for (size_t i = 0; i + 2 < map.obstacle_xyz.size(); i += 3) {
        std::fprintf(file, "%f %f %f\n", map.obstacle_xyz[i], map.obstacle_xyz[i + 1], map.obstacle_xyz[i + 2]);
    }
    std::fclose(file);
}

}  // namespace qmini
