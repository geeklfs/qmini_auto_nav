#include "perception.hpp"
#include "planner.hpp"
#include "visualizer.hpp"
#include "zed_session.hpp"

#include <opencv2/highgui.hpp>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <sys/stat.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void printUsage() {
    std::cout
        << "用法:\n"
        << "  qmini_auto_nav [nav|map|probe] [选项]\n\n"
        << "模式:\n"
        << "  nav    定位 + 局部障碍 + 避障速度建议 + 浏览器画面。默认。\n"
        << "  map    在 nav 的基础上做三维网格重建，退出时保存 mesh.obj。\n"
        << "  probe  打开摄像头，打印型号、分辨率、IMU、一帧深度后退出。\n\n"
        << "选项:\n"
        << "  --res vga|hd720     默认 vga。Nano 4GB 上先用 vga。\n"
        << "  --fps N             默认 15。\n"
        << "  --port N            可视化端口，默认 8080。\n"
        << "  --no-http           不打开浏览器预览。\n"
        << "  --display           本机有桌面时额外弹 OpenCV 窗口。\n"
        << "  --seconds N         运行 N 秒后退出。0 表示一直运行到 Ctrl-C。\n"
        << "  --output DIR        输出目录，默认 output。\n"
        << "  --camera-height M   相机离地高度，地板检测失败时使用，默认 0.25。\n"
        << "  --robot-radius M    障碍膨胀半径，默认 0.18。\n"
        << "  --area-memory       打开视觉回环。Nano 上会更吃算力。\n"
        << "  --load-area FILE    载入之前保存的 .area。\n";
}

bool parseArgs(int argc, char** argv, qmini::AppConfig* config) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "缺少参数: " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (arg == "nav") {
            config->mode = qmini::RunMode::Nav;
        } else if (arg == "map") {
            config->mode = qmini::RunMode::Map;
        } else if (arg == "probe") {
            config->mode = qmini::RunMode::Probe;
        } else if (arg == "--res") {
            const char* value = need("--res");
            if (!value) return false;
            config->resolution = value;
        } else if (arg == "--fps") {
            const char* value = need("--fps");
            if (!value) return false;
            config->fps = std::atoi(value);
        } else if (arg == "--port") {
            const char* value = need("--port");
            if (!value) return false;
            config->http_port = std::atoi(value);
        } else if (arg == "--seconds") {
            const char* value = need("--seconds");
            if (!value) return false;
            config->seconds = std::atoi(value);
        } else if (arg == "--output") {
            const char* value = need("--output");
            if (!value) return false;
            config->output_dir = value;
        } else if (arg == "--camera-height") {
            const char* value = need("--camera-height");
            if (!value) return false;
            config->camera_height = std::atof(value);
        } else if (arg == "--robot-radius") {
            const char* value = need("--robot-radius");
            if (!value) return false;
            config->robot_radius = std::atof(value);
        } else if (arg == "--load-area") {
            const char* value = need("--load-area");
            if (!value) return false;
            config->load_area = value;
        } else if (arg == "--no-http") {
            config->http = false;
        } else if (arg == "--display") {
            config->display = true;
        } else if (arg == "--area-memory") {
            config->area_memory = true;
        } else if (arg == "-h" || arg == "--help") {
            printUsage();
            return false;
        } else {
            std::cerr << "无法识别的参数: " << arg << "\n";
            printUsage();
            return false;
        }
    }
    if (config->resolution != "vga" && config->resolution != "hd720") {
        std::cerr << "--res 只能是 vga 或 hd720\n";
        return false;
    }
    return true;
}

const char* modeName(qmini::RunMode mode) {
    switch (mode) {
        case qmini::RunMode::Map:
            return "map";
        case qmini::RunMode::Probe:
            return "probe";
        case qmini::RunMode::Nav:
        default:
            return "nav";
    }
}

}  // namespace

int main(int argc, char** argv) {
    qmini::AppConfig config;
    if (argc > 1 && !parseArgs(argc, argv, &config)) {
        return argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") ? 0 : 1;
    }
    if (argc == 1) {
        printUsage();
        std::cout << "\n未给参数，按 nav 模式启动。\n";
    }

    ::mkdir(config.output_dir.c_str(), 0755);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    qmini::ZedSession zed(config);
    std::string error;
    if (!zed.open(&error)) {
        std::cerr << "ZED 打开失败: " << error << "\n";
        std::cerr << "确认摄像头插在 USB 3.0 口，并且没有别的程序（ZED Explorer / ROS 节点）占用它。\n";
        return 1;
    }
    const qmini::CameraReport& report = zed.report();
    std::cout << "摄像头: " << report.summary << "\n";
    std::cout << "深度: PERFORMANCE，坐标系 X前 Y左 Z上，最近 0.40 m。\n";
    if (!report.imu) {
        std::cout << "这是初代 ZED，没有 IMU。定位只靠视觉，转身和快速移动时会漂，纹理少的地面也会丢。\n";
    }

    if (config.mode == qmini::RunMode::Probe) {
        sl::Mat left, depth, cloud;
        qmini::Pose2D pose;
        int ok_frames = 0;
        for (int i = 0; i < 30 && ok_frames < 8 && !g_stop; ++i) {
            if (!zed.grab(&pose, &left, &depth, &cloud, &error)) {
                std::cerr << "抓帧失败: " << error << "\n";
                continue;
            }
            ++ok_frames;
            qmini::LocalMap local;
            qmini::FloorModel floor;
            qmini::buildLocalMap(cloud, floor, config, &local);
            std::cout << "frame " << ok_frames << "  tracking=" << pose.tracking_state
                      << "  confidence=" << pose.confidence << "  center_depth=" << local.center_depth
                      << " m  valid_points=" << local.valid_points << "\n";
        }
        std::cout << "probe 完成，抓到 " << ok_frames << " 帧。\n";
        return ok_frames > 0 ? 0 : 1;
    }

    std::unique_ptr<qmini::PreviewServer> preview;
    if (config.http) {
        preview.reset(new qmini::PreviewServer(config.http_port));
        if (preview->ok()) {
            std::cout << "可视化: http://172.16.31.154:" << config.http_port << "\n";
        } else {
            std::cerr << "浏览器预览端口 " << config.http_port << " 绑定失败，继续只写文件。\n";
            preview.reset();
        }
    }
    if (config.display && std::getenv("DISPLAY") != nullptr) {
        cv::namedWindow("qmini_auto_nav", cv::WINDOW_NORMAL);
    }

    std::ofstream pose_csv(config.output_dir + "/pose.csv");
    std::ofstream cmd_csv(config.output_dir + "/cmd_vel.csv");
    pose_csv << "stamp,x,y,z,roll,pitch,yaw,confidence,valid,state\n";
    cmd_csv << "stamp,linear_x,angular_z,clearance,heading,reason\n";
    pose_csv.flush();
    cmd_csv.flush();

    qmini::GlobalMap global_map;
    qmini::FloorModel floor;
    const auto started = std::chrono::steady_clock::now();
    auto fps_window = started;
    int fps_count = 0;
    float fps = 0.f;
    int frame_id = 0;

    std::cout << "速度只写到 " << config.output_dir
              << "/cmd_vel.csv，不会驱动电机。确认画面后再把开发板装到 Qmini 上。\n";
    if (config.mode == qmini::RunMode::Map) {
        std::cout << "建图时让相机缓慢扫过有纹理的场景。Ctrl-C 后保存 mesh.obj。\n";
    }

    while (!g_stop) {
        if (config.seconds > 0) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            if (elapsed >= config.seconds) {
                break;
            }
        }

        const auto t0 = std::chrono::steady_clock::now();
        sl::Mat left, depth, cloud;
        qmini::Pose2D pose;
        if (!zed.grab(&pose, &left, &depth, &cloud, &error)) {
            std::cerr << "抓帧失败: " << error << "\n";
            continue;
        }
        const auto t1 = std::chrono::steady_clock::now();

        if (frame_id % 8 == 0) {
            qmini::FloorModel detected;
            if (zed.detectFloor(&detected)) {
                floor = detected;
            }
        }
        ++floor.age;

        qmini::LocalMap local;
        qmini::buildLocalMap(cloud, floor, config, &local);
        qmini::updateGlobalMap(pose, local, &global_map);
        static float preferred_heading = 0.f;
        static bool has_preferred = false;
        qmini::NavCommand command =
            qmini::plan(local, config, config.mode == qmini::RunMode::Map, preferred_heading, has_preferred);
        if (command.linear_x > 0.f || std::fabs(command.angular_z) > 0.f) {
            preferred_heading = command.heading;
            has_preferred = true;
        }
        command.stamp = pose.stamp;
        const auto t2 = std::chrono::steady_clock::now();

        ++frame_id;
        ++fps_count;
        const double fps_elapsed = std::chrono::duration<double>(t2 - fps_window).count();
        if (fps_elapsed >= 1.0) {
            fps = static_cast<float>(fps_count / fps_elapsed);
            fps_count = 0;
            fps_window = t2;
        }

        qmini::FrameView view;
        view.pose = pose;
        view.local = local;
        view.command = command;
        view.floor = floor;
        view.grab_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        view.process_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        view.fps = fps;
        view.frame_id = frame_id;
        view.mode_name = modeName(config.mode);
        view.camera_line = report.summary;

        pose_csv << pose.stamp << "," << pose.x << "," << pose.y << "," << pose.z << "," << pose.roll << ","
                 << pose.pitch << "," << pose.yaw << "," << pose.confidence << "," << (pose.valid ? 1 : 0) << ","
                 << pose.tracking_state << "\n";
        cmd_csv << command.stamp << "," << command.linear_x << "," << command.angular_z << "," << command.clearance
                << "," << command.heading << ",\"" << command.reason << "\"\n";

        if (frame_id % 3 == 0) {
            cv::Mat dashboard = qmini::renderDashboard(left, depth, view, global_map);
            const std::vector<unsigned char> jpeg = qmini::encodeJpeg(dashboard);
            const std::string json = qmini::statusJson(view);
            if (preview) {
                preview->publish(jpeg, json);
            }
            if (frame_id % 15 == 0) {
                cv::imwrite(config.output_dir + "/latest.jpg", dashboard);
                qmini::writeObstaclePly(config.output_dir + "/obstacles_latest.ply", local);
                pose_csv.flush();
                cmd_csv.flush();
                std::cout << "frame " << frame_id << "  fps " << fps << "  " << pose.tracking_state
                          << "  x=" << pose.x << " y=" << pose.y << "  v=" << command.linear_x
                          << " w=" << command.angular_z << "  floor=" << (local.used_floor ? "yes" : "no")
                          << "  near_invalid=" << local.near_invalid_ratio << "  " << command.reason << "\n";
            }
            if (config.display && std::getenv("DISPLAY") != nullptr) {
                cv::imshow("qmini_auto_nav", dashboard);
                cv::waitKey(1);
            }
        }
    }

    std::cout << "\n正在退出...\n";
    if (config.mode == qmini::RunMode::Map) {
        const std::string mesh_path = config.output_dir + "/mesh.obj";
        std::cout << "提取网格并写入 " << mesh_path << "\n";
        if (!zed.exportMesh(mesh_path, &error)) {
            std::cerr << error << "\n";
        } else {
            std::cout << "网格已保存。\n";
        }
    }
    if (config.area_memory || config.mode == qmini::RunMode::Map) {
        zed.saveArea(config.output_dir + "/area.area");
    }
    return 0;
}
