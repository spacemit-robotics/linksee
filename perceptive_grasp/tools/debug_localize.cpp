/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file debug_localize.cpp
    * @brief 单独测试检测 + 定位流程
    *
    * 用法:
    *   ./debug_localize --config ../config/grasp_pipeline.yaml
    *   ./debug_localize --config ../config/grasp_pipeline.yaml --frames 5
    *   ./debug_localize --config ../config/grasp_pipeline.yaml --target apple
    *
    * 功能:
    *   1. 根据主配置打开立体相机后端
    *   2. 运行 VisionService 检测
    *   3. 对每个检测目标计算中心点附近 5x5 深度中值
    *   4. 反投影到相机坐标系
    *   5. 通过手眼标定变换到机械臂基座坐标系
    *   6. 输出定位结果并保存可视化图片
    */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "grasp_planner.h"
#include "stereo_camera.h"
#include "vision_service.h"
#include "vision_result_adapter.h"

namespace fs = std::filesystem;
using perceptive_grasp::GraspPlanner;
using perceptive_grasp::GraspPlannerConfig;
using perceptive_grasp::Pose3D;
using perceptive_grasp::StereoCameraConfig;

static constexpr const char* kDefaultConfigPath = "../config/grasp_pipeline.yaml";

static const cv::Scalar kColors[] = {
    {255, 0, 0},   {0, 255, 0},   {0, 0, 255},   {255, 255, 0},
    {255, 0, 255}, {0, 255, 255}, {128, 128, 0}, {255, 128, 0},
};
static constexpr int kNumColors = sizeof(kColors) / sizeof(kColors[0]);

struct AppConfig {
    std::string config_path;
    std::string output_dir = "./debug_localize_output";
    std::string target_name;
    int num_frames = 1;
    int warmup_frames = -1;
};

static std::vector<std::string> LoadLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream f(path);
    if (!f.is_open()) return labels;
    std::string line;
    while (std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            line.pop_back();
        }
        if (!line.empty()) labels.push_back(line);
    }
    return labels;
}

static std::optional<uint16_t> MedianDepth5x5(const cv::Mat& depth, int cx, int cy) {
    std::vector<uint16_t> vals;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int x = cx + dx;
            int y = cy + dy;
            if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows) continue;
            uint16_t d = depth.at<uint16_t>(y, x);
            if (d > 0) vals.push_back(d);
        }
    }
    if (vals.empty()) return std::nullopt;
    std::sort(vals.begin(), vals.end());
    return vals[vals.size() / 2];
}

static AppConfig ParseArgs(int argc, char* argv[]) {
    AppConfig cfg;
    cfg.config_path = kDefaultConfigPath;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--help") || (arg == "-h")) {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                        << "Options:\n"
                        << "  --config <yaml>    Pipeline config "
                        "(default: ../config/grasp_pipeline.yaml)\n"
                        << "  --output <dir>     Output directory "
                        "(default: ./debug_localize_output)\n"
                        << "  --frames <N>       Number of frames to capture (default: 1)\n"
                        << "  --target <name>    Filter target class name (e.g. apple)\n"
                        << "  --warmup <N>       Warmup frames "
                        "(default: backend specific)\n";
            std::exit(0);
        } else if (arg == "--config" && i + 1 < argc) {
            cfg.config_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            cfg.output_dir = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            cfg.num_frames = std::atoi(argv[++i]);
        } else if (arg == "--target" && i + 1 < argc) {
            cfg.target_name = argv[++i];
        } else if (arg == "--warmup" && i + 1 < argc) {
            cfg.warmup_frames = std::atoi(argv[++i]);
        }
    }
    return cfg;
}

static void ResolveConfigPath(const fs::path& config_dir,
                            std::string* path) {
    if (path == nullptr || path->empty()) return;

    std::string expanded = *path;
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
        if (expanded == "~") {
            expanded = home;
        } else if (expanded.rfind("~/", 0) == 0) {
            expanded = (fs::path(home) / expanded.substr(2)).string();
        } else if (expanded.rfind("$HOME/", 0) == 0) {
            expanded = (fs::path(home) / expanded.substr(6)).string();
        }
    }

    fs::path resolved(expanded);
    if (!resolved.is_absolute()) resolved = config_dir / resolved;
    *path = fs::weakly_canonical(resolved).string();
}

static StereoCameraConfig LoadCameraConfig(
    const std::string& pipeline_config) {
    StereoCameraConfig config;
    const YAML::Node root = YAML::LoadFile(pipeline_config);
    const YAML::Node camera = root["camera"];
    if (!camera || !camera.IsMap()) {
        throw std::runtime_error("camera configuration is required");
    }

    config.type = camera["type"].as<std::string>(config.type);
    if (config.type == "realsense" || config.type == "d435i") {
        if (const YAML::Node realsense = camera["realsense"]) {
            auto& settings = config.realsense;
            settings.width = realsense["width"].as<int>(settings.width);
            settings.height = realsense["height"].as<int>(settings.height);
            settings.fps = realsense["fps"].as<int>(settings.fps);
            settings.motion_flush_frames =
                realsense["motion_flush_frames"].as<int>(
                    settings.motion_flush_frames);
            settings.align_depth = realsense["align_depth"].as<bool>(
                settings.align_depth);
            if (const YAML::Node filters = realsense["depth_filter"]) {
                settings.spatial_filter = filters["spatial"].as<bool>(
                    settings.spatial_filter);
                settings.temporal_filter = filters["temporal"].as<bool>(
                    settings.temporal_filter);
                settings.hole_filling = filters["hole_filling"].as<bool>(
                    settings.hole_filling);
            }
        }
    } else if (config.type == "spacemit_las2") {
        const YAML::Node las2 = camera["spacemit_las2"];
        if (!las2 || !las2.IsMap()) {
            throw std::runtime_error(
                "camera.spacemit_las2 configuration is required");
        }
        auto& settings = config.spacemit_las2;
        settings.video_device = las2["video_device"].as<std::string>(
            settings.video_device);
        settings.model_path = las2["model_path"].as<std::string>(
            settings.model_path);
        settings.calib_path = las2["calib_path"].as<std::string>(
            settings.calib_path);
        settings.core_count = las2["core_count"].as<int>(
            settings.core_count);
        settings.core_affinity = las2["core_affinity"].as<std::string>(
            settings.core_affinity);
        if (const YAML::Node depth = las2["depth"]) {
            settings.min_depth_m = depth["min_m"].as<float>(
                settings.min_depth_m);
            settings.max_depth_m = depth["max_m"].as<float>(
                settings.max_depth_m);
        }
    } else {
        throw std::runtime_error("unsupported camera.type: " + config.type);
    }

    fs::path config_dir = fs::path(pipeline_config).parent_path();
    if (config_dir.empty()) config_dir = ".";
    ResolveConfigPath(config_dir, &config.spacemit_las2.model_path);
    ResolveConfigPath(config_dir, &config.spacemit_las2.calib_path);
    return config;
}

static std::string ResolveDetectConfig(const std::string& pipeline_config) {
    const YAML::Node root = YAML::LoadFile(pipeline_config);
    std::string detect_config =
        root["detection"]["config_path"].as<std::string>();
    fs::path config_dir = fs::path(pipeline_config).parent_path();
    if (config_dir.empty()) config_dir = ".";
    ResolveConfigPath(config_dir, &detect_config);
    return detect_config;
}

static GraspPlannerConfig LoadPlannerConfig(const std::string& pipeline_config) {
    YAML::Node root = YAML::LoadFile(pipeline_config);
    GraspPlannerConfig cfg;

    auto t = root["calibration"]["T_base_camera"]["translation"];
    auto r = root["calibration"]["T_base_camera"]["rotation"];
    for (int i = 0; i < 3; ++i) {
        cfg.t_base_camera[i] = t[i].as<float>();
        cfg.r_base_camera[i] = r[i].as<float>();
    }

    auto grasp = root["grasp"];
    cfg.approach_height = grasp["approach_height"].as<float>();
    cfg.grasp_depth = grasp["grasp_depth"].as<float>();

    auto ws = grasp["workspace"];
    cfg.workspace.x_min = ws["x_min"].as<float>();
    cfg.workspace.x_max = ws["x_max"].as<float>();
    cfg.workspace.y_min = ws["y_min"].as<float>();
    cfg.workspace.y_max = ws["y_max"].as<float>();
    cfg.workspace.z_min = ws["z_min"].as<float>();
    cfg.workspace.z_max = ws["z_max"].as<float>();

    return cfg;
}

int main(int argc, char* argv[]) {
    AppConfig app = ParseArgs(argc, argv);
    if (app.config_path.empty()) {
        std::cerr << "[debug_localize] Error: --config is required" << std::endl;
        return 1;
    }
    if (app.num_frames < 1 || app.warmup_frames < -1) {
        std::cerr << "[debug_localize] --frames must be positive; --warmup "
                    "must be -1 or non-negative"
                << std::endl;
        return 1;
    }

    fs::create_directories(app.output_dir);
    std::cout << "[debug_localize] Output: " << fs::absolute(app.output_dir)
            << std::endl;

    StereoCameraConfig camera_config;
    std::string detect_config;
    GraspPlannerConfig planner_config;
    try {
        camera_config = LoadCameraConfig(app.config_path);
        detect_config = ResolveDetectConfig(app.config_path);
        planner_config = LoadPlannerConfig(app.config_path);
    } catch (const std::exception& e) {
        std::cerr << "[debug_localize] Config error: " << e.what()
                << std::endl;
        return 1;
    }

    if (app.warmup_frames < 0) {
        app.warmup_frames = camera_config.type == "spacemit_las2" ? 1 : 30;
    }

    std::cout << "[debug_localize] Initializing camera backend: "
            << camera_config.type << std::endl;
    auto camera = perceptive_grasp::CreateStereoCamera(camera_config);
    if (!camera || !camera->Init()) {
        std::cerr << "[debug_localize] Failed to initialize camera backend: "
                << camera_config.type << std::endl;
        return 1;
    }

    cv::Mat color;
    cv::Mat depth;
    std::cout << "[debug_localize] Warming up (" << app.warmup_frames
            << " frames)..." << std::endl;
    for (int i = 0; i < app.warmup_frames; ++i) {
        if (!camera->GetFrames(color, depth)) {
            std::cerr << "[debug_localize] Failed to acquire warmup frame "
                    << (i + 1) << std::endl;
            return 1;
        }
    }

    GraspPlanner planner(planner_config);

    auto vision = VisionService::Create(detect_config, "", false);
    if (!vision) {
        std::cerr << "[debug_localize] VisionService create failed: "
                    << VisionService::LastCreateError() << std::endl;
        return 1;
    }

    std::vector<std::string> labels;
    std::string label_path = vision->GetConfigPathValue("label_file_path");
    if (!label_path.empty()) {
        labels = LoadLabels(label_path);
    }

    std::cout << "[debug_localize] Capturing " << app.num_frames
            << " frame(s)..." << std::endl;

    for (int frame_idx = 1; frame_idx <= app.num_frames; ++frame_idx) {
        if (!camera->GetFrames(color, depth) || color.empty() ||
            depth.empty()) {
            std::cerr << "[debug_localize] Failed to acquire frame "
                    << frame_idx << std::endl;
            return 1;
        }
        cv::Mat annotated = color.clone();

        std::vector<VisionServiceResult> results;
        auto t0 = std::chrono::steady_clock::now();
        auto status = InferImageDetections(vision.get(), color, &results);
        auto t1 = std::chrono::steady_clock::now();
        double infer_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::ostringstream prefix;
        prefix << app.output_dir << "/frame_" << std::setw(3) << std::setfill('0') << frame_idx;
        std::ofstream out(prefix.str() + "_localize.txt");
        out << "# Frame " << frame_idx << " | Inference: " << std::fixed << std::setprecision(1)
            << infer_ms << " ms\n";

        if (status != VISION_SERVICE_OK) {
            out << "Detection failed, status=" << status << "\n";
            std::cerr << "[debug_localize] Detection failed, status=" << status << std::endl;
            continue;
        }

        int kept = 0;
        for (size_t i = 0; i < results.size(); ++i) {
            const auto& r = results[i];
            std::string label =
                (r.label >= 0 &&
                r.label < static_cast<int>(labels.size()))
                ? labels[r.label]
                : ("class_" + std::to_string(r.label));
            if (!app.target_name.empty() && label != app.target_name) {
                continue;
            }

            const int cx = static_cast<int>((r.x1 + r.x2) / 2);
            const int cy = static_cast<int>((r.y1 + r.y2) / 2);
            auto depth_mm_opt = MedianDepth5x5(depth, cx, cy);
            if (!depth_mm_opt.has_value()) {
                out << label << ": invalid depth around (" << cx << ", "
                    << cy << ")\n";
                continue;
            }

            const float depth_m = *depth_mm_opt * 0.001f;
            float cam_point[3] = {0.0f, 0.0f, 0.0f};
            if (!camera->Deproject(cx, cy, *depth_mm_opt, cam_point)) {
                out << label << ": deprojection failed at (" << cx << ", "
                    << cy << ")\n";
                continue;
            }

            float base_point[3] = {0.0f, 0.0f, 0.0f};
            planner.CameraToBase(cam_point, base_point);

            Pose3D grasp_pose{}, pre_grasp_pose{};
            const bool in_workspace = planner.PlanTopGrasp(
                base_point, grasp_pose, pre_grasp_pose);

            const int color_index = std::max(0, r.label) % kNumColors;
            const cv::Scalar color = kColors[color_index];
            cv::rectangle(annotated,
                        cv::Point(static_cast<int>(r.x1),
                                    static_cast<int>(r.y1)),
                        cv::Point(static_cast<int>(r.x2),
                                    static_cast<int>(r.y2)),
                        color, 2);
            cv::circle(annotated, cv::Point(cx, cy), 4, color, -1);

            std::ostringstream text;
            text << label << " z=" << std::fixed << std::setprecision(3)
                << depth_m << "m";
            cv::putText(
                annotated, text.str(),
                cv::Point(static_cast<int>(r.x1),
                        std::max(20, static_cast<int>(r.y1) - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

            out << "Detection " << kept + 1 << ": " << label
                << " score=" << std::fixed << std::setprecision(3)
                << r.score << "\n"
                << "  pixel_center: [" << cx << ", " << cy << "]\n"
                << "  median_depth_mm_5x5: " << *depth_mm_opt << "\n"
                << "  camera_point_m: [" << cam_point[0] << ", "
                << cam_point[1] << ", " << cam_point[2] << "]\n"
                << "  base_point_m:   [" << base_point[0] << ", "
                << base_point[1] << ", " << base_point[2] << "]\n"
                << "  in_workspace: " << (in_workspace ? "yes" : "no") << "\n";
            if (in_workspace) {
                out << "  pre_grasp_m:    ["
                    << pre_grasp_pose.x << ", "
                    << pre_grasp_pose.y << ", "
                    << pre_grasp_pose.z << "]\n"
                    << "  grasp_m:        ["
                    << grasp_pose.x << ", "
                    << grasp_pose.y << ", "
                    << grasp_pose.z << "]\n";
            }
            out << "\n";
            ++kept;
        }

        cv::imwrite(prefix.str() + "_localize.png", annotated);
        std::cout << "[Frame " << frame_idx << "] localize: " << prefix.str() + "_localize.txt"
                    << " | image: " << prefix.str() + "_localize.png"
                    << " | detections: " << kept << std::endl;
    }

    std::cout << "[debug_localize] Done." << std::endl;
    return 0;
}
