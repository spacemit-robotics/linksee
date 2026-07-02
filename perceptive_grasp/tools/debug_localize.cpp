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
    *   1. 打开 RealSense D435i 相机
    *   2. 运行 VisionService 检测
    *   3. 对每个检测目标计算中心点附近 5x5 深度中值
    *   4. 反投影到相机坐标系
    *   5. 通过手眼标定变换到机械臂基座坐标系
    *   6. 输出定位结果并保存可视化图片
    */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "grasp_planner.h"
#include "vision_service.h"
#include "vision_result_adapter.h"

namespace fs = std::filesystem;
using perceptive_grasp::GraspPlanner;
using perceptive_grasp::GraspPlannerConfig;
using perceptive_grasp::Pose3D;

static constexpr const char* kDefaultConfigPath = "../config/grasp_pipeline.yaml";

static const cv::Scalar COLORS[] = {
    {255, 0, 0},   {0, 255, 0},   {0, 0, 255},   {255, 255, 0},
    {255, 0, 255}, {0, 255, 255}, {128, 128, 0}, {255, 128, 0},
};
static constexpr int NUM_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);

struct AppConfig {
    std::string config_path;
    std::string output_dir = "./debug_localize_output";
    std::string target_name;
    int num_frames = 1;
    int warmup_frames = 30;
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
                        << "  --config <yaml>    Pipeline config (default: ../config/grasp_pipeline.yaml)\n"
                        << "  --output <dir>     Output directory (default: ./debug_localize_output)\n"
                        << "  --frames <N>       Number of frames to capture (default: 1)\n"
                        << "  --target <name>    Filter target class name (e.g. apple)\n"
                        << "  --warmup <N>       Warmup frames (default: 30)\n";
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

static std::string ResolveDetectConfig(const std::string& pipeline_config) {
    YAML::Node root = YAML::LoadFile(pipeline_config);
    auto detect_cfg = root["detection"]["config_path"].as<std::string>();
    if (fs::path(detect_cfg).is_absolute()) return detect_cfg;
    return fs::weakly_canonical(fs::path(pipeline_config).parent_path() / detect_cfg).string();
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

    fs::create_directories(app.output_dir);
    std::cout << "[debug_localize] Output: " << fs::absolute(app.output_dir) << std::endl;

    std::string detect_config = ResolveDetectConfig(app.config_path);
    auto planner_cfg = LoadPlannerConfig(app.config_path);
    GraspPlanner planner(planner_cfg);

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

    rs2::pipeline pipe;
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    rs2::pipeline_profile profile = pipe.start(rs_cfg);
    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intr = color_stream.get_intrinsics();
    rs2::align align(RS2_STREAM_COLOR);

    for (int i = 0; i < app.warmup_frames; ++i) pipe.wait_for_frames();

    std::cout << "[debug_localize] Capturing " << app.num_frames << " frame(s)..." << std::endl;

    for (int frame_idx = 1; frame_idx <= app.num_frames; ++frame_idx) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frameset aligned = align.process(frames);
        auto color_frame = aligned.get_color_frame();
        auto depth_frame = aligned.get_depth_frame();

        cv::Mat color(cv::Size(640, 480), CV_8UC3,
                        (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(640, 480), CV_16UC1,
                        (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
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
            std::string label = (r.label >= 0 && r.label < (int)labels.size())
                ? labels[r.label]
                : ("class_" + std::to_string(r.label));
            if (!app.target_name.empty() && label != app.target_name) {
                continue;
            }

            int cx = (int)((r.x1 + r.x2) / 2);
            int cy = (int)((r.y1 + r.y2) / 2);
            auto depth_mm_opt = MedianDepth5x5(depth, cx, cy);
            if (!depth_mm_opt.has_value()) {
                out << label << ": invalid depth around (" << cx << ", " << cy << ")\n";
                continue;
            }

            float depth_m = (*depth_mm_opt) * depth_scale;
            float pixel[2] = {static_cast<float>(cx), static_cast<float>(cy)};
            float cam_point[3] = {0.0f, 0.0f, 0.0f};
            rs2_deproject_pixel_to_point(cam_point, &intr, pixel, depth_m);

            float base_point[3] = {0.0f, 0.0f, 0.0f};
            planner.CameraToBase(cam_point, base_point);

            Pose3D grasp_pose{}, pre_grasp_pose{};
            bool in_workspace = planner.PlanTopGrasp(base_point, grasp_pose, pre_grasp_pose);

            cv::Scalar clr = COLORS[r.label % NUM_COLORS];
            cv::rectangle(annotated, cv::Point((int)r.x1, (int)r.y1), cv::Point((int)r.x2, (int)r.y2), clr, 2);
            cv::circle(annotated, cv::Point(cx, cy), 4, clr, -1);

            std::ostringstream text;
            text << label << " z=" << std::fixed << std::setprecision(3) << depth_m << "m";
            cv::putText(annotated, text.str(), cv::Point((int)r.x1, std::max(20, (int)r.y1 - 5)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, clr, 2);

            out << "Detection " << kept + 1 << ": " << label
                << " score=" << std::fixed << std::setprecision(3) << r.score << "\n"
                << "  pixel_center: [" << cx << ", " << cy << "]\n"
                << "  median_depth_mm_5x5: " << *depth_mm_opt << "\n"
                << "  camera_point_m: [" << cam_point[0] << ", " << cam_point[1] << ", " << cam_point[2] << "]\n"
                << "  base_point_m:   [" << base_point[0] << ", " << base_point[1] << ", " << base_point[2] << "]\n"
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

    pipe.stop();
    std::cout << "[debug_localize] Done." << std::endl;
    return 0;
}
