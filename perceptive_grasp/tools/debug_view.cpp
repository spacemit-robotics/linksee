/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file debug_view.cpp
    * @brief 调试工具 - 查看相机图像和检测结果
    *
    * 用法:
    *   ./debug_view --config ../config/grasp_pipeline.yaml
    *   ./debug_view --config ../config/grasp_pipeline.yaml --frames 5
    *   ./debug_view --config ../config/grasp_pipeline.yaml --output /tmp/debug
    *   ./debug_view --config ../config/grasp_pipeline.yaml --no-detect
    *
    * 功能:
    *   1. 根据主配置打开立体相机后端
    *   2. 抓取对齐的彩色图和深度图
    *   3. (可选) 运行 VisionService 检测并绘制 bbox
    *   4. 保存图像到指定目录
    *
    * 输出文件:
    *   <output>/frame_001_color.png     - 原始彩色图
    *   <output>/frame_001_depth.png     - 深度图 (伪彩色)
    *   <output>/frame_001_detect.png    - 检测结果标注图
    *   <output>/frame_001_result.txt    - 检测结果文本
    */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "stereo_camera.h"

#ifndef NO_VISION_SERVICE
#include "vision_service.h"
#include "vision_result_adapter.h"
#endif

namespace fs = std::filesystem;

static constexpr const char* kDefaultConfigPath = "../config/grasp_pipeline.yaml";

struct DebugViewConfig {
    perceptive_grasp::StereoCameraConfig camera;
    std::string detection_config_path = "yolov8_seg.yaml";
};

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

static DebugViewConfig LoadDebugViewConfig(const std::string& config_path) {
    DebugViewConfig config;
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node camera = root["camera"];
    if (!camera || !camera.IsMap()) {
        throw std::runtime_error("camera configuration is required");
    }

    config.camera.type = camera["type"].as<std::string>(config.camera.type);
    if (config.camera.type == "realsense" ||
        config.camera.type == "d435i") {
        const YAML::Node realsense = camera["realsense"];
        if (realsense) {
            auto& settings = config.camera.realsense;
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
    } else if (config.camera.type == "spacemit_las2") {
        const YAML::Node las2 = camera["spacemit_las2"];
        if (!las2 || !las2.IsMap()) {
            throw std::runtime_error(
                "camera.spacemit_las2 configuration is required");
        }
        auto& settings = config.camera.spacemit_las2;
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
        throw std::runtime_error("unsupported camera.type: " +
                                config.camera.type);
    }

    if (const YAML::Node detection = root["detection"]) {
        config.detection_config_path =
            detection["config_path"].as<std::string>(
                config.detection_config_path);
    }

    fs::path config_dir = fs::path(config_path).parent_path();
    if (config_dir.empty()) config_dir = ".";
    ResolveConfigPath(config_dir,
                    &config.camera.spacemit_las2.model_path);
    ResolveConfigPath(config_dir,
                    &config.camera.spacemit_las2.calib_path);
    ResolveConfigPath(config_dir, &config.detection_config_path);
    return config;
}

// ============ 颜色表 (用于绘制不同类别) ============
static const cv::Scalar COLORS[] = {
    {255, 0, 0},     {0, 255, 0},     {0, 0, 255},     {255, 255, 0},
    {255, 0, 255},   {0, 255, 255},   {128, 0, 0},     {0, 128, 0},
    {0, 0, 128},     {128, 128, 0},   {128, 0, 128},   {0, 128, 128},
    {255, 128, 0},   {255, 0, 128},   {128, 255, 0},   {0, 255, 128},
    {128, 0, 255},   {0, 128, 255},   {200, 200, 200}, {100, 100, 100},
};
static constexpr int NUM_COLORS = sizeof(COLORS) / sizeof(COLORS[0]);

// ============ 加载 COCO 标签 ============
static std::vector<std::string> LoadLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream f(path);
    if (!f.is_open()) return labels;
    std::string line;
    while (std::getline(f, line)) {
        // 去除首尾空白
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
            line.pop_back();
        labels.push_back(line);
    }
    return labels;
}

// ============ 深度图转伪彩色 ============
static cv::Mat DepthToColormap(const cv::Mat& depth_raw, float max_dist_mm = 3000.0f) {
    cv::Mat normalized;
    depth_raw.convertTo(normalized, CV_8UC1, 255.0 / max_dist_mm);
    cv::Mat colormap;
    cv::applyColorMap(normalized, colormap, cv::COLORMAP_JET);
    // 无效深度 (0) 显示为黑色
    colormap.setTo(cv::Scalar(0, 0, 0), depth_raw == 0);
    return colormap;
}

// ============ 主程序 ============
int main(int argc, char* argv[]) {
    // 参数解析
    std::string config_path = kDefaultConfigPath;
    std::string output_dir = "debug_view_output";
    int num_frames = 1;
    bool do_detect = true;
    int warmup_frames = -1;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                        << "Options:\n"
                        << "  --config <yaml>    Pipeline config "
                        "(default: ../config/grasp_pipeline.yaml)\n"
                        << "  --output <dir>     Output directory "
                        "(default: ./debug_view_output)\n"
                        << "  --frames <N>       Number of frames to capture (default: 1)\n"
                        << "  --no-detect        Skip detection, only save raw images\n"
                        << "  --warmup <N>       Override backend warmup frame count\n"
                        << "  --help             Show this help\n"
                        << "\nExamples:\n"
                        << "  " << argv[0] << " --config ../config/grasp_pipeline.yaml\n"
                        << "  " << argv[0]
                        << " --config ../config/grasp_pipeline.yaml "
                        "--frames 5 --output /tmp/debug\n"
                        << "  " << argv[0] << " --no-detect --frames 3\n";
            return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            num_frames = std::atoi(argv[++i]);
        } else if (arg == "--no-detect") {
            do_detect = false;
        } else if (arg == "--warmup" && i + 1 < argc) {
            warmup_frames = std::atoi(argv[++i]);
        }
    }

    DebugViewConfig config;
    try {
        config = LoadDebugViewConfig(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[debug_view] Config error: " << e.what() << std::endl;
        return 1;
    }

    if (num_frames < 1 || warmup_frames < -1) {
        std::cerr << "[debug_view] --frames must be positive; --warmup "
                    "must be -1 or non-negative"
                << std::endl;
        return 1;
    }
    if (warmup_frames < 0) {
        warmup_frames =
            config.camera.type == "spacemit_las2" ? 1 : 30;
    }

    // 创建输出目录
    fs::create_directories(output_dir);
    std::cout << "[debug_view] Output: " << fs::absolute(output_dir).string() << std::endl;

    // ============ 初始化相机 ============
    std::cout << "[debug_view] Initializing camera backend: "
            << config.camera.type << std::endl;
    auto camera = perceptive_grasp::CreateStereoCamera(config.camera);
    if (!camera || !camera->Init()) {
        std::cerr << "[debug_view] Failed to initialize camera backend: "
                << config.camera.type << std::endl;
        return 1;
    }

    std::cout << "[debug_view] Camera ready" << std::endl;

    // 等待自动曝光稳定
    std::cout << "[debug_view] Warming up (" << warmup_frames << " frames)..." << std::endl;
    cv::Mat color;
    cv::Mat depth;
    for (int i = 0; i < warmup_frames; i++) {
        if (!camera->GetFrames(color, depth)) {
            std::cerr << "[debug_view] Failed to acquire warmup frame "
                    << (i + 1) << std::endl;
            return 1;
        }
    }

    // ============ 初始化检测器 (可选) ============
#ifndef NO_VISION_SERVICE
    std::unique_ptr<VisionService> vision;
    std::vector<std::string> labels;

    if (do_detect) {
        std::cout << "[debug_view] Detection config: "
                << config.detection_config_path << std::endl;
        vision = VisionService::Create(
            config.detection_config_path, "", false);
        if (!vision) {
            std::cerr << "[debug_view] VisionService create failed: "
                    << VisionService::LastCreateError() << std::endl;
            do_detect = false;
        } else {
            const std::string label_path =
                vision->GetConfigPathValue("label_file_path");
            if (!label_path.empty()) {
                labels = LoadLabels(label_path);
            }
            std::cout << "[debug_view] VisionService ready, "
                    << labels.size() << " labels loaded" << std::endl;
        }
    }
#else
    if (do_detect) {
        std::cerr << "[debug_view] Built without VisionService, detection disabled\n";
        do_detect = false;
    }
#endif

    // ============ 抓取并处理帧 ============
    std::cout << "[debug_view] Capturing " << num_frames << " frame(s)...\n" << std::endl;

    for (int frame_idx = 1; frame_idx <= num_frames; frame_idx++) {
        if (!camera->GetFrames(color, depth) || color.empty() ||
            depth.empty()) {
            std::cerr << "[debug_view] Failed to acquire frame "
                    << frame_idx << std::endl;
            return 1;
        }

        // 文件名前缀
        std::ostringstream prefix;
        prefix << output_dir << "/frame_" << std::setw(3) << std::setfill('0') << frame_idx;
        std::string pfx = prefix.str();

        // 保存原始彩色图
        cv::imwrite(pfx + "_color.png", color);
        std::cout << "[Frame " << frame_idx << "] Color: " << pfx + "_color.png" << std::endl;

        // 保存深度伪彩色图
        cv::Mat depth_vis = DepthToColormap(depth);
        cv::imwrite(pfx + "_depth.png", depth_vis);
        std::cout << "[Frame " << frame_idx << "] Depth: " << pfx + "_depth.png" << std::endl;

        // 检测
#ifndef NO_VISION_SERVICE
        if (do_detect && vision) {
            std::vector<VisionServiceResult> results;
            VisionServiceResponse response;
            auto t0 = std::chrono::steady_clock::now();
            auto status = InferImageDetections(
                vision.get(), color, &results, &response);
            auto t1 = std::chrono::steady_clock::now();
            double infer_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

            if (status == VISION_SERVICE_OK) {
                // 1) 使用 VisionService::Draw() 绘制带 mask 的官方可视化
                cv::Mat seg_vis;
                auto draw_status = vision->Draw(color, response, &seg_vis);
                if (draw_status == VISION_SERVICE_OK && !seg_vis.empty()) {
                    cv::imwrite(pfx + "_seg.png", seg_vis);
                    std::cout << "[Frame " << frame_idx << "] Seg:    " << pfx + "_seg.png"
                                << " (masks + bbox)" << std::endl;
                }

                // 2) 自定义标注图 (bbox + 深度 + 标签)
                cv::Mat annotated = color.clone();

                // 先绘制 mask 半透明叠加
                for (size_t i = 0; i < results.size(); i++) {
                    const auto& r = results[i];
                    if (!r.mask.empty()) {
                        cv::Scalar clr = COLORS[r.label % NUM_COLORS];
                        // mask 可能是原图尺寸或 bbox 区域尺寸，需要处理
                        cv::Mat mask_resized;
                        if (r.mask.rows == annotated.rows && r.mask.cols == annotated.cols) {
                            mask_resized = r.mask;
                        } else {
                            // mask 是 bbox 区域大小，需要放到全图
                            mask_resized = cv::Mat::zeros(annotated.size(), CV_8UC1);
                            int bx1 = std::max(0, (int)r.x1);
                            int by1 = std::max(0, (int)r.y1);
                            int bx2 = std::min(annotated.cols, (int)r.x2);
                            int by2 = std::min(annotated.rows, (int)r.y2);
                            int bw = bx2 - bx1;
                            int bh = by2 - by1;
                            if (bw > 0 && bh > 0) {
                                cv::Mat roi_mask;
                                cv::resize(r.mask, roi_mask, cv::Size(bw, bh));
                                if (roi_mask.type() != CV_8UC1) {
                                    roi_mask.convertTo(roi_mask, CV_8UC1, 255.0);
                                }
                                roi_mask.copyTo(mask_resized(cv::Rect(bx1, by1, bw, bh)));
                            }
                        }
                        // 二值化 mask
                        cv::Mat binary_mask;
                        if (mask_resized.type() == CV_32FC1) {
                            mask_resized.convertTo(binary_mask, CV_8UC1, 255.0);
                        } else {
                            binary_mask = mask_resized;
                        }
                        cv::threshold(binary_mask, binary_mask, 127, 255, cv::THRESH_BINARY);

                        // 半透明叠加
                        cv::Mat color_overlay(annotated.size(), CV_8UC3, clr);
                        cv::Mat blended;
                        cv::addWeighted(annotated, 0.6, color_overlay, 0.4, 0, blended);
                        blended.copyTo(annotated, binary_mask);
                    }
                }

                // 再绘制 bbox + 文字 (在 mask 之上)
                std::ofstream result_file(pfx + "_result.txt");
                result_file << "# Frame " << frame_idx
                            << " | Inference: " << std::fixed << std::setprecision(1)
                            << infer_ms << " ms | Detections: " << results.size() << "\n";
                result_file
                    << "# label_id  label_name  score  x1  y1  x2  y2  "
                    << "center_x  center_y  depth_mm  has_mask\n";

                for (size_t i = 0; i < results.size(); i++) {
                    const auto& r = results[i];
                    cv::Scalar clr = COLORS[r.label % NUM_COLORS];

                    // 画 bbox
                    cv::rectangle(annotated,
                                    cv::Point((int)r.x1, (int)r.y1),
                                    cv::Point((int)r.x2, (int)r.y2),
                                    clr, 2);

                    // 标签文字
                    std::string label_text;
                    if (r.label >= 0 && r.label < (int)labels.size()) {
                        label_text = labels[r.label];
                    } else {
                        label_text = "class_" + std::to_string(r.label);
                    }
                    std::ostringstream oss;
                    oss << label_text << " " << std::fixed << std::setprecision(2) << r.score;

                    // 中心点深度
                    int cx = std::clamp(
                        static_cast<int>((r.x1 + r.x2) / 2),
                        0, depth.cols - 1);
                    int cy = std::clamp(
                        static_cast<int>((r.y1 + r.y2) / 2),
                        0, depth.rows - 1);
                    uint16_t depth_val = depth.at<uint16_t>(cy, cx);
                    float depth_m = depth_val * 0.001f;

                    oss << " " << std::setprecision(3) << depth_m << "m";

                    // 画文字背景
                    int baseline = 0;
                    cv::Size text_size = cv::getTextSize(oss.str(), cv::FONT_HERSHEY_SIMPLEX,
                                                        0.5, 1, &baseline);
                    cv::rectangle(annotated,
                                    cv::Point((int)r.x1, (int)r.y1 - text_size.height - 4),
                                    cv::Point((int)r.x1 + text_size.width, (int)r.y1),
                                    clr, -1);
                    cv::putText(annotated, oss.str(),
                                cv::Point((int)r.x1, (int)r.y1 - 2),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(255, 255, 255), 1);

                    // 画中心点
                    cv::circle(annotated, cv::Point(cx, cy), 4, clr, -1);

                    // 写入结果文件
                    result_file << r.label << "  " << label_text << "  "
                                << std::fixed << std::setprecision(3) << r.score << "  "
                                << (int)r.x1 << "  " << (int)r.y1 << "  "
                                << (int)r.x2 << "  " << (int)r.y2 << "  "
                                << cx << "  " << cy << "  " << depth_val
                                << "  " << (!r.mask.empty() ? "yes" : "no") << "\n";
                }

                result_file.close();
                cv::imwrite(pfx + "_detect.png", annotated);

                std::cout << "[Frame " << frame_idx << "] Detect: " << pfx + "_detect.png"
                            << " (" << results.size() << " objects, "
                            << std::fixed << std::setprecision(1) << infer_ms
                            << " ms)" << std::endl;
                std::cout << "[Frame " << frame_idx << "] Result: "
                        << pfx + "_result.txt" << std::endl;
            } else {
                std::cerr << "[Frame " << frame_idx << "] Detection failed (status="
                            << status << ")" << std::endl;
            }
        }
#endif

        std::cout << std::endl;
    }

    camera.reset();
    std::cout << "[debug_view] Done. Files saved to: "
            << fs::absolute(output_dir).string() << std::endl;

    return 0;
}
