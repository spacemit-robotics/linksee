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
    *   3. 使用分割 mask 和对齐深度构建稀疏目标点云
    *   4. 估计桌面平面和目标三维尺寸
    *   5. 生成顶抓/侧抓候选
    *   6. 输出定位结果、点云和可视化图片
    */

#include <algorithm>
#include <chrono>
#include <cmath>
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

#include "camera_calibration.h"
#include "grasp_geometry.h"
#include "grasp_planner.h"
#include "stereo_camera.h"
#include "vision_service.h"
#include "vision_result_adapter.h"

namespace fs = std::filesystem;
using perceptive_grasp::GraspPlanner;
using perceptive_grasp::GraspPlannerConfig;
using perceptive_grasp::GraspGeometryConfig;
using perceptive_grasp::GraspGeometryPlanner;
using perceptive_grasp::GraspGeometryResult;
using perceptive_grasp::GraspStrategyName;
using perceptive_grasp::Pose3D;
using perceptive_grasp::StereoCameraConfig;
using perceptive_grasp::DetectionTarget;

static constexpr const char* kDefaultConfigPath = "../config/grasp_pipeline.yaml";

static const cv::Scalar kColors[] = {
    {255, 0, 0},   {0, 255, 0},   {0, 0, 255},   {255, 255, 0},
    {255, 0, 255}, {0, 255, 255}, {128, 128, 0}, {255, 128, 0},
};
static constexpr int kNumColors = sizeof(kColors) / sizeof(kColors[0]);

struct DebugLocalizeAppConfig {
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

static cv::Mat NormalizeDetectionMask(
    const cv::Mat& detection_mask,
    const cv::Size& image_size) {
    if (detection_mask.empty()) return {};

    cv::Mat single_channel;
    if (detection_mask.channels() == 1) {
        single_channel = detection_mask;
    } else {
        cv::cvtColor(
            detection_mask, single_channel, cv::COLOR_BGR2GRAY);
    }

    double max_value = 0.0;
    cv::minMaxLoc(single_channel, nullptr, &max_value);
    cv::Mat normalized;
    single_channel.convertTo(
        normalized, CV_8UC1, max_value <= 1.0 ? 255.0 : 1.0);
    if (normalized.size() != image_size) {
        cv::resize(
            normalized, normalized, image_size, 0.0, 0.0,
            cv::INTER_NEAREST);
    }
    cv::threshold(normalized, normalized, 0, 255, cv::THRESH_BINARY);
    return normalized;
}

static uint16_t DepthPercentile(
    std::vector<uint16_t> values,
    float quantile) {
    if (values.empty()) return 0;
    quantile = std::clamp(quantile, 0.0f, 1.0f);
    const size_t index = static_cast<size_t>(std::lround(
        quantile * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

static void SaveObjectPointCloud(
    const fs::path& path,
    const std::vector<cv::Point3f>& points) {
    std::ofstream output(path);
    output << "ply\nformat ascii 1.0\n"
            << "element vertex " << points.size() << "\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "end_header\n";
    output << std::fixed << std::setprecision(6);
    for (const cv::Point3f& point : points) {
        output << point.x << " " << point.y << " " << point.z << "\n";
    }
}

static DebugLocalizeAppConfig ParseArgs(int argc, char* argv[]) {
    DebugLocalizeAppConfig cfg;
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
    } else if (config.type == "mujoco") {
        const YAML::Node mujoco = camera["mujoco"];
        if (!mujoco || !mujoco.IsMap()) {
            throw std::runtime_error("camera.mujoco configuration is required");
        }
        auto& settings = config.mujoco;
        settings.xml_path = mujoco["xml_path"].as<std::string>(
            settings.xml_path);
        settings.camera_name = mujoco["camera_name"].as<std::string>(
            settings.camera_name);
        settings.width = mujoco["width"].as<int>(settings.width);
        settings.height = mujoco["height"].as<int>(settings.height);
        if (const YAML::Node depth = mujoco["depth"]) {
            settings.min_depth_m = depth["min_m"].as<float>(
                settings.min_depth_m);
            settings.max_depth_m = depth["max_m"].as<float>(
                settings.max_depth_m);
        }
    } else if (config.type == "remote_mujoco") {
        const YAML::Node remote = camera["remote_mujoco"];
        if (!remote || !remote.IsMap()) {
            throw std::runtime_error(
                "camera.remote_mujoco configuration is required");
        }
        auto& settings = config.remote_mujoco;
        settings.host = remote["host"].as<std::string>(settings.host);
        settings.port = remote["port"].as<int>(settings.port);
        settings.timeout_ms =
            remote["timeout_ms"].as<int>(settings.timeout_ms);
    } else {
        throw std::runtime_error("unsupported camera.type: " + config.type);
    }

    fs::path config_dir = fs::path(pipeline_config).parent_path();
    if (config_dir.empty()) config_dir = ".";
    ResolveConfigPath(config_dir, &config.spacemit_las2.model_path);
    ResolveConfigPath(config_dir, &config.spacemit_las2.calib_path);
    ResolveConfigPath(config_dir, &config.mujoco.xml_path);
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

    const std::string camera_type =
        root["camera"]["type"].as<std::string>("realsense");
    perceptive_grasp::LoadCameraCalibration(root, camera_type, &cfg);

    const YAML::Node grasp = root["grasp"];
    const YAML::Node top = grasp["top"];
    if (!top) {
        throw std::runtime_error("grasp.top configuration is required");
    }
    cfg.approach_height = top["approach_height"].as<float>(
        cfg.approach_height);
    cfg.grasp_depth = top["grasp_depth"].as<float>(cfg.grasp_depth);
    cfg.gripper_offset = top["gripper_offset"].as<float>(
        cfg.gripper_offset);

    auto ws = grasp["workspace"];
    cfg.workspace.x_min = ws["x_min"].as<float>();
    cfg.workspace.x_max = ws["x_max"].as<float>();
    cfg.workspace.y_min = ws["y_min"].as<float>();
    cfg.workspace.y_max = ws["y_max"].as<float>();
    cfg.workspace.z_min = ws["z_min"].as<float>();
    cfg.workspace.z_max = ws["z_max"].as<float>();

    return cfg;
}

static GraspGeometryConfig LoadGeometryConfig(
    const std::string& pipeline_config) {
    const YAML::Node root = YAML::LoadFile(pipeline_config);
    const YAML::Node grasp = root["grasp"];
    const YAML::Node geometry = grasp["geometry"];
    const YAML::Node side = grasp["side"];
    GraspGeometryConfig config;

    config.strategy = grasp["strategy"].as<std::string>(config.strategy);
    if (geometry) {
        config.sample_stride = geometry["sample_stride"].as<int>(
            config.sample_stride);
        config.max_object_points = geometry["max_object_points"].as<int>(
            config.max_object_points);
        config.min_object_points = geometry["min_object_points"].as<int>(
            config.min_object_points);
        config.plane_distance_threshold_m =
            geometry["plane_distance_threshold_m"].as<float>(
                config.plane_distance_threshold_m);
        config.table_clearance_m = geometry["table_clearance_m"].as<float>(
            config.table_clearance_m);
        config.footprint_padding_m =
            geometry["footprint_padding_m"].as<float>(
                config.footprint_padding_m);
        config.gripper_max_width_m =
            geometry["gripper_max_width_m"].as<float>(
                config.gripper_max_width_m);
        config.planning_timeout_ms = geometry["planning_timeout_ms"].as<int>(
            config.planning_timeout_ms);
        config.perception_budget_ms =
            geometry["perception_budget_ms"].as<int>(
                config.perception_budget_ms);
    }
    if (side) {
        config.side_min_height_m = side["min_height_m"].as<float>(
            config.side_min_height_m);
        config.side_approach_distance_m =
            side["approach_distance_m"].as<float>(
                config.side_approach_distance_m);
        config.side_entry_clearance_m =
            side["entry_clearance_m"].as<float>(
                config.side_entry_clearance_m);
        config.side_pregrasp_min_x_m =
            side["pregrasp_min_x_m"].as<float>(
                config.side_pregrasp_min_x_m);
        config.side_single_sided_gripper =
            side["single_sided_gripper"].as<bool>(
                config.side_single_sided_gripper);
        config.side_gripper_offset_m = side["gripper_offset_m"].as<float>(
            config.side_gripper_offset_m);
        config.side_visible_surface_offset_m =
            side["visible_surface_offset_m"].as<float>(
                config.side_visible_surface_offset_m);
        config.side_grasp_forward_offset_m =
            side["grasp_forward_offset_m"].as<float>(
                config.side_grasp_forward_offset_m);
        config.side_grasp_height_ratio =
            side["grasp_height_ratio"].as<float>(
                config.side_grasp_height_ratio);
        config.side_initial_lift_m = side["initial_lift_m"].as<float>(
            config.side_initial_lift_m);
        config.side_lift_retreat_m = side["lift_retreat_m"].as<float>(
            config.side_lift_retreat_m);
    }
    return config;
}

static void SaveCaptureContext(
    const DebugLocalizeAppConfig& app,
    const StereoCameraConfig& camera_config) {
    const fs::path output_dir = fs::absolute(app.output_dir);
    const fs::path source_config = fs::absolute(app.config_path);
    const fs::path config_snapshot =
        output_dir / "pipeline_config_snapshot.yaml";
    std::error_code copy_error;
    fs::copy_file(
        source_config, config_snapshot,
        fs::copy_options::overwrite_existing, copy_error);
    if (copy_error) {
        throw std::runtime_error(
            "failed to save pipeline config snapshot: " +
            copy_error.message());
    }

    YAML::Node manifest;
    manifest["format_version"] = 1;
    manifest["camera_backend"] = camera_config.type;
    manifest["source_config"] = source_config.string();
    manifest["config_snapshot"] = config_snapshot.filename().string();
    manifest["target_filter"] =
        app.target_name.empty() ? "*" : app.target_name;
    manifest["frames"] = app.num_frames;
    manifest["warmup_frames"] = app.warmup_frames;
    manifest["coordinate_frame"]["object_cloud"] = "manipulator_base";
    manifest["units"]["depth"] = "millimeter";
    manifest["units"]["point_cloud"] = "meter";
    manifest["artifacts"]["color"] = "frame_NNN_color.png";
    manifest["artifacts"]["depth"] = "frame_NNN_depth_mm.png";
    manifest["artifacts"]["mask"] = "frame_NNN_mask_INDEX.png";
    manifest["artifacts"]["object_cloud"] =
        "frame_NNN_object_INDEX.ply";
    manifest["artifacts"]["plan"] = "frame_NNN_localize.txt";
    manifest["artifacts"]["visualization"] =
        "frame_NNN_localize.png";

    const fs::path manifest_path = output_dir / "capture_manifest.yaml";
    std::ofstream output(manifest_path);
    if (!output.is_open()) {
        throw std::runtime_error(
            "failed to open capture manifest: " +
            manifest_path.string());
    }
    output << manifest << "\n";
    if (!output.good()) {
        throw std::runtime_error(
            "failed to write capture manifest: " +
            manifest_path.string());
    }
}

int main(int argc, char* argv[]) {
    DebugLocalizeAppConfig app = ParseArgs(argc, argv);
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
    GraspGeometryConfig geometry_config;
    try {
        camera_config = LoadCameraConfig(app.config_path);
        detect_config = ResolveDetectConfig(app.config_path);
        planner_config = LoadPlannerConfig(app.config_path);
        geometry_config = LoadGeometryConfig(app.config_path);
    } catch (const std::exception& e) {
        std::cerr << "[debug_localize] Config error: " << e.what()
                << std::endl;
        return 1;
    }

    if (app.warmup_frames < 0) {
        app.warmup_frames =
            (camera_config.type == "spacemit_las2" ||
                camera_config.type == "mujoco" ||
                camera_config.type == "remote_mujoco") ? 1 : 30;
    }
    try {
        SaveCaptureContext(app, camera_config);
    } catch (const std::exception& error) {
        std::cerr << "[debug_localize] Failed to save capture context: "
            << error.what() << std::endl;
        return 1;
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
    GraspGeometryPlanner geometry_planner(geometry_config, planner_config);

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
        const auto capture_started_at = std::chrono::steady_clock::now();
        if (!camera->GetFrames(color, depth) || color.empty() ||
            depth.empty()) {
            std::cerr << "[debug_localize] Failed to acquire frame "
                    << frame_idx << std::endl;
            return 1;
        }
        const double capture_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - capture_started_at)
                .count();
        cv::Mat annotated = color.clone();

        std::vector<VisionServiceResult> results;
        auto t0 = std::chrono::steady_clock::now();
        auto status = InferImageDetections(vision.get(), color, &results);
        auto t1 = std::chrono::steady_clock::now();
        double infer_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::ostringstream prefix;
        prefix << app.output_dir << "/frame_" << std::setw(3) << std::setfill('0') << frame_idx;
        cv::imwrite(prefix.str() + "_color.png", color);
        cv::imwrite(prefix.str() + "_depth_mm.png", depth);
        std::ofstream out(prefix.str() + "_localize.txt");
        out << "# Frame " << frame_idx
            << " | Capture: " << std::fixed << std::setprecision(1)
            << capture_ms
            << " ms | Inference: " << infer_ms
            << " ms | Perception: " << capture_ms + infer_ms << " ms\n";

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

            DetectionTarget target{};
            target.x1 = r.x1;
            target.y1 = r.y1;
            target.x2 = r.x2;
            target.y2 = r.y2;
            target.score = r.score;
            target.label = r.label;
            target.label_name = label;
            target.center = cv::Point2f(
                (r.x1 + r.x2) * 0.5f, (r.y1 + r.y2) * 0.5f);
            target.mask = r.mask;
            target.area = (r.x2 - r.x1) * (r.y2 - r.y1);

            const cv::Mat normalized_mask =
                NormalizeDetectionMask(target.mask, depth.size());
            std::vector<uint16_t> mask_depths;
            if (!normalized_mask.empty()) {
                mask_depths.reserve(cv::countNonZero(normalized_mask));
                for (int y = 0; y < depth.rows; ++y) {
                    const uint8_t* mask_row =
                        normalized_mask.ptr<uint8_t>(y);
                    const uint16_t* depth_row = depth.ptr<uint16_t>(y);
                    for (int x = 0; x < depth.cols; ++x) {
                        if (mask_row[x] != 0 && depth_row[x] != 0) {
                            mask_depths.push_back(depth_row[x]);
                        }
                    }
                }
                cv::imwrite(
                    prefix.str() + "_mask_" +
                        std::to_string(kept + 1) + ".png",
                    normalized_mask);
            }

            GraspGeometryResult geometry_result;
            const auto geometry_started_at = std::chrono::steady_clock::now();
            const bool geometry_valid = geometry_planner.Plan(
                depth, target, *camera, planner, geometry_result);
            const double geometry_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - geometry_started_at)
                    .count();
            const auto candidate = std::find_if(
                geometry_result.candidates.begin(),
                geometry_result.candidates.end(),
                [](const perceptive_grasp::GraspCandidate& value) {
                    return value.geometry_valid;
                });
            const bool in_workspace = geometry_valid &&
                candidate != geometry_result.candidates.end();
            Pose3D grasp_pose{};
            Pose3D pre_grasp_pose{};
            if (geometry_result.geometry.valid) {
                base_point[0] = geometry_result.geometry.center.x;
                base_point[1] = geometry_result.geometry.center.y;
                base_point[2] = geometry_result.geometry.center.z;
                SaveObjectPointCloud(
                    prefix.str() + "_object_" + std::to_string(kept + 1) +
                        ".ply",
                    geometry_result.object_points);
            }
            if (in_workspace) {
                grasp_pose = candidate->grasp_pose;
                pre_grasp_pose = candidate->pre_grasp_pose;
            }

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
            text << label;
            if (geometry_result.geometry.valid) {
                text << (in_workspace
                        ? std::string(" ") +
                            GraspStrategyName(candidate->strategy) + " "
                        : " no-grasp ")
                    << std::fixed << std::setprecision(0)
                    << geometry_result.geometry.length_m * 1000.0f << "x"
                    << geometry_result.geometry.width_m * 1000.0f << "x"
                    << geometry_result.geometry.height_m * 1000.0f << "mm";
            } else {
                text << " geometry failed";
            }
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
                << "  geometry_ms: " << geometry_ms << "\n"
                << "  geometry_result: "
                << (geometry_valid ? "valid" : geometry_result.error) << "\n"
                << "  in_workspace: " << (in_workspace ? "yes" : "no") << "\n";
            if (!mask_depths.empty()) {
                out << "  mask_depth_samples: " << mask_depths.size() << "\n"
                    << "  mask_depth_percentiles_mm: ["
                    << DepthPercentile(mask_depths, 0.02f) << ", "
                    << DepthPercentile(mask_depths, 0.10f) << ", "
                    << DepthPercentile(mask_depths, 0.25f) << ", "
                    << DepthPercentile(mask_depths, 0.50f) << ", "
                    << DepthPercentile(mask_depths, 0.75f) << ", "
                    << DepthPercentile(mask_depths, 0.90f) << ", "
                    << DepthPercentile(mask_depths, 0.98f) << "]\n";
            }
            if (geometry_result.geometry.valid) {
                out << "  dimensions_m:  ["
                    << geometry_result.geometry.length_m << ", "
                    << geometry_result.geometry.width_m << ", "
                    << geometry_result.geometry.height_m << "]\n"
                    << std::setprecision(6)
                    << "  support_plane: normal=["
                    << geometry_result.geometry.table.normal.x << ", "
                    << geometry_result.geometry.table.normal.y << ", "
                    << geometry_result.geometry.table.normal.z << "] d="
                    << geometry_result.geometry.table.d
                    << " inliers="
                    << geometry_result.geometry.table.inlier_count << "\n"
                    << std::setprecision(3);
            }
            if (in_workspace) {
                out << "  selected: "
                    << GraspStrategyName(candidate->strategy)
                    << " score=" << candidate->score
                    << " required_width_m="
                    << candidate->required_width_m << "\n";
                out << "  pre_grasp_m:    ["
                    << pre_grasp_pose.x << ", "
                    << pre_grasp_pose.y << ", "
                    << pre_grasp_pose.z << "]\n"
                    << "  pre_grasp_quat: ["
                    << pre_grasp_pose.qw << ", "
                    << pre_grasp_pose.qx << ", "
                    << pre_grasp_pose.qy << ", "
                    << pre_grasp_pose.qz << "]\n"
                    << "  grasp_m:        ["
                    << grasp_pose.x << ", "
                    << grasp_pose.y << ", "
                    << grasp_pose.z << "]\n";
                const float approach_dx =
                    grasp_pose.x - pre_grasp_pose.x;
                const float approach_dy =
                    grasp_pose.y - pre_grasp_pose.y;
                const float approach_dz =
                    grasp_pose.z - pre_grasp_pose.z;
                out << "  approach_delta_m: distance="
                    << std::sqrt(
                        approach_dx * approach_dx +
                        approach_dy * approach_dy +
                        approach_dz * approach_dz)
                    << " dx=" << approach_dx
                    << " dy=" << approach_dy
                    << " dz=" << approach_dz << "\n";
            }
            for (const auto& value : geometry_result.candidates) {
                out << "  candidate: strategy="
                    << GraspStrategyName(value.strategy)
                    << " score=" << value.score
                    << " required_width_m=" << value.required_width_m
                    << " valid=" << (value.geometry_valid ? "yes" : "no")
                    << " reason=" << value.rejection_reason << "\n";
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
