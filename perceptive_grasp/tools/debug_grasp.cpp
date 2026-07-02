/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file debug_grasp.cpp
    * @brief 单独测试抓取规划与执行流程
    *
    * 功能:
    *   1. 打开 RealSense + VisionService
    *   2. 检测目标并完成定位/规划
    *   3. 打印预抓取位、抓取位和 IK 结果
    *   4. 默认只做 dry-run，不移动机械臂
    *   5. 使用 --execute 时才真正执行抓取
    *   6. 使用 --step 时每步暂停确认，便于安全测试
    */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "grasp_planner.h"
#include "orientation_estimator.h"
#include "vision_service.h"
#include "vision_result_adapter.h"

extern "C" {
#include "kinematics_interface.h"
}

namespace fs = std::filesystem;
using perceptive_grasp::ComputeGraspPixel;
using perceptive_grasp::ComputeGraspYaw;
using perceptive_grasp::ComputeOrientationFromBbox;
using perceptive_grasp::ComputeOrientationFromMask;
using perceptive_grasp::DetectionTarget;
using perceptive_grasp::GraspPlanner;
using perceptive_grasp::GraspPlannerConfig;
using perceptive_grasp::OrientationConfig;
using perceptive_grasp::Pose3D;

static constexpr const char* kDefaultConfigPath = "../config/grasp_pipeline.yaml";

static float NormalizeAnglePi(float angle) {
    while (angle > static_cast<float>(M_PI)) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle <= -static_cast<float>(M_PI)) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

static float ImageLineAngleFromHorizontal(float image_angle) {
    float angle = -image_angle;  // OpenCV image y-axis points down.
    while (angle < 0.0f) {
        angle += static_cast<float>(M_PI);
    }
    while (angle >= static_cast<float>(M_PI)) {
        angle -= static_cast<float>(M_PI);
    }
    return angle;
}

struct AppConfig {
    std::string config_path;
    std::string target_name;
    std::string output_dir = "./debug_grasp_output";
    int warmup_frames = 30;
    int detect_frames = 1;
    bool execute = false;
    bool step = false;
    bool save_images = true;
};

struct IKConfig {
    std::string urdf_path;
    std::string base_link = "base_link";
    std::string tip_link = "gripper_frame_link";
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

static std::optional<std::pair<cv::Point, uint16_t>> FindValidDepthInBox(
    const cv::Mat& depth, int x1, int y1, int x2, int y2) {
    int left = std::max(0, std::min(x1, x2));
    int right = std::min(depth.cols - 1, std::max(x1, x2));
    int top = std::max(0, std::min(y1, y2));
    int bottom = std::min(depth.rows - 1, std::max(y1, y2));
    int cx = (left + right) / 2;
    int cy = (top + bottom) / 2;

    std::optional<std::pair<cv::Point, uint16_t>> best;
    int best_dist2 = std::numeric_limits<int>::max();

    for (int y = top; y <= bottom; ++y) {
        for (int x = left; x <= right; ++x) {
            auto d = MedianDepth5x5(depth, x, y);
            if (!d.has_value()) continue;
            int dx = x - cx;
            int dy = y - cy;
            int dist2 = dx * dx + dy * dy;
            if (!best.has_value() || dist2 < best_dist2) {
                best = std::make_pair(cv::Point(x, y), *d);
                best_dist2 = dist2;
            }
        }
    }
    return best;
}

static AppConfig ParseArgs(int argc, char* argv[]) {
    AppConfig cfg;
    cfg.config_path = kDefaultConfigPath;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                        << "Options:\n"
                        << "  --config <yaml>     Pipeline config (default: ../config/grasp_pipeline.yaml)\n"
                        << "  --target <name>     Target label (e.g. apple)\n"
                        << "  --output <dir>      Output directory\n"
                        << "  --warmup <N>        Warmup frames (default: 30)\n"
                        << "  --frames <N>        Number of frames to capture (default: 1)\n"
                        << "  --execute           Actually move arm and grasp\n"
                        << "  --step              Pause before execute\n"
                        << "  --no-save           Do not save images\n";
            std::exit(0);
        } else if (arg == "--config" && i + 1 < argc) {
            cfg.config_path = argv[++i];
        } else if (arg == "--target" && i + 1 < argc) {
            cfg.target_name = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            cfg.output_dir = argv[++i];
        } else if (arg == "--warmup" && i + 1 < argc) {
            cfg.warmup_frames = std::atoi(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            cfg.detect_frames = std::atoi(argv[++i]);
        } else if (arg == "--execute") {
            cfg.execute = true;
        } else if (arg == "--step") {
            cfg.step = true;
        } else if (arg == "--no-save") {
            cfg.save_images = false;
        }
    }
    return cfg;
}

static std::string ResolveDetectConfig(const std::string& pipeline_config) {
    YAML::Node root = YAML::LoadFile(pipeline_config);
    if (!root["detection"] || !root["detection"]["config_path"]) {
        throw std::runtime_error("missing detection.config_path in pipeline config");
    }
    auto detect_cfg = root["detection"]["config_path"].as<std::string>();
    if (fs::path(detect_cfg).is_absolute()) return detect_cfg;
    return fs::weakly_canonical(fs::path(pipeline_config).parent_path() / detect_cfg).string();
}

static GraspPlannerConfig LoadPlannerConfig(const std::string& pipeline_config) {
    YAML::Node root = YAML::LoadFile(pipeline_config);
    GraspPlannerConfig cfg;

    if (root["calibration"] && root["calibration"]["T_base_camera"]) {
        auto t = root["calibration"]["T_base_camera"]["translation"];
        auto r = root["calibration"]["T_base_camera"]["rotation"];
        if (t && r && t.size() >= 3 && r.size() >= 3) {
            for (int i = 0; i < 3; ++i) {
                cfg.t_base_camera[i] = t[i].as<float>();
                cfg.r_base_camera[i] = r[i].as<float>();
            }
        }
    }

    auto grasp = root["grasp"];
    if (grasp) {
        cfg.approach_height = grasp["approach_height"].as<float>(cfg.approach_height);
        cfg.grasp_depth = grasp["grasp_depth"].as<float>(cfg.grasp_depth);
        cfg.gripper_offset = grasp["gripper_offset"].as<float>(cfg.gripper_offset);

        auto ws = grasp["workspace"];
        if (ws) {
            cfg.workspace.x_min = ws["x_min"].as<float>(cfg.workspace.x_min);
            cfg.workspace.x_max = ws["x_max"].as<float>(cfg.workspace.x_max);
            cfg.workspace.y_min = ws["y_min"].as<float>(cfg.workspace.y_min);
            cfg.workspace.y_max = ws["y_max"].as<float>(cfg.workspace.y_max);
            cfg.workspace.z_min = ws["z_min"].as<float>(cfg.workspace.z_min);
            cfg.workspace.z_max = ws["z_max"].as<float>(cfg.workspace.z_max);
        }
    }

    return cfg;
}

static IKConfig LoadIkConfig(const std::string& pipeline_config) {
    YAML::Node root = YAML::LoadFile(pipeline_config);
    IKConfig cfg;

    auto m = root["manipulator"];
    if (m) {
        cfg.urdf_path = m["urdf_path"].as<std::string>(cfg.urdf_path);
        cfg.base_link = m["base_link"].as<std::string>(cfg.base_link);
        cfg.tip_link = m["tip_link"].as<std::string>(cfg.tip_link);
    }

    if (!cfg.urdf_path.empty() && !fs::path(cfg.urdf_path).is_absolute()) {
        fs::path config_dir = fs::path(pipeline_config).parent_path();
        fs::path resolved = config_dir / cfg.urdf_path;
        if (fs::exists(resolved)) {
            cfg.urdf_path = fs::canonical(resolved).string();
        }
    }
    return cfg;
}

static std::optional<std::vector<double>> SolveArmIk(const IKConfig& cfg,
                                                        const Pose3D& pose,
                                                        std::string& extra_info) {
    if (cfg.urdf_path.empty()) {
        extra_info = "urdf_path is empty";
        return std::nullopt;
    }

    kin_solver_t* kin = kin_create(nullptr,
                                    cfg.urdf_path.c_str(),
                                    cfg.base_link.c_str(),
                                    cfg.tip_link.c_str());
    if (!kin) {
        extra_info = "kin_create failed";
        return std::nullopt;
    }

    kin_pose_t target{};
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    kin_joints_t out{};
    int ret = kin_inverse(kin, &target, nullptr, nullptr, &out);
    if (ret != KIN_OK) {
        extra_info = "kin_inverse failed: " + std::to_string(ret);
        kin_destroy(kin);
        return std::nullopt;
    }

    std::vector<double> arm_joints;
    int arm_joint_count = std::min<int>(5, out.count);
    arm_joints.reserve(arm_joint_count);
    for (int i = 0; i < arm_joint_count; ++i) arm_joints.push_back(out.q[i]);

    if (out.count > arm_joint_count) {
        std::ostringstream oss;
        oss << "extra joints(rad): [";
        for (int i = arm_joint_count; i < out.count; ++i) {
            if (i > arm_joint_count) oss << ", ";
            oss << std::fixed << std::setprecision(3) << out.q[i];
        }
        oss << "]";
        extra_info = oss.str();
    } else {
        extra_info.clear();
    }

    kin_destroy(kin);
    return arm_joints;
}

static bool GetJointLimit(const IKConfig& cfg, int joint_index,
                            double& lower, double& upper) {
    if (cfg.urdf_path.empty()) return false;

    kin_solver_t* kin = kin_create(nullptr,
                                    cfg.urdf_path.c_str(),
                                    cfg.base_link.c_str(),
                                    cfg.tip_link.c_str());
    if (!kin) return false;

    int n_joints = kin_get_num_joints(kin);
    if (joint_index < 0 || joint_index >= n_joints) {
        kin_destroy(kin);
        return false;
    }

    std::vector<double> lowers(n_joints), uppers(n_joints);
    kin_get_joint_limits(kin, lowers.data(), uppers.data());
    lower = lowers[joint_index];
    upper = uppers[joint_index];
    kin_destroy(kin);
    return true;
}

static void PrintIkResult(const char* tag,
                            const IKConfig& ik_cfg,
                            const Pose3D& pose) {
    std::string ik_extra_info;
    auto ik_joints = SolveArmIk(ik_cfg, pose, ik_extra_info);
    std::cout << tag;
    if (ik_joints.has_value()) {
        std::cout << "[";
        for (size_t i = 0; i < ik_joints->size(); ++i) {
            if (i) std::cout << ", ";
            std::cout << std::fixed << std::setprecision(3) << (*ik_joints)[i];
        }
        std::cout << "]" << std::endl;
        if (!ik_extra_info.empty()) {
            std::cout << "ik_note:        " << ik_extra_info << std::endl;
        }
    } else {
        std::cout << "<failed>" << std::endl;
        if (!ik_extra_info.empty()) {
            std::cout << "ik_note:        " << ik_extra_info << std::endl;
        }
    }
}

static bool WaitEnter(const std::string& msg) {
    std::cout << msg << std::endl;
    std::string line;
    std::getline(std::cin, line);
    return true;
}

int main(int argc, char* argv[]) {
    try {
        AppConfig app = ParseArgs(argc, argv);
        if (app.config_path.empty()) {
            std::cerr << "[debug_grasp] Error: --config is required" << std::endl;
            return 1;
        }

        fs::create_directories(app.output_dir);
        std::cout << "[debug_grasp] Stage 1: load config" << std::endl;
        std::cout << "[debug_grasp] Output: " << fs::absolute(app.output_dir) << std::endl;

        auto planner_cfg = LoadPlannerConfig(app.config_path);
        auto ik_cfg = LoadIkConfig(app.config_path);
        std::string detect_config = ResolveDetectConfig(app.config_path);

        // 加载抓取点位置比例
        float grasp_point_x_ratio = 0.0f;
        {
            YAML::Node root = YAML::LoadFile(app.config_path);
            if (root["grasp"] && root["grasp"]["grasp_point_x_ratio"]) {
                grasp_point_x_ratio = root["grasp"]["grasp_point_x_ratio"].as<float>(0.0f);
            }
        }

        std::cout << "[debug_grasp] detect config: " << detect_config << std::endl;
        std::cout << "[debug_grasp] grasp_point_x_ratio: " << grasp_point_x_ratio << std::endl;

        std::cout << "[debug_grasp] Stage 2: create planner" << std::endl;
        GraspPlanner planner(planner_cfg);
        std::cout << "[debug_grasp] Stage 3: create vision service" << std::endl;
        auto vision = VisionService::Create(detect_config, "", false);
        if (!vision) {
            std::cerr << "[debug_grasp] VisionService create failed: "
                        << VisionService::LastCreateError() << std::endl;
            return 1;
        }

        std::vector<std::string> labels;
        std::string label_path = vision->GetConfigPathValue("label_file_path");
        if (!label_path.empty()) labels = LoadLabels(label_path);

        std::cout << "[debug_grasp] Stage 4: open camera" << std::endl;
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

        std::vector<VisionServiceResult> results;
        cv::Mat color, depth, annotated;
        bool found = false;
        VisionServiceResult best{};
        float best_score = -1.0f;
        std::string best_label;
        int best_cx = 0, best_cy = 0;
        uint16_t best_depth_mm = 0;
        float cam_point[3] = {0.0f, 0.0f, 0.0f};
        float base_point[3] = {0.0f, 0.0f, 0.0f};
        Pose3D grasp_pose{}, pre_grasp_pose{};
        bool in_workspace = false;

        std::cout << "[debug_grasp] Stage 5: detect target" << std::endl;
        for (int frame_idx = 1; frame_idx <= app.detect_frames; ++frame_idx) {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::frameset aligned = align.process(frames);
            auto color_frame = aligned.get_color_frame();
            auto depth_frame = aligned.get_depth_frame();

            color = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP).clone();
            depth = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP).clone();
            annotated = color.clone();

            results.clear();
            auto status = InferImageDetections(vision.get(), color, &results);
            if (status != VISION_SERVICE_OK) {
                std::cerr << "[debug_grasp] Detection failed, status=" << status << std::endl;
                return 1;
            }

            for (size_t i = 0; i < results.size(); ++i) {
                const auto& r = results[i];
                std::string label =
                    (r.label >= 0 && r.label < static_cast<int>(labels.size()))
                    ? labels[r.label]
                    : ("class_" + std::to_string(r.label));
                if (!app.target_name.empty() && label != app.target_name) continue;
                if (!found || r.score > best_score) {
                    found = true;
                    best = r;
                    best_score = r.score;
                    best_label = label;
                }
            }
        }
        if (!found) {
            std::cerr << "[debug_grasp] No target found" << std::endl;
            return 1;
        }

        // 构造 DetectionTarget 用于方向感知的抓取点计算
        DetectionTarget det_target;
        det_target.x1 = best.x1;
        det_target.y1 = best.y1;
        det_target.x2 = best.x2;
        det_target.y2 = best.y2;
        det_target.center = cv::Point2f((best.x1 + best.x2) / 2.0f,
                                        (best.y1 + best.y2) / 2.0f);
        if (!best.mask.empty()) {
            det_target.mask = best.mask.clone();
        }

        OrientationConfig orient_cfg;
        {
            YAML::Node root = YAML::LoadFile(app.config_path);
            if (root["orientation"]) {
                orient_cfg.aspect_ratio_threshold =
                    root["orientation"]["aspect_ratio_threshold"].as<float>(1.5f);
                orient_cfg.camera_yaw_offset =
                    root["orientation"]["camera_yaw_offset"].as<float>(0.0f);
            }
        }

        // 沿短轴方向偏移到固定爪侧边缘
        float grasp_px_f, grasp_py_f;
        float offset_dir_angle = NAN;
        if (!ComputeGraspPixel(det_target, grasp_px_f, grasp_py_f,
                                grasp_point_x_ratio, orient_cfg,
                                &offset_dir_angle)) {
            grasp_px_f = det_target.center.x;
            grasp_py_f = det_target.center.y;
        }
        best_cx = static_cast<int>(grasp_px_f);
        best_cy = static_cast<int>(grasp_py_f);
        int center_cx = static_cast<int>((best.x1 + best.x2) / 2.0f);
        int center_cy = static_cast<int>((best.y1 + best.y2) / 2.0f);

        auto depth_mm_opt = MedianDepth5x5(depth, best_cx, best_cy);
        if (!depth_mm_opt.has_value()) {
            auto fallback = FindValidDepthInBox(depth,
                                                    static_cast<int>(best.x1),
                                                    static_cast<int>(best.y1),
                                                    static_cast<int>(best.x2),
                                                    static_cast<int>(best.y2));
            if (!fallback.has_value()) {
                std::cerr << "[debug_grasp] No valid depth inside target box" << std::endl;
                return 1;
            }
            best_cx = fallback->first.x;
            best_cy = fallback->first.y;
            best_depth_mm = fallback->second;
            std::cout << "[debug_grasp] center depth missing, fallback pixel: ["
                        << best_cx << ", " << best_cy << "]" << std::endl;
        } else {
            best_depth_mm = *depth_mm_opt;
        }

        std::cout << "[debug_grasp] Stage 6: localize and plan" << std::endl;
        float pixel[2] = {static_cast<float>(best_cx), static_cast<float>(best_cy)};
        float depth_m = best_depth_mm * depth_scale;
        rs2_deproject_pixel_to_point(cam_point, &intr, pixel, depth_m);
        planner.CameraToBase(cam_point, base_point);

        in_workspace = planner.PlanTopGrasp(base_point, grasp_pose, pre_grasp_pose);
        if (!in_workspace) {
            std::cerr << "[debug_grasp] Target out of workspace" << std::endl;
            std::cerr << "  base_point: ["
                        << base_point[0] << ", "
                        << base_point[1] << ", "
                        << base_point[2] << "]" << std::endl;
            return 1;
        }

        std::cout << "\n=== Debug Grasp Plan ===" << std::endl;
        std::cout << "label: " << best_label
                    << " score=" << std::fixed << std::setprecision(3)
                    << best.score << std::endl;
        std::cout << "bbox: ["
                    << static_cast<int>(best.x1) << ", "
                    << static_cast<int>(best.y1) << ", "
                    << static_cast<int>(best.x2) << ", "
                    << static_cast<int>(best.y2) << "]" << std::endl;
        std::cout << "pixel_center: [" << center_cx << ", " << center_cy << "]" << std::endl;
        std::cout << "pixel_grasp (ratio=" << grasp_point_x_ratio << "): ["
                    << best_cx << ", " << best_cy << "]" << std::endl;
        std::cout << "median_depth_mm_5x5: " << best_depth_mm << std::endl;
        std::cout << "camera_point_m: ["
                    << cam_point[0] << ", "
                    << cam_point[1] << ", "
                    << cam_point[2] << "]" << std::endl;
        std::cout << "base_point_m:   ["
                    << base_point[0] << ", "
                    << base_point[1] << ", "
                    << base_point[2] << "]" << std::endl;
        std::cout << "pre_grasp_m:    ["
                    << pre_grasp_pose.x << ", "
                    << pre_grasp_pose.y << ", "
                    << pre_grasp_pose.z << "]" << std::endl;
        std::cout << "grasp_m:        ["
                    << grasp_pose.x << ", "
                    << grasp_pose.y << ", "
                    << grasp_pose.z << "]" << std::endl;

        PrintIkResult("ik_pre_grasp:   ", ik_cfg, pre_grasp_pose);
        PrintIkResult("ik_grasp:       ", ik_cfg, grasp_pose);

        // === 方向估计调试 ===
        // 计算中间值用于调试
        float aspect_ratio = 1.0f;
        float image_angle = 0.0f;
        if (!det_target.mask.empty()) {
            image_angle = ComputeOrientationFromMask(det_target.mask, aspect_ratio);
        } else {
            image_angle = ComputeOrientationFromBbox(best.x1, best.y1, best.x2, best.y2, aspect_ratio);
        }

        float grasp_yaw = NAN;
        if (!std::isnan(offset_dir_angle)) {
            grasp_yaw = ImageLineAngleFromHorizontal(offset_dir_angle);
        } else {
            grasp_yaw = ComputeGraspYaw(det_target, orient_cfg);
        }

        std::cout << "\n=== Orientation Debug ===" << std::endl;
        std::cout << "image_angle:    " << image_angle << " rad ("
                    << image_angle * 180.0f / M_PI << "°)" << std::endl;
        std::cout << "aspect_ratio:   " << aspect_ratio << std::endl;
        std::cout << "threshold:      " << orient_cfg.aspect_ratio_threshold << std::endl;
        std::cout << "camera_yaw_off: " << orient_cfg.camera_yaw_offset << " rad" << std::endl;
        if (!std::isnan(offset_dir_angle)) {
            std::cout << "offset_dir:     " << offset_dir_angle << " rad ("
                        << offset_dir_angle * 180.0f / M_PI << "°)" << std::endl;
        }
        if (std::isnan(grasp_yaw)) {
            std::cout << "grasp_yaw:      NAN (object is symmetric, no alignment)" << std::endl;
        } else {
            std::cout << "grasp_yaw:      " << grasp_yaw << " rad ("
                        << grasp_yaw * 180.0f / M_PI << "°)" << std::endl;
            // 显示补偿后的 joint5 (需要 IK 的 joint0)
            std::string ik_info;
            auto ik_j = SolveArmIk(ik_cfg, pre_grasp_pose, ik_info);
            if (ik_j.has_value() && ik_j->size() >= 5) {
                float j0 = static_cast<float>((*ik_j)[0]);
                float scale = -0.721f;
                {
                    YAML::Node root = YAML::LoadFile(app.config_path);
                    if (auto m = root["manipulator"]) {
                        scale = m["wrist_yaw_scale"].as<float>(scale);
                    }
                }
                float j5_raw = (grasp_yaw - j0) / scale;
                double j5_min = 0.0;
                double j5_max = 0.0;
                bool have_limit = GetJointLimit(ik_cfg, 4, j5_min, j5_max);
                float j5_limited = j5_raw;
                if (have_limit) {
                    if (j5_limited < static_cast<float>(j5_min)) {
                        j5_limited = static_cast<float>(j5_min);
                    }
                    if (j5_limited > static_cast<float>(j5_max)) {
                        j5_limited = static_cast<float>(j5_max);
                    }
                }
                std::cout << "  -> joint5 raw: " << j5_raw
                            << ", limited: " << j5_limited
                            << " (j0=" << j0 << ", scale=" << scale;
                if (have_limit) {
                    std::cout << ", limit=[" << j5_min << ", " << j5_max << "]";
                }
                std::cout << ")" << std::endl;
            } else {
                std::cout << "  -> joint5 (no IK): " << grasp_yaw << " rad" << std::endl;
            }
        }

        if (app.save_images) {
            cv::rectangle(annotated,
                            cv::Point(static_cast<int>(best.x1),
                                    static_cast<int>(best.y1)),
                            cv::Point(static_cast<int>(best.x2),
                                    static_cast<int>(best.y2)),
                            cv::Scalar(0, 255, 0), 2);
            // 绿色圆点 = bbox 中心
            cv::circle(annotated, cv::Point(center_cx, center_cy), 4, cv::Scalar(0, 255, 0), -1);
            // 红色圆点 = 实际抓取点 (根据 ratio 偏移)
            cv::circle(annotated, cv::Point(best_cx, best_cy), 6, cv::Scalar(0, 0, 255), -1);
            // 连线显示偏移
            cv::line(annotated, cv::Point(center_cx, center_cy), cv::Point(best_cx, best_cy), cv::Scalar(0, 0, 255), 1);

            // === 方向可视化 ===
            if (!std::isnan(grasp_yaw)) {
                int line_len = 40;  // 可视化线段半长（像素）

                // 蓝色线 = 物体长轴方向 (image_angle)
                int ax1 = center_cx - static_cast<int>(line_len * std::cos(image_angle));
                int ay1 = center_cy - static_cast<int>(line_len * std::sin(image_angle));
                int ax2 = center_cx + static_cast<int>(line_len * std::cos(image_angle));
                int ay2 = center_cy + static_cast<int>(line_len * std::sin(image_angle));
                cv::line(annotated, cv::Point(ax1, ay1), cv::Point(ax2, ay2),
                        cv::Scalar(255, 100, 0), 2);  // 蓝色 = 物体长轴

                // 黄色线 = 夹爪张开方向 (垂直于长轴 = image_angle + 90°)
                float jaw_angle = image_angle + static_cast<float>(M_PI) / 2.0f;
                int jx1 = center_cx - static_cast<int>(line_len * std::cos(jaw_angle));
                int jy1 = center_cy - static_cast<int>(line_len * std::sin(jaw_angle));
                int jx2 = center_cx + static_cast<int>(line_len * std::cos(jaw_angle));
                int jy2 = center_cy + static_cast<int>(line_len * std::sin(jaw_angle));
                cv::line(annotated, cv::Point(jx1, jy1), cv::Point(jx2, jy2),
                        cv::Scalar(0, 255, 255), 2);  // 黄色 = 夹爪方向

                // 标注文字
                std::ostringstream yaw_text;
                yaw_text << "yaw=" << std::fixed << std::setprecision(1)
                        << grasp_yaw * 180.0f / M_PI << " deg";
                cv::putText(annotated, yaw_text.str(),
                            cv::Point((int)best.x1, (int)best.y1 - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 255), 1);

                // 标注图例
                cv::putText(annotated, "Blue=obj axis  Yellow=jaw dir",
                            cv::Point(10, annotated.rows - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            cv::Scalar(200, 200, 200), 1);
            }

            std::ostringstream prefix;
            prefix << app.output_dir << "/debug_grasp";
            cv::imwrite(prefix.str() + "_color.png", color);
            cv::imwrite(prefix.str() + "_annotated.png", annotated);
        }

        if (app.execute) {
            std::cerr
                << "[debug_grasp] --execute is temporarily disabled on K3 "
                << "for stability; use this tool for dry-run "
                << "planning/localization only." << std::endl;
            return 2;
        }

        if (app.step) {
            WaitEnter("[step] Dry-run complete. Press Enter to exit...");
        }

        std::cout << "\n[dry-run] Planning/localization completed successfully. No arm motion executed." << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[debug_grasp] Exception: " << e.what() << std::endl;
        return 1;
    }
}
