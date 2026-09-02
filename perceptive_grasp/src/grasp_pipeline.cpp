/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_pipeline.cpp
    * @brief 视觉抓取主 Pipeline 实现
    */

#include "grasp_pipeline.h"
#include "grasp_depth.h"
#include "pipeline_debug_writer.h"
#include "voice_command_parser.h"

#ifdef MOCK_DETECTOR
#include "mock/mock_detector.h"
#endif
#ifdef MOCK_EXECUTOR
#include "mock/mock_executor.h"
#endif
#ifdef HAVE_MUJOCO_EXECUTOR
#include "mujoco_grasp_executor.h"
#endif
#ifdef HAVE_REMOTE_MUJOCO
#include "remote_mujoco_grasp_executor.h"
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {

namespace {

constexpr int kMaxGeometryAttempts = 12;
constexpr int kGeometryRefreshAttempt = 6;
constexpr int kTopGeometryRecoveryAttempt = 1;
constexpr int kMotionGeometryRequiredConsistentPairs = 1;
constexpr int kMotionGeometryMaxSamples = 15;
constexpr int kMotionGeometryMaxRefreshes = 1;
constexpr int kMinimumCandidateValidationTimeoutMs = 750;
constexpr float kTopAlignmentMaximumFrameDeltaM = 0.05f;

bool BuildSupportPlane(const TablePlane& table, SupportPlane& support_plane) {
    const float normal_norm = std::sqrt(
        table.normal.x * table.normal.x +
        table.normal.y * table.normal.y +
        table.normal.z * table.normal.z);
    if (table.inlier_count <= 0 || !table.bounds_valid ||
        !std::isfinite(normal_norm) || normal_norm < 0.9f ||
        !std::isfinite(table.d)) {
        return false;
    }
    support_plane.normal_x = table.normal.x;
    support_plane.normal_y = table.normal.y;
    support_plane.normal_z = table.normal.z;
    support_plane.d = table.d;
    support_plane.valid = true;
    support_plane.min_x = table.min_x;
    support_plane.max_x = table.max_x;
    support_plane.min_y = table.min_y;
    support_plane.max_y = table.max_y;
    support_plane.bounds_valid = true;
    return true;
}

bool BuildWorkspaceSupportPlane(
    const WorkspaceLimits& workspace,
    SupportPlane& support_plane) {
    if (!std::isfinite(workspace.x_min) ||
        !std::isfinite(workspace.x_max) ||
        !std::isfinite(workspace.y_min) ||
        !std::isfinite(workspace.y_max) ||
        !std::isfinite(workspace.z_min) ||
        workspace.x_min > workspace.x_max ||
        workspace.y_min > workspace.y_max) {
        return false;
    }
    support_plane.normal_x = 0.0f;
    support_plane.normal_y = 0.0f;
    support_plane.normal_z = 1.0f;
    support_plane.d = -workspace.z_min;
    support_plane.valid = true;
    support_plane.min_x = workspace.x_min;
    support_plane.max_x = workspace.x_max;
    support_plane.min_y = workspace.y_min;
    support_plane.max_y = workspace.y_max;
    support_plane.bounds_valid = true;
    return true;
}

float EnforceTopSupportClearance(
    GraspCandidate& candidate,
    const SupportPlane& support_plane,
    float minimum_clearance_m) {
    const float normal_norm = std::sqrt(
        support_plane.normal_x * support_plane.normal_x +
        support_plane.normal_y * support_plane.normal_y +
        support_plane.normal_z * support_plane.normal_z);
    if (!support_plane.valid || minimum_clearance_m <= 0.0f ||
        !std::isfinite(normal_norm) || normal_norm < 1e-6f) {
        return 0.0f;
    }

    float normal_x = support_plane.normal_x / normal_norm;
    float normal_y = support_plane.normal_y / normal_norm;
    float normal_z = support_plane.normal_z / normal_norm;
    float plane_d = support_plane.d / normal_norm;
    if (normal_z < 0.0f) {
        normal_x = -normal_x;
        normal_y = -normal_y;
        normal_z = -normal_z;
        plane_d = -plane_d;
    }

    const float current_clearance =
        normal_x * candidate.grasp_pose.x +
        normal_y * candidate.grasp_pose.y +
        normal_z * candidate.grasp_pose.z + plane_d;
    if (!std::isfinite(current_clearance)) {
        return 0.0f;
    }
    const float adjustment = std::max(
        0.0f, minimum_clearance_m - current_clearance);
    if (adjustment <= 0.0f) {
        return 0.0f;
    }

    auto shift_pose = [normal_x, normal_y, normal_z, adjustment](
        Pose3D& pose) {
        pose.x += normal_x * adjustment;
        pose.y += normal_y * adjustment;
        pose.z += normal_z * adjustment;
    };
    shift_pose(candidate.grasp_pose);
    shift_pose(candidate.pre_grasp_pose);
    shift_pose(candidate.retreat_pose);
    shift_pose(candidate.lift_pose);
    return adjustment;
}

void WriteStructuredLine(const std::string& text) {
    const std::string line = text + "\n";
    ssize_t written;
    do {
        written = write(STDOUT_FILENO, line.data(), line.size());
    } while (written < 0 && errno == EINTR);
    if (written < 0) {
        std::cout << line << std::flush;
    }
}

float NormalizeAnglePi(float angle) {
    while (angle > static_cast<float>(M_PI)) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle <= -static_cast<float>(M_PI)) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

float ImageLineAngleFromHorizontal(float image_angle) {
    float angle = -image_angle;  // OpenCV image y-axis points down.
    while (angle < 0.0f) {
        angle += static_cast<float>(M_PI);
    }
    while (angle >= static_cast<float>(M_PI)) {
        angle -= static_cast<float>(M_PI);
    }
    return angle;
}

const char* GraspResultName(GraspResult result) {
    switch (result) {
        case GraspResult::SUCCESS: return "SUCCESS";
        case GraspResult::EMPTY: return "EMPTY";
        case GraspResult::IK_FAILED: return "IK_FAILED";
        case GraspResult::OUT_OF_RANGE: return "OUT_OF_RANGE";
        case GraspResult::MOVE_FAILED: return "MOVE_FAILED";
        case GraspResult::TIMEOUT: return "TIMEOUT";
    }
    return "UNKNOWN";
}

int ClampPixel(int value, int limit) {
    return std::clamp(value, 0, std::max(0, limit - 1));
}

bool MedianDepthAtPixel(const cv::Mat& depth, int cx, int cy, int roi_size,
                        uint16_t& depth_mm) {
    if (depth.empty()) {
        return false;
    }

    cx = ClampPixel(cx, depth.cols);
    cy = ClampPixel(cy, depth.rows);

    const int x_start = std::max(0, cx - roi_size);
    const int y_start = std::max(0, cy - roi_size);
    const int x_end = std::min(depth.cols - 1, cx + roi_size);
    const int y_end = std::min(depth.rows - 1, cy + roi_size);

    std::vector<uint16_t> depth_values;
    for (int y = y_start; y <= y_end; y++) {
        for (int x = x_start; x <= x_end; x++) {
            uint16_t d = depth.at<uint16_t>(y, x);
            if (IsValidGraspDepth(d)) {
                depth_values.push_back(d);
            }
        }
    }

    if (depth_values.empty()) {
        return false;
    }

    std::sort(depth_values.begin(), depth_values.end());
    depth_mm = depth_values[depth_values.size() / 2];
    return true;
}

bool ForegroundDepthFromMask(const cv::Mat& depth, const cv::Mat& input_mask,
                            uint16_t& depth_mm, size_t& sample_count) {
    sample_count = 0;
    if (depth.empty() || depth.type() != CV_16UC1 || input_mask.empty()) {
        return false;
    }

    cv::Mat mask;
    if (input_mask.size() != depth.size()) {
        cv::resize(input_mask, mask, depth.size(), 0.0, 0.0,
                cv::INTER_NEAREST);
    } else {
        mask = input_mask.clone();
    }
    if (mask.channels() == 3) {
        cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
    } else if (mask.channels() == 4) {
        cv::cvtColor(mask, mask, cv::COLOR_BGRA2GRAY);
    } else if (mask.channels() != 1) {
        return false;
    }
    if (mask.type() != CV_8UC1) {
        mask.convertTo(mask, CV_8UC1);
    }
    cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);

    cv::Mat inner_mask;
    cv::erode(mask, inner_mask, cv::Mat(), cv::Point(-1, -1), 1);
    if (cv::countNonZero(inner_mask) < 20) {
        inner_mask = mask;
    }

    std::vector<uint16_t> depth_values;
    depth_values.reserve(static_cast<size_t>(cv::countNonZero(inner_mask)));
    for (int y = 0; y < depth.rows; ++y) {
        const auto* depth_row = depth.ptr<uint16_t>(y);
        const auto* mask_row = inner_mask.ptr<uint8_t>(y);
        for (int x = 0; x < depth.cols; ++x) {
            if (mask_row[x] != 0 &&
                IsValidGraspDepth(depth_row[x])) {
                depth_values.push_back(depth_row[x]);
            }
        }
    }
    if (depth_values.size() < 12) {
        return false;
    }

    // Favor the object surface over the farther support plane.
    std::sort(depth_values.begin(), depth_values.end());
    sample_count = depth_values.size();
    depth_mm = depth_values[depth_values.size() / 4];
    return true;
}

std::int64_t ProcessCpuMillis() {
    timespec value{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
        return 0;
    }
    return static_cast<std::int64_t>(value.tv_sec) * 1000 +
        static_cast<std::int64_t>(value.tv_nsec) / 1000000;
}

}  // namespace

GraspPipeline::GraspPipeline(const PipelineConfig& config) : config_(config) {}

GraspPipeline::~GraspPipeline() {
    Stop();

    // LAS2 and VisionService both own inference runtime resources. Stop the
    // camera workers before destroying the detector to avoid cross-runtime
    // teardown deadlocks.
    mobile_base_.reset();
    executor_.reset();
    geometry_planner_.reset();
    planner_.reset();
    std::cout << "[Pipeline] Releasing camera..." << std::endl;
    camera_.reset();
    std::cout << "[Pipeline] Camera released" << std::endl;
    std::cout << "[Pipeline] Releasing detector..." << std::endl;
    detector_.reset();
    std::cout << "[Pipeline] Detector released" << std::endl;
}

bool GraspPipeline::Init() {
    const auto pipeline_start = std::chrono::steady_clock::now();
    const auto log_init_time = [this](const char* module,
                                    const auto& started_at,
                                    std::int64_t started_cpu_ms) {
        if (!config_.performance_log_enabled) return;
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started_at)
                .count();
        std::cout << "[Init] END module=" << module
                << " elapsed_ms=" << elapsed_ms
                << " cpu_ms=" << ProcessCpuMillis() - started_cpu_ms
                << " result=SUCCESS" << std::endl;
    };
    const auto log_init_failure = [&pipeline_start](const char* module) {
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - pipeline_start)
                .count();
        std::cerr << "[Init] END module=" << module
                << " result=FAILED" << std::endl;
        std::cerr << "[Init] SUMMARY result=FAILED"
                << " elapsed_ms=" << elapsed_ms << std::endl;
    };

    std::cout << "\n[Init] START pipeline" << std::endl;

    // 初始化立体相机
    std::cout << "[Init] START module=camera" << std::endl;
    const auto camera_start = std::chrono::steady_clock::now();
    const auto camera_cpu_start = ProcessCpuMillis();
    camera_ = CreateStereoCamera(config_.camera);
    if (!camera_) {
        std::cerr << "[Pipeline] Failed to create stereo camera backend: "
                    << config_.camera.type << std::endl;
        log_init_failure("camera");
        return false;
    }
    if (!camera_->Init()) {
        std::cerr << "[Pipeline] Failed to init stereo camera backend: "
                    << config_.camera.type << std::endl;
        log_init_failure("camera");
        return false;
    }
    log_init_time("camera", camera_start, camera_cpu_start);

    // LAS2 lazily prepares its inference graph on the first GetFrames call.
    // Finish that work before creating VisionService so the two ONNX Runtime
    // backends do not perform their one-time graph setup at the same time.
    const auto camera_warmup_start = std::chrono::steady_clock::now();
    const auto camera_warmup_cpu_start = ProcessCpuMillis();
    std::cout << "[Init] START module=camera_warmup" << std::endl;
    cv::Mat warmup_color, warmup_depth;
    if (!camera_->GetFrames(warmup_color, warmup_depth)) {
        std::cerr << "[Pipeline] Failed to warm up stereo camera" << std::endl;
        log_init_failure("camera_warmup");
        return false;
    }
    log_init_time("camera_warmup", camera_warmup_start,
                camera_warmup_cpu_start);

    // 初始化目标检测器
    std::cout << "[Init] START module=detector" << std::endl;
    const auto detector_start = std::chrono::steady_clock::now();
    const auto detector_cpu_start = ProcessCpuMillis();
#ifdef MOCK_DETECTOR
    detector_ = std::make_unique<MockDetector>(config_.detector);
#else
    detector_ = std::make_unique<TargetDetector>(config_.detector);
#endif
    if (!detector_->Init()) {
        std::cerr << "[Pipeline] Failed to init detector" << std::endl;
        log_init_failure("detector");
        return false;
    }
    log_init_time("detector", detector_start, detector_cpu_start);

    // Pay the detector's first-inference cost during startup. The warmup frame
    // is discarded; LAS2 will select the newest prepared frame on the next
    // GetFrames call.
    const auto detector_warmup_start = std::chrono::steady_clock::now();
    const auto detector_warmup_cpu_start = ProcessCpuMillis();
    std::cout << "[Init] START module=detector_warmup" << std::endl;
    std::vector<DetectionTarget> warmup_targets;
    if (!detector_->Detect(warmup_color, warmup_targets)) {
        std::cerr << "[Pipeline] Failed to warm up detector" << std::endl;
        log_init_failure("detector_warmup");
        return false;
    }
    log_init_time("detector_warmup", detector_warmup_start,
                detector_warmup_cpu_start);
    std::cout << "[Init] Stereo camera and detector warmup complete"
            << std::endl;

    // 初始化抓取规划器
    planner_ = std::make_unique<GraspPlanner>(config_.planner);
    geometry_planner_ = std::make_unique<GraspGeometryPlanner>(
        config_.geometry, config_.planner);

    // 初始化执行器
    std::cout << "[Init] START module=executor" << std::endl;
    const auto executor_start = std::chrono::steady_clock::now();
    const auto executor_cpu_start = ProcessCpuMillis();
    if (config_.executor.manip_driver == "remote_mujoco") {
#ifdef HAVE_REMOTE_MUJOCO
        executor_ = std::make_unique<RemoteMujocoGraspExecutor>(
            config_.executor);
#else
        std::cerr << "[Pipeline] manipulator.driver=remote_mujoco requires "
                << "-DENABLE_REMOTE_MUJOCO=ON" << std::endl;
        log_init_failure("executor");
        return false;
#endif
    } else if (config_.executor.manip_driver == "mujoco" ||
        config_.executor.manip_driver == "mujoco_ur5e") {
#ifdef HAVE_MUJOCO_EXECUTOR
        executor_ = std::make_unique<MujocoGraspExecutor>(config_.executor);
#else
        std::cerr << "[Pipeline] manipulator.driver="
                << config_.executor.manip_driver
                << " requires -DENABLE_MUJOCO_EXECUTOR=ON" << std::endl;
        log_init_failure("executor");
        return false;
#endif
    } else {
#ifdef MOCK_EXECUTOR
        executor_ = std::make_unique<MockExecutor>(config_.executor);
#else
        executor_ = std::make_unique<GraspExecutor>(config_.executor);
#endif
    }
    if (!executor_->Init()) {
        std::cerr << "[Pipeline] Failed to init executor" << std::endl;
        log_init_failure("executor");
        return false;
    }
    log_init_time("executor", executor_start, executor_cpu_start);

    const auto mobile_base_start = std::chrono::steady_clock::now();
    const auto mobile_base_cpu_start = ProcessCpuMillis();
    std::cout << "[Init] START module=mobile_base" << std::endl;
    mobile_base_ = std::make_unique<MobileBaseController>(config_.mobile_base);
    if (!mobile_base_->Init()) {
        std::cerr << "[Pipeline] Failed to init mobile base" << std::endl;
        log_init_failure("mobile_base");
        return false;
    }
    log_init_time("mobile_base", mobile_base_start, mobile_base_cpu_start);

    initialization_elapsed_ms_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - pipeline_start)
            .count();
    std::cout << "[Init] SUMMARY result=SUCCESS"
            << " elapsed_ms=" << initialization_elapsed_ms_
            << " cpu_ms=" << ProcessCpuMillis() - camera_cpu_start
            << std::endl;
    SetState(PipelineState::IDLE, "Ready");
    return true;
}

bool GraspPipeline::TriggerGrasp() {
    if (state_.load() != PipelineState::IDLE || HasActiveAction()) {
        std::cerr << "[Pipeline] Cannot trigger: not idle" << std::endl;
        return false;
    }
    if (object_may_be_held_.load()) {
        std::cerr << "[Pipeline] Cannot trigger: gripper may still hold an "
                    "object; remove it before starting another task"
                << std::endl;
        return false;
    }
    target_label_.clear();
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    geometry_retry_count_ = 0;
    base_align_attempts_ = 0;
    target_stationary_confirmed_ = false;
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    top_alignment_reference_valid_ = false;
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    last_base_motion_odometry_confirmed_ = false;
    base_align_travel_m_ = 0.0f;
    base_align_direction_reversals_ = 0;
    task_id_.clear();
    target_track_ = TargetTrack{};
    grasp_strategy_ = GraspStrategy::TOP;
    observation_strategy_selected_ = false;
    grasp_opening_ = NAN;
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
    perception_cycle_active_ = false;
    auto_loop_target_label_.clear();
    auto_loop_iteration_ = config_.auto_loop ? 1 : 0;
    last_valid_geometry_ = ObjectGeometry3D{};
    last_valid_geometry_available_ = false;
    last_valid_strategy_ = GraspStrategy::TOP;
    last_valid_strategy_available_ = false;
    top_geometry_recovery_active_ = false;
    last_top_support_plane_valid_ = false;
    last_top_support_plane_ = SupportPlane{};
    BeginTaskTiming();
    if (config_.auto_loop) {
        std::cout << "\n[Loop] START iteration=1 target=auto" << std::endl;
    }
    SetState(PipelineState::DETECTING,
            "Detecting target before selecting observation pose");
    return true;
}

bool GraspPipeline::TriggerGrasp(const std::string& target_label) {
    if (state_.load() != PipelineState::IDLE || HasActiveAction()) {
        std::cerr << "[Pipeline] Cannot trigger: not idle" << std::endl;
        return false;
    }
    if (object_may_be_held_.load()) {
        std::cerr << "[Pipeline] Cannot trigger: gripper may still hold an "
                    "object; remove it before starting another task"
                << std::endl;
        return false;
    }
    target_label_ = target_label;
    if (executor_) {
        executor_->SetTargetLabel(target_label_);
    }
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    geometry_retry_count_ = 0;
    base_align_attempts_ = 0;
    target_stationary_confirmed_ = false;
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    top_alignment_reference_valid_ = false;
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    last_base_motion_odometry_confirmed_ = false;
    base_align_travel_m_ = 0.0f;
    base_align_direction_reversals_ = 0;
    task_id_.clear();
    target_track_ = TargetTrack{};
    grasp_strategy_ = GraspStrategy::TOP;
    observation_strategy_selected_ = false;
    grasp_opening_ = NAN;
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
    perception_cycle_active_ = false;
    auto_loop_target_label_ = target_label;
    auto_loop_iteration_ = config_.auto_loop ? 1 : 0;
    last_valid_geometry_ = ObjectGeometry3D{};
    last_valid_geometry_available_ = false;
    last_valid_strategy_ = GraspStrategy::TOP;
    last_valid_strategy_available_ = false;
    top_geometry_recovery_active_ = false;
    last_top_support_plane_valid_ = false;
    last_top_support_plane_ = SupportPlane{};
    BeginTaskTiming();
    if (config_.auto_loop) {
        std::cout << "\n[Loop] START iteration=1 target="
                    << target_label << std::endl;
    }
    SetState(PipelineState::DETECTING,
            "Detecting target before observation, target: " + target_label);
    return true;
}

bool GraspPipeline::TriggerVoiceCommand(const std::string& command_text) {
    std::string text = VoiceCommandParser::NormalizeText(command_text);
    if (text.empty()) {
        std::cerr << "[Voice] Empty command" << std::endl;
        return false;
    }

    VoiceCommandParser parser(config_.voice);
    if (parser.IsCancelCommand(command_text)) {
        if (state_.load() == PipelineState::IDLE && !config_.voice.enabled) {
            std::cerr << "[Voice] Cancel ignored: no active task" << std::endl;
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(voice_queue_mutex_);
            waiting_voice_target_ = false;
            while (!voice_queue_.empty()) {
                voice_queue_.pop();
            }
            voice_queue_.push({
                PendingVoiceCommand::Type::CANCEL,
                "",
                command_text,
            });
        }
        cancel_requested_.store(true);
        std::cout << "[Voice] Cancel command queued" << std::endl;
        return true;
    }

    if (parser.IsHomeCommand(command_text)) {
        if (state_.load() != PipelineState::IDLE || HasActiveAction()) {
            std::cerr << "[Voice] Busy, home command rejected: "
                        << command_text << std::endl;
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(voice_queue_mutex_);
            waiting_voice_target_ = false;
            if (!voice_queue_.empty()) {
                std::cerr << "[Voice] Pending command exists, home rejected: "
                            << command_text << std::endl;
                return false;
            }
            voice_queue_.push({
                PendingVoiceCommand::Type::HOME,
                "",
                command_text,
            });
        }
        std::cout << "[Voice] Home command queued" << std::endl;
        return true;
    }

    auto target = parser.ParseTarget(command_text);
    if (!target.has_value()) {
        bool can_complete_split = false;
        {
            std::lock_guard<std::mutex> lock(voice_queue_mutex_);
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - waiting_voice_target_since_)
                    .count();
            if (waiting_voice_target_ &&
                elapsed_ms <= config_.voice.split_command_timeout_ms) {
                can_complete_split = true;
            } else {
                waiting_voice_target_ = false;
            }
        }

        if (can_complete_split) {
            target = parser.ResolveKnownTargetText(command_text);
        }

        if (target.has_value() && !target->empty()) {
            std::cout << "[Voice] Split command target: " << *target
                        << std::endl;
        }
    }

    if (!target.has_value()) {
        std::cerr << "[Voice] No valid command or configured target matched: "
                    << command_text << std::endl;
        return false;
    }

    if (target->empty()) {
        if (!config_.voice.enabled) {
            std::cerr << "[Voice] Trigger word found but target is empty: "
                        << command_text << std::endl;
            return false;
        }
        if (state_.load() != PipelineState::IDLE) {
            std::cerr << "[Voice] Busy, incomplete command ignored: "
                        << command_text << std::endl;
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(voice_queue_mutex_);
            waiting_voice_target_ = true;
            waiting_voice_target_since_ = std::chrono::steady_clock::now();
        }
        std::cout << "[Voice] Trigger word found, waiting for target ("
                    << config_.voice.split_command_timeout_ms << "ms)"
                    << std::endl;
        return true;
    }

    if (state_.load() != PipelineState::IDLE || HasActiveAction()) {
        std::cerr << "[Voice] Busy, command rejected: "
                    << command_text << std::endl;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(voice_queue_mutex_);
        waiting_voice_target_ = false;
        if (!voice_queue_.empty()) {
            std::cerr << "[Voice] Pending command exists, command rejected: "
                        << command_text << std::endl;
            return false;
        }
        voice_queue_.push({
            PendingVoiceCommand::Type::GRASP,
            *target,
            command_text,
        });
    }

    std::cout << "[Voice] Command queued: target=" << *target << std::endl;
    return true;
}

void GraspPipeline::Stop() {
    cancel_requested_.store(true);
    {
        std::lock_guard<std::mutex> lock(voice_queue_mutex_);
        waiting_voice_target_ = false;
        while (!voice_queue_.empty()) {
            voice_queue_.pop();
        }
    }
    const PipelineState current_state = state_.load();
    if (current_state != PipelineState::IDLE &&
        !IsTerminalPipelineState(current_state)) {
        SetState(PipelineState::IDLE, "Stopped");
    }
    std::cout << std::flush;
}

void GraspPipeline::RequestGracefulShutdown() {
    if (graceful_shutdown_requested_.exchange(true)) return;

    {
        std::lock_guard<std::mutex> lock(voice_queue_mutex_);
        waiting_voice_target_ = false;
        while (!voice_queue_.empty()) {
            voice_queue_.pop();
        }
    }
    cancel_requested_.store(true);
    if (config_.plan_only) {
        std::cout << "[Pipeline] Graceful shutdown requested; exiting "
                    "plan-only mode without motion"
                    << std::endl;
    } else {
        std::cout << "[Pipeline] Graceful shutdown requested; stopping loop "
                    "after the current action and returning home"
                    << std::endl;
    }
}

void GraspPipeline::SetCallback(PipelineCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
}

void GraspPipeline::ResetTaskState() {
    target_label_.clear();
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    geometry_retry_count_ = 0;
    base_align_attempts_ = 0;
    target_stationary_confirmed_ = false;
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    top_alignment_reference_valid_ = false;
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    last_base_motion_odometry_confirmed_ = false;
    base_align_travel_m_ = 0.0f;
    base_align_direction_reversals_ = 0;
    {
        std::lock_guard<std::mutex> lock(voice_queue_mutex_);
        waiting_voice_target_ = false;
    }
    return_to_observe_pending_ = false;
    return_to_home_pending_ = false;
    place_possible_object_pending_ = false;
    grasp_yaw_rad_ = NAN;
    grasp_opening_ = NAN;
    current_target_ = DetectionTarget{};
    last_top_support_plane_valid_ = false;
    last_top_support_plane_ = SupportPlane{};
    target_track_ = TargetTrack{};
    grasp_pose_ = Pose3D{};
    pre_grasp_pose_ = Pose3D{};
    retreat_pose_ = Pose3D{};
    lift_pose_ = Pose3D{};
    grasp_strategy_ = GraspStrategy::TOP;
    observation_strategy_selected_ = false;
    grasp_geometry_result_ = GraspGeometryResult{};
    last_valid_geometry_ = ObjectGeometry3D{};
    last_valid_geometry_available_ = false;
    last_valid_strategy_ = GraspStrategy::TOP;
    last_valid_strategy_available_ = false;
    top_geometry_recovery_active_ = false;
    base_alignment_command_ = MobileBaseAlignmentCommand{};
    last_candidates_.clear();
    current_color_.release();
    current_depth_.release();
    task_id_.clear();
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
    failure_recovery_active_ = false;
    failure_recovery_succeeded_ = false;
    pending_failure_message_.clear();
}

void GraspPipeline::RestartAutoLoop(const char* previous_result) {
    const std::string target = auto_loop_target_label_;
    ResetTaskState();
    target_label_ = target;
    ++auto_loop_iteration_;
    BeginTaskTiming();

    std::cout << "\n[Loop] START iteration=" << auto_loop_iteration_
                << " previous_result=" << previous_result
                << " target=" << (target_label_.empty() ? "auto"
                                                        : target_label_)
                << std::endl;
    SetState(
        PipelineState::DETECTING,
        target_label_.empty()
            ? "Loop: detecting the next stable target"
            : "Loop: detecting stable target " + target_label_);
}

std::string GraspPipeline::FormatCandidates(size_t max_items) const {
    if (last_candidates_.empty()) return "none";

    std::ostringstream oss;
    const size_t count = std::min(max_items, last_candidates_.size());
    for (size_t i = 0; i < count; ++i) {
        const auto& t = last_candidates_[i];
        if (i > 0) oss << ", ";
        oss << (t.label_name.empty() ? std::to_string(t.label) : t.label_name)
            << "(" << std::fixed << std::setprecision(2) << t.score << ")";
    }
    if (last_candidates_.size() > count) {
        oss << ", ...";
    }
    return oss.str();
}

std::string GraspPipeline::ResultMessage(const std::string& phase,
                                        GraspResult result) const {
    ExecutorDiagnostics diag;
    if (executor_) diag = executor_->GetDiagnostics();

    std::ostringstream oss;
    oss << phase << ": ";
    switch (result) {
        case GraspResult::SUCCESS:
            oss << "success";
            break;
        case GraspResult::EMPTY:
            oss << "grasp empty";
            break;
        case GraspResult::IK_FAILED:
            oss << "IK failed";
            break;
        case GraspResult::OUT_OF_RANGE:
            oss << "motion outside configured joint or Cartesian limits";
            break;
        case GraspResult::TIMEOUT:
            oss << "timeout";
            break;
        case GraspResult::MOVE_FAILED:
        default:
            oss << "motion or device error";
            break;
    }

    if (!diag.last_action.empty()) {
        oss << " (action=" << diag.last_action;
        if (!diag.last_detail.empty()) {
            oss << ", detail=" << diag.last_detail;
        }
        oss << ")";
    }
    return oss.str();
}

bool GraspPipeline::RetryRecoverableMotion(
    const std::string& phase, GraspResult result) {
    if (result != GraspResult::OUT_OF_RANGE &&
        result != GraspResult::IK_FAILED) {
        return false;
    }

    retry_count_++;
    if (retry_count_ >= config_.max_retries) {
        return false;
    }

    stable_count_ = 0;
    missing_count_ = 0;
    geometry_retry_count_ = 0;
    current_color_.release();
    current_depth_.release();
    perception_cycle_active_ = false;

    std::ostringstream message;
    message << phase << " was not reachable; returning to the "
            << (grasp_strategy_ == GraspStrategy::SIDE ? "side" : "top")
            << " observation pose for re-detection and replanning (retry "
            << retry_count_ << "/" << config_.max_retries - 1 << ")";
    SetState(PipelineState::OBSERVING, message.str());
    return true;
}

bool GraspPipeline::FlushCameraAfterMotion(const char* reason) {
    target_stationary_confirmed_ = false;
    stable_count_ = 0;
    motion_geometry_confirmation_pending_ = true;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    camera_->ResetAfterMotion();

    const auto start = std::chrono::steady_clock::now();
    const std::int64_t previous_frame_id = camera_->LastFrameId();
    int discarded_frames = 0;
    if (config_.camera.type == "realsense") {
        const int maximum_discard_count =
            config_.camera.realsense.motion_flush_frames;
        if (maximum_discard_count <= 0) return true;
        discarded_frames =
            camera_->DiscardQueuedFrames(maximum_discard_count);
        if (discarded_frames < 0) {
            std::cerr << "[Pipeline] Failed to discard queued realsense "
                "frames after " << reason << std::endl;
            return false;
        }
    }

    cv::Mat color;
    cv::Mat depth;
    constexpr int kMaximumRefreshAttempts = 3;
    for (int attempt = 0; attempt < kMaximumRefreshAttempts; ++attempt) {
        if (!camera_->GetFrames(color, depth)) {
            std::cerr << "[Pipeline] Failed to refresh camera frame after "
                << reason << std::endl;
            return false;
        }
        const std::int64_t current_frame_id = camera_->LastFrameId();
        if (previous_frame_id < 0 || current_frame_id < 0 ||
            current_frame_id != previous_frame_id) {
            const auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start)
                    .count();
            std::ostringstream refresh_log;
            refresh_log << "[Timing] stage=CAMERA_REFRESH backend="
                << config_.camera.type << " reason=\"" << reason
                << "\" frame_id_before=" << previous_frame_id
                << " frame_id_after=" << current_frame_id
                << " discarded_frames=" << discarded_frames
                << " elapsed_ms=" << elapsed_ms << " result=SUCCESS";
            WriteStructuredLine(refresh_log.str());
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    std::cerr << "[Pipeline] Camera frame did not advance after "
        << reason << ": backend=" << config_.camera.type
        << " frame_id=" << previous_frame_id << std::endl;
    return false;
}

bool GraspPipeline::SaveStepCameraDebug(const char* phase) {
    if (!config_.step_mode || !config_.save_debug_data) return true;

    cv::Mat color;
    cv::Mat depth;
    if (!camera_->GetFrames(color, depth)) return false;
    return SavePipelineStepCameraDebug(
        config_.debug_output_dir, phase, color, depth, task_id_);
}

void GraspPipeline::SaveGraspDebug(float grasp_px, float grasp_py,
                                    uint16_t depth_mm,
                                    const float cam_point[3],
                                    const float base_point[3]) {
    if (!config_.save_debug_data) return;

    PipelinePlanDebugData data;
    data.color = current_color_;
    data.target_mask = current_target_.mask;
    data.target_detected = current_target_.label_name;
    data.target_score = current_target_.score;
    data.target_bbox = {
        current_target_.x1, current_target_.y1,
        current_target_.x2, current_target_.y2};
    data.target_center = current_target_.center;
    data.target_requested = target_label_;
    data.candidates = FormatCandidates();
    data.grasp_px = grasp_px;
    data.grasp_py = grasp_py;
    data.depth_mm = depth_mm;
    data.camera_point_m = {
        cam_point[0], cam_point[1], cam_point[2]};
    data.base_point_m = {
        base_point[0], base_point[1], base_point[2]};
    data.grasp_strategy = GraspStrategyName(grasp_strategy_);
    data.object_dimensions_m = {
        grasp_geometry_result_.geometry.length_m,
        grasp_geometry_result_.geometry.width_m,
        grasp_geometry_result_.geometry.height_m};
    data.geometry_elapsed_ms = grasp_geometry_result_.elapsed_ms;
    data.grasp_yaw_rad = grasp_yaw_rad_;
    data.pre_grasp_pose = pre_grasp_pose_;
    data.grasp_pose = grasp_pose_;
    data.retreat_pose = retreat_pose_;
    data.lift_pose = lift_pose_;
    SavePipelinePlanDebug(
        config_.debug_output_dir, data, task_id_,
        last_debug_image_path_, last_debug_json_path_);
}

void GraspPipeline::SaveTaskResultDebug(PipelineState terminal_state,
                                        const std::string& message) {
    if (!config_.save_debug_data) return;
    if (task_id_.empty() && current_color_.empty() && current_depth_.empty() &&
        target_label_.empty()) {
        return;
    }

    PipelineTaskResultDebugData data;
    data.terminal_state = PipelineStateName(terminal_state);
    data.message = message;
    data.target_requested = target_label_;
    data.target_detected = current_target_.label_name;
    data.candidates = FormatCandidates();
    if (executor_) data.executor = executor_->GetDiagnostics();
    SavePipelineTaskResultDebug(
        config_.debug_output_dir, data, task_id_,
        last_debug_image_path_, last_debug_json_path_);
}

bool GraspPipeline::ConsumeVoiceCommand() {
    PendingVoiceCommand command;
    {
        std::lock_guard<std::mutex> lock(voice_queue_mutex_);
        if (voice_queue_.empty()) return false;
        command = std::move(voice_queue_.front());
        voice_queue_.pop();
    }

    if (command.type == PendingVoiceCommand::Type::CANCEL) {
        ResetTaskState();
        std::cout << "[Voice] Cancel command consumed" << std::endl;
        return true;
    }

    if (command.type == PendingVoiceCommand::Type::HOME) {
        ResetTaskState();
        return_to_home_pending_ = true;
        std::cout << "[Voice] Home command consumed" << std::endl;
        return true;
    }

    std::cout << "[Voice] Command: target=" << command.target << std::endl;
    return TriggerGrasp(command.target);
}

bool GraspPipeline::WaitForConfirm(const std::string& prompt) {
    if (!config_.step_mode) return true;

    std::cout << "\n[Step] " << prompt << std::endl;
    std::cout << "[Step] 继续? (y=继续 / n=中止 / s=跳过后续确认): "
                << std::flush;

    std::string input;
    if (!std::getline(std::cin, input)) {
        std::cerr << "[Step] Input closed; aborting the task and returning home"
                << std::endl;
        RequestGracefulShutdown();
        return false;
    }
    if (input.empty()) {
        // 默认回车 = 继续
        return true;
    }

    char c = input[0];
    if (c == 'n' || c == 'N') {
        std::cout << "[Step] 用户中止" << std::endl;
        RequestGracefulShutdown();
        return false;
    }
    if (c == 's' || c == 'S') {
        std::cout << "[Step] 已关闭单步模式，后续不再暂停" << std::endl;
        config_.step_mode = false;
        return true;
    }
    return true;
}

void GraspPipeline::Run(
        const std::function<bool()>& external_shutdown_requested) {
    std::cout << "[Pipeline] Running main loop..." << std::endl;
    while (true) {
        if (external_shutdown_requested &&
            external_shutdown_requested()) {
            RequestGracefulShutdown();
        }
        SpinOnce(0.05f);

        // 非循环/非语音常驻模式下，完成或出错即退出。
        // 语音模式需要回到 IDLE 继续等待下一条 ASR 命令。
        auto s = state_.load();
        if (shutdown_requested_.load() && !action_.active) {
            break;
        }
        if (!config_.auto_loop && !config_.voice.enabled &&
            (s == PipelineState::DONE || s == PipelineState::ERROR)) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void GraspPipeline::SpinOnce(float dt_s) {
    // 更新执行器状态
    if (executor_ && !action_.active) executor_->Tick(dt_s);

    if (cancel_requested_.exchange(false)) {
        const bool graceful_shutdown =
            graceful_shutdown_requested_.load();
        const bool object_may_be_held = object_may_be_held_.load();
        SetState(
            PipelineState::IDLE,
            graceful_shutdown
                ? "Shutdown requested; waiting for current action"
                : (object_may_be_held
                    ? "Cancelling while gripper may hold an object; "
                        "placing it before returning home"
                    : "Cancelling; keeping observe pose"));
        std::cout << std::flush;
        if (action_.active) {
            action_.cancelling = true;
        } else {
            SaveTaskResultDebug(
                PipelineState::IDLE,
                graceful_shutdown ? "Graceful shutdown requested"
                                    : "Cancelled");
            ResetTaskState();
            if (config_.plan_only) {
                if (graceful_shutdown) {
                    shutdown_requested_.store(true);
                }
                SetState(
                    PipelineState::IDLE,
                    graceful_shutdown
                        ? "Plan-only shutdown complete; exiting without motion"
                        : "Plan-only task cancelled without motion");
            } else if (object_may_be_held) {
                place_possible_object_pending_ = true;
            } else if (graceful_shutdown) {
                return_to_home_pending_ = true;
            } else {
                return_to_observe_pending_ = true;
            }
        }
        return;
    }

    switch (state_.load()) {
        case PipelineState::IDLE:
            HandleIdle();
            break;
        case PipelineState::OBSERVING:
            HandleObserving();
            break;
        case PipelineState::DETECTING:
            HandleDetecting();
            break;
        case PipelineState::PLANNING:
            HandlePlanning();
            break;
        case PipelineState::BASE_ALIGNING:
            HandleBaseAligning();
            break;
        case PipelineState::APPROACHING:
            HandleApproaching();
            break;
        case PipelineState::GRASPING:
            HandleGrasping();
            break;
        case PipelineState::LIFTING:
            HandleLifting();
            break;
        case PipelineState::PLACING:
            HandlePlacing();
            break;
        case PipelineState::HOMING:
            HandleHoming();
            break;
        case PipelineState::RECOVERING:
            HandleRecovering();
            break;
        case PipelineState::DONE:
            if (config_.voice.enabled) {
                ResetTaskState();
                SetState(PipelineState::IDLE,
                        "Voice: waiting for next command");
            } else if (config_.auto_loop) {
                RestartAutoLoop("success");
            }
            break;
        case PipelineState::ERROR:
            if (!object_may_be_held_.load()) {
                if (config_.voice.enabled) {
                    ResetTaskState();
                    SetState(PipelineState::IDLE,
                            "Voice: waiting for next command");
                } else if (config_.auto_loop) {
                    RestartAutoLoop("failure");
                }
            }
            break;
    }
}

// --- State Handlers ---

void GraspPipeline::HandleIdle() {
    if (action_.active) {
        const bool was_cancelling = action_.cancelling;
        const std::string action_name = action_.name;
        auto result = PollAction(action_.owner, true);
        if (result.has_value()) {
            if (action_name == "return_to_observe_after_cancel") {
                SaveTaskResultDebug(PipelineState::IDLE,
                                    "Returned to observe after cancel: " +
                                        std::string(GraspResultName(*result)));
                ResetTaskState();
                if (graceful_shutdown_requested_.load()) {
                    return_to_home_pending_ = true;
                    SetState(PipelineState::IDLE,
                            "Shutdown requested; returning home");
                    return;
                }
                const char* next_message = config_.voice.enabled
                    ? "Voice: waiting for next command"
                    : "Cancelled";
                SetState(PipelineState::IDLE, next_message);
                return;
            }
            if (action_name == "return_to_home_on_command") {
                SaveTaskResultDebug(PipelineState::IDLE,
                                    "Returned home on command: " +
                                        std::string(GraspResultName(*result)));
                const bool home_succeeded =
                    *result == GraspResult::SUCCESS;
                ResetTaskState();
                shutdown_requested_.store(true);
                if (home_succeeded) {
                    SetState(PipelineState::IDLE,
                            "Home position reached; exiting");
                } else {
                    SetState(
                        PipelineState::ERROR,
                        "Home return failed during shutdown: " +
                            std::string(GraspResultName(*result)));
                }
                return;
            }
            if (action_name == "place_possible_object_after_cancel") {
                SaveTaskResultDebug(
                    PipelineState::IDLE,
                    "Safe cancellation recovery finished with " +
                        std::string(GraspResultName(*result)));
                const bool recovery_succeeded =
                    *result == GraspResult::SUCCESS;
                ResetTaskState();
                shutdown_requested_.store(true);
                if (recovery_succeeded) {
                    SetState(
                        PipelineState::IDLE,
                        "Possible object placed; home position reached");
                } else {
                    SetState(
                        PipelineState::ERROR,
                        "Safe cancellation recovery failed: " +
                            std::string(GraspResultName(*result)));
                }
                return;
            }
            SaveTaskResultDebug(PipelineState::IDLE,
                                "Cancelled; active action finished with " +
                                    std::string(GraspResultName(*result)));
            ResetTaskState();
            if (was_cancelling) {
                if (config_.plan_only) {
                    const bool graceful_shutdown =
                        graceful_shutdown_requested_.load();
                    if (graceful_shutdown) {
                        shutdown_requested_.store(true);
                    }
                    SetState(
                        PipelineState::IDLE,
                        graceful_shutdown
                            ? "Plan-only shutdown complete; exiting without "
                                "motion"
                            : "Plan-only task cancelled without motion");
                } else if (object_may_be_held_.load()) {
                    place_possible_object_pending_ = true;
                } else if (graceful_shutdown_requested_.load()) {
                    return_to_home_pending_ = true;
                } else {
                    return_to_observe_pending_ = true;
                }
            } else {
                SetState(PipelineState::IDLE, "Cancelled");
            }
        }
        return;
    }
    if (place_possible_object_pending_) {
        place_possible_object_pending_ = false;
        SetState(
            PipelineState::IDLE,
            "Possible object held; placing it before returning home");
        if (!StartAction(
                PipelineState::IDLE, "place_possible_object_after_cancel",
                [this]() {
                    return PlacePossibleObjectAndReturnHome();
                })) {
            shutdown_requested_.store(true);
            SetState(
                PipelineState::ERROR,
                "Failed to start safe cancellation recovery");
        }
        return;
    }
    if (return_to_observe_pending_) {
        return_to_observe_pending_ = false;
        if (config_.plan_only) {
            SetState(
                PipelineState::IDLE,
                "Plan-only task cancelled without motion");
            return;
        }
        SetState(PipelineState::IDLE, "Cancelled; returning to observe position");
        StartAction(PipelineState::IDLE, "return_to_observe_after_cancel",
                    [this]() {
                        if (observation_strategy_selected_ &&
                            grasp_strategy_ == GraspStrategy::SIDE) {
                            return executor_->MoveToSideObserve();
                        }
                        return executor_->MoveToObserve();
        });
        return;
    }
    if (return_to_home_pending_) {
        return_to_home_pending_ = false;
        if (config_.plan_only) {
            shutdown_requested_.store(true);
            SetState(
                PipelineState::IDLE,
                "Plan-only shutdown complete; exiting without motion");
            return;
        }
        SetState(PipelineState::IDLE, "Returning home; exiting after home");
        StartAction(PipelineState::IDLE, "return_to_home_on_command",
                    [this]() {
                        return executor_->MoveToHome();
                    });
        return;
    }
    ConsumeVoiceCommand();
}

GraspResult GraspPipeline::PlacePossibleObjectAndReturnHome() {
    GraspResult result = executor_->MoveToPlace();
    if (result != GraspResult::SUCCESS) return result;

    result = executor_->ReleaseObject();
    if (result != GraspResult::SUCCESS) return result;
    object_may_be_held_.store(false);

    result = executor_->CloseGripper();
    if (result != GraspResult::SUCCESS) return result;
    return executor_->MoveToHome();
}

void GraspPipeline::HandleObserving() {
    if (config_.plan_only) {
        SetState(PipelineState::DETECTING,
                "Plan-only: using current arm pose without motion");
        return;
    }
    if (!observation_strategy_selected_) {
        SetState(PipelineState::ERROR,
                "Observation strategy was not selected before arm motion");
        return;
    }
    const bool use_side_observation =
        grasp_strategy_ == GraspStrategy::SIDE;
    if (!action_.active) {
        const char* prompt = use_side_observation
            ? "即将移动到侧抓观察位 (side_ready_joints)"
            : "即将移动到顶抓观察位 (observe_joints)";
        if (!WaitForConfirm(prompt)) return;
        const char* action_name = use_side_observation
            ? "move_to_side_observe"
            : "move_to_top_observe";
        StartAction(
            PipelineState::OBSERVING, action_name,
            [this, use_side_observation]() {
            auto result = use_side_observation
                ? executor_->MoveToSideObserve()
                : executor_->MoveToObserve();
            if (result != GraspResult::SUCCESS) return result;
            std::this_thread::sleep_for(std::chrono::milliseconds(
                config_.executor.timing.observe_settle_ms));
            return GraspResult::SUCCESS;
        });
        return;
    }

    auto result = PollAction(PipelineState::OBSERVING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        const char* motion_name = use_side_observation
            ? "side observation motion"
            : "top observation motion";
        if (!FlushCameraAfterMotion(motion_name)) {
            SetState(PipelineState::ERROR,
                    "Failed to refresh camera after observe motion");
            return;
        }
        stable_count_ = 0;
        SetState(
            PipelineState::DETECTING,
            use_side_observation
                ? "Side observation pose reached; detecting target again"
                : "Top observation pose reached; detecting target again");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Observe move failed", *result));
    }
}

void GraspPipeline::HandleDetecting() {
    const auto detection_stage_start = std::chrono::steady_clock::now();
    perception_cycle_started_at_ = detection_stage_start;
    perception_cycle_active_ = true;
    const auto detection_stage_cpu_start = ProcessCpuMillis();
    cv::Mat color, depth;
    if (!camera_->GetFrames(color, depth)) {
        std::cerr << "[Pipeline] Failed to get camera frames" << std::endl;
        return;
    }
    const auto capture_end = std::chrono::steady_clock::now();
    const auto capture_cpu_end = ProcessCpuMillis();

    const auto detector_start = capture_end;

    DetectionTarget target{};
    std::vector<DetectionTarget> targets;

    detector_->Detect(color, targets);
    const TargetAssociationResult association = SelectTargetInstance(
        targets, target_label_, target_track_);
    const bool found = association.index >= 0;
    if (found) {
        target = targets[static_cast<size_t>(association.index)];
        if (association.matched_existing_track) {
            std::cout << "[Pipeline] Target association: label="
                        << target.label_name << " cost="
                        << association.cost << std::endl;
        }
    }
    last_candidates_ = targets;

    const auto detector_end = std::chrono::steady_clock::now();
    const auto detector_cpu_end = ProcessCpuMillis();
    if (config_.performance_log_enabled) {
        const auto detector_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                detector_end - detector_start)
                .count();
        const auto capture_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capture_end - detection_stage_start)
                .count();
        const auto stage_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                detector_end - detection_stage_start)
                .count();
        std::cout << "[Timing] stage=DETECTING"
                << " elapsed_ms=" << stage_ms
                << " cpu_ms="
                << detector_cpu_end - detection_stage_cpu_start
                << " camera_ms=" << capture_ms
                << " camera_cpu_ms="
                << capture_cpu_end - detection_stage_cpu_start
                << " detector_ms=" << detector_ms
                << " detector_cpu_ms="
                << detector_cpu_end - capture_cpu_end
                << " result=" << (found ? "FOUND" : "NOT_FOUND")
                << std::endl;
    }

    if (!found) {
        target_stationary_confirmed_ = false;
        stable_count_ = 0;
        if (target_track_.valid && !association.reason.empty()) {
            std::cout << "[Pipeline] Target association: "
                        << association.reason << std::endl;
        }
        if (!target_label_.empty()) {
            const bool first_missing_report =
                missing_count_ < config_.target_missing_frames;
            missing_count_ = std::min(
                missing_count_ + 1, config_.target_missing_frames);
            std::cout << "[Pipeline] Target not detected: " << target_label_
                        << " (" << missing_count_ << "/"
                        << config_.target_missing_frames << ")"
                        << ", candidates: " << FormatCandidates()
                        << std::endl;
            if (missing_count_ >= config_.target_missing_frames) {
                if (config_.auto_loop) {
                    if (first_missing_report) {
                        std::cout
                            << "[Loop] Target " << target_label_
                            << " is temporarily unavailable; waiting at the "
                            << (observation_strategy_selected_
                                ? "current observation pose"
                                : "current safe pose")
                            << std::endl;
                    }
                } else {
                    SetState(PipelineState::ERROR,
                            "Target not found: " + target_label_ +
                            "; candidates: " + FormatCandidates());
                }
            }
        } else {
            std::cout << "[Pipeline] No target detected, retrying..." << std::endl;
        }
        return;
    }

    missing_count_ = 0;
    const TargetTrack current_track = UpdateTargetTrack(target);
    std::string stationarity_detail;
    const bool stationary = AreTargetTracksStationary(
        target_track_, current_track, &stationarity_detail);
    target_track_ = current_track;

    if (!target_stationary_confirmed_) {
        stable_count_ = stationary ? stable_count_ + 1 : 1;
        if (stable_count_ < config_.detect_stable_frames) {
            return;
        }
        target_stationary_confirmed_ = true;
        std::cout << "[Pipeline] Target stationary: frames="
                    << stable_count_ << " " << stationarity_detail
                    << std::endl;
    } else if (!stationary && config_.detect_stable_frames > 1) {
        target_stationary_confirmed_ = false;
        stable_count_ = 1;
        std::cout << "[Pipeline] Target moved; waiting for it to settle: "
                    << stationarity_detail << std::endl;
        return;
    }

    current_target_ = target;
    std::cout << "[Pipeline] Target detected: " << target.label_name
                << " (score=" << target.score << ", center=["
                << target.center.x << "," << target.center.y << "])"
                << std::endl;

    current_color_ = color.clone();
    current_depth_ = depth.clone();
    SetState(PipelineState::PLANNING, "Target stable, planning grasp...");
}

bool GraspPipeline::BuildMaskTopGrasp(
    GraspCandidate& candidate,
    float& grasp_px,
    float& grasp_py,
    uint16_t& depth_mm,
    float cam_point[3],
    float base_point[3],
    std::string& error,
    float fallback_depth_mm,
    const SupportPlane* fallback_support_plane,
    bool allow_global_mask_depth) {
    float offset_dir_angle = NAN;
    if (!ComputeGraspPixel(
            current_target_, grasp_px, grasp_py,
            config_.top_grasp_point_x_ratio,
            config_.orientation, &offset_dir_angle)) {
        grasp_px = current_target_.center.x;
        grasp_py = current_target_.center.y;
    }

    int cx = ClampPixel(
        static_cast<int>(std::lround(grasp_px)), current_depth_.cols);
    int cy = ClampPixel(
        static_cast<int>(std::lround(grasp_py)), current_depth_.rows);
    const int intended_grasp_x = cx;
    const int intended_grasp_y = cy;
    constexpr int kDepthSearchRadius = 8;
    GraspDepthSample depth_sample;
    if (SampleMaskedDepthNearPixel(
            current_depth_, current_target_.mask, cx, cy,
            kDepthSearchRadius, &depth_sample)) {
        cx = depth_sample.x;
        cy = depth_sample.y;
        depth_mm = depth_sample.depth_mm;
        std::cout << "[Pipeline] Top-grasp depth "
                    "source=grasp_pixel_mask value="
                    << depth_mm << "mm samples="
                    << depth_sample.sample_count
                    << std::endl;
    } else {
        cx = ClampPixel(
            static_cast<int>(std::lround(current_target_.center.x)),
            current_depth_.cols);
        cy = ClampPixel(
            static_cast<int>(std::lround(current_target_.center.y)),
            current_depth_.rows);
        if (!SampleMaskedDepthNearPixel(
                current_depth_, current_target_.mask, cx, cy,
                kDepthSearchRadius, &depth_sample)) {
            uint16_t mask_depth_mm = 0;
            size_t mask_depth_sample_count = 0;
            float support_depth_mm = NAN;
            const bool has_mask_depth =
                allow_global_mask_depth &&
                ForegroundDepthFromMask(
                    current_depth_, current_target_.mask,
                    mask_depth_mm, mask_depth_sample_count);
            const bool has_geometry_depth =
                std::isfinite(fallback_depth_mm) &&
                fallback_depth_mm > 0.0f;
            const bool has_support_depth =
                fallback_support_plane != nullptr &&
                EstimateSupportPlaneDepth(
                    *fallback_support_plane,
                    intended_grasp_x,
                    intended_grasp_y,
                    support_depth_mm);
            if (!has_mask_depth && !has_geometry_depth &&
                !has_support_depth) {
                error =
                    "top-grasp mask depth is invalid near grasp pixel and "
                    "center";
                return false;
            }
            cx = intended_grasp_x;
            cy = intended_grasp_y;
            const float recovered_depth_mm = has_mask_depth
                ? static_cast<float>(mask_depth_mm)
                : (has_geometry_depth
                    ? fallback_depth_mm
                    : support_depth_mm);
            depth_mm = static_cast<uint16_t>(std::clamp(
                std::lround(recovered_depth_mm),
                1L, static_cast<long>(UINT16_MAX)));
            if (has_mask_depth) {
                std::cout
                    << "[Pipeline] Top-grasp depth "
                    << "source=target_mask_foreground value="
                    << depth_mm << "mm samples="
                    << mask_depth_sample_count
                    << " at intended grasp pixel"
                    << std::endl;
            } else if (has_geometry_depth) {
                std::cout
                    << "[Pipeline] Top-grasp depth "
                    << "source=geometry_foreground_cluster value="
                    << depth_mm
                    << "mm at intended grasp pixel"
                    << std::endl;
            } else {
                std::cout
                    << "[Pipeline] Top-grasp depth "
                    << "source=support_plane_intersection value="
                    << depth_mm
                    << "mm at intended grasp pixel"
                    << std::endl;
            }
        } else {
            cx = depth_sample.x;
            cy = depth_sample.y;
            depth_mm = depth_sample.depth_mm;
            std::cout << "[Pipeline] Top-grasp depth "
                        "source=target_center_mask value="
                        << depth_mm << "mm samples="
                        << depth_sample.sample_count << std::endl;
        }
    }

    // At the top observation pose the arm can overlap a valid target mask.
    // A mask-local sample then reports the much nearer gripper rather than the
    // low-profile object. Top grasps are selected below the side-grasp height
    // threshold, so a sample more than 100 mm in front of the measured support
    // plane cannot belong to the intended object. Recover at the same pixel
    // with the support-plane intersection; PlanTopGrasp applies the configured
    // grasp depth from that surface.
    float support_depth_mm = NAN;
    constexpr float kMaximumTopForegroundLeadMm = 100.0f;
    if (config_.top_support_plane_occlusion_recovery &&
        fallback_support_plane != nullptr &&
        EstimateSupportPlaneDepth(
            *fallback_support_plane, cx, cy, support_depth_mm) &&
        support_depth_mm - static_cast<float>(depth_mm) >
            kMaximumTopForegroundLeadMm) {
        const uint16_t occluded_depth_mm = depth_mm;
        depth_mm = static_cast<uint16_t>(std::clamp(
            std::lround(support_depth_mm),
            1L, static_cast<long>(UINT16_MAX)));
        std::cout
            << "[Pipeline] Top-grasp depth "
            << "source=support_plane_occlusion_recovery value="
            << depth_mm << "mm rejected_mask_depth="
            << occluded_depth_mm << "mm lead="
            << support_depth_mm - static_cast<float>(occluded_depth_mm)
            << "mm" << std::endl;
    }

    if (!camera_->Deproject(cx, cy, depth_mm, cam_point)) {
        error = "top-grasp camera deprojection failed";
        return false;
    }
    planner_->CameraToBase(cam_point, base_point);

    // The mask-local depth is the top surface of a low-profile object. A
    // fixed 15 mm descent from a 30-40 mm-tall banana leaves the fingers well
    // above the support surface, while historical boundary pixels only worked
    // because they accidentally sampled the table. Anchor the final TCP
    // height to the already validated support plane instead. The calibrated
    // support plane can sit several millimetres above the manipulator's known
    // workspace floor. Keep the requested TCP slightly above that measured
    // plane: the finger envelope still spans the low-profile object, while
    // the SO101 can preserve the safety-critical top-down wrist constraint.
    // Requesting the TCP at or below the plane drives joint 4 into its top-
    // grasp limit and leaves an otherwise safe pose outside the vertical IK
    // residual.
    // PlanTopGrasp() subtracts grasp_depth from base_point.z, so include it in
    // the synthetic surface point passed to the planner.
    constexpr float kTopGraspSupportClearanceM = 0.004f;
    if (config_.top_support_plane_height_anchor &&
        fallback_support_plane != nullptr &&
        fallback_support_plane->valid &&
        std::fabs(fallback_support_plane->normal_z) > 0.5f) {
        const float measured_base_z = base_point[2];
        const float support_z = -(
            fallback_support_plane->normal_x * base_point[0] +
            fallback_support_plane->normal_y * base_point[1] +
            fallback_support_plane->d) /
            fallback_support_plane->normal_z;
        constexpr float kTopGraspFloorClearanceM = 0.001f;
        const float planned_tcp_z = std::max(
            config_.planner.workspace.z_min +
                kTopGraspFloorClearanceM,
            support_z + kTopGraspSupportClearanceM);
        const float support_anchored_base_z =
            planned_tcp_z + config_.planner.grasp_depth;
        if (std::isfinite(support_anchored_base_z)) {
            base_point[2] = support_anchored_base_z;
            std::cout
                << "[Pipeline] Top-grasp height "
                << "source=support_plane measured_surface_z="
                << measured_base_z << "m support_z=" << support_z
                << "m planned_tcp_z=" << planned_tcp_z << "m"
                << std::endl;
        }
    }
    grasp_px = static_cast<float>(cx);
    grasp_py = static_cast<float>(cy);
    std::cout << "[Pipeline] Top-grasp pixel (offset_ratio="
                << config_.top_grasp_point_x_ratio << "): ["
                << cx << ", " << cy << "]" << std::endl;

    Pose3D grasp_pose;
    Pose3D pre_grasp_pose;
    if (!planner_->PlanTopGrasp(
            base_point, grasp_pose, pre_grasp_pose, false)) {
        std::ostringstream message;
        message << "top-grasp pose generation failed for point ["
                << base_point[0] << "," << base_point[1] << ","
                << base_point[2] << "]";
        error = message.str();
        return false;
    }

    float grasp_yaw = NAN;
    if (config_.auto_orient && std::isfinite(offset_dir_angle)) {
        grasp_yaw = ImageLineAngleFromHorizontal(offset_dir_angle);
        std::cout << "[Pipeline] Top-grasp yaw source=2d_mask value="
                    << grasp_yaw << std::endl;
    } else if (config_.auto_orient) {
        grasp_yaw = ComputeGraspYaw(
            current_target_, config_.orientation);
    }

    const float top_offset = config_.planner.gripper_offset;
    if (top_offset != 0.0f) {
        const float yaw = std::isnan(grasp_yaw) ? 0.0f : grasp_yaw;
        const float dx = -top_offset * std::sin(yaw);
        const float dy = top_offset * std::cos(yaw);
        grasp_pose.x += dx;
        grasp_pose.y += dy;
        pre_grasp_pose.x += dx;
        pre_grasp_pose.y += dy;
        std::cout << "[Pipeline] Top-grasp jaw offset="
                    << top_offset << "m dx=" << dx << " dy=" << dy
                    << std::endl;
    }

    candidate.grasp_pose = grasp_pose;
    candidate.pre_grasp_pose = pre_grasp_pose;
    candidate.retreat_pose = pre_grasp_pose;
    if (config_.top_verification_lift_m > 0.0f) {
        candidate.retreat_pose = grasp_pose;
        candidate.retreat_pose.z = std::min(
            pre_grasp_pose.z,
            grasp_pose.z + config_.top_verification_lift_m);
    }
    candidate.lift_pose = pre_grasp_pose;
    candidate.grasp_yaw_rad = grasp_yaw;
    candidate.geometry_valid = true;
    candidate.rejection_reason.clear();
    float support_clearance_adjustment = 0.0f;
    if (fallback_support_plane != nullptr) {
        support_clearance_adjustment = EnforceTopSupportClearance(
            candidate, *fallback_support_plane,
            config_.top_minimum_grasp_height_m);
    }
    base_point[0] = candidate.grasp_pose.x;
    base_point[1] = candidate.grasp_pose.y;
    base_point[2] = candidate.grasp_pose.z;
    std::cout << "[Pipeline] Top-grasp plan: target=["
                << candidate.grasp_pose.x << ","
                << candidate.grasp_pose.y << ","
                << candidate.grasp_pose.z << "] yaw=" << grasp_yaw
                << " support_clearance_adjustment="
                << support_clearance_adjustment << "m" << std::endl;
    error.clear();
    return true;
}

bool GraspPipeline::EstimateSupportPlaneDepth(
    const SupportPlane& support_plane,
    int pixel_x,
    int pixel_y,
    float& depth_mm) const {
    if (!support_plane.valid || camera_ == nullptr || planner_ == nullptr) {
        return false;
    }

    float camera_origin[3] = {};
    float camera_point[3] = {};
    if (!camera_->Deproject(pixel_x, pixel_y, 1000, camera_point)) {
        return false;
    }

    float base_origin[3] = {};
    float base_point[3] = {};
    planner_->CameraToBase(camera_origin, base_origin);
    planner_->CameraToBase(camera_point, base_point);

    const float direction[3] = {
        base_point[0] - base_origin[0],
        base_point[1] - base_origin[1],
        base_point[2] - base_origin[2],
    };
    const float denominator =
        support_plane.normal_x * direction[0] +
        support_plane.normal_y * direction[1] +
        support_plane.normal_z * direction[2];
    if (!std::isfinite(denominator) ||
        std::fabs(denominator) < 1e-6f) {
        return false;
    }

    const float origin_distance =
        support_plane.normal_x * base_origin[0] +
        support_plane.normal_y * base_origin[1] +
        support_plane.normal_z * base_origin[2] +
        support_plane.d;
    const float scale = -origin_distance / denominator;
    const float estimated_depth_mm = scale * 1000.0f;
    constexpr float kMinimumFallbackDepthMm = 50.0f;
    constexpr float kMaximumFallbackDepthMm = 5000.0f;
    if (!std::isfinite(estimated_depth_mm) ||
        estimated_depth_mm < kMinimumFallbackDepthMm ||
        estimated_depth_mm > kMaximumFallbackDepthMm) {
        return false;
    }

    depth_mm = estimated_depth_mm;
    return true;
}

bool GraspPipeline::ProjectTopCandidateToMaskCenter(
    const ObjectGeometry3D& geometry,
    GraspCandidate& candidate,
    float& grasp_px,
    float& grasp_py,
    uint16_t& depth_mm,
    float cam_point[3],
    float base_point[3],
    std::string& error) {
    SupportPlane support_plane;
    std::string support_source;
    if (!ResolveTopSupportPlane(
            geometry.table, support_plane, support_source)) {
        error = "projected center requires a support plane";
        return false;
    }

    const float center_height = 0.5f * geometry.height_m;
    SupportPlane center_plane = support_plane;
    center_plane.d -= center_height;
    grasp_px = current_target_.center.x;
    grasp_py = current_target_.center.y;
    const int cx = ClampPixel(
        static_cast<int>(std::lround(grasp_px)), current_depth_.cols);
    const int cy = ClampPixel(
        static_cast<int>(std::lround(grasp_py)), current_depth_.rows);
    float center_depth_mm = NAN;
    if (!EstimateSupportPlaneDepth(
            center_plane, cx, cy, center_depth_mm)) {
        error = "projected center ray is invalid";
        return false;
    }
    depth_mm = static_cast<uint16_t>(std::clamp(
        std::lround(center_depth_mm),
        1L, static_cast<long>(UINT16_MAX)));
    if (!camera_->Deproject(cx, cy, depth_mm, cam_point)) {
        error = "projected center deprojection failed";
        return false;
    }

    float projected_center[3] = {};
    planner_->CameraToBase(cam_point, projected_center);
    const float footprint_aspect_ratio = geometry.width_m > 1e-6f
        ? geometry.length_m / geometry.width_m
        : 1.0f;
    constexpr float kProjectionBlendStartAspectRatio = 2.5f;
    constexpr float kProjectionBlendEndAspectRatio = 4.5f;
    const float geometry_blend_scale = std::clamp(
        (kProjectionBlendEndAspectRatio - footprint_aspect_ratio) /
            (kProjectionBlendEndAspectRatio -
                kProjectionBlendStartAspectRatio),
        0.0f, 1.0f);
    const float blend =
        config_.top_projected_center_blend * geometry_blend_scale;
    const float dx = blend *
        (projected_center[0] - candidate.grasp_pose.x);
    const float dy = blend *
        (projected_center[1] - candidate.grasp_pose.y);
    candidate.grasp_pose.x += dx;
    candidate.grasp_pose.y += dy;
    candidate.pre_grasp_pose.x += dx;
    candidate.pre_grasp_pose.y += dy;
    candidate.retreat_pose.x += dx;
    candidate.retreat_pose.y += dy;
    candidate.lift_pose.x += dx;
    candidate.lift_pose.y += dy;
    const float support_clearance_adjustment = EnforceTopSupportClearance(
        candidate, support_plane, config_.top_minimum_grasp_height_m);
    base_point[0] = candidate.grasp_pose.x;
    base_point[1] = candidate.grasp_pose.y;
    base_point[2] = candidate.grasp_pose.z;
    std::cout << "[Pipeline] Top-grasp position "
            << "source=projected_geometry_center support="
            << support_source << " center_height="
            << center_height << "m support_clearance_adjustment="
            << support_clearance_adjustment
            << "m projected_center_blend="
            << config_.top_projected_center_blend
            << " effective_blend=" << blend
            << " footprint_aspect_ratio=" << footprint_aspect_ratio
            << " center_shift=[" << dx << "," << dy
            << "]m" << std::endl;
    error.clear();
    return true;
}

bool GraspPipeline::ResolveTopSupportPlane(
    const TablePlane& table,
    SupportPlane& support_plane,
    std::string& source) {
    if (last_top_support_plane_valid_) {
        support_plane = last_top_support_plane_;
        source = "cached_pre_motion_depth";
        std::cout << "[Pipeline] Reusing pre-motion support surface: normal=["
                    << support_plane.normal_x << ","
                    << support_plane.normal_y << ","
                    << support_plane.normal_z << "] d="
                    << support_plane.d << std::endl;
        return true;
    }
    if (BuildSupportPlane(table, support_plane)) {
        last_top_support_plane_ = support_plane;
        last_top_support_plane_valid_ = true;
        source = "current_depth";
        return true;
    }
    if (BuildWorkspaceSupportPlane(
            config_.planner.workspace, support_plane)) {
        source = "workspace_floor";
        return true;
    }
    source.clear();
    return false;
}

bool GraspPipeline::BuildLowProfileTopFallback(
    const GraspGeometryResult& failed_geometry,
    GraspGeometryResult& recovered_geometry) {
    SupportPlane support_plane;
    std::string support_plane_source;
    if (!ResolveTopSupportPlane(
            failed_geometry.geometry.table,
            support_plane,
            support_plane_source)) {
        return false;
    }

    GraspCandidate candidate;
    float grasp_px = current_target_.center.x;
    float grasp_py = current_target_.center.y;
    uint16_t depth_mm = 0;
    float camera_point[3] = {};
    float base_point[3] = {};
    std::string recovery_error;
    const bool confirmed_top_after_base_motion =
        base_align_attempts_ > 0 &&
        last_valid_geometry_available_ &&
        last_valid_strategy_available_ &&
        last_valid_strategy_ == GraspStrategy::TOP;
    const SupportPlane* fallback_depth_plane =
        confirmed_top_after_base_motion ? &support_plane : nullptr;
    if (!BuildMaskTopGrasp(
            candidate, grasp_px, grasp_py, depth_mm,
            camera_point, base_point, recovery_error,
            failed_geometry.foreground_depth_mm,
            fallback_depth_plane,
            confirmed_top_after_base_motion)) {
        return false;
    }

    TablePlane table = failed_geometry.geometry.table;
    if (table.inlier_count <= 0 || !table.bounds_valid) {
        table.normal = cv::Point3f(
            support_plane.normal_x,
            support_plane.normal_y,
            support_plane.normal_z);
        table.d = support_plane.d;
        table.inlier_count = 1;
        table.min_x = support_plane.min_x;
        table.max_x = support_plane.max_x;
        table.min_y = support_plane.min_y;
        table.max_y = support_plane.max_y;
        table.bounds_valid = support_plane.bounds_valid;
    }
    const cv::Point3f measured(
        base_point[0], base_point[1], base_point[2]);
    const float signed_table_distance =
        table.normal.x * measured.x +
        table.normal.y * measured.y +
        table.normal.z * measured.z +
        table.d;
    const float minimum_height_m = std::max(
        0.0f, config_.geometry.object_min_height_m);
    constexpr float kMaximumBelowSupportPlaneM = 0.010f;
    if (!std::isfinite(signed_table_distance) ||
        signed_table_distance < -kMaximumBelowSupportPlaneM ||
        signed_table_distance > config_.geometry.side_min_height_m) {
        return false;
    }

    ObjectGeometry3D geometry = confirmed_top_after_base_motion
        ? last_valid_geometry_
        : ObjectGeometry3D{};
    geometry.valid = true;
    geometry.table_center = cv::Point3f(
        measured.x - table.normal.x * signed_table_distance,
        measured.y - table.normal.y * signed_table_distance,
        measured.z - table.normal.z * signed_table_distance);
    if (confirmed_top_after_base_motion) {
        geometry.center = cv::Point3f(
            geometry.table_center.x +
                table.normal.x * geometry.height_m * 0.5f,
            geometry.table_center.y +
                table.normal.y * geometry.height_m * 0.5f,
            geometry.table_center.z +
                table.normal.z * geometry.height_m * 0.5f);
    } else {
        geometry.height_m = std::max(
            minimum_height_m, signed_table_distance);
        geometry.center = cv::Point3f(
            geometry.table_center.x +
                table.normal.x * geometry.height_m * 0.5f,
            geometry.table_center.y +
                table.normal.y * geometry.height_m * 0.5f,
            geometry.table_center.z +
                table.normal.z * geometry.height_m * 0.5f);
    }
    geometry.table = table;

    recovered_geometry = GraspGeometryResult{};
    recovered_geometry.geometry = geometry;
    recovered_geometry.candidates.push_back(candidate);
    recovered_geometry.foreground_depth_mm =
        static_cast<float>(depth_mm);
    recovered_geometry.center_depth_mm =
        failed_geometry.center_depth_mm;
    recovered_geometry.elapsed_ms = failed_geometry.elapsed_ms;
    recovered_geometry.error =
        "low-profile target recovered with mask-guided top grasp; "
        "support_plane=" + support_plane_source;
    return true;
}

bool GraspPipeline::ConfirmTopAlignmentPoint(
    const float alignment_point[3]) {
    if (!config_.mobile_base.enabled) {
        top_alignment_reference_valid_ = false;
        return true;
    }
    if (!top_alignment_reference_valid_) {
        std::copy_n(
            alignment_point, 3, top_alignment_reference_.begin());
        top_alignment_reference_valid_ = true;
        stable_count_ = 0;
        perception_cycle_active_ = false;
        SetState(
            PipelineState::DETECTING,
            "Confirming top-grasp base alignment distance");
        return false;
    }

    const float planar_delta = std::hypot(
        alignment_point[0] - top_alignment_reference_[0],
        alignment_point[1] - top_alignment_reference_[1]);
    if (!std::isfinite(planar_delta) ||
        planar_delta > kTopAlignmentMaximumFrameDeltaM) {
        std::copy_n(
            alignment_point, 3, top_alignment_reference_.begin());
        stable_count_ = 0;
        perception_cycle_active_ = false;
        std::ostringstream message;
        message << "Top-grasp base alignment depth changed by "
                << planar_delta
                << "m; confirming another frame before chassis motion";
        SetState(PipelineState::DETECTING, message.str());
        return false;
    }

    top_alignment_reference_valid_ = false;
    std::cout << "[Pipeline] Top-grasp base alignment confirmed: "
                << "frame_delta=" << planar_delta << "m" << std::endl;
    return true;
}

void GraspPipeline::HandleTopPlanning() {
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_cpu_start = ProcessCpuMillis();

    GraspGeometryResult safety_geometry;
    geometry_planner_->Plan(
        current_depth_, current_target_, *camera_, *planner_,
        safety_geometry);
    SupportPlane support_plane;
    std::string support_plane_source;
    if (!ResolveTopSupportPlane(
            safety_geometry.geometry.table, support_plane,
            support_plane_source)) {
        SetState(
            PipelineState::ERROR,
            "Top-grasp safety validation failed: support surface "
            "configuration is invalid");
        return;
    }
    std::cout << "[Pipeline] Top-grasp support surface source="
                << support_plane_source << " normal=["
                << support_plane.normal_x << ","
                << support_plane.normal_y << ","
                << support_plane.normal_z << "] d="
                << support_plane.d << " bounds=["
                << support_plane.min_x << ","
                << support_plane.max_x << ","
                << support_plane.min_y << ","
                << support_plane.max_y << "]" << std::endl;
    executor_->SetSupportPlane(support_plane);

    GraspCandidate candidate;
    float grasp_px = current_target_.center.x;
    float grasp_py = current_target_.center.y;
    uint16_t depth_mm = 0;
    float cam_point[3] = {};
    float base_point[3] = {};
    std::string error;
    bool top_grasp_built = false;
    if (config_.top_position_source == "projected_geometry_center") {
        const GraspCandidate* geometry_candidate = nullptr;
        for (const GraspCandidate& current : safety_geometry.candidates) {
            if (current.strategy != GraspStrategy::TOP ||
                !current.geometry_valid) {
                continue;
            }
            if (geometry_candidate == nullptr ||
                current.score > geometry_candidate->score) {
                geometry_candidate = &current;
            }
        }
        if (geometry_candidate == nullptr) {
            if (safety_geometry.candidates.empty()) {
                top_grasp_built = BuildMaskTopGrasp(
                    candidate, grasp_px, grasp_py, depth_mm,
                    cam_point, base_point, error,
                    safety_geometry.foreground_depth_mm,
                    &support_plane);
                if (top_grasp_built) {
                    const int projected_x = ClampPixel(
                        static_cast<int>(std::lround(grasp_px)),
                        current_depth_.cols);
                    const int projected_y = ClampPixel(
                        static_cast<int>(std::lround(grasp_py)),
                        current_depth_.rows);
                    float support_depth_mm = NAN;
                    float support_camera_point[3] = {};
                    float support_base_point[3] = {};
                    if (!EstimateSupportPlaneDepth(
                            support_plane, projected_x, projected_y,
                            support_depth_mm) ||
                        !camera_->Deproject(
                            projected_x, projected_y,
                            static_cast<uint16_t>(std::clamp(
                                std::lround(support_depth_mm),
                                1L, static_cast<long>(UINT16_MAX))),
                            support_camera_point)) {
                        top_grasp_built = false;
                        error = "sparse top-grasp support projection failed";
                    } else {
                        planner_->CameraToBase(
                            support_camera_point, support_base_point);
                        const float blend =
                            config_.top_sparse_projected_center_blend;
                        const float dx = blend *
                            (support_base_point[0] -
                                candidate.grasp_pose.x);
                        const float dy = blend *
                            (support_base_point[1] -
                                candidate.grasp_pose.y);
                        candidate.grasp_pose.x += dx;
                        candidate.grasp_pose.y += dy;
                        candidate.pre_grasp_pose.x += dx;
                        candidate.pre_grasp_pose.y += dy;
                        candidate.retreat_pose.x += dx;
                        candidate.retreat_pose.y += dy;
                        candidate.lift_pose.x += dx;
                        candidate.lift_pose.y += dy;
                        base_point[0] = candidate.grasp_pose.x;
                        base_point[1] = candidate.grasp_pose.y;
                        std::cout
                            << "[Pipeline] Sparse top-grasp position "
                            << "source=projected_support_footprint"
                            << " sparse_projected_center_blend=" << blend
                            << " center_shift=[" << dx << "," << dy
                            << "]m" << std::endl;
                    }
                }
                if (top_grasp_built) {
                    std::cout
                        << "[Pipeline] Top-grasp point cloud is sparse; "
                        << "using the mask footprint projected onto the "
                        << "validated support plane" << std::endl;
                }
            }
            if (!top_grasp_built) {
                std::ostringstream diagnostic;
                diagnostic
                    << "point-cloud geometry did not produce a top candidate";
                for (const GraspCandidate& current :
                        safety_geometry.candidates) {
                    if (current.strategy == GraspStrategy::TOP) {
                        diagnostic << ": " << current.rejection_reason;
                        break;
                    }
                }
                if (!safety_geometry.error.empty()) {
                    diagnostic << ": " << safety_geometry.error;
                }
                error = diagnostic.str();
            }
        } else {
            candidate = *geometry_candidate;
            top_grasp_built = ProjectTopCandidateToMaskCenter(
                safety_geometry.geometry, candidate,
                grasp_px, grasp_py, depth_mm,
                cam_point, base_point, error);
        }
    } else {
        top_grasp_built = BuildMaskTopGrasp(
            candidate, grasp_px, grasp_py, depth_mm,
            cam_point, base_point, error,
            safety_geometry.foreground_depth_mm,
            &support_plane);
    }
    if (!top_grasp_built) {
        if (RetryTransientTopPlanning(error)) return;
        SetState(PipelineState::ERROR,
                "Top-grasp planning failed: " + error);
        return;
    }
    geometry_retry_count_ = 0;
    motion_geometry_refresh_count_ = 0;

    std::cout << "[Pipeline] 3D position (base): [" << base_point[0] << ", "
                << base_point[1] << ", " << base_point[2] << "]" << std::endl;

    // Align against the validated 3D grasp point. At the top observation
    // pose, the gripper can overlap the segmentation mask and make its center
    // depth refer to the arm (for example 166 mm) instead of the object
    // (roughly 360 mm). Re-deprojecting that contaminated center would command
    // a large, incorrect base reversal even though the selected grasp point is
    // already in the reachable window.
    float alignment_point[3] = {
        candidate.grasp_pose.x,
        candidate.grasp_pose.y,
        candidate.grasp_pose.z};
    std::cout << "[Pipeline] Mobile base alignment source="
                << "validated_top_grasp_point target=["
                << alignment_point[0] << ", " << alignment_point[1] << ", "
                << alignment_point[2] << "]" << std::endl;

    if (!ConfirmTopAlignmentPoint(alignment_point)) return;

    bool base_alignment_soft_stopped = false;
    base_alignment_command_ = PlanMobileBaseAlignment(
        config_.mobile_base, alignment_point, base_align_attempts_);

    if (!ValidateMobileBaseVisualProgress(alignment_point)) return;

    if (base_alignment_command_.type !=
        MobileBaseAlignmentCommand::Type::NONE) {
        std::string transition_error;
        if (!ValidateBaseAlignmentCommandTransition(
                base_alignment_command_, transition_error)) {
            std::cout
                << "[Pipeline] Mobile base correction stopped: "
                << transition_error
                << "; continuing with workspace, IK and path validation"
                << std::endl;
            base_alignment_command_ = MobileBaseAlignmentCommand{};
            base_alignment_soft_stopped = true;
        }
    }
    if (base_alignment_command_.type !=
        MobileBaseAlignmentCommand::Type::NONE) {
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::DRIVE) {
            const float next_travel =
                std::fabs(base_alignment_command_.linear_x) *
                static_cast<float>(base_alignment_command_.duration_ms) /
                1000.0f;
            if (base_align_travel_m_ + next_travel >
                config_.mobile_base.max_total_travel_m + 1e-6f) {
                stable_count_ = 0;
                SetState(
                    PipelineState::ERROR,
                    "Base alignment stopped: cumulative travel safety "
                    "limit reached");
                return;
            }
        }
        std::copy_n(
            alignment_point, 3, previous_base_alignment_point_.begin());
        previous_base_alignment_command_ = base_alignment_command_;
        have_previous_base_alignment_point_ = true;
        std::cout << "[Pipeline] Mobile base alignment needed: "
                    << base_alignment_command_.reason << " (attempt "
                    << base_align_attempts_ + 1 << "/"
                    << config_.mobile_base.max_align_attempts << ")"
                    << std::endl;
        std::ostringstream message;
        message << "Target center=[" << alignment_point[0] << ", "
                << alignment_point[1]
                << "]m; " << base_alignment_command_.reason << "; ";
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::ROTATE) {
            message << "rotate wz=" << base_alignment_command_.angular_z;
        } else {
            message << "drive vx=" << base_alignment_command_.linear_x;
        }
        message << " duration_ms=" << base_alignment_command_.duration_ms;
        SetState(PipelineState::BASE_ALIGNING, message.str());
        return;
    }
    if (base_alignment_command_.max_attempts_reached) {
        std::cout
            << "[Pipeline] Mobile base reached the alignment attempt "
            << "limit; continuing with workspace, IK and path validation "
            << "at the current position"
            << std::endl;
        base_alignment_command_.max_attempts_reached = false;
        base_alignment_soft_stopped = true;
    }
    if (config_.mobile_base.enabled && base_alignment_soft_stopped) {
        std::cout
            << "[Pipeline] Mobile base correction stopped within the soft "
            << "alignment policy; arm safety validation remains required"
            << std::endl;
    } else if (config_.mobile_base.enabled) {
        const float stable_x_limit =
            config_.mobile_base.x_tolerance +
            std::max(0.0f, config_.mobile_base.x_hysteresis);
        const float stable_y_limit =
            config_.mobile_base.y_tolerance +
            std::max(0.0f, config_.mobile_base.y_hysteresis);
        std::cout << "[Pipeline] Mobile base target in comfortable range: "
                    << "x=" << alignment_point[0] << "m in ["
                    << config_.mobile_base.target_x -
                        stable_x_limit
                    << ", "
                    << config_.mobile_base.target_x +
                        stable_x_limit
                    << "]m, y=" << alignment_point[1]
                    << "m within stable +/-" << stable_y_limit << "m"
                    << std::endl;
    }

    if (!planner_->InWorkspace(
            candidate.grasp_pose.x, candidate.grasp_pose.y,
            candidate.grasp_pose.z)) {
        SetState(
            PipelineState::ERROR,
            "Top-grasp pose out of workspace: pose=[" +
                std::to_string(candidate.grasp_pose.x) + ", " +
                std::to_string(candidate.grasp_pose.y) + ", " +
                std::to_string(candidate.grasp_pose.z) + "]");
        return;
    }
    if (!planner_->InWorkspace(
            candidate.pre_grasp_pose.x, candidate.pre_grasp_pose.y,
            candidate.pre_grasp_pose.z)) {
        SetState(
            PipelineState::ERROR,
            "Adjusted grasp pose out of workspace after gripper_offset: "
            "pose=[" + std::to_string(candidate.pre_grasp_pose.x) + ", " +
                std::to_string(candidate.pre_grasp_pose.y) + ", " +
                std::to_string(candidate.pre_grasp_pose.z) + "], offset=" +
                std::to_string(config_.planner.gripper_offset));
        return;
    }

    const int validation_timeout_ms = std::max(
        config_.geometry.planning_timeout_ms,
        kMinimumCandidateValidationTimeoutMs);
    std::string validation_detail;
    const GraspResult validation_result = executor_->ValidateGraspPoses(
        candidate.pre_grasp_pose, candidate.grasp_pose,
        candidate.retreat_pose, candidate.lift_pose,
        candidate.entry_clearance_z_m,
        candidate.grasp_yaw_rad, true,
        validation_timeout_ms, &validation_detail);
    if (validation_result != GraspResult::SUCCESS) {
        std::string message = "Top-grasp plan validation failed";
        if (!validation_detail.empty()) {
            message += ": " + validation_detail;
        } else {
            message += ": ";
            message += GraspResultName(validation_result);
        }
        SetState(PipelineState::ERROR, message);
        return;
    }

    grasp_strategy_ = GraspStrategy::TOP;
    grasp_pose_ = candidate.grasp_pose;
    pre_grasp_pose_ = candidate.pre_grasp_pose;
    retreat_pose_ = candidate.retreat_pose;
    lift_pose_ = candidate.lift_pose;
    grasp_yaw_rad_ = candidate.grasp_yaw_rad;
    grasp_opening_ = NAN;
    perception_cycle_active_ = false;
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;

    SaveGraspDebug(
        grasp_px, grasp_py, depth_mm, cam_point, base_point);

    if (config_.plan_only) {
        SetState(PipelineState::DONE,
                "Plan validated without motion: strategy=top");
        return;
    }

    if (config_.performance_log_enabled) {
        const auto planning_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - planning_start)
                .count();
        std::cout << "[Timing] stage=PLANNING"
                    << " elapsed_ms=" << planning_ms
                    << " cpu_ms="
                    << ProcessCpuMillis() - planning_cpu_start
                    << " stable_count=" << stable_count_
                    << " result=GRASP_READY" << std::endl;
    }

    SetState(PipelineState::APPROACHING, "Moving to pre-grasp...");
}

bool GraspPipeline::RetryTransientTopPlanning(const std::string& reason) {
    stable_count_ = 0;
    perception_cycle_active_ = false;
    ++geometry_retry_count_;

    if (geometry_retry_count_ == 1 && config_.save_debug_data) {
        SavePipelineStepCameraDebug(
            config_.debug_output_dir, "top_depth_unavailable",
            current_color_, current_depth_, task_id_);
    }

    if (geometry_retry_count_ >= kGeometryRefreshAttempt &&
        motion_geometry_refresh_count_ < kMotionGeometryMaxRefreshes) {
        ++motion_geometry_refresh_count_;
        if (!FlushCameraAfterMotion("top-grasp depth recovery")) {
            SetState(
                PipelineState::ERROR,
                "Failed to refresh camera after transient top-grasp "
                "perception failure");
            return true;
        }
        SetState(
            PipelineState::DETECTING,
            "Refreshed camera after transient top-grasp perception "
            "failure: " + reason);
        return true;
    }

    if (geometry_retry_count_ < kMaxGeometryAttempts) {
        std::ostringstream message;
        message << "Top-grasp perception unavailable, retrying frame "
                << geometry_retry_count_ + 1 << "/"
                << kMaxGeometryAttempts << ": ";
        message << reason;
        SetState(PipelineState::DETECTING, message.str());
        return true;
    }
    return false;
}

void GraspPipeline::HandlePlanning() {
    if (current_depth_.empty()) {
        stable_count_ = 0;
        SetState(PipelineState::DETECTING, "No cached depth, detecting again");
        return;
    }

    const bool explicit_top_strategy =
        config_.geometry.strategy == "top";
    if (!observation_strategy_selected_ &&
        explicit_top_strategy) {
        GraspGeometryResult observation_geometry;
        geometry_planner_->Plan(
            current_depth_, current_target_, *camera_, *planner_,
            observation_geometry);
        SupportPlane observation_support_plane;
        std::string support_plane_source;
        if (!ResolveTopSupportPlane(
                observation_geometry.geometry.table,
                observation_support_plane,
                support_plane_source)) {
            SetState(
                PipelineState::ERROR,
                "Top observation safety validation failed: support surface "
                "configuration is invalid");
            return;
        }
        std::cout << "[Pipeline] Top observation support surface source="
                    << support_plane_source << std::endl;
        executor_->SetSupportPlane(observation_support_plane);

        grasp_strategy_ = GraspStrategy::TOP;
        observation_strategy_selected_ = true;
        std::cout << "[Pipeline] Explicit top-grasp strategy selected "
                    "before arm motion" << std::endl;
        if (!config_.plan_only) {
            stable_count_ = 0;
            perception_cycle_active_ = false;
            SetState(
                PipelineState::OBSERVING,
                "Strategy selected: top; moving to matching observation pose");
            return;
        }
    }

    if (observation_strategy_selected_ &&
        grasp_strategy_ == GraspStrategy::TOP) {
        HandleTopPlanning();
        return;
    }

    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_cpu_start = ProcessCpuMillis();

    bool geometry_from_point_cloud = geometry_planner_->Plan(
        current_depth_, current_target_, *camera_, *planner_,
        grasp_geometry_result_,
        observation_strategy_selected_
            ? std::optional<GraspStrategy>(grasp_strategy_)
            : std::nullopt);
    bool geometry_recovered = false;
    if (!geometry_from_point_cloud) {
        stable_count_ = 0;
        geometry_retry_count_++;
        const bool can_recover_top_geometry =
            observation_strategy_selected_ &&
            grasp_strategy_ == GraspStrategy::TOP &&
            last_valid_geometry_available_ &&
            grasp_geometry_result_.geometry.table.inlier_count > 0 &&
            (top_geometry_recovery_active_ ||
            geometry_retry_count_ >= kTopGeometryRecoveryAttempt);
        if (can_recover_top_geometry) {
            const TablePlane current_table =
                grasp_geometry_result_.geometry.table;
            const std::string point_cloud_error =
                grasp_geometry_result_.error;
            GraspCandidate candidate;
            float grasp_px = current_target_.center.x;
            float grasp_py = current_target_.center.y;
            uint16_t depth_mm = 0;
            float camera_point[3] = {};
            float base_point[3] = {};
            std::string recovery_error;
            if (BuildMaskTopGrasp(
                    candidate, grasp_px, grasp_py, depth_mm,
                    camera_point, base_point, recovery_error)) {
                ObjectGeometry3D recovered = last_valid_geometry_;
                recovered.table = current_table;
                const cv::Point3f measured(
                    base_point[0], base_point[1], base_point[2]);
                const float signed_table_distance =
                    current_table.normal.x * measured.x +
                    current_table.normal.y * measured.y +
                    current_table.normal.z * measured.z +
                    current_table.d;
                recovered.table_center = cv::Point3f(
                    measured.x -
                        current_table.normal.x * signed_table_distance,
                    measured.y -
                        current_table.normal.y * signed_table_distance,
                    measured.z -
                        current_table.normal.z * signed_table_distance);
                recovered.center = cv::Point3f(
                    recovered.table_center.x +
                        current_table.normal.x * recovered.height_m * 0.5f,
                    recovered.table_center.y +
                        current_table.normal.y * recovered.height_m * 0.5f,
                    recovered.table_center.z +
                        current_table.normal.z * recovered.height_m * 0.5f);
                recovered.valid = true;

                GraspGeometryResult recovery_result;
                recovery_result.geometry = recovered;
                recovery_result.candidates.push_back(candidate);
                recovery_result.foreground_depth_mm =
                    static_cast<float>(depth_mm);
                recovery_result.center_depth_mm =
                    grasp_geometry_result_.center_depth_mm;
                recovery_result.elapsed_ms =
                    grasp_geometry_result_.elapsed_ms;
                recovery_result.error =
                    "top geometry recovered after point-cloud failure: " +
                    point_cloud_error;
                grasp_geometry_result_ = std::move(recovery_result);
                geometry_retry_count_ = 0;
                top_geometry_recovery_active_ = true;
                geometry_recovered = true;
                std::cout
                    << "[Pipeline] Recovered low-profile top-grasp geometry "
                        "using the current support plane and the last "
                        "confirmed object dimensions"
                    << std::endl;
            }
        }
        if (!observation_strategy_selected_ &&
            geometry_retry_count_ >= kTopGeometryRecoveryAttempt) {
            GraspGeometryResult fallback_geometry;
            if (BuildLowProfileTopFallback(
                    grasp_geometry_result_, fallback_geometry)) {
                grasp_geometry_result_ =
                    std::move(fallback_geometry);
                geometry_retry_count_ = 0;
                geometry_recovered = true;
                std::cout
                    << "[Pipeline] Recovered low-profile target with "
                    << "mask-guided top grasp before observation"
                    << std::endl;
            }
        }
    }

    if (!geometry_from_point_cloud && !geometry_recovered) {
        if (motion_geometry_confirmation_pending_ &&
            geometry_retry_count_ >= kGeometryRefreshAttempt &&
            motion_geometry_refresh_count_ <
                kMotionGeometryMaxRefreshes) {
            geometry_retry_count_ = 0;
            motion_geometry_refresh_count_++;
            perception_cycle_active_ = false;
            if (!FlushCameraAfterMotion(
                    "geometry estimation retry")) {
                SetState(PipelineState::ERROR,
                    "Failed to refresh camera after invalid "
                    "post-motion 3D geometry");
                return;
            }
            SetState(PipelineState::DETECTING,
                "Refreshed camera after transient post-motion "
                "3D geometry");
            return;
        }
        if (geometry_retry_count_ < kMaxGeometryAttempts) {
            perception_cycle_active_ = false;
            std::ostringstream retry_message;
            retry_message << "3D geometry unavailable, retrying frame "
                            << geometry_retry_count_ + 1 << "/"
                            << kMaxGeometryAttempts;
            SetState(PipelineState::DETECTING, retry_message.str());
            return;
        }
        SetState(PipelineState::ERROR,
                "3D grasp geometry failed: " +
                    grasp_geometry_result_.error);
        return;
    }
    if (geometry_from_point_cloud) {
        top_geometry_recovery_active_ = false;
    }
    if (motion_geometry_confirmation_pending_) {
        const ObjectGeometry3D& current_geometry =
            grasp_geometry_result_.geometry;
        motion_geometry_sample_count_++;
        bool consistent = false;
        std::string consistency_detail;
        if (motion_geometry_reference_valid_) {
            consistent = AreObjectGeometriesConsistent(
                motion_geometry_reference_, current_geometry,
                &consistency_detail);
            motion_geometry_consistent_count_ = consistent
                ? motion_geometry_consistent_count_ + 1
                : 0;
        }
        motion_geometry_reference_ = current_geometry;
        motion_geometry_reference_valid_ = true;

        if (motion_geometry_consistent_count_ <
            kMotionGeometryRequiredConsistentPairs) {
            stable_count_ = 0;
            perception_cycle_active_ = false;
            if (motion_geometry_sample_count_ >=
                kMotionGeometryMaxSamples) {
                if (motion_geometry_refresh_count_ <
                    kMotionGeometryMaxRefreshes) {
                    motion_geometry_refresh_count_++;
                    if (!FlushCameraAfterMotion(
                            "geometry confirmation retry")) {
                        SetState(PipelineState::ERROR,
                            "Failed to refresh camera while confirming "
                            "3D geometry");
                        return;
                    }
                    SetState(PipelineState::DETECTING,
                        "Refreshing transient 3D geometry after motion");
                    return;
                }
                std::cout
                    << "[Pipeline] 3D geometry did not reach the temporal "
                    << "stability target; using the latest valid geometry "
                    << "and continuing with workspace, IK and path "
                    << "validation: " << consistency_detail
                    << std::endl;
                motion_geometry_consistent_count_ =
                    kMotionGeometryRequiredConsistentPairs;
            }
            if (motion_geometry_consistent_count_ <
                kMotionGeometryRequiredConsistentPairs) {
                std::ostringstream confirmation_message;
                confirmation_message
                    << "Confirming 3D geometry after motion "
                    << motion_geometry_sample_count_ << "/"
                    << kMotionGeometryMaxSamples;
                if (!consistency_detail.empty()) {
                    confirmation_message << "; "
                        << (consistent ? "stable" : "changed");
                }
                SetState(
                    PipelineState::DETECTING,
                    confirmation_message.str());
                return;
            }
        }

        std::cout << "[Pipeline] Post-motion 3D geometry confirmed with "
                    << motion_geometry_sample_count_ << " samples: "
                    << consistency_detail << std::endl;
        motion_geometry_confirmation_pending_ = false;
        motion_geometry_reference_valid_ = false;
        motion_geometry_sample_count_ = 0;
        motion_geometry_consistent_count_ = 0;
        motion_geometry_refresh_count_ = 0;
    }

    SupportPlane support_plane;
    if (!BuildSupportPlane(
            grasp_geometry_result_.geometry.table, support_plane)) {
        SetState(
            PipelineState::ERROR,
            "3D grasp safety validation failed: support surface is invalid");
        return;
    }
    std::cout << "[Pipeline] Grasp support surface normal=["
                << support_plane.normal_x << ","
                << support_plane.normal_y << ","
                << support_plane.normal_z << "] d="
                << support_plane.d << " bounds=["
                << support_plane.min_x << ","
                << support_plane.max_x << ","
                << support_plane.min_y << ","
                << support_plane.max_y << "]" << std::endl;
    if (!observation_strategy_selected_ &&
        !last_top_support_plane_valid_) {
        last_top_support_plane_ = support_plane;
        last_top_support_plane_valid_ = true;
        std::cout << "[Pipeline] Cached pre-motion support surface: normal=["
                    << support_plane.normal_x << ","
                    << support_plane.normal_y << ","
                    << support_plane.normal_z << "] d="
                    << support_plane.d << std::endl;
    }
    executor_->SetSupportPlane(support_plane);

    const auto matches_observation_strategy =
        [this](const GraspCandidate& candidate) {
            return !observation_strategy_selected_ ||
                candidate.strategy == grasp_strategy_;
        };
    GraspCandidate* preferred_candidate = nullptr;
    for (GraspCandidate& candidate : grasp_geometry_result_.candidates) {
        if (candidate.geometry_valid &&
            matches_observation_strategy(candidate)) {
            preferred_candidate = &candidate;
            break;
        }
    }
    if (!preferred_candidate) {
        std::ostringstream message;
        if (observation_strategy_selected_) {
            message << "Selected observation strategy "
                    << GraspStrategyName(grasp_strategy_)
                    << " is unavailable after re-detection";
        } else {
            message << "No geometry-valid grasp strategy before observation";
        }
        for (const GraspCandidate& candidate :
            grasp_geometry_result_.candidates) {
            if (matches_observation_strategy(candidate)) {
                message << "; " << GraspStrategyName(candidate.strategy)
                        << "=" << candidate.rejection_reason;
            }
        }
        if (observation_strategy_selected_) {
            geometry_retry_count_++;
            stable_count_ = 0;
            target_track_ = TargetTrack{};
            perception_cycle_active_ = false;
            if (geometry_retry_count_ < kMaxGeometryAttempts) {
                std::ostringstream retry_message;
                retry_message
                    << "Selected "
                    << GraspStrategyName(grasp_strategy_)
                    << " strategy temporarily unavailable; retaining "
                    "the strategy and retrying geometry "
                    << geometry_retry_count_ << "/"
                    << kMaxGeometryAttempts;
                SetState(PipelineState::DETECTING,
                        retry_message.str() + ": " + message.str());
                return;
            }
        }
        SetState(PipelineState::ERROR, message.str());
        return;
    }

    geometry_retry_count_ = 0;
    last_valid_geometry_ = grasp_geometry_result_.geometry;
    last_valid_geometry_available_ = true;
    last_valid_strategy_ = preferred_candidate->strategy;
    last_valid_strategy_available_ = true;

    std::cout << "[Pipeline] Object geometry: dimensions=["
                << grasp_geometry_result_.geometry.length_m << ", "
                << grasp_geometry_result_.geometry.width_m << ", "
                << grasp_geometry_result_.geometry.height_m << "]m"
                << " points="
                << grasp_geometry_result_.geometry.object_point_count
                << " table_inliers="
                << grasp_geometry_result_.geometry.table.inlier_count
                << " elapsed_ms=" << grasp_geometry_result_.elapsed_ms
                << std::endl;

    float debug_grasp_px = current_target_.center.x;
    float debug_grasp_py = current_target_.center.y;
    int cx = ClampPixel(static_cast<int>(std::lround(debug_grasp_px)),
                        current_depth_.cols);
    int cy = ClampPixel(static_cast<int>(std::lround(debug_grasp_py)),
                        current_depth_.rows);

    std::cout << "[Pipeline] Geometry reference pixel: ["
                << cx << ", " << cy << "]" << std::endl;

    // Keep a representative camera point in the debug record. Base alignment
    // and grasp planning use the table-relative point-cloud center below.
    constexpr int roi_size = 5;
    uint16_t depth_mm = 0;
    float cam_point[3] = {0.0f, 0.0f, 0.0f};
    if (std::isfinite(grasp_geometry_result_.foreground_depth_mm) &&
        grasp_geometry_result_.foreground_depth_mm > 0.0f) {
        depth_mm = static_cast<uint16_t>(std::clamp(
            std::lround(grasp_geometry_result_.foreground_depth_mm),
            1L, static_cast<long>(UINT16_MAX)));
        std::cout << "[Pipeline] Depth source=geometry_foreground_cluster"
                    << " value=" << depth_mm << "mm samples="
                    << grasp_geometry_result_.geometry.source_point_count
                    << " center_depth="
                    << grasp_geometry_result_.center_depth_mm << "mm"
                    << std::endl;
    } else if (MedianDepthAtPixel(
                    current_depth_, cx, cy, roi_size, depth_mm)) {
        std::cout << "[Pipeline] Depth source=target_center_roi value="
                    << depth_mm << "mm" << std::endl;
    } else {
        std::cout << "[Pipeline] Depth source=geometry_silhouette"
                    << " value=unavailable" << std::endl;
    }

    if (depth_mm != 0 &&
        !camera_->Deproject(cx, cy, depth_mm, cam_point)) {
        std::cout << "[Pipeline] Representative camera point unavailable"
                    << std::endl;
        depth_mm = 0;
        cam_point[0] = 0.0f;
        cam_point[1] = 0.0f;
        cam_point[2] = 0.0f;
    }

    // Keep the table-relative object center for diagnostics. Base alignment
    // below uses the selected candidate's actual grasp or pre-grasp point.
    float base_point[3] = {
        grasp_geometry_result_.geometry.center.x,
        grasp_geometry_result_.geometry.center.y,
        grasp_geometry_result_.geometry.center.z};

    if (preferred_candidate &&
        preferred_candidate->strategy == GraspStrategy::TOP &&
        config_.top_position_source == "mask_depth") {
        std::string top_error;
        if (!BuildMaskTopGrasp(
                *preferred_candidate, debug_grasp_px, debug_grasp_py,
                depth_mm, cam_point, base_point, top_error,
                grasp_geometry_result_.foreground_depth_mm)) {
            SetState(PipelineState::ERROR,
                    "Top-grasp planning failed: " + top_error);
            return;
        }
        cx = ClampPixel(
            static_cast<int>(std::lround(debug_grasp_px)),
            current_depth_.cols);
        cy = ClampPixel(
            static_cast<int>(std::lround(debug_grasp_py)),
            current_depth_.rows);
    } else if (preferred_candidate &&
            preferred_candidate->strategy == GraspStrategy::TOP) {
        std::string top_error;
        if (!ProjectTopCandidateToMaskCenter(
                grasp_geometry_result_.geometry,
                *preferred_candidate,
                debug_grasp_px, debug_grasp_py,
                depth_mm, cam_point, base_point, top_error)) {
            SetState(PipelineState::ERROR,
                    "Top-grasp planning failed: " + top_error);
            return;
        }
        cx = ClampPixel(
            static_cast<int>(std::lround(debug_grasp_px)),
            current_depth_.cols);
        cy = ClampPixel(
            static_cast<int>(std::lround(debug_grasp_py)),
            current_depth_.rows);
    }

    std::cout << "[Pipeline] 3D position (base): [" << base_point[0] << ", "
                << base_point[1] << ", " << base_point[2] << "]" << std::endl;

    MobileBaseAlignmentConfig alignment_config = config_.mobile_base;
    bool side_pregrasp_alignment_active = false;
    bool side_overshoot_accepted = false;
    bool base_alignment_soft_stopped = false;
    float alignment_point[3] = {
        base_point[0], base_point[1], base_point[2]};
    for (const GraspCandidate& candidate :
            grasp_geometry_result_.candidates) {
        if (!candidate.geometry_valid ||
            !matches_observation_strategy(candidate)) {
            continue;
        }
        if (candidate.strategy == GraspStrategy::TOP) {
            alignment_point[0] = candidate.grasp_pose.x;
            alignment_point[1] = candidate.grasp_pose.y;
            alignment_point[2] = candidate.grasp_pose.z;
            std::cout << "[Pipeline] Top-grasp alignment point: ["
                        << alignment_point[0] << ", "
                        << alignment_point[1] << "]m"
                        << std::endl;
            break;
        }

        side_pregrasp_alignment_active = true;
        // The UART chassis moves about 20-30 mm per minimum effective pulse.
        // Align to a window instead of a single point so one minimum pulse
        // cannot overshoot and immediately request motion in reverse.
        constexpr float kSidePreGraspAlignmentWindowM = 0.030f;
        constexpr float kSidePreGraspUpperHysteresisM = 0.005f;
        const float minimum_pregrasp_x =
            config_.geometry.side_pregrasp_min_x_m;
        const float maximum_pregrasp_x =
            minimum_pregrasp_x + kSidePreGraspAlignmentWindowM;
        const float preferred_pregrasp_x =
            0.5f * (minimum_pregrasp_x + maximum_pregrasp_x);
        float required_shift = 0.0f;
        if (candidate.pre_grasp_pose.x < minimum_pregrasp_x ||
            candidate.pre_grasp_pose.x >
                maximum_pregrasp_x +
                    kSidePreGraspUpperHysteresisM) {
            required_shift =
                preferred_pregrasp_x - candidate.pre_grasp_pose.x;
            const float minimum_target_x =
                config_.planner.workspace.x_min + 0.01f;
            const float maximum_target_x =
                config_.planner.workspace.x_max - 0.01f;
            alignment_config.target_x = std::clamp(
                base_point[0] + required_shift,
                minimum_target_x, maximum_target_x);
            alignment_config.x_tolerance =
                0.5f * kSidePreGraspAlignmentWindowM;
            alignment_config.x_hysteresis =
                kSidePreGraspUpperHysteresisM;
            std::cout << "[Pipeline] Aligning closed-gripper side sweep: "
                        << "pre_grasp_x="
                        << candidate.pre_grasp_pose.x
                        << "m desired_range=[" << minimum_pregrasp_x
                        << "," << maximum_pregrasp_x
                        << "]m; requesting base target_x="
                        << alignment_config.target_x << "m"
                        << std::endl;
        } else {
            // Keep the current longitudinal distance once the closed gripper
            // is aligned with the horizontal side-ready sweep radius.
            alignment_config.target_x = base_point[0];
            alignment_config.x_tolerance = std::max(
                alignment_config.x_tolerance, 0.005f);
            std::cout << "[Pipeline] Side pre-grasp distance is aligned: x="
                        << candidate.pre_grasp_pose.x
                        << "m; preserving current base distance"
                        << std::endl;
        }
        break;
    }

    base_alignment_command_ = PlanMobileBaseAlignment(
        alignment_config, alignment_point, base_align_attempts_);

    if (side_pregrasp_alignment_active &&
        have_previous_base_alignment_point_ &&
        IsMobileBaseDirectionReversal(
            previous_base_alignment_command_,
            base_alignment_command_)) {
        side_overshoot_accepted = true;
        base_alignment_command_ = MobileBaseAlignmentCommand{};
        base_alignment_command_.reason =
            "minimum chassis pulse crossed the side alignment target";
        std::cout
            << "[Pipeline] Minimum chassis pulse crossed the side "
            << "alignment target; stopping base motion and validating "
            << "arm reachability at the current position"
            << std::endl;
    }

    if (!ValidateMobileBaseVisualProgress(alignment_point)) return;

    if (base_alignment_command_.type !=
        MobileBaseAlignmentCommand::Type::NONE) {
        std::string transition_error;
        if (!ValidateBaseAlignmentCommandTransition(
                base_alignment_command_, transition_error)) {
            std::cout
                << "[Pipeline] Mobile base correction stopped: "
                << transition_error
                << "; continuing with workspace, IK and path validation"
                << std::endl;
            base_alignment_command_ = MobileBaseAlignmentCommand{};
            base_alignment_soft_stopped = true;
        }
    }
    if (base_alignment_command_.type !=
        MobileBaseAlignmentCommand::Type::NONE) {
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::DRIVE) {
            const float next_travel =
                std::fabs(base_alignment_command_.linear_x) *
                static_cast<float>(base_alignment_command_.duration_ms) /
                1000.0f;
            if (base_align_travel_m_ + next_travel >
                config_.mobile_base.max_total_travel_m + 1e-6f) {
                stable_count_ = 0;
                SetState(PipelineState::ERROR,
                    "Base alignment stopped: cumulative travel safety "
                    "limit reached");
                return;
            }
        }
        std::copy_n(
            alignment_point, 3, previous_base_alignment_point_.begin());
        previous_base_alignment_command_ = base_alignment_command_;
        have_previous_base_alignment_point_ = true;
        std::cout << "[Pipeline] Mobile base alignment needed: "
                    << base_alignment_command_.reason
                    << " (attempt " << (base_align_attempts_ + 1)
                    << "/" << config_.mobile_base.max_align_attempts << ")"
                    << std::endl;
        std::ostringstream message;
        message << "Target=[" << alignment_point[0] << ", "
                << alignment_point[1]
                << "]m; " << base_alignment_command_.reason << "; ";
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::ROTATE) {
            message << "rotate wz=" << base_alignment_command_.angular_z;
        } else {
            message << "drive vx=" << base_alignment_command_.linear_x;
        }
        message << " duration_ms=" << base_alignment_command_.duration_ms;
        SetState(PipelineState::BASE_ALIGNING, message.str());
        return;
    }
    if (base_alignment_command_.max_attempts_reached) {
        std::cout
            << "[Pipeline] Mobile base reached the alignment attempt "
            << "limit; continuing with workspace, IK and path validation "
            << "at the current position"
            << std::endl;
        base_alignment_command_.max_attempts_reached = false;
        base_alignment_soft_stopped = true;
    }
    if (config_.mobile_base.enabled && base_alignment_soft_stopped) {
        std::cout
            << "[Pipeline] Mobile base correction stopped within the soft "
            << "alignment policy; arm safety validation remains required"
            << std::endl;
    } else if (config_.mobile_base.enabled && side_overshoot_accepted) {
        std::cout << "[Pipeline] Mobile base side alignment accepted after "
                    "minimum-pulse overshoot; workspace and IK validation "
                    "remain required"
                    << std::endl;
    } else if (config_.mobile_base.enabled) {
        const float stable_x_tolerance =
            alignment_config.x_tolerance +
            std::max(0.0f, alignment_config.x_hysteresis);
        const float stable_y_limit =
            config_.mobile_base.y_tolerance +
            std::max(0.0f, config_.mobile_base.y_hysteresis);
        std::cout << "[Pipeline] Mobile base target in comfortable range: "
                    << "x=" << alignment_point[0] << "m in ["
                    << alignment_config.target_x -
                        stable_x_tolerance
                    << ", "
                    << alignment_config.target_x +
                        stable_x_tolerance
                    << "]m, y=" << alignment_point[1]
                    << "m within stable +/-"
                    << stable_y_limit << "m"
                    << std::endl;
    }

    if (!observation_strategy_selected_) {
        grasp_strategy_ = preferred_candidate->strategy;
        observation_strategy_selected_ = true;
        std::cout << "[Pipeline] Observation strategy selected after base "
                    "alignment: "
                    << GraspStrategyName(grasp_strategy_) << std::endl;
        if (!config_.plan_only) {
            stable_count_ = 0;
            perception_cycle_active_ = false;
            SetState(
                PipelineState::OBSERVING,
                "Strategy selected: " +
                    std::string(GraspStrategyName(grasp_strategy_)) +
                    "; moving to matching observation pose");
            return;
        }
    }

    GraspCandidate* selected_candidate = nullptr;
    for (GraspCandidate& candidate : grasp_geometry_result_.candidates) {
        if (!matches_observation_strategy(candidate)) continue;
        if (!candidate.geometry_valid) {
            std::cout << "[Pipeline] Candidate rejected: strategy="
                        << GraspStrategyName(candidate.strategy)
                        << " reason=" << candidate.rejection_reason
                        << std::endl;
            continue;
        }
        if (!planner_->InWorkspace(
                candidate.grasp_pose.x,
                candidate.grasp_pose.y,
                candidate.grasp_pose.z)) {
            candidate.rejection_reason =
                "grasp pose remains outside workspace after base alignment";
            std::cout << "[Pipeline] Candidate requires additional base "
                        "alignment: strategy="
                        << GraspStrategyName(candidate.strategy)
                        << " pose=[" << candidate.grasp_pose.x << ","
                        << candidate.grasp_pose.y << ","
                        << candidate.grasp_pose.z << "]"
                        << std::endl;
            continue;
        }

        const int validation_timeout_ms = std::max(
            config_.geometry.planning_timeout_ms,
            kMinimumCandidateValidationTimeoutMs);
        std::string ik_detail;
        const bool use_top_constraints =
            candidate.strategy == GraspStrategy::TOP;
        const GraspResult ik_result = executor_->ValidateGraspPoses(
            candidate.pre_grasp_pose, candidate.grasp_pose,
            candidate.retreat_pose, candidate.lift_pose,
            candidate.entry_clearance_z_m,
            candidate.grasp_yaw_rad,
            use_top_constraints, validation_timeout_ms, &ik_detail);
        if (ik_result == GraspResult::SUCCESS) {
            const ExecutorDiagnostics diagnostics =
                executor_->GetDiagnostics();
            candidate.ik_margin_rad =
                diagnostics.validation_min_joint_margin_rad;
            if (std::isfinite(candidate.ik_margin_rad)) {
                candidate.score += 0.10f * std::clamp(
                    candidate.ik_margin_rad / 0.30f, 0.0f, 1.0f);
            }
            selected_candidate = &candidate;
            break;
        }
        candidate.rejection_reason = ik_detail.empty()
            ? "IK validation failed"
            : ik_detail;
        std::cout << "[Pipeline] Candidate rejected: strategy="
                    << GraspStrategyName(candidate.strategy)
                    << " reason=" << candidate.rejection_reason
                    << std::endl;
    }

    if (!selected_candidate) {
        std::ostringstream message;
        message << "No safe 3D grasp candidate";
        for (const GraspCandidate& candidate :
            grasp_geometry_result_.candidates) {
            if (!matches_observation_strategy(candidate)) continue;
            message << "; " << GraspStrategyName(candidate.strategy)
                    << "=" << candidate.rejection_reason;
        }
        SetState(PipelineState::ERROR, message.str());
        return;
    }

    grasp_strategy_ = selected_candidate->strategy;
    grasp_pose_ = selected_candidate->grasp_pose;
    pre_grasp_pose_ = selected_candidate->pre_grasp_pose;
    retreat_pose_ = selected_candidate->retreat_pose;
    lift_pose_ = selected_candidate->lift_pose;
    grasp_yaw_rad_ = selected_candidate->grasp_yaw_rad;
    grasp_opening_ = grasp_strategy_ == GraspStrategy::SIDE
        ? 1.0f
        : NAN;
    std::cout << "[Pipeline] Selected 3D grasp: strategy="
                << GraspStrategyName(grasp_strategy_)
                << " score=" << selected_candidate->score
                << " required_width_m="
                << selected_candidate->required_width_m
                << " width_margin_m="
                << selected_candidate->width_margin_m
                << " depth_quality="
                << selected_candidate->depth_quality
                << " path_clearance_m="
                << selected_candidate->path_clearance_m
                << " workspace_margin_m="
                << selected_candidate->workspace_margin_m
                << " ik_margin_rad="
                << selected_candidate->ik_margin_rad
                << " gripper_opening=" << grasp_opening_ << std::endl;

    const auto perception_elapsed_ms = perception_cycle_active_
        ? std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() -
                perception_cycle_started_at_)
                .count()
        : 0;
    const bool within_perception_budget =
        !perception_cycle_active_ ||
        perception_elapsed_ms <= config_.geometry.perception_budget_ms;
    std::cout << "[Timing] stage=PERCEPTION_PLANNING"
                << " elapsed_ms=" << perception_elapsed_ms
                << " budget_ms=" << config_.geometry.perception_budget_ms
                << " result="
                << (within_perception_budget ? "SUCCESS" : "OVERRUN")
                << std::endl;
    perception_cycle_active_ = false;
    if (!within_perception_budget) {
        std::cout
            << "[Pipeline] Perception and planning exceeded the "
            << "performance budget; continuing because workspace, IK and "
            << "path safety validation succeeded"
            << std::endl;
    }

    SaveGraspDebug(debug_grasp_px, debug_grasp_py, depth_mm,
                    cam_point, base_point);

    if (config_.plan_only) {
        SetState(PipelineState::DONE,
                "Plan validated without motion: strategy=" +
                    std::string(GraspStrategyName(grasp_strategy_)));
        return;
    }

    if (config_.performance_log_enabled) {
        const auto planning_end = std::chrono::steady_clock::now();
        const auto planning_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                planning_end - planning_start)
                .count();
        std::cout << "[Timing] stage=PLANNING"
                << " elapsed_ms=" << planning_ms
                << " cpu_ms="
                << ProcessCpuMillis() - planning_cpu_start
                << " stable_count=" << stable_count_
                << " result=GRASP_READY" << std::endl;
    }

    SetState(PipelineState::APPROACHING, "Moving to pre-grasp...");
}

bool GraspPipeline::ValidateMobileBaseVisualProgress(
    const float alignment_point[3]) {
    if (!have_previous_base_alignment_point_) return true;

    const float progress = MeasureMobileBaseAlignmentProgress(
        previous_base_alignment_point_.data(), alignment_point,
        previous_base_alignment_command_);
    const float required_progress = RequiredMobileBaseAlignmentProgress(
        config_.mobile_base, previous_base_alignment_point_.data(),
        previous_base_alignment_command_);
    std::cout << "[Pipeline] Mobile base visual progress: "
                << progress << "m (required >= "
                << required_progress << "m)" << std::endl;

    const bool still_needs_alignment =
        base_alignment_command_.type !=
            MobileBaseAlignmentCommand::Type::NONE ||
        base_alignment_command_.max_attempts_reached;
    const bool odometry_confirmed =
        last_base_motion_odometry_confirmed_;
    have_previous_base_alignment_point_ = false;
    last_base_motion_odometry_confirmed_ = false;
    if (!still_needs_alignment) return true;

    const float maximum_regression = std::max(
        0.0f, config_.mobile_base.max_visual_regression_m);
    if (odometry_confirmed && progress >= -maximum_regression) {
        std::cout
            << "[Pipeline] Mobile base odometry already confirmed the "
            << "commanded motion; using refreshed vision for the next "
            << "closed-loop correction" << std::endl;
        return true;
    }
    if (progress >= required_progress) {
        std::cout << "[Pipeline] Mobile base visual motion confirmed; "
                    "continuing closed-loop alignment" << std::endl;
        return true;
    }

    std::ostringstream message;
    if (progress < -maximum_regression) {
        message << "Base alignment stopped: visual progress regressed";
    } else {
        message << "Base alignment stopped: visual feedback did not confirm "
                    "commanded motion";
    }
    message << " (measured_m=" << progress
            << " required_m=" << required_progress << ")";
    base_alignment_command_ = MobileBaseAlignmentCommand{};
    stable_count_ = 0;
    SetState(PipelineState::ERROR, message.str());
    return false;
}

bool GraspPipeline::ValidateBaseAlignmentCommandTransition(
    const MobileBaseAlignmentCommand& command,
    std::string& error) {
    if (base_align_attempts_ <= 0 ||
        !IsMobileBaseDirectionReversal(
            previous_base_alignment_command_, command)) {
        error.clear();
        return true;
    }

    base_align_direction_reversals_++;
    std::cout << "[Pipeline] Mobile base direction reversal: count="
                << base_align_direction_reversals_ << "/"
                << config_.mobile_base.max_direction_reversals
                << std::endl;
    if (base_align_direction_reversals_ <=
        config_.mobile_base.max_direction_reversals) {
        error.clear();
        return true;
    }

    error =
        "Base alignment stopped: repeated direction reversals indicate "
        "oscillation; widen the comfortable range or inspect odometry";
    return false;
}

void GraspPipeline::HandleBaseAligning() {
    if (!mobile_base_) {
        SetState(PipelineState::ERROR, "Mobile base controller not initialized");
        return;
    }
    if (!action_.active) {
        if (!WaitForConfirm("即将移动底盘对齐目标")) return;
        last_base_motion_odometry_confirmed_ = false;
        StartAction(PipelineState::BASE_ALIGNING, "mobile_base_align",
                    [this]() {
                        return mobile_base_->Execute(base_alignment_command_);
                    });
        return;
    }

    auto result = PollAction(PipelineState::BASE_ALIGNING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        const MobileBaseMotionReport motion_report =
            mobile_base_->LastMotionReport();
        last_base_motion_odometry_confirmed_ =
            motion_report.odometry_available &&
            motion_report.motion_confirmed;
        std::cout << "[Pipeline] Mobile base motion: odom="
                    << (motion_report.odometry_available
                        ? "available" : "unavailable")
                    << " measured=" << motion_report.signed_progress
                    << " required=" << motion_report.required_progress
                    << " detail=" << motion_report.detail << std::endl;
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::DRIVE) {
            const float commanded_travel =
                std::fabs(base_alignment_command_.linear_x) *
                static_cast<float>(base_alignment_command_.duration_ms) /
                1000.0f;
            base_align_travel_m_ += motion_report.odometry_available
                ? motion_report.translation_m
                : commanded_travel;
        }
        base_align_attempts_++;
        if (motion_report.odometry_available &&
            !motion_report.motion_confirmed) {
            base_align_attempts_ = std::max(
                base_align_attempts_,
                config_.mobile_base.max_align_attempts);
            std::cout
                << "[Pipeline] Mobile base odometry did not confirm motion; "
                << "disabling further chassis corrections for this task"
                << std::endl;
        }
        stable_count_ = 0;
        missing_count_ = 0;
        top_alignment_reference_valid_ = false;
        current_color_.release();
        current_depth_.release();
        motion_geometry_refresh_count_ = 0;
        if (!FlushCameraAfterMotion("base motion")) {
            SetState(PipelineState::ERROR,
                    "Failed to refresh camera after base motion");
            return;
        }
        SetState(PipelineState::DETECTING,
                "Base aligned, detecting target again");
    } else {
        std::string message =
            ResultMessage("Mobile base alignment failed", *result);
        const MobileBaseMotionReport motion_report =
            mobile_base_->LastMotionReport();
        if (!motion_report.detail.empty()) {
            message += "; " + motion_report.detail;
        }
        SetState(PipelineState::ERROR, message);
    }
}

}  // namespace perceptive_grasp
