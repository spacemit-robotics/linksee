/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_pipeline.cpp
    * @brief 视觉抓取主 Pipeline 实现
    */

#include "grasp_pipeline.h"
#include "voice_command_parser.h"

#ifdef MOCK_DETECTOR
#include "mock/mock_detector.h"
#endif
#ifdef MOCK_EXECUTOR
#include "mock/mock_executor.h"
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
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

namespace fs = std::filesystem;

constexpr int kMaxGeometryAttempts = 12;
constexpr int kGeometryRefreshAttempt = 6;
constexpr int kTopGeometryRecoveryAttempt = 2;
constexpr int kMotionGeometryRequiredConsistentPairs = 2;
constexpr int kMotionGeometryMaxSamples = 15;
constexpr int kMotionGeometryMaxRefreshes = 1;

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

const char* PipelineStateName(PipelineState state) {
    switch (state) {
        case PipelineState::IDLE: return "IDLE";
        case PipelineState::OBSERVING: return "OBSERVING";
        case PipelineState::DETECTING: return "DETECTING";
        case PipelineState::PLANNING: return "PLANNING";
        case PipelineState::BASE_ALIGNING: return "BASE_ALIGNING";
        case PipelineState::APPROACHING: return "APPROACHING";
        case PipelineState::GRASPING: return "GRASPING";
        case PipelineState::LIFTING: return "LIFTING";
        case PipelineState::PLACING: return "PLACING";
        case PipelineState::HOMING: return "HOMING";
        case PipelineState::RECOVERING: return "RECOVERING";
        case PipelineState::DONE: return "DONE";
        case PipelineState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
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
            if (d > 0) {
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
            if (mask_row[x] != 0 && depth_row[x] > 0) {
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

std::string TimestampString() {
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm = {};
    localtime_r(&tt, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

std::string JsonEscape(const std::string& input) {
    std::ostringstream oss;
    for (char ch : input) {
        switch (ch) {
            case '\\': oss << "\\\\"; break;
            case '"': oss << "\\\""; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default: oss << ch; break;
        }
    }
    return oss.str();
}

void WritePoseJson(std::ofstream& ofs, const char* name, const Pose3D& pose,
                    bool trailing_comma) {
    ofs << "  \"" << name << "\": {"
        << "\"x\": " << pose.x << ", "
        << "\"y\": " << pose.y << ", "
        << "\"z\": " << pose.z << ", "
        << "\"qw\": " << pose.qw << ", "
        << "\"qx\": " << pose.qx << ", "
        << "\"qy\": " << pose.qy << ", "
        << "\"qz\": " << pose.qz << "}";
    if (trailing_comma) ofs << ",";
    ofs << "\n";
}

bool IsTerminalState(PipelineState state) {
    return state == PipelineState::DONE || state == PipelineState::ERROR;
}

bool IsTaskStage(PipelineState state) {
    return state != PipelineState::IDLE && !IsTerminalState(state);
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
#ifdef MOCK_EXECUTOR
    executor_ = std::make_unique<MockExecutor>(config_.executor);
#else
    executor_ = std::make_unique<GraspExecutor>(config_.executor);
#endif
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
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    base_align_commanded_travel_m_ = 0.0f;
    task_id_.clear();
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
    top_geometry_recovery_active_ = false;
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
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    geometry_retry_count_ = 0;
    base_align_attempts_ = 0;
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    base_align_commanded_travel_m_ = 0.0f;
    task_id_.clear();
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
    top_geometry_recovery_active_ = false;
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
        !IsTerminalState(current_state)) {
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
    std::cout << "[Pipeline] Graceful shutdown requested; stopping loop after "
                "the current action and returning home"
                << std::endl;
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
    motion_geometry_confirmation_pending_ = false;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_refresh_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    base_align_commanded_travel_m_ = 0.0f;
    {
        std::lock_guard<std::mutex> lock(voice_queue_mutex_);
        waiting_voice_target_ = false;
    }
    return_to_observe_pending_ = false;
    return_to_home_pending_ = false;
    grasp_yaw_rad_ = NAN;
    grasp_opening_ = NAN;
    current_target_ = DetectionTarget{};
    grasp_pose_ = Pose3D{};
    pre_grasp_pose_ = Pose3D{};
    retreat_pose_ = Pose3D{};
    lift_pose_ = Pose3D{};
    grasp_strategy_ = GraspStrategy::TOP;
    observation_strategy_selected_ = false;
    grasp_geometry_result_ = GraspGeometryResult{};
    last_valid_geometry_ = ObjectGeometry3D{};
    last_valid_geometry_available_ = false;
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
    const bool observation_strategy_selected =
        observation_strategy_selected_;
    const GraspStrategy grasp_strategy = grasp_strategy_;
    ResetTaskState();
    target_label_ = target;
    observation_strategy_selected_ = observation_strategy_selected;
    grasp_strategy_ = grasp_strategy;
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
            ? "Loop: detecting before selecting observation pose"
            : "Loop: detecting target before observation, target: " +
                target_label_);
}

bool GraspPipeline::StartAction(PipelineState owner, const std::string& name,
                                std::function<GraspResult()> fn) {
    if (action_.active) {
        return false;
    }
    action_.active = true;
    action_.cancelling = false;
    action_.owner = owner;
    action_.name = name;
    action_.started_at = std::chrono::steady_clock::now();
    action_.started_cpu_ms = ProcessCpuMillis();
    action_.future = std::async(std::launch::async, [name, fn = std::move(fn)]() {
        try {
            return fn();
        } catch (const std::exception& e) {
            std::cerr << "[Pipeline] Async action exception (" << name
                        << "): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[Pipeline] Async action unknown exception (" << name
                        << ")" << std::endl;
        }
        return GraspResult::MOVE_FAILED;
    });
    std::ostringstream action_log;
    action_log << "[Action] START stage=" << PipelineStateName(owner)
            << " name=" << name;
    WriteStructuredLine(action_log.str());
    return true;
}

std::optional<GraspResult> GraspPipeline::PollAction(
    PipelineState owner, bool accept_any_owner) {
    if (!action_.active) return std::nullopt;
    if (!accept_any_owner && action_.owner != owner) return std::nullopt;

    if (action_.future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
        return std::nullopt;
    }

    GraspResult result = action_.future.get();
    const auto action_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - action_.started_at)
            .count();
    std::ostringstream action_log;
    action_log << "[Action] END stage=" << PipelineStateName(action_.owner)
            << " name=" << action_.name
            << " elapsed_ms=" << action_ms;
    if (config_.performance_log_enabled) {
        action_log << " cpu_ms="
                << ProcessCpuMillis() - action_.started_cpu_ms;
    }
    action_log << " result=" << GraspResultName(result);
    WriteStructuredLine(action_log.str());
    action_.active = false;
    action_.cancelling = false;
    action_.name.clear();
    action_.owner = PipelineState::IDLE;
    return result;
}

void GraspPipeline::ClearAction() {
    if (!action_.active) return;
    if (action_.future.valid()) {
        action_.future.wait();
    }
    action_.active = false;
    action_.cancelling = false;
    action_.name.clear();
    action_.owner = PipelineState::IDLE;
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
    motion_geometry_confirmation_pending_ = true;
    motion_geometry_reference_valid_ = false;
    motion_geometry_sample_count_ = 0;
    motion_geometry_consistent_count_ = 0;
    motion_geometry_reference_ = ObjectGeometry3D{};
    camera_->ResetAfterMotion();

    if (config_.camera.type == "spacemit_las2") {
        const auto start = std::chrono::steady_clock::now();
        const std::int64_t previous_frame_id = camera_->LastFrameId();
        cv::Mat color;
        cv::Mat depth;
        constexpr int kMaxRefreshAttempts = 3;
        for (int attempt = 0; attempt < kMaxRefreshAttempts; ++attempt) {
            if (!camera_->GetFrames(color, depth)) {
                std::cerr << "[Pipeline] Failed to refresh LAS2 frame after "
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
                refresh_log << "[Timing] stage=CAMERA_REFRESH reason=\""
                            << reason << "\" frame_id_before="
                            << previous_frame_id << " frame_id_after="
                            << current_frame_id << " elapsed_ms="
                            << elapsed_ms << " result=SUCCESS";
                WriteStructuredLine(refresh_log.str());
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        std::cerr << "[Pipeline] LAS2 frame did not advance after "
                << reason << ": frame_id=" << previous_frame_id
                << std::endl;
        return false;
    }

    const int count = config_.camera.realsense.motion_flush_frames;
    if (count <= 0) return true;

    const auto start = std::chrono::steady_clock::now();
    cv::Mat color;
    cv::Mat depth;
    for (int i = 0; i < count; ++i) {
        if (!camera_->GetFrames(color, depth)) {
            std::cerr << "[Pipeline] Failed to flush queued camera frame "
                        << (i + 1) << "/" << count
                        << " after " << reason << std::endl;
            return false;
        }
    }
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start)
            .count();
    std::cout << "[Pipeline] Flushed " << count
                << " queued camera frames after " << reason
                << " in " << elapsed_ms << "ms" << std::endl;
    return true;
}

bool GraspPipeline::SaveStepCameraDebug(const char* phase) {
    if (!config_.step_mode || !config_.save_debug_data) return true;

    cv::Mat color;
    cv::Mat depth;
    if (!camera_->GetFrames(color, depth)) return false;

    try {
        fs::path out_dir(config_.debug_output_dir);
        fs::create_directories(out_dir);
        if (task_id_.empty()) task_id_ = TimestampString();
        const std::string stem = "grasp_" + task_id_ + "_" + phase;
        const fs::path color_path = out_dir / (stem + ".png");
        const fs::path depth_path = out_dir / (stem + "_depth.png");
        cv::imwrite(color_path.string(), color);

        cv::Mat depth_8u;
        cv::Mat depth_color;
        depth.convertTo(depth_8u, CV_8UC1, 255.0 / 1000.0);
        cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_TURBO);
        depth_color.setTo(cv::Scalar(0, 0, 0), depth == 0);
        cv::imwrite(depth_path.string(), depth_color);
        std::cout << "[Pipeline] Step camera debug saved: "
                    << color_path << ", " << depth_path << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "[Pipeline] Failed to save step camera debug: "
                    << error.what() << std::endl;
        return false;
    }
    return true;
}

void GraspPipeline::SaveGraspDebug(float grasp_px, float grasp_py,
                                    uint16_t depth_mm,
                                    const float cam_point[3],
                                    const float base_point[3]) {
    if (!config_.save_debug_data) return;

    try {
        fs::path out_dir(config_.debug_output_dir);
        fs::create_directories(out_dir);
        if (task_id_.empty()) task_id_ = TimestampString();
        fs::path image_path = out_dir / ("grasp_" + task_id_ + ".png");
        fs::path json_path = out_dir / ("grasp_" + task_id_ + ".json");
        last_debug_image_path_ = image_path.string();
        last_debug_json_path_ = json_path.string();

        if (!current_color_.empty()) {
            cv::Mat annotated = current_color_.clone();
            cv::rectangle(
                annotated,
                cv::Point(static_cast<int>(current_target_.x1),
                            static_cast<int>(current_target_.y1)),
                cv::Point(static_cast<int>(current_target_.x2),
                            static_cast<int>(current_target_.y2)),
                cv::Scalar(0, 255, 0), 2);
            cv::circle(annotated, current_target_.center, 5,
                        cv::Scalar(255, 0, 0), -1);
            cv::circle(annotated, cv::Point2f(grasp_px, grasp_py), 6,
                        cv::Scalar(0, 0, 255), -1);
            cv::line(annotated, current_target_.center,
                    cv::Point2f(grasp_px, grasp_py),
                    cv::Scalar(0, 255, 255), 2);
            cv::putText(annotated, current_target_.label_name,
                        cv::Point(static_cast<int>(current_target_.x1),
                                    std::max(20, static_cast<int>(current_target_.y1) - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);
            cv::imwrite(image_path.string(), annotated);
        }

        std::ofstream ofs(json_path);
        ofs << "{\n";
        ofs << "  \"task_id\": \"" << JsonEscape(task_id_) << "\",\n";
        ofs << "  \"target\": \"" << JsonEscape(current_target_.label_name)
            << "\",\n";
        ofs << "  \"target_requested\": \"" << JsonEscape(target_label_)
            << "\",\n";
        ofs << "  \"score\": " << current_target_.score << ",\n";
        ofs << "  \"bbox\": [" << current_target_.x1 << ", "
            << current_target_.y1 << ", " << current_target_.x2 << ", "
            << current_target_.y2 << "],\n";
        ofs << "  \"pixel_center\": [" << current_target_.center.x << ", "
            << current_target_.center.y << "],\n";
        ofs << "  \"pixel_grasp\": [" << grasp_px << ", " << grasp_py << "],\n";
        ofs << "  \"depth_mm\": ";
        if (depth_mm != 0) {
            ofs << depth_mm;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "  \"camera_point_m\": ";
        if (depth_mm != 0) {
            ofs << "[" << cam_point[0] << ", "
                << cam_point[1] << ", " << cam_point[2] << "]";
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "  \"base_point_m\": [" << base_point[0] << ", "
            << base_point[1] << ", " << base_point[2] << "],\n";
        ofs << "  \"grasp_strategy\": \""
            << GraspStrategyName(grasp_strategy_) << "\",\n";
        ofs << "  \"object_dimensions_m\": ["
            << grasp_geometry_result_.geometry.length_m << ", "
            << grasp_geometry_result_.geometry.width_m << ", "
            << grasp_geometry_result_.geometry.height_m << "],\n";
        ofs << "  \"geometry_elapsed_ms\": "
            << grasp_geometry_result_.elapsed_ms << ",\n";
        ofs << "  \"grasp_yaw_rad\": ";
        if (std::isfinite(grasp_yaw_rad_)) {
            ofs << grasp_yaw_rad_;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "  \"candidates\": \"" << JsonEscape(FormatCandidates())
            << "\",\n";
        WritePoseJson(ofs, "pre_grasp_pose", pre_grasp_pose_, true);
        WritePoseJson(ofs, "grasp_pose", grasp_pose_, true);
        WritePoseJson(ofs, "retreat_pose", retreat_pose_, true);
        WritePoseJson(ofs, "lift_pose", lift_pose_, false);
        ofs << "}\n";

        std::cout << "[Pipeline] Debug saved: " << image_path
                    << ", " << json_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Pipeline] Failed to save debug data: "
                    << e.what() << std::endl;
    }
}

void GraspPipeline::SaveTaskResultDebug(PipelineState terminal_state,
                                        const std::string& message) {
    if (!config_.save_debug_data) return;
    if (task_id_.empty() && current_color_.empty() && current_depth_.empty() &&
        target_label_.empty()) {
        return;
    }

    try {
        fs::path out_dir(config_.debug_output_dir);
        fs::create_directories(out_dir);
        if (task_id_.empty()) task_id_ = TimestampString();

        fs::path result_path = out_dir / ("grasp_" + task_id_ + "_result.json");
        ExecutorDiagnostics diag;
        if (executor_) diag = executor_->GetDiagnostics();

        std::ofstream ofs(result_path);
        ofs << "{\n";
        ofs << "  \"task_id\": \"" << JsonEscape(task_id_) << "\",\n";
        ofs << "  \"terminal_state\": \""
            << PipelineStateName(terminal_state) << "\",\n";
        ofs << "  \"message\": \"" << JsonEscape(message) << "\",\n";
        ofs << "  \"target_requested\": \"" << JsonEscape(target_label_)
            << "\",\n";
        ofs << "  \"target_detected\": \""
            << JsonEscape(current_target_.label_name) << "\",\n";
        ofs << "  \"candidates\": \"" << JsonEscape(FormatCandidates())
            << "\",\n";
        ofs << "  \"debug_image\": \"" << JsonEscape(last_debug_image_path_)
            << "\",\n";
        ofs << "  \"debug_plan_json\": \"" << JsonEscape(last_debug_json_path_)
            << "\",\n";
        ofs << "  \"last_executor_result\": \""
            << GraspResultName(diag.last_result) << "\",\n";
        ofs << "  \"last_executor_action\": \""
            << JsonEscape(diag.last_action) << "\",\n";
        ofs << "  \"last_executor_detail\": \""
            << JsonEscape(diag.last_detail) << "\",\n";
        ofs << "  \"gripper_check\": {\n";
        ofs << "    \"phase\": \"" << JsonEscape(diag.gripper_check.phase)
            << "\",\n";
        ofs << "    \"state\": \"" << JsonEscape(diag.gripper_check.state)
            << "\",\n";
        ofs << "    \"holding_count\": "
            << diag.gripper_check.holding_count << ",\n";
        ofs << "    \"load_holding_count\": "
            << diag.gripper_check.load_holding_count << ",\n";
        ofs << "    \"check_count\": " << diag.gripper_check.check_count
            << ",\n";
        ofs << "    \"load_threshold\": "
            << diag.gripper_check.load_threshold << ",\n";
        ofs << "    \"empty_closed_position\": ";
        if (std::isfinite(diag.gripper_check.empty_closed_position)) {
            ofs << diag.gripper_check.empty_closed_position;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"min_object_position\": ";
        if (std::isfinite(diag.gripper_check.min_object_position)) {
            ofs << diag.gripper_check.min_object_position;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"position\": ";
        if (std::isfinite(diag.gripper_check.position)) {
            ofs << diag.gripper_check.position;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"load\": ";
        if (std::isfinite(diag.gripper_check.load)) {
            ofs << diag.gripper_check.load;
        } else {
            ofs << "null";
        }
        ofs << "\n";
        ofs << "  },\n";
        ofs << "  \"wrist_yaw\": {\n";
        ofs << "    \"valid\": " << (diag.wrist_yaw.valid ? "true" : "false")
            << ",\n";
        ofs << "    \"target_yaw_rad\": ";
        if (std::isfinite(diag.wrist_yaw.target_yaw)) {
            ofs << diag.wrist_yaw.target_yaw;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"target_yaw_deg\": ";
        if (std::isfinite(diag.wrist_yaw.target_yaw)) {
            ofs << diag.wrist_yaw.target_yaw * 180.0f / M_PI;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"joint0\": ";
        if (std::isfinite(diag.wrist_yaw.joint0)) {
            ofs << diag.wrist_yaw.joint0;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"scale\": ";
        if (std::isfinite(diag.wrist_yaw.scale)) {
            ofs << diag.wrist_yaw.scale;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"joint5_raw\": ";
        if (std::isfinite(diag.wrist_yaw.joint5_raw)) {
            ofs << diag.wrist_yaw.joint5_raw;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"joint5_limited\": ";
        if (std::isfinite(diag.wrist_yaw.joint5_limited)) {
            ofs << diag.wrist_yaw.joint5_limited;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"joint5_min\": ";
        if (std::isfinite(diag.wrist_yaw.joint5_min)) {
            ofs << diag.wrist_yaw.joint5_min;
        } else {
            ofs << "null";
        }
        ofs << ",\n";
        ofs << "    \"joint5_max\": ";
        if (std::isfinite(diag.wrist_yaw.joint5_max)) {
            ofs << diag.wrist_yaw.joint5_max;
        } else {
            ofs << "null";
        }
        ofs << "\n";
        ofs << "  }\n";
        ofs << "}\n";

        std::cout << "[Pipeline] Task result debug saved: "
                    << result_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Pipeline] Failed to save task result debug: "
                    << e.what() << std::endl;
    }
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
        const bool return_home =
            graceful_shutdown || object_may_be_held_.load();
        SetState(
            PipelineState::IDLE,
            graceful_shutdown
                ? "Shutdown requested; waiting for current action"
                : (return_home
                    ? "Cancelling while gripper may hold an object; "
                        "returning home"
                    : "Cancelling; keeping observe pose"));
        std::cout << std::flush;
        if (action_.active) {
            action_.cancelling = true;
            if (return_home) {
                return_to_home_pending_ = true;
            } else {
                return_to_observe_pending_ = true;
            }
        } else {
            SaveTaskResultDebug(
                PipelineState::IDLE,
                graceful_shutdown ? "Graceful shutdown requested"
                                    : "Cancelled");
            ResetTaskState();
            if (return_home) {
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
            SaveTaskResultDebug(PipelineState::IDLE,
                                "Cancelled; active action finished with " +
                                    std::string(GraspResultName(*result)));
            ResetTaskState();
            if (was_cancelling) {
                if (graceful_shutdown_requested_.load() ||
                    object_may_be_held_.load()) {
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
    if (return_to_observe_pending_) {
        return_to_observe_pending_ = false;
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
        SetState(PipelineState::IDLE, "Returning home; exiting after home");
        StartAction(PipelineState::IDLE, "return_to_home_on_command",
                    [this]() {
                        return executor_->MoveToHome();
                    });
        return;
    }
    ConsumeVoiceCommand();
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

    DetectionTarget target;
    bool found = false;
    std::vector<DetectionTarget> targets;

    if (target_label_.empty()) {
        found = detector_->Detect(color, targets) && !targets.empty();
        if (found) target = targets[0];
    } else {
        if (detector_->Detect(color, targets)) {
            for (const auto& candidate : targets) {
                if (candidate.label_name == target_label_) {
                    target = candidate;
                    found = true;
                    break;
                }
            }
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
        stable_count_ = 0;
        if (!target_label_.empty()) {
            missing_count_++;
            std::cout << "[Pipeline] Target not detected: " << target_label_
                        << " (" << missing_count_ << "/"
                        << config_.target_missing_frames << ")"
                        << ", candidates: " << FormatCandidates()
                        << std::endl;
            if (missing_count_ >= config_.target_missing_frames) {
                if (config_.auto_loop) {
                    missing_count_ = 0;
                    std::cout
                        << "[Loop] Waiting for next " << target_label_
                        << "; staying in DETECTING without returning home"
                        << std::endl;
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

    // 稳定性检查: 连续多帧检测到才执行
    stable_count_++;
    if (stable_count_ < config_.detect_stable_frames) {
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
    std::string& error) {
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
    constexpr int kDepthRoiSize = 5;
    size_t mask_depth_samples = 0;
    if (ForegroundDepthFromMask(
            current_depth_, current_target_.mask, depth_mm,
            mask_depth_samples)) {
        std::cout << "[Pipeline] Top-grasp depth "
                    "source=mask_foreground_q25 value="
                    << depth_mm << "mm samples=" << mask_depth_samples
                    << std::endl;
    } else if (!MedianDepthAtPixel(
                    current_depth_, cx, cy, kDepthRoiSize, depth_mm)) {
        cx = ClampPixel(
            static_cast<int>(std::lround(current_target_.center.x)),
            current_depth_.cols);
        cy = ClampPixel(
            static_cast<int>(std::lround(current_target_.center.y)),
            current_depth_.rows);
        if (!MedianDepthAtPixel(
                current_depth_, cx, cy, kDepthRoiSize, depth_mm)) {
            error = "top-grasp depth is invalid at grasp pixel and center";
            return false;
        }
        std::cout << "[Pipeline] Top-grasp depth "
                    "source=target_center_roi value="
                    << depth_mm << "mm" << std::endl;
    } else {
        std::cout << "[Pipeline] Top-grasp depth "
                    "source=grasp_pixel_roi value="
                    << depth_mm << "mm" << std::endl;
    }

    if (!camera_->Deproject(cx, cy, depth_mm, cam_point)) {
        error = "top-grasp camera deprojection failed";
        return false;
    }
    planner_->CameraToBase(cam_point, base_point);
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
    candidate.lift_pose = pre_grasp_pose;
    candidate.grasp_yaw_rad = grasp_yaw;
    candidate.geometry_valid = true;
    candidate.rejection_reason.clear();
    std::cout << "[Pipeline] Top-grasp plan: target=["
                << grasp_pose.x << "," << grasp_pose.y << ","
                << grasp_pose.z << "] yaw=" << grasp_yaw << std::endl;
    error.clear();
    return true;
}

void GraspPipeline::HandleTopPlanning() {
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_cpu_start = ProcessCpuMillis();

    GraspCandidate candidate;
    float grasp_px = current_target_.center.x;
    float grasp_py = current_target_.center.y;
    uint16_t depth_mm = 0;
    float cam_point[3] = {};
    float base_point[3] = {};
    std::string error;
    if (!BuildMaskTopGrasp(
            candidate, grasp_px, grasp_py, depth_mm,
            cam_point, base_point, error)) {
        SetState(PipelineState::ERROR,
                "Top-grasp planning failed: " + error);
        return;
    }

    std::cout << "[Pipeline] 3D position (base): [" << base_point[0] << ", "
                << base_point[1] << ", " << base_point[2] << "]" << std::endl;

    float alignment_point[3] = {
        base_point[0], base_point[1], base_point[2]};
    const int center_x = ClampPixel(
        static_cast<int>(std::lround(current_target_.center.x)),
        current_depth_.cols);
    const int center_y = ClampPixel(
        static_cast<int>(std::lround(current_target_.center.y)),
        current_depth_.rows);
    uint16_t center_depth_mm = 0;
    constexpr int kCenterDepthRoiSize = 5;
    if (MedianDepthAtPixel(
            current_depth_, center_x, center_y,
            kCenterDepthRoiSize, center_depth_mm)) {
        float center_cam_point[3] = {};
        if (camera_->Deproject(
                center_x, center_y, center_depth_mm,
                center_cam_point)) {
            planner_->CameraToBase(center_cam_point, alignment_point);
        }
    }
    std::cout << "[Pipeline] Mobile base alignment target center: ["
                << alignment_point[0] << ", " << alignment_point[1] << ", "
                << alignment_point[2] << "]" << std::endl;

    base_alignment_command_ = PlanMobileBaseAlignment(
        config_.mobile_base, alignment_point, base_align_attempts_);

    if (have_previous_base_alignment_point_) {
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
        if (still_needs_alignment && progress < required_progress) {
            stable_count_ = 0;
            std::ostringstream message;
            message << "Base alignment stopped: visual progress "
                    << progress << "m below required " << required_progress
                    << "m after chassis motion; check depth and motion "
                        "direction";
            SetState(PipelineState::ERROR, message.str());
            return;
        }
        have_previous_base_alignment_point_ = false;
    }

    if (base_alignment_command_.type !=
        MobileBaseAlignmentCommand::Type::NONE) {
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::DRIVE) {
            const float next_travel =
                std::fabs(base_alignment_command_.linear_x) *
                static_cast<float>(base_alignment_command_.duration_ms) /
                1000.0f;
            if (base_align_commanded_travel_m_ + next_travel >
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
        stable_count_ = 0;
        SetState(
            PipelineState::ERROR,
            "Base alignment failed: max attempts reached while target "
            "remains outside the comfortable range");
        return;
    }
    if (config_.mobile_base.enabled) {
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
        std::string validation_detail;
        const GraspResult validation_result = executor_->ValidateGraspPoses(
            candidate.pre_grasp_pose, candidate.grasp_pose,
            candidate.retreat_pose, candidate.lift_pose,
            candidate.entry_clearance_z_m,
            candidate.grasp_yaw_rad, true,
            config_.geometry.planning_timeout_ms, &validation_detail);
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

void GraspPipeline::HandlePlanning() {
    if (current_depth_.empty()) {
        stable_count_ = 0;
        SetState(PipelineState::DETECTING, "No cached depth, detecting again");
        return;
    }

    const bool explicit_top_strategy =
        config_.geometry.strategy == "top";
    const bool auto_selects_top =
        config_.geometry.strategy == "auto" &&
        current_target_.label_name != "cup" &&
        current_target_.label_name != "bottle";
    if (!observation_strategy_selected_ &&
        (explicit_top_strategy || auto_selects_top)) {
        grasp_strategy_ = GraspStrategy::TOP;
        observation_strategy_selected_ = true;
        std::cout << "[Pipeline] Top-grasp strategy selected from target "
                    "class before arm motion: "
                    << current_target_.label_name << std::endl;
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
        grasp_geometry_result_);
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
    geometry_retry_count_ = 0;

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
                SetState(PipelineState::ERROR,
                    "3D geometry remained unstable after robot motion: " +
                        consistency_detail);
                return;
            }
            std::ostringstream confirmation_message;
            confirmation_message << "Confirming 3D geometry after motion "
                                << motion_geometry_sample_count_ << "/"
                                << kMotionGeometryMaxSamples;
            if (!consistency_detail.empty()) {
                confirmation_message << "; "
                                    << (consistent ? "stable" : "changed");
            }
            SetState(PipelineState::DETECTING,
                    confirmation_message.str());
            return;
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

    if (geometry_from_point_cloud) {
        last_valid_geometry_ = grasp_geometry_result_.geometry;
        last_valid_geometry_available_ = true;
    }

    SupportPlane support_plane;
    support_plane.normal_x = grasp_geometry_result_.geometry.table.normal.x;
    support_plane.normal_y = grasp_geometry_result_.geometry.table.normal.y;
    support_plane.normal_z = grasp_geometry_result_.geometry.table.normal.z;
    support_plane.d = grasp_geometry_result_.geometry.table.d;
    support_plane.valid = true;
    support_plane.min_x = grasp_geometry_result_.geometry.table.min_x;
    support_plane.max_x = grasp_geometry_result_.geometry.table.max_x;
    support_plane.min_y = grasp_geometry_result_.geometry.table.min_y;
    support_plane.max_y = grasp_geometry_result_.geometry.table.max_y;
    support_plane.bounds_valid =
        grasp_geometry_result_.geometry.table.bounds_valid;
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
        SetState(PipelineState::ERROR, message.str());
        return;
    }
    if (!observation_strategy_selected_) {
        grasp_strategy_ = preferred_candidate->strategy;
        observation_strategy_selected_ = true;
        std::cout << "[Pipeline] Observation strategy selected before arm "
                    "motion: "
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
        preferred_candidate->strategy == GraspStrategy::TOP) {
        std::string top_error;
        if (!BuildMaskTopGrasp(
                *preferred_candidate, debug_grasp_px, debug_grasp_py,
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
            alignment_config.x_tolerance = std::min(
                alignment_config.x_tolerance,
                kSidePreGraspUpperHysteresisM);
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

    if (have_previous_base_alignment_point_) {
        const float progress = MeasureMobileBaseAlignmentProgress(
            previous_base_alignment_point_.data(), alignment_point,
            previous_base_alignment_command_);
        const float required_progress =
            RequiredMobileBaseAlignmentProgress(
                config_.mobile_base,
                previous_base_alignment_point_.data(),
                previous_base_alignment_command_);
        std::cout << "[Pipeline] Mobile base visual progress: "
                    << progress << "m (required >= "
                    << required_progress << "m)"
                    << std::endl;
        const bool still_needs_alignment =
            base_alignment_command_.type !=
                MobileBaseAlignmentCommand::Type::NONE ||
            base_alignment_command_.max_attempts_reached;
        const float maximum_regression =
            std::max(0.0f,
                    config_.mobile_base.max_visual_regression_m);
        if (still_needs_alignment &&
            progress < -maximum_regression) {
            stable_count_ = 0;
            std::ostringstream message;
            message << "Base alignment stopped: visual progress "
                    << progress << "m exceeded allowed regression "
                    << maximum_regression
                    << "m after chassis motion; check depth and motion "
                        "direction";
            SetState(PipelineState::ERROR, message.str());
            return;
        }
        if (still_needs_alignment && progress < required_progress) {
            std::cout << "[Pipeline] Mobile base made limited but valid "
                        "progress; continuing closed-loop alignment"
                        << std::endl;
        }
        have_previous_base_alignment_point_ = false;
    }

    if (base_alignment_command_.type !=
        MobileBaseAlignmentCommand::Type::NONE) {
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::DRIVE) {
            const float next_travel =
                std::fabs(base_alignment_command_.linear_x) *
                static_cast<float>(base_alignment_command_.duration_ms) /
                1000.0f;
            if (base_align_commanded_travel_m_ + next_travel >
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
        stable_count_ = 0;
        SetState(PipelineState::ERROR,
            "Base alignment failed: max attempts reached while target "
            "remains outside the comfortable range");
        return;
    }
    if (config_.mobile_base.enabled) {
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

    const auto planning_deadline = planning_start + std::chrono::milliseconds(
        config_.geometry.planning_timeout_ms);
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

        const auto now = std::chrono::steady_clock::now();
        if (now >= planning_deadline) {
            candidate.rejection_reason = "planning deadline exceeded";
            break;
        }
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                planning_deadline - now)
                .count());
        std::string ik_detail;
        const bool use_top_constraints =
            candidate.strategy == GraspStrategy::TOP;
        const GraspResult ik_result = executor_->ValidateGraspPoses(
            candidate.pre_grasp_pose, candidate.grasp_pose,
            candidate.retreat_pose, candidate.lift_pose,
            candidate.entry_clearance_z_m,
            candidate.grasp_yaw_rad,
            use_top_constraints, remaining_ms, &ik_detail);
        if (ik_result == GraspResult::SUCCESS) {
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
                << (within_perception_budget ? "SUCCESS" : "TIMEOUT")
                << std::endl;
    perception_cycle_active_ = false;
    if (!within_perception_budget) {
        SetState(PipelineState::ERROR,
                "Perception and planning exceeded the configured budget");
        return;
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

void GraspPipeline::HandleBaseAligning() {
    if (!mobile_base_) {
        SetState(PipelineState::ERROR, "Mobile base controller not initialized");
        return;
    }
    if (!action_.active) {
        if (!WaitForConfirm("即将移动底盘对齐目标")) return;
        StartAction(PipelineState::BASE_ALIGNING, "mobile_base_align",
                    [this]() {
                        return mobile_base_->Execute(base_alignment_command_);
                    });
        return;
    }

    auto result = PollAction(PipelineState::BASE_ALIGNING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        if (base_alignment_command_.type ==
            MobileBaseAlignmentCommand::Type::DRIVE) {
            base_align_commanded_travel_m_ +=
                std::fabs(base_alignment_command_.linear_x) *
                static_cast<float>(base_alignment_command_.duration_ms) /
                1000.0f;
        }
        base_align_attempts_++;
        stable_count_ = 0;
        missing_count_ = 0;
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
        SetState(PipelineState::ERROR,
                ResultMessage("Mobile base alignment failed", *result));
    }
}

void GraspPipeline::HandleApproaching() {
    if (!action_.active) {
        const char* prompt = grasp_strategy_ == GraspStrategy::SIDE
            ? "即将移动到目标上方安全预抓取位 (safe_pre_grasp)"
            : "即将移动到预抓取位 (pre_grasp)";
        if (!WaitForConfirm(prompt)) return;
        StartAction(PipelineState::APPROACHING, "move_to_pre_grasp", [this]() {
            return executor_->MoveToPreGrasp(
                pre_grasp_pose_, grasp_yaw_rad_,
                grasp_strategy_ == GraspStrategy::TOP);
        });
        return;
    }

    auto result = PollAction(PipelineState::APPROACHING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        const char* debug_name = grasp_strategy_ == GraspStrategy::SIDE
            ? "safe_pre_grasp"
            : "pre_grasp";
        if (config_.step_mode &&
            (!FlushCameraAfterMotion("pre-grasp motion") ||
            !SaveStepCameraDebug(debug_name))) {
            SetState(PipelineState::ERROR,
                    "Failed to capture pre-grasp camera verification");
            return;
        }
        if (grasp_strategy_ == GraspStrategy::SIDE) {
            const float dx = grasp_pose_.x - pre_grasp_pose_.x;
            const float dy = grasp_pose_.y - pre_grasp_pose_.y;
            const float dz = grasp_pose_.z - pre_grasp_pose_.z;
            std::ostringstream message;
            message << "At safe pre-grasp above target; open gripper, "
                    << "descend, then advance distance="
                    << std::sqrt(dx * dx + dy * dy + dz * dz)
                    << "m dx=" << dx
                    << "m dy=" << dy
                    << "m dz=" << dz << "m";
            SetState(PipelineState::GRASPING, message.str());
        } else {
            SetState(
                PipelineState::GRASPING,
                "At pre-grasp, executing grasp...");
        }
    } else {
        if (RetryRecoverableMotion("Pre-grasp motion", *result)) return;
        SetState(PipelineState::ERROR,
                ResultMessage("Pre-grasp move failed", *result));
    }
}

void GraspPipeline::HandleGrasping() {
    if (!action_.active) {
        if (!WaitForConfirm(
                "即将张开夹爪、下降、水平进给并闭合抓取")) return;
        StartAction(PipelineState::GRASPING, "open_move_close_gripper", [this]() {
            auto result = executor_->OpenGripperForGrasp(grasp_opening_);
            if (result != GraspResult::SUCCESS) return result;
            object_may_be_held_.store(false);
            result = executor_->MoveToGrasp(
                grasp_pose_, grasp_yaw_rad_,
                grasp_strategy_ == GraspStrategy::TOP);
            if (result != GraspResult::SUCCESS) return result;
            if (config_.step_mode &&
                (!FlushCameraAfterMotion("grasp motion") ||
                !SaveStepCameraDebug("grasp_pose"))) {
                return GraspResult::MOVE_FAILED;
            }
            // Once closing starts, retain a conservative held-object state
            // until the executor explicitly reports EMPTY or release succeeds.
            object_may_be_held_.store(true);
            return executor_->CloseGripperAndCheck();
        });
        return;
    }

    auto result = PollAction(PipelineState::GRASPING);
    if (!result.has_value()) return;

    switch (*result) {
        case GraspResult::SUCCESS:
            object_may_be_held_.store(true);
            SetState(PipelineState::LIFTING, "Object held, lifting...");
            break;

        case GraspResult::EMPTY:
            object_may_be_held_.store(false);
            retry_count_++;
            if (retry_count_ < config_.max_retries) {
                std::cout << "[Pipeline] Retry " << retry_count_ << "/"
                            << config_.max_retries << std::endl;
                stable_count_ = 0;
                // 回到观察位再重新检测:
                // 1. 臂收起避免遮挡前视立体相机
                // 2. 物体可能被碰移位，需要重新定位
                SetState(PipelineState::OBSERVING,
                        "Retry: retracting arm for re-detection");
            } else {
                SetState(PipelineState::ERROR,
                        "Grasp empty; max retries reached");
            }
            break;

        case GraspResult::IK_FAILED:
        case GraspResult::OUT_OF_RANGE:
            if (RetryRecoverableMotion("Grasp motion", *result)) break;
            SetState(PipelineState::ERROR,
                    ResultMessage("Grasp move failed", *result));
            break;

        case GraspResult::TIMEOUT:
            SetState(PipelineState::ERROR,
                    ResultMessage("Gripper close timeout", *result));
            break;

        case GraspResult::MOVE_FAILED:
            SetState(PipelineState::ERROR,
                    ResultMessage("Gripper close failed", *result));
            break;

        default:
            SetState(PipelineState::ERROR,
                    ResultMessage("Grasp failed", *result));
            break;
    }
}

void GraspPipeline::HandleLifting() {
    if (!action_.active) {
        if (!WaitForConfirm("抓取成功，即将抬起到预抓取位")) return;
        StartAction(PipelineState::LIFTING, "lift_from_grasp", [this]() {
            return executor_->LiftFromGrasp(
                retreat_pose_, lift_pose_, grasp_yaw_rad_,
                grasp_strategy_ == GraspStrategy::TOP);
        });
        return;
    }

    auto result = PollAction(PipelineState::LIFTING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        SetState(PipelineState::PLACING,
                "Lift completed and object holding confirmed; placing...");
    } else if (*result == GraspResult::EMPTY) {
        object_may_be_held_.store(false);
        retry_count_++;
        if (retry_count_ < config_.max_retries) {
            stable_count_ = 0;
            SetState(PipelineState::OBSERVING,
                    "Object lost after lift; retracting for re-detection");
        } else {
            SetState(PipelineState::ERROR,
                    "Object not held after lift; max retries reached");
        }
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Lift failed", *result));
    }
}

void GraspPipeline::HandlePlacing() {
    if (!action_.active) {
        if (!WaitForConfirm("抓取成功，即将移动到放置位")) return;
        StartAction(PipelineState::PLACING, "place_and_release", [this]() {
            auto result = executor_->MoveToPlace();
            if (result == GraspResult::SUCCESS) {
                result = executor_->ReleaseObject();
                if (result == GraspResult::SUCCESS) {
                    object_may_be_held_.store(false);
                }
            }
            if (result == GraspResult::SUCCESS) {
                result = executor_->CloseGripper();
            }
            return result;
        });
        return;
    }

    auto result = PollAction(PipelineState::PLACING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        const bool return_to_observe =
            config_.voice.enabled || config_.auto_loop;
        const char* target_pose =
            return_to_observe ? "observe position" : "home position";
        SetState(PipelineState::HOMING,
                std::string("Object released, returning to ") +
                    target_pose + "...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Place failed", *result));
    }
}

void GraspPipeline::HandleHoming() {
    const bool return_to_observe =
        config_.voice.enabled || config_.auto_loop;
    if (!action_.active) {
        if (!WaitForConfirm(return_to_observe ? "即将回到观察位"
                                            : "即将回到 Home 位")) {
            return;
        }
        const std::string action_name = return_to_observe
            ? "move_to_observe_after_place"
            : "move_to_home_after_place";
        StartAction(PipelineState::HOMING, action_name,
                    [this, return_to_observe]() {
            if (!return_to_observe) {
                return executor_->MoveToHome();
            }
            if (observation_strategy_selected_ &&
                grasp_strategy_ == GraspStrategy::SIDE) {
                return executor_->MoveToSideObserve();
            }
            return executor_->MoveToObserve();
        });
        return;
    }

    auto result = PollAction(PipelineState::HOMING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        SetState(PipelineState::DONE, "Task completed!");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Final move failed", *result));
    }
}

void GraspPipeline::HandleRecovering() {
    const bool carrying_object = object_may_be_held_.load();
    const bool return_to_observe = config_.auto_loop && !carrying_object;
    const bool use_side_observation =
        observation_strategy_selected_ &&
        grasp_strategy_ == GraspStrategy::SIDE;
    const char* recovery_target =
        return_to_observe ? "observation position" : "home position";
    if (!action_.active) {
        const char* action_name = return_to_observe
            ? "return_observe_after_failure"
            : "return_home_after_failure";
        if (!StartAction(
                PipelineState::RECOVERING, action_name,
                [this, return_to_observe, use_side_observation]() {
                    if (!return_to_observe) {
                        return executor_->MoveToHome();
                    }
                    return use_side_observation
                        ? executor_->MoveToSideObserve()
                        : executor_->MoveToObserve();
                })) {
            failure_recovery_succeeded_ = false;
            const std::string message =
                pending_failure_message_ +
                "; failed to start recovery action for " +
                recovery_target;
            if (return_to_observe) {
                observation_strategy_selected_ = false;
            }
            if (carrying_object) {
                shutdown_requested_.store(true);
            }
            SetState(PipelineState::ERROR, message);
            failure_recovery_active_ = false;
            pending_failure_message_.clear();
        }
        return;
    }

    auto result = PollAction(PipelineState::RECOVERING);
    if (!result.has_value()) return;

    failure_recovery_succeeded_ = *result == GraspResult::SUCCESS;
    std::string message = pending_failure_message_;
    if (failure_recovery_succeeded_) {
        message += "; returned to ";
        message += recovery_target;
    } else {
        if (return_to_observe) {
            observation_strategy_selected_ = false;
        }
        message += "; " +
            ResultMessage(
                return_to_observe
                    ? "Observation recovery failed"
                    : "Home recovery failed",
                *result);
    }

    if (carrying_object) {
        message +=
            "; automatic restart disabled because the gripper may hold an "
            "object";
        shutdown_requested_.store(true);
    }
    SetState(PipelineState::ERROR, message);
    failure_recovery_active_ = false;
    failure_recovery_succeeded_ = false;
    pending_failure_message_.clear();
}

void GraspPipeline::BeginTaskTiming() {
    failure_recovery_active_ = false;
    failure_recovery_succeeded_ = false;
    pending_failure_message_.clear();
    task_timing_active_ = true;
    stage_timing_active_ = false;
    stage_sequence_ = 0;
    stage_timings_.clear();
    task_started_at_ = std::chrono::steady_clock::now();
}

void GraspPipeline::PrintTaskSummary(PipelineState terminal_state,
                                    const std::string& message) {
    const auto total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - task_started_at_)
            .count();
    const char* result = terminal_state == PipelineState::DONE
        ? "SUCCESS"
        : (terminal_state == PipelineState::ERROR ? "FAILED" : "CANCELLED");

    std::cout << "\n========== PIPELINE SUMMARY ==========" << std::endl;
    std::cout << "result=" << result
            << " target=" << (target_label_.empty() ? "auto" : target_label_)
            << " initialization_ms=" << initialization_elapsed_ms_
            << " task_ms=" << total_ms
            << " end_to_end_ms=" << initialization_elapsed_ms_ + total_ms
            << " base_align_attempts=" << base_align_attempts_
            << std::endl;
    for (const auto& timing : stage_timings_) {
        std::cout << "  [" << std::setw(2) << std::setfill('0')
                << timing.sequence << std::setfill(' ') << "] "
                << PipelineStateName(timing.state)
                << " elapsed_ms=" << timing.elapsed_ms
                << " result=" << timing.result << std::endl;
    }
    if (!message.empty()) {
        std::cout << "message=" << message << std::endl;
    }
    std::cout << "======================================" << std::endl;

    task_timing_active_ = false;
    stage_timing_active_ = false;
}

void GraspPipeline::SetState(PipelineState new_state,
                            const std::string& msg) {
    const PipelineState requested_state = new_state;
    const PipelineState previous_state = state_.load();
    std::string state_message = msg;
    const bool start_failure_recovery =
        requested_state == PipelineState::ERROR &&
        task_timing_active_ && executor_ && !config_.plan_only &&
        !failure_recovery_active_;
    if (start_failure_recovery) {
        pending_failure_message_ =
            msg.empty() ? last_status_message_ : msg;
        failure_recovery_active_ = true;
        failure_recovery_succeeded_ = false;
        new_state = PipelineState::RECOVERING;
        state_message =
            config_.auto_loop && !object_may_be_held_.load()
            ? "Task failed; returning to observation position"
            : "Task failed; returning to home position";
    }

    if (!state_message.empty()) {
        last_status_message_ = state_message;
    }

    if (task_timing_active_ && stage_timing_active_ &&
        IsTaskStage(previous_state) && previous_state != new_state) {
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - stage_started_at_)
                .count();
        const bool recovery_completed =
            previous_state == PipelineState::RECOVERING &&
            requested_state == PipelineState::ERROR &&
            failure_recovery_succeeded_;
        const char* result = recovery_completed
            ? "SUCCESS"
            : (requested_state == PipelineState::ERROR
                ? "FAILED"
                : (new_state == PipelineState::IDLE
                    ? "CANCELLED"
                    : "SUCCESS"));
        stage_timings_.push_back({
            stage_sequence_, previous_state, elapsed_ms, result,
        });
        std::ostringstream stage_log;
        stage_log << "[Stage " << stage_sequence_ << "] END   "
                << PipelineStateName(previous_state)
                << " elapsed_ms=" << elapsed_ms
                << " result=" << result;
        WriteStructuredLine(stage_log.str());
        stage_timing_active_ = false;
    }

    state_.store(new_state);

    if (task_timing_active_ && IsTaskStage(new_state) &&
        previous_state != new_state) {
        stage_sequence_++;
        stage_started_at_ = std::chrono::steady_clock::now();
        stage_timing_active_ = true;
        std::ostringstream stage_log;
        stage_log << "\n[Stage " << stage_sequence_ << "] START "
                << PipelineStateName(new_state);
        if (!state_message.empty()) {
            stage_log << " | " << state_message;
        }
        WriteStructuredLine(stage_log.str());
    } else if (!task_timing_active_ && !state_message.empty()) {
        std::ostringstream pipeline_log;
        pipeline_log << "[Pipeline] " << PipelineStateName(new_state)
                    << " | " << state_message;
        WriteStructuredLine(pipeline_log.str());
    }

    if (IsTerminalState(new_state)) {
        SaveTaskResultDebug(
            new_state,
            state_message.empty() ? last_status_message_ : state_message);
    }

    if (task_timing_active_ &&
        (IsTerminalState(new_state) || new_state == PipelineState::IDLE)) {
        PrintTaskSummary(
            new_state,
            state_message.empty() ? last_status_message_ : state_message);
    }

    // 通知回调
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
        callback_(new_state, state_message);
    }
}

}  // namespace perceptive_grasp
