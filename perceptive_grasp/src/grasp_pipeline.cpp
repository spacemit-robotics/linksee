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

    // The lower quartile favors the object surface over the farther floor,
    // while remaining robust against isolated near-depth outliers.
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
    target_label_.clear();
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    base_align_attempts_ = 0;
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    base_align_commanded_travel_m_ = 0.0f;
    task_id_.clear();
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
    BeginTaskTiming();
    SetState(PipelineState::OBSERVING, "Moving to observe position");
    return true;
}

bool GraspPipeline::TriggerGrasp(const std::string& target_label) {
    if (state_.load() != PipelineState::IDLE || HasActiveAction()) {
        std::cerr << "[Pipeline] Cannot trigger: not idle" << std::endl;
        return false;
    }
    target_label_ = target_label;
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    base_align_attempts_ = 0;
    have_previous_base_alignment_point_ = false;
    previous_base_alignment_command_ = MobileBaseAlignmentCommand{};
    base_align_commanded_travel_m_ = 0.0f;
    task_id_.clear();
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
    BeginTaskTiming();
    SetState(PipelineState::OBSERVING,
            "Moving to observe, target: " + target_label);
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
        std::cerr << "[Voice] No trigger word matched: "
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

void GraspPipeline::SetCallback(PipelineCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
}

void GraspPipeline::ResetTaskState() {
    target_label_.clear();
    retry_count_ = 0;
    stable_count_ = 0;
    missing_count_ = 0;
    base_align_attempts_ = 0;
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
    current_target_ = DetectionTarget{};
    grasp_pose_ = Pose3D{};
    pre_grasp_pose_ = Pose3D{};
    base_alignment_command_ = MobileBaseAlignmentCommand{};
    last_candidates_.clear();
    current_color_.release();
    current_depth_.release();
    task_id_.clear();
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
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
            oss << "target out of workspace";
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

bool GraspPipeline::FlushCameraAfterMotion(const char* reason) {
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

void GraspPipeline::SaveGraspDebug(float grasp_px, float grasp_py,
                                    uint16_t depth_mm,
                                    const float cam_point[3],
                                    const float base_point[3],
                                    float offset_dir_angle) {
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
        ofs << "  \"depth_mm\": " << depth_mm << ",\n";
        ofs << "  \"camera_point_m\": [" << cam_point[0] << ", "
            << cam_point[1] << ", " << cam_point[2] << "],\n";
        ofs << "  \"base_point_m\": [" << base_point[0] << ", "
            << base_point[1] << ", " << base_point[2] << "],\n";
        ofs << "  \"offset_dir_angle_rad\": " << offset_dir_angle << ",\n";
        ofs << "  \"grasp_yaw_rad\": " << grasp_yaw_rad_ << ",\n";
        ofs << "  \"offset_dir_angle_deg\": "
            << offset_dir_angle * 180.0f / M_PI << ",\n";
        ofs << "  \"grasp_yaw_deg\": "
            << grasp_yaw_rad_ * 180.0f / M_PI << ",\n";
        ofs << "  \"candidates\": \"" << JsonEscape(FormatCandidates())
            << "\",\n";
        WritePoseJson(ofs, "pre_grasp_pose", pre_grasp_pose_, true);
        WritePoseJson(ofs, "grasp_pose", grasp_pose_, false);
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
    if (!std::getline(std::cin, input) || input.empty()) {
        // 默认回车 = 继续
        return true;
    }

    char c = input[0];
    if (c == 'n' || c == 'N') {
        std::cout << "[Step] 用户中止" << std::endl;
        Stop();
        return false;
    }
    if (c == 's' || c == 'S') {
        std::cout << "[Step] 已关闭单步模式，后续不再暂停" << std::endl;
        config_.step_mode = false;
        return true;
    }
    return true;
}

void GraspPipeline::Run() {
    std::cout << "[Pipeline] Running main loop..." << std::endl;
    while (true) {
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
        SetState(PipelineState::IDLE, "Cancelling; keeping observe pose");
        std::cout << std::flush;
        if (action_.active) {
            action_.cancelling = true;
            return_to_observe_pending_ = true;
        } else {
            SaveTaskResultDebug(PipelineState::IDLE, "Cancelled");
            ResetTaskState();
            return_to_observe_pending_ = true;
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
        case PipelineState::DONE:
            if (config_.voice.enabled) {
                ResetTaskState();
                SetState(PipelineState::IDLE,
                        "Voice: waiting for next command");
            } else if (config_.auto_loop) {
                ResetTaskState();
                SetState(PipelineState::OBSERVING,
                        "Loop: restarting from observe position");
            }
            break;
        case PipelineState::ERROR:
            if (config_.voice.enabled) {
                ResetTaskState();
                SetState(PipelineState::IDLE,
                        "Voice: waiting for next command");
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
                ResetTaskState();
                shutdown_requested_.store(true);
                SetState(PipelineState::IDLE, "Home position reached; exiting");
                return;
            }
            SaveTaskResultDebug(PipelineState::IDLE,
                                "Cancelled; active action finished with " +
                                    std::string(GraspResultName(*result)));
            ResetTaskState();
            if (was_cancelling) {
                return_to_observe_pending_ = true;
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
    if (!action_.active) {
        // 将臂收到观察姿态:
        // Linksee 结构中立体相机在机身顶部面朝前方，
        // 臂需要收起/侧收，避免遮挡相机前方视野
        if (!WaitForConfirm("即将移动到观察位 (observe_joints)")) return;
        StartAction(PipelineState::OBSERVING, "move_to_observe", [this]() {
            auto result = executor_->MoveToObserve();
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
        if (!FlushCameraAfterMotion("observe motion")) {
            SetState(PipelineState::ERROR,
                    "Failed to refresh camera after observe motion");
            return;
        }
        SetState(PipelineState::DETECTING, "Arm retracted, detecting targets...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Observe move failed", *result));
    }
}

void GraspPipeline::HandleDetecting() {
    const auto detection_stage_start = std::chrono::steady_clock::now();
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
                SetState(PipelineState::ERROR,
                        "Target not found: " + target_label_ +
                        "; candidates: " + FormatCandidates());
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

void GraspPipeline::HandlePlanning() {
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_cpu_start = ProcessCpuMillis();

    if (current_depth_.empty()) {
        stable_count_ = 0;
        SetState(PipelineState::DETECTING, "No cached depth, detecting again");
        return;
    }

    // 计算抓取像素坐标:
    // 沿物体短轴方向偏移，方向由 yaw 约束自动选择
    // offset_ratio 控制偏移程度: 0=中心, 1=短轴边缘
    float grasp_px, grasp_py;
    float offset_dir_angle = NAN;
    float offset_ratio = config_.grasp_point_x_ratio;  // 复用此参数作为短轴偏移比例
    if (!ComputeGraspPixel(current_target_, grasp_px, grasp_py, offset_ratio,
                            config_.orientation, &offset_dir_angle)) {
        grasp_px = current_target_.center.x;
        grasp_py = current_target_.center.y;
    }

    int cx = ClampPixel(static_cast<int>(std::lround(grasp_px)),
                        current_depth_.cols);
    int cy = ClampPixel(static_cast<int>(std::lround(grasp_py)),
                        current_depth_.rows);

    std::cout << "[Pipeline] Grasp pixel (offset_ratio=" << offset_ratio << "): ["
                << cx << ", " << cy << "]";
    if (cx != static_cast<int>(std::lround(grasp_px)) ||
        cy != static_cast<int>(std::lround(grasp_py))) {
        std::cout << " clamped from [" << grasp_px << ", " << grasp_py << "]";
    }
    std::cout << std::endl;

    // 优先使用分割 mask 内的近景稳健深度。低矮物体的抓取点靠近边缘时，
    // 局部中值容易被地面深度主导，导致底盘误判目标始终距离不变。
    constexpr int roi_size = 5;
    uint16_t depth_mm = 0;
    uint16_t mask_depth_mm = 0;
    size_t mask_depth_samples = 0;
    const bool have_mask_depth = ForegroundDepthFromMask(
        current_depth_, current_target_.mask, mask_depth_mm,
        mask_depth_samples);
    if (have_mask_depth) {
        depth_mm = mask_depth_mm;
        std::cout << "[Pipeline] Depth source=mask_foreground_q25 value="
                    << depth_mm << "mm samples=" << mask_depth_samples
                    << std::endl;
    } else if (!MedianDepthAtPixel(
                current_depth_, cx, cy, roi_size, depth_mm)) {
        const int fallback_cx = ClampPixel(
            static_cast<int>(std::lround(current_target_.center.x)),
            current_depth_.cols);
        const int fallback_cy = ClampPixel(
            static_cast<int>(std::lround(current_target_.center.y)),
            current_depth_.rows);
        std::cout << "[Pipeline] No valid depth at grasp pixel, fallback to "
                << "target center [" << fallback_cx << ", " << fallback_cy
                << "]" << std::endl;
        cx = fallback_cx;
        cy = fallback_cy;
        if (!MedianDepthAtPixel(current_depth_, cx, cy, roi_size, depth_mm)) {
            std::cerr << "[Pipeline] No valid depth at grasp pixel or target "
                    << "center" << std::endl;
            stable_count_ = 0;
            SetState(PipelineState::ERROR,
                    "Target depth invalid at grasp pixel and center");
            return;
        }
        std::cout << "[Pipeline] Depth source=target_center_roi value="
                    << depth_mm << "mm" << std::endl;
    } else {
        std::cout << "[Pipeline] Depth source=grasp_pixel_roi value="
                    << depth_mm << "mm" << std::endl;
    }

    // 反投影到 3D
    float cam_point[3];
    if (!camera_->Deproject(cx, cy, depth_mm, cam_point)) {
        std::cerr << "[Pipeline] Deproject failed" << std::endl;
        stable_count_ = 0;
        SetState(PipelineState::ERROR, "Camera deprojection failed");
        return;
    }

    // 转换到基坐标系
    float base_point[3];
    planner_->CameraToBase(cam_point, base_point);

    std::cout << "[Pipeline] 3D position (base): [" << base_point[0] << ", "
                << base_point[1] << ", " << base_point[2] << "]" << std::endl;

    base_alignment_command_ = PlanMobileBaseAlignment(
        config_.mobile_base, base_point, base_align_attempts_);

    if (have_previous_base_alignment_point_) {
        const float progress = MeasureMobileBaseAlignmentProgress(
            previous_base_alignment_point_.data(), base_point,
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
        if (still_needs_alignment &&
            progress < required_progress) {
            stable_count_ = 0;
            std::ostringstream message;
            message << "Base alignment stopped: visual progress "
                    << progress << "m below required "
                    << required_progress
                    << "m after chassis motion; check depth and motion "
                    << "direction";
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
                SetState(PipelineState::ERROR,
                    "Base alignment stopped: cumulative travel safety "
                    "limit reached");
                return;
            }
        }
        std::copy_n(base_point, 3, previous_base_alignment_point_.begin());
        previous_base_alignment_command_ = base_alignment_command_;
        have_previous_base_alignment_point_ = true;
        std::cout << "[Pipeline] Mobile base alignment needed: "
                    << base_alignment_command_.reason
                    << " (attempt " << (base_align_attempts_ + 1)
                    << "/" << config_.mobile_base.max_align_attempts << ")"
                    << std::endl;
        std::ostringstream message;
        message << "Target=[" << base_point[0] << ", " << base_point[1]
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
        const float stable_y_limit =
            config_.mobile_base.y_tolerance +
            std::max(0.0f, config_.mobile_base.y_hysteresis);
        std::cout << "[Pipeline] Mobile base target in comfortable range: "
                    << "x=" << base_point[0] << "m in ["
                    << config_.mobile_base.target_x -
                        config_.mobile_base.x_tolerance
                    << ", "
                    << config_.mobile_base.target_x +
                        config_.mobile_base.x_tolerance
                    << "]m, y=" << base_point[1] << "m within stable +/-"
                    << stable_y_limit << "m"
                    << std::endl;
    }

    // 规划抓取
    if (!planner_->PlanTopGrasp(base_point, grasp_pose_, pre_grasp_pose_)) {
        SetState(PipelineState::ERROR,
                "Target out of workspace: base_point=[" +
                    std::to_string(base_point[0]) + ", " +
                    std::to_string(base_point[1]) + ", " +
                    std::to_string(base_point[2]) + "]");
        return;
    }

    // 计算基座系目标夹爪方向 (根据偏移方向自动对齐)
    // 偏移方向角由 ComputeGraspPixel 输出；executor 会再根据 IK 解的 joint0
    // 转换成实际 wrist_roll(joint5) 命令。
    if (config_.auto_orient && !std::isnan(offset_dir_angle)) {
        // 夹爪角度直接使用黄色检测线与图像水平线的夹角。
        // OpenCV 图像坐标 y 轴向下，因此需对 offset_dir_angle 取反后
        // 归一化到 [0, pi)。例如 offset_dir=76° 时，夹角为 104°。
        float yaw = ImageLineAngleFromHorizontal(offset_dir_angle);
        grasp_yaw_rad_ = yaw;
        std::cout << "[Pipeline] Yaw from offset_dir: offset_dir_angle="
                    << offset_dir_angle * 180.0f / M_PI << "° -> yaw="
                    << grasp_yaw_rad_ * 180.0f / M_PI << "°" << std::endl;
    } else if (config_.auto_orient) {
        // 物体接近圆形时回退到长轴方向
        grasp_yaw_rad_ = ComputeGraspYaw(current_target_, config_.orientation);
    } else {
        grasp_yaw_rad_ = NAN;
    }

    // 沿夹爪方向偏移: SO101 左爪固定，抓取点需往固定爪方向偏移
    // gripper_offset > 0 表示往固定爪侧偏移
    float offset = config_.planner.gripper_offset;
    if (offset != 0.0f) {
        float yaw = std::isnan(grasp_yaw_rad_) ? 0.0f : grasp_yaw_rad_;
        // 夹爪方向垂直于张开方向，偏移沿张开方向 (垂直于夹爪轴)
        // 在基座标系中: yaw=0 时夹爪张开方向为 Y 轴
        float dx = -offset * std::sin(yaw);
        float dy =  offset * std::cos(yaw);
        grasp_pose_.x += dx;
        grasp_pose_.y += dy;
        pre_grasp_pose_.x += dx;
        pre_grasp_pose_.y += dy;
        std::cout << "[Pipeline] Gripper offset (along jaw dir): "
                    << offset << "m, yaw=" << yaw << " rad"
                    << " -> dx=" << dx << " dy=" << dy << std::endl;
    }

    SaveGraspDebug(static_cast<float>(cx), static_cast<float>(cy), depth_mm,
                    cam_point, base_point, offset_dir_angle);

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
        if (!WaitForConfirm("即将移动到预抓取位 (pre_grasp)")) return;
        StartAction(PipelineState::APPROACHING, "move_to_pre_grasp", [this]() {
            return executor_->MoveToPreGrasp(pre_grasp_pose_, grasp_yaw_rad_);
        });
        return;
    }

    auto result = PollAction(PipelineState::APPROACHING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        SetState(PipelineState::GRASPING, "At pre-grasp, executing grasp...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Pre-grasp move failed", *result));
    }
}

void GraspPipeline::HandleGrasping() {
    if (!action_.active) {
        if (!WaitForConfirm("即将张开夹爪、下探并闭合抓取")) return;
        StartAction(PipelineState::GRASPING, "open_move_close_gripper", [this]() {
            auto result = executor_->OpenGripperForGrasp();
            if (result != GraspResult::SUCCESS) return result;
            result = executor_->MoveToGrasp(grasp_pose_, grasp_yaw_rad_);
            if (result != GraspResult::SUCCESS) return result;
            return executor_->CloseGripperAndCheck();
        });
        return;
    }

    auto result = PollAction(PipelineState::GRASPING);
    if (!result.has_value()) return;

    switch (*result) {
        case GraspResult::SUCCESS:
            SetState(PipelineState::LIFTING, "Object held, lifting...");
            break;

        case GraspResult::EMPTY:
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
            return executor_->LiftFromGrasp(pre_grasp_pose_, grasp_yaw_rad_);
        });
        return;
    }

    auto result = PollAction(PipelineState::LIFTING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        SetState(PipelineState::PLACING,
                "Lift completed and object holding confirmed; placing...");
    } else if (*result == GraspResult::EMPTY) {
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
        const char* target_pose = config_.voice.enabled ? "observe position"
                                                        : "home position";
        SetState(PipelineState::HOMING,
                std::string("Object released, returning to ") +
                    target_pose + "...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Place failed", *result));
    }
}

void GraspPipeline::HandleHoming() {
    if (!action_.active) {
        if (!WaitForConfirm(config_.voice.enabled ? "即将回到观察位"
                                                : "即将回到 Home 位")) {
            return;
        }
        const std::string action_name = config_.voice.enabled
            ? "move_to_observe_after_place"
            : "move_to_home_after_place";
        StartAction(PipelineState::HOMING, action_name, [this]() {
            return config_.voice.enabled ? executor_->MoveToObserve()
                                        : executor_->MoveToHome();
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

void GraspPipeline::BeginTaskTiming() {
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
    const PipelineState previous_state = state_.load();
    if (!msg.empty()) {
        last_status_message_ = msg;
    }

    if (task_timing_active_ && stage_timing_active_ &&
        IsTaskStage(previous_state) && previous_state != new_state) {
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - stage_started_at_)
                .count();
        const char* result = new_state == PipelineState::ERROR
            ? "FAILED"
            : (new_state == PipelineState::IDLE ? "CANCELLED" : "SUCCESS");
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
        if (!msg.empty()) {
            stage_log << " | " << msg;
        }
        WriteStructuredLine(stage_log.str());
    } else if (!task_timing_active_ && !msg.empty()) {
        std::ostringstream pipeline_log;
        pipeline_log << "[Pipeline] " << PipelineStateName(new_state)
                    << " | " << msg;
        WriteStructuredLine(pipeline_log.str());
    }

    if (IsTerminalState(new_state)) {
        SaveTaskResultDebug(new_state, msg.empty() ? last_status_message_ : msg);
    }

    if (task_timing_active_ &&
        (IsTerminalState(new_state) || new_state == PipelineState::IDLE)) {
        PrintTaskSummary(new_state,
                        msg.empty() ? last_status_message_ : msg);
    }

    // 通知回调
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
        callback_(new_state, msg);
    }
}

}  // namespace perceptive_grasp
