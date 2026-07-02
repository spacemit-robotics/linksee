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
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {

namespace {

namespace fs = std::filesystem;

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

}  // namespace

GraspPipeline::GraspPipeline(const PipelineConfig& config) : config_(config) {}

GraspPipeline::~GraspPipeline() { Stop(); }

bool GraspPipeline::Init() {
    // 初始化深度相机
    camera_ = std::make_unique<DepthCamera>(config_.camera);
    if (!camera_->Init()) {
        std::cerr << "[Pipeline] Failed to init depth camera" << std::endl;
        return false;
    }

    // 初始化目标检测器
#ifdef MOCK_DETECTOR
    detector_ = std::make_unique<MockDetector>(config_.detector);
#else
    detector_ = std::make_unique<TargetDetector>(config_.detector);
#endif
    if (!detector_->Init()) {
        std::cerr << "[Pipeline] Failed to init detector" << std::endl;
        return false;
    }

    // 初始化抓取规划器
    planner_ = std::make_unique<GraspPlanner>(config_.planner);

    // 初始化执行器
#ifdef MOCK_EXECUTOR
    executor_ = std::make_unique<MockExecutor>(config_.executor);
#else
    executor_ = std::make_unique<GraspExecutor>(config_.executor);
#endif
    if (!executor_->Init()) {
        std::cerr << "[Pipeline] Failed to init executor" << std::endl;
        return false;
    }

    std::cout << "[Pipeline] All modules initialized successfully" << std::endl;
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
    task_id_.clear();
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
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
    task_id_.clear();
    last_debug_image_path_.clear();
    last_debug_json_path_.clear();
    last_status_message_.clear();
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
    SetState(PipelineState::IDLE, "Stopped");
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
    std::cout << "[Pipeline] Async action started: " << name << std::endl;
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
    std::cout << "[Pipeline] Async action finished: " << action_.name
                << " -> " << GraspResultName(result) << std::endl;
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
                return_to_observe_pending_ = true;
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
        // Linksee 结构中 D435i 在机身顶部面朝前方，
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
        SetState(PipelineState::DETECTING, "Arm retracted, detecting targets...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Observe move failed", *result));
    }
}

void GraspPipeline::HandleDetecting() {
    cv::Mat color, depth;
    if (!camera_->GetFrames(color, depth)) {
        std::cerr << "[Pipeline] Failed to get camera frames" << std::endl;
        return;
    }

    const auto detector_start = std::chrono::steady_clock::now();

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
    if (config_.performance_log_enabled) {
        const auto detector_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                detector_end - detector_start)
                .count();
        std::cout << "[Perf] detection_inference_ms=" << detector_ms
                    << " found=" << (found ? 1 : 0) << std::endl;
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

    int cx = static_cast<int>(grasp_px);
    int cy = static_cast<int>(grasp_py);

    std::cout << "[Pipeline] Grasp pixel (offset_ratio=" << offset_ratio << "): ["
                << cx << ", " << cy << "]" << std::endl;

    // 取深度值 (取中心区域的中值，更鲁棒)
    int roi_size = 5;
    int x_start = std::max(0, cx - roi_size);
    int y_start = std::max(0, cy - roi_size);
    int x_end = std::min(current_depth_.cols - 1, cx + roi_size);
    int y_end = std::min(current_depth_.rows - 1, cy + roi_size);

    std::vector<uint16_t> depth_values;
    for (int y = y_start; y <= y_end; y++) {
        for (int x = x_start; x <= x_end; x++) {
            uint16_t d = current_depth_.at<uint16_t>(y, x);
            if (d > 0) depth_values.push_back(d);
        }
    }

    if (depth_values.empty()) {
        std::cerr << "[Pipeline] No valid depth at target center" << std::endl;
        stable_count_ = 0;
        SetState(PipelineState::ERROR, "Target depth invalid at grasp pixel");
        return;
    }

    // 取中值
    std::sort(depth_values.begin(), depth_values.end());
    uint16_t depth_mm = depth_values[depth_values.size() / 2];

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

    SaveGraspDebug(grasp_px, grasp_py, depth_mm, cam_point, base_point,
                    offset_dir_angle);

    if (config_.performance_log_enabled) {
        const auto planning_end = std::chrono::steady_clock::now();
        const auto planning_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                planning_end - planning_start)
                .count();
        std::cout << "[Perf] planning_stage_ms=" << planning_ms
                    << " stable_count=" << stable_count_ << std::endl;
    }

    SetState(PipelineState::APPROACHING, "Moving to pre-grasp...");
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
                // 1. 臂收起避免遮挡前视 D435i
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
        SetState(PipelineState::PLACING, "Lift completed, placing...");
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
        SetState(PipelineState::HOMING,
                "Object released, returning to observe position...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Place failed", *result));
    }
}

void GraspPipeline::HandleHoming() {
    if (!action_.active) {
        if (!WaitForConfirm("即将回到观察位")) return;
        StartAction(PipelineState::HOMING, "move_to_observe_after_place", [this]() {
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
                ResultMessage("Observe move failed", *result));
    }
}

void GraspPipeline::SetState(PipelineState new_state,
                            const std::string& msg) {
    if (!msg.empty()) {
        last_status_message_ = msg;
    }
    state_.store(new_state);
    if (!msg.empty()) {
        std::cout << "[Pipeline] State -> " << PipelineStateName(new_state)
                    << "(" << static_cast<int>(new_state) << ")"
                    << ": " << msg << std::endl;
    }

    if (IsTerminalState(new_state)) {
        SaveTaskResultDebug(new_state, msg.empty() ? last_status_message_ : msg);
    }

    // 通知回调
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
        callback_(new_state, msg);
    }
}

}  // namespace perceptive_grasp
