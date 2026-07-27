/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_executor.cpp
    * @brief 抓取执行模块实现 - 机械臂 + 夹爪协调控制
    */

#include "grasp_executor.h"

extern "C" {
#include "grasp.h"
#include "kinematics_interface.h"
#include "manipulator.h"
#include "so101_utils.h"
#include "so101_gripper.h"
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

namespace perceptive_grasp {

namespace {

constexpr int kSideMotionCompletionTimeoutMs = 5000;

const char* GraspStateName(grasp_state_t state) {
    switch (state) {
        case GRASP_STATE_IDLE: return "IDLE";
        case GRASP_STATE_MOVING: return "MOVING";
        case GRASP_STATE_HOLDING: return "HOLDING";
        case GRASP_STATE_EMPTY: return "EMPTY";
        case GRASP_STATE_ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

bool ResolveFixedJawWristYaw(float target_yaw,
                            float joint0,
                            float scale,
                            float lower,
                            float upper,
                            float& raw_wrist,
                            float& resolved_wrist) {
    if (std::fabs(scale) < 1e-6f || lower > upper) return false;

    // The Linksee gripper has a fixed jaw. A yaw shifted by pi preserves the
    // closing axis but swaps the fixed jaw to the opposite side, so it is not
    // an equivalent grasp direction. Use a one-to-one yaw mapping.
    raw_wrist = (target_yaw - joint0) / scale;
    resolved_wrist = std::clamp(raw_wrist, lower, upper);
    return true;
}

std::array<double, 3> ToolAxisY(double qw, double qx,
                                double qy, double qz) {
    return {
        2.0 * (qx * qy - qw * qz),
        1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz + qw * qx),
    };
}

std::array<double, 3> ToolAxisZ(double qw, double qx,
                                double qy, double qz) {
    return {
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    };
}

double AxisAngleDegrees(const std::array<double, 3>& lhs,
                        const std::array<double, 3>& rhs) {
    const double lhs_norm = std::sqrt(
        lhs[0] * lhs[0] + lhs[1] * lhs[1] + lhs[2] * lhs[2]);
    const double rhs_norm = std::sqrt(
        rhs[0] * rhs[0] + rhs[1] * rhs[1] + rhs[2] * rhs[2]);
    if (lhs_norm < 1e-9 || rhs_norm < 1e-9) return 180.0;
    const double cosine = std::clamp(
        (lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]) /
            (lhs_norm * rhs_norm),
        -1.0, 1.0);
    return std::acos(cosine) * 180.0 / M_PI;
}

bool PosesMatch(const Pose3D& lhs, const Pose3D& rhs) {
    constexpr float kPositionToleranceM = 1e-4f;
    constexpr float kQuaternionTolerance = 1e-4f;
    return std::fabs(lhs.x - rhs.x) <= kPositionToleranceM &&
        std::fabs(lhs.y - rhs.y) <= kPositionToleranceM &&
        std::fabs(lhs.z - rhs.z) <= kPositionToleranceM &&
        std::fabs(lhs.qw - rhs.qw) <= kQuaternionTolerance &&
        std::fabs(lhs.qx - rhs.qx) <= kQuaternionTolerance &&
        std::fabs(lhs.qy - rhs.qy) <= kQuaternionTolerance &&
        std::fabs(lhs.qz - rhs.qz) <= kQuaternionTolerance;
}

float PosePositionError(const Pose3D& actual, const Pose3D& target) {
    const float dx = actual.x - target.x;
    const float dy = actual.y - target.y;
    const float dz = actual.z - target.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ApplyJointLimits(const std::vector<JointConstraint>& limits,
                        std::vector<double>& lower,
                        std::vector<double>& upper,
                        std::string* detail = nullptr) {
    for (const JointConstraint& limit : limits) {
        if (limit.joint_index < 0 ||
            limit.joint_index >= static_cast<int>(lower.size()) ||
            limit.min_rad > limit.max_rad) {
            if (detail) *detail = "invalid hardware joint limit";
            return false;
        }
        const size_t index = static_cast<size_t>(limit.joint_index);
        lower[index] = std::max(
            lower[index], static_cast<double>(limit.min_rad));
        upper[index] = std::min(
            upper[index], static_cast<double>(limit.max_rad));
        if (lower[index] > upper[index]) {
            if (detail) {
                *detail = "hardware and URDF limits do not overlap for joint " +
                    std::to_string(limit.joint_index);
            }
            return false;
        }
    }
    return true;
}

bool JointsWithinLimits(const std::vector<float>& joints,
                        const std::vector<JointConstraint>& limits,
                        std::string* detail = nullptr) {
    for (const JointConstraint& limit : limits) {
        if (limit.joint_index < 0 ||
            limit.joint_index >= static_cast<int>(joints.size())) {
            if (detail) *detail = "hardware joint limit index is invalid";
            return false;
        }
        const float value = joints[limit.joint_index];
        if (value < limit.min_rad || value > limit.max_rad) {
            if (detail) {
                *detail = "joint " + std::to_string(limit.joint_index) +
                    " target " + std::to_string(value) +
                    " is outside hardware range [" +
                    std::to_string(limit.min_rad) + "," +
                    std::to_string(limit.max_rad) + "]";
            }
            return false;
        }
    }
    return true;
}

bool ClampJointsToLimits(std::vector<float>& joints,
                        const std::vector<JointConstraint>& limits) {
    for (const JointConstraint& limit : limits) {
        if (limit.joint_index < 0 ||
            limit.joint_index >= static_cast<int>(joints.size()) ||
            limit.min_rad > limit.max_rad) {
            return false;
        }
        float& value = joints[limit.joint_index];
        value = std::clamp(value, limit.min_rad, limit.max_rad);
    }
    return true;
}

}  // namespace

void GraspExecutor::RecordResult(GraspResult result, const std::string& action,
                                const std::string& detail) {
    diagnostics_.last_result = result;
    diagnostics_.last_action = action;
    diagnostics_.last_detail = detail;
}

GraspExecutor::~GraspExecutor() {
    if (gripper_) {
        grasp_stop(gripper_);
        grasp_free(gripper_);
    }
    if (arm_) {
        manip_free(arm_);
        // kin_ 由 manip_free 自动释放 (所有权已转移)
    }
}

bool GraspExecutor::Init() {
    // 构造 SO101 机械臂配置
    struct so101_config arm_cfg = {};
    arm_cfg.uart_path = config_.uart_device.c_str();
    arm_cfg.baud = static_cast<uint32_t>(config_.baudrate);
    arm_cfg.ids[0] = 1;
    arm_cfg.ids[1] = 2;
    arm_cfg.ids[2] = 3;
    arm_cfg.ids[3] = 4;
    arm_cfg.ids[4] = 5;
    arm_cfg.urdf_path = config_.urdf_path.c_str();
    arm_cfg.kin_solver_name = nullptr;  // 使用默认 pinocchio

    arm_ = manip_alloc(config_.manip_driver.c_str(), &arm_cfg);
    if (!arm_) {
        std::cerr << "[GraspExecutor] Failed to create manipulator" << std::endl;
        return false;
    }

    // 创建运动学求解器
    kin_ = kin_create(nullptr,  // 使用默认求解器 (tracik/pinocchio)
                        config_.urdf_path.c_str(), config_.base_link.c_str(),
                        config_.tip_link.c_str());
    if (!kin_) {
        std::cerr << "[GraspExecutor] Failed to create kinematics solver"
                    << std::endl;
        return false;
    }

    // 绑定运动学到机械臂 (所有权转移)
    int ret = manip_set_kinematics(arm_, kin_);
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] Failed to bind kinematics: " << ret
                    << std::endl;
        kin_destroy(kin_);
        kin_ = nullptr;
        return false;
    }

    arm_path_safety_ = std::make_unique<ArmPathSafety>();
    std::string path_safety_error;
    if (!arm_path_safety_->Init(
            config_.urdf_path, config_.base_link, config_.tip_link,
            &path_safety_error)) {
        std::cerr << "[GraspExecutor] Failed to initialize arm path safety: "
                << path_safety_error << std::endl;
        return false;
    }

    // 构造 SO101 夹爪配置 (与机械臂共用同一串口)
    struct so101_gripper_config grip_cfg = {};
    grip_cfg.uart_path = config_.uart_device.c_str();
    grip_cfg.baud = static_cast<uint32_t>(config_.baudrate);
    grip_cfg.id = 6;  // SO101 夹爪默认 ID=6
    grip_cfg.grasp_cfg.max_effort = config_.gripper_effort;
    grip_cfg.grasp_cfg.hold_threshold = config_.gripper_hold_load_threshold;
    grip_cfg.grasp_cfg.timeout_ms =
        static_cast<uint32_t>(config_.gripper_timeout_ms);

    gripper_ = grasp_alloc("so101_gripper", &grip_cfg);
    if (!gripper_) {
        std::cerr << "[GraspExecutor] Failed to create gripper" << std::endl;
        return false;
    }

    std::cout << "[GraspExecutor] Initialized: arm=" << config_.manip_driver
                << ", urdf=" << config_.urdf_path << std::endl;
    return true;
}

void GraspExecutor::SetSupportPlane(const SupportPlane& support_plane) {
    support_plane_ = support_plane;
    validated_side_staging_joints_.clear();
    validated_side_sweep_joints_.clear();
    validated_side_entry_pose_ = Pose3D{};
    validated_side_entry_joints_.clear();
    validated_side_poses_.clear();
    validated_side_joint_path_.clear();
    validated_side_path_index_ = 0;
}

GraspResult GraspExecutor::MoveToObserve() {
    // 先闭合夹爪并等待完成
    grasp_set_position(gripper_, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.observe_gripper_close_wait_ms));

    // Coordinate the large elbow/wrist changes so the shorter joint motion
    // cannot finish first and lower the gripper into the support surface.
    std::vector<float> current_joints;
    bool starts_near_home =
        GetCurrentJoints(current_joints) &&
        current_joints.size() == config_.home_joints.size();
    constexpr float kHomeJointToleranceRad = 0.25f;
    if (starts_near_home) {
        for (size_t index = 0; index < current_joints.size(); ++index) {
            if (std::fabs(
                    current_joints[index] - config_.home_joints[index]) >
                kHomeJointToleranceRad) {
                starts_near_home = false;
                break;
            }
        }
    }
    const bool use_coordinated_path =
        starts_near_home &&
        !NeedsCollisionAvoidance(current_joints, config_.observe_joints);
    GraspResult result = use_coordinated_path
        ? MoveToJointsCoordinated(config_.observe_joints)
        : MoveToJointsCollisionSafe(config_.observe_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_observe", "move_joints failed");
        return result;
    }
    if (!use_coordinated_path && !WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_observe",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    CaptureEmptyClosedPosition();
    RecordResult(GraspResult::SUCCESS, "move_to_observe");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToSideObserve() {
    grasp_set_position(gripper_, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.observe_gripper_close_wait_ms));

    constexpr size_t kFourthJointIndex = 3;
    constexpr float kOtherJointLeadProgress = 0.45f;
    constexpr float kFourthJointLeadProgress = 0.65f;
    constexpr float kSideObserveFinalToleranceRad = 0.080f;
    std::vector<float> current_joints;
    if (!GetCurrentJoints(current_joints) ||
        current_joints.size() != config_.side_ready_joints.size() ||
        current_joints.size() <= kFourthJointIndex) {
        RecordResult(
            GraspResult::MOVE_FAILED, "move_to_side_observe",
            "failed to read joints or side-ready pose is incomplete");
        return GraspResult::MOVE_FAILED;
    }
    if (!arm_path_safety_ || !support_plane_.valid) {
        RecordResult(
            GraspResult::OUT_OF_RANGE, "move_to_side_observe",
            "support surface is unavailable");
        return GraspResult::OUT_OF_RANGE;
    }

    const std::vector<float> measured_joints = current_joints;
    if (!ClampJointsToLimits(current_joints, config_.joint_limits)) {
        RecordResult(
            GraspResult::OUT_OF_RANGE, "move_to_side_observe",
            "configured joint limits are invalid");
        return GraspResult::OUT_OF_RANGE;
    }
    if (current_joints != measured_joints) {
        std::cout << "[GraspExecutor] side observation: clamped measured "
                    "joint drift to configured limits"
                    << std::endl;
    }

    const auto build_lead_path =
        [this](const std::vector<float>& start,
                std::vector<std::vector<float>>& path,
                std::string& detail) {
            std::vector<float> lead = start;
            for (size_t index = 0; index < lead.size(); ++index) {
                const float progress = index == kFourthJointIndex
                    ? kFourthJointLeadProgress
                    : kOtherJointLeadProgress;
                lead[index] += progress *
                    (config_.side_ready_joints[index] - start[index]);
            }

            const ArmPathSafetyResult lead_result =
                arm_path_safety_->CheckPath(
                    start, lead, support_plane_,
                    config_.support_surface_clearance_m,
                    config_.path_joint_step_rad);
            if (!lead_result.safe) {
                detail = "coordinated lead path is unsafe: " +
                    lead_result.detail;
                return false;
            }
            const ArmPathSafetyResult finish_result =
                arm_path_safety_->CheckPath(
                    lead, config_.side_ready_joints, support_plane_,
                    config_.support_surface_clearance_m,
                    config_.path_joint_step_rad);
            if (!finish_result.safe) {
                detail = "coordinated finish path is unsafe: " +
                    finish_result.detail;
                return false;
            }
            path.push_back(std::move(lead));
            path.push_back(config_.side_ready_joints);
            return true;
        };

    std::vector<std::vector<float>> joint_path;
    std::string path_detail;
    if (!build_lead_path(current_joints, joint_path, path_detail)) {
        const ArmPathSafetyResult home_result =
            arm_path_safety_->CheckPath(
                current_joints, config_.home_joints, support_plane_,
                config_.support_surface_clearance_m,
                config_.path_joint_step_rad);
        std::vector<std::vector<float>> home_to_side_path;
        std::string home_to_side_detail;
        if (!home_result.safe ||
            !build_lead_path(
                config_.home_joints, home_to_side_path,
                home_to_side_detail)) {
            const std::string detail = !home_result.safe
                ? "cannot recover through home: " + home_result.detail
                : home_to_side_detail;
            RecordResult(
                GraspResult::OUT_OF_RANGE, "move_to_side_observe",
                path_detail + "; " + detail);
            return GraspResult::OUT_OF_RANGE;
        }
        joint_path.push_back(config_.home_joints);
        joint_path.insert(
            joint_path.end(),
            home_to_side_path.begin(), home_to_side_path.end());
        std::cout << "[GraspExecutor] side observation: recovering through "
                    "home with a continuous path"
                    << std::endl;
    }

    std::cout << "[GraspExecutor] side observation: coordinated motion; "
                "fourth joint leads while all joints move continuously"
                << std::endl;
    const GraspResult result = ExecuteContinuousJointPath(
        joint_path, config_.move_speed, config_.move_speed,
        kSideObserveFinalToleranceRad);
    if (result != GraspResult::SUCCESS) {
        RecordResult(
            result, "move_to_side_observe",
            "coordinated side-observation motion failed");
        return result;
    }

    CaptureEmptyClosedPosition();
    RecordResult(GraspResult::SUCCESS, "move_to_side_observe");
    return GraspResult::SUCCESS;
}

void GraspExecutor::CaptureEmptyClosedPosition() {
    if (std::isfinite(empty_closed_position_)) return;

    float position = NAN;
    float load = NAN;
    grasp_tick(gripper_, 0.05f);
    if (grasp_get_feedback(gripper_, &position, &load) != GRASP_OK ||
        std::isnan(position)) {
        return;
    }
    if (std::isfinite(load) &&
        load >= config_.gripper_hold_load_threshold) {
        std::cout << "[GraspExecutor] empty gripper baseline skipped: "
                    "load indicates a possible held object"
                    << std::endl;
        return;
    }

    empty_closed_position_ = position;
    std::cout << "[GraspExecutor] empty gripper baseline: position="
            << empty_closed_position_ << ", load=" << load << std::endl;
}

GraspResult GraspExecutor::MoveToHome() {
    GraspResult result = MoveToJointsCollisionSafe(config_.home_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_home", "move_joints failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_home", "motion timeout");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_home");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                            float grasp_yaw_rad,
                                            bool use_top_constraints) {
    if (!use_top_constraints) {
        const GraspResult close_result = CloseGripper();
        if (close_result != GraspResult::SUCCESS) {
            RecordResult(close_result, "move_to_pre_grasp",
                        "failed to close gripper before side-ready motion");
            return close_result;
        }
    }

    bool has_yaw = !std::isnan(grasp_yaw_rad);
    if (has_yaw) {
        std::cout << "[GraspExecutor] Approach with yaw override: "
                    << grasp_yaw_rad << " rad ("
                    << grasp_yaw_rad * 180.0f / M_PI << "°)" << std::endl;
    }

    GraspResult result;
    if (has_yaw) {
        result = MoveToPoseWithYaw(
            pre_grasp_pose, config_.move_speed, grasp_yaw_rad);
    } else if (use_top_constraints) {
        result = MoveToPoseConstrained(pre_grasp_pose, config_.move_speed);
    } else {
        result = MoveToSidePreGrasp(pre_grasp_pose, config_.move_speed);
        if (result == GraspResult::SUCCESS) {
            RecordResult(GraspResult::SUCCESS, "move_to_pre_grasp");
            return GraspResult::SUCCESS;
        }
    }
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_pre_grasp",
                    result == GraspResult::IK_FAILED ? "ik failed"
                                                        : "move command failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_pre_grasp",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    if (!VerifyPoseReached("move_to_pre_grasp", pre_grasp_pose)) {
        RecordResult(GraspResult::MOVE_FAILED, "move_to_pre_grasp",
                    "pose verification failed");
        return GraspResult::MOVE_FAILED;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_pre_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::OpenGripperForGrasp(float minimum_opening) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.pre_grasp_settle_ms));
    float opening = config_.gripper_open;
    if (std::isfinite(minimum_opening)) {
        opening = std::max(opening, std::clamp(minimum_opening, 0.0f, 1.0f));
    }
    std::cout << "[GraspExecutor] opening gripper: configured="
                << config_.gripper_open << " requested=" << minimum_opening
                << " command=" << opening << std::endl;
    if (grasp_set_position(gripper_, opening) != GRASP_OK) {
        RecordResult(GraspResult::MOVE_FAILED, "open_gripper_for_grasp",
                    "failed to set grasp opening");
        return GraspResult::MOVE_FAILED;
    }
    if (!WaitGripperOpening(opening)) {
        RecordResult(GraspResult::TIMEOUT, "open_gripper_for_grasp",
                    "gripper did not reach requested opening");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "open_gripper_for_grasp");
    return GraspResult::SUCCESS;
}

bool GraspExecutor::WaitGripperOpening(float target_position) {
    constexpr int kPollIntervalMs = 50;
    constexpr float kPositionTolerance = 0.03f;
    const int timeout_ms = std::max(
        config_.gripper_timeout_ms,
        config_.timing.gripper_open_wait_ms);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    float position = NAN;
    float load = NAN;
    while (std::chrono::steady_clock::now() < deadline) {
        grasp_tick(
            gripper_, static_cast<float>(kPollIntervalMs) / 1000.0f);
        const grasp_state_t state = grasp_get_state(gripper_);
        if (state == GRASP_STATE_ERROR) return false;
        if (grasp_get_feedback(gripper_, &position, &load) == GRASP_OK &&
            std::isfinite(position) &&
            position >= target_position - kPositionTolerance) {
            std::cout << "[GraspExecutor] gripper opening reached: target="
                        << target_position << " position=" << position
                        << " load=" << load << std::endl;
            return true;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kPollIntervalMs));
    }

    std::cerr << "[GraspExecutor] gripper opening timeout: target="
                << target_position << " position=" << position
                << " load=" << load << std::endl;
    return false;
}

GraspResult GraspExecutor::MoveToGrasp(const Pose3D& grasp_pose,
                                        float grasp_yaw_rad,
                                        bool use_top_constraints) {
    bool has_yaw = !std::isnan(grasp_yaw_rad);
    GraspResult result;
    if (has_yaw) {
        result = MoveToPoseWithYaw(
            grasp_pose, config_.line_speed, grasp_yaw_rad);
    } else if (use_top_constraints) {
        result = MoveToPoseConstrained(grasp_pose, config_.line_speed);
    } else {
        result = MoveToPoseSide(grasp_pose, config_.line_speed);
        if (result == GraspResult::SUCCESS) {
            RecordResult(GraspResult::SUCCESS, "move_to_grasp");
            return GraspResult::SUCCESS;
        }
    }
    if (result != GraspResult::SUCCESS) {
        std::string detail = result == GraspResult::IK_FAILED
            ? "ik failed"
            : "move command failed";
        if (diagnostics_.last_action == "move_to_side_pose" &&
            diagnostics_.last_result == result &&
            !diagnostics_.last_detail.empty()) {
            detail = diagnostics_.last_detail;
        }
        RecordResult(result, "move_to_grasp",
                    detail);
        return result;
    }
    constexpr float kTopGraspJointToleranceRad = 0.020f;
    if (!WaitMotionDone(-1, kTopGraspJointToleranceRad)) {
        RecordResult(GraspResult::TIMEOUT, "move_to_grasp",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    if (!VerifyPoseReached("move_to_grasp", grasp_pose)) {
        RecordResult(GraspResult::MOVE_FAILED, "move_to_grasp",
                    "pose verification failed");
        return GraspResult::MOVE_FAILED;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CloseGripperAndCheck() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.grasp_settle_ms));
    if (grasp_execute(gripper_, GRASP_CMD_GRAB,
                    config_.gripper_effort) != GRASP_OK) {
        RecordResult(GraspResult::MOVE_FAILED, "close_gripper_and_check",
                    "failed to start gripper close");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.gripper_close_wait_ms));

    return CheckGripperHolding("after_close", false);
}

GraspResult GraspExecutor::CheckGripperHolding(const char* phase,
                                            bool after_lift) {
    grasp_state_t state = GRASP_STATE_ERROR;
    int holding_count = 0;
    int load_holding_count = 0;
    int performed_checks = 0;
    float position = NAN;
    float load = NAN;

    const int required_holding =
        std::max(1, config_.timing.grasp_check_count / 2);
    const int maximum_checks = std::max(
        config_.timing.grasp_check_count,
        config_.gripper_timeout_ms /
            std::max(1, config_.timing.grasp_check_interval_ms));
    constexpr float kAfterLiftOpeningHysteresis = 0.010f;
    const float position_margin = after_lift
        ? std::max(
            0.0f,
            config_.gripper_empty_position_margin -
                kAfterLiftOpeningHysteresis)
        : config_.gripper_empty_position_margin;
    const float min_object_position = std::isnan(empty_closed_position_)
        ? 0.05f
        : empty_closed_position_ + position_margin;
    for (int i = 0; i < maximum_checks; i++) {
        const float interval_s =
            static_cast<float>(config_.timing.grasp_check_interval_ms) /
            1000.0f;
        grasp_tick(gripper_, interval_s);
        performed_checks++;
        state = grasp_get_state(gripper_);
        if (state == GRASP_STATE_HOLDING) {
            holding_count++;
        } else {
            holding_count = 0;
        }

        float cur_position = NAN;
        float cur_load = NAN;
        if (grasp_get_feedback(gripper_, &cur_position, &cur_load) == GRASP_OK) {
            position = cur_position;
            load = cur_load;
            if (cur_position > min_object_position &&
                cur_load >= config_.gripper_hold_load_threshold) {
                load_holding_count++;
            } else {
                load_holding_count = 0;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(
            config_.timing.grasp_check_interval_ms));
        if (load_holding_count >= required_holding) {
            break;
        }
        if (performed_checks >= config_.timing.grasp_check_count &&
            (state == GRASP_STATE_EMPTY || state == GRASP_STATE_IDLE ||
            state == GRASP_STATE_ERROR) &&
            load_holding_count == 0) {
            break;
        }
    }

    diagnostics_.gripper_check.phase = phase;
    diagnostics_.gripper_check.state = GraspStateName(state);
    diagnostics_.gripper_check.holding_count = holding_count;
    diagnostics_.gripper_check.load_holding_count = load_holding_count;
    diagnostics_.gripper_check.check_count = performed_checks;
    diagnostics_.gripper_check.load_threshold =
        config_.gripper_hold_load_threshold;
    diagnostics_.gripper_check.empty_closed_position =
        empty_closed_position_;
    diagnostics_.gripper_check.min_object_position = min_object_position;
    diagnostics_.gripper_check.position = position;
    diagnostics_.gripper_check.load = load;

    std::cout << "[GraspExecutor] grasp check: phase=" << phase
                << ", state="
                << GraspStateName(state)
                << ", holding=" << holding_count << "/"
                << performed_checks
                << ", load_holding=" << load_holding_count << "/"
                << performed_checks
                << ", load_threshold="
                << config_.gripper_hold_load_threshold
                << ", min_object_position=" << min_object_position
                << ", position=" << position
                << ", load=" << load << std::endl;

    const bool opening_indicates_object =
        !std::isnan(position) && position > min_object_position;
    const bool holding_confirmed =
        load_holding_count >= required_holding &&
        opening_indicates_object;
    if (holding_confirmed) {
        const char* action = after_lift ? "verify_grasp_after_lift"
                                        : "close_gripper_and_check";
        RecordResult(GraspResult::SUCCESS, action,
                    "sustained load and baseline-relative opening confirmed");
        return GraspResult::SUCCESS;
    }
    if (state == GRASP_STATE_EMPTY ||
        state == GRASP_STATE_IDLE ||
        (!std::isnan(position) && position <= min_object_position)) {
        std::cout << "[GraspExecutor] Grasp empty - nothing grabbed" << std::endl;
        RecordResult(GraspResult::EMPTY,
                    after_lift ? "verify_grasp_after_lift"
                            : "close_gripper_and_check",
                    "gripper closed without object");
        return GraspResult::EMPTY;
    }
    if (after_lift) {
        RecordResult(GraspResult::EMPTY, "verify_grasp_after_lift",
                    "holding state or load was not sustained after lift");
        return GraspResult::EMPTY;
    }
    if (state == GRASP_STATE_MOVING) {
        std::cerr << "[GraspExecutor] Gripper still moving after close check"
                    << std::endl;
        RecordResult(GraspResult::TIMEOUT, "close_gripper_and_check",
                    "gripper still moving after close check");
        return GraspResult::TIMEOUT;
    }
    std::cerr << "[GraspExecutor] Gripper error during close check" << std::endl;
    RecordResult(GraspResult::MOVE_FAILED, "close_gripper_and_check",
                "gripper error during close check");
    return GraspResult::MOVE_FAILED;
}

GraspResult GraspExecutor::LiftFromGrasp(const Pose3D& retreat_pose,
                                        const Pose3D& lift_pose,
                                        float grasp_yaw_rad,
                                        bool use_top_constraints) {
    const auto move_and_wait = [this, grasp_yaw_rad, use_top_constraints](
                                    const Pose3D& pose,
                                    const char* action) -> GraspResult {
        const bool has_yaw = !std::isnan(grasp_yaw_rad);
        GraspResult result;
        if (has_yaw) {
            result = MoveToPoseWithYaw(pose, config_.line_speed,
                                        grasp_yaw_rad);
        } else if (use_top_constraints) {
            result = MoveToPoseConstrained(pose, config_.line_speed);
        } else {
            result = MoveToPoseSide(pose, config_.line_speed);
        }
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;
        if (!VerifyPoseReached(action, pose)) return GraspResult::MOVE_FAILED;
        return GraspResult::SUCCESS;
    };

    const bool use_side_path =
        !use_top_constraints && std::isnan(grasp_yaw_rad);
    GraspResult result;
    if (use_side_path) {
        std::cout << "[GraspExecutor] side lift phase=vertical target=["
                    << retreat_pose.x << "," << retreat_pose.y << ","
                    << retreat_pose.z << "]" << std::endl;
        std::cout << "[GraspExecutor] side lift phase=retreat target=["
                    << lift_pose.x << "," << lift_pose.y << ","
                    << lift_pose.z << "]" << std::endl;
        result = MoveToSideLift(
            retreat_pose, lift_pose, config_.line_speed);
        if (result != GraspResult::SUCCESS) {
            RecordResult(result, "lift_from_grasp",
                        result == GraspResult::IK_FAILED
                            ? "side lift ik failed"
                            : "side lift failed");
            return result;
        }
    } else {
        result = move_and_wait(retreat_pose, "retreat_from_grasp");
        if (result != GraspResult::SUCCESS) {
            RecordResult(result, "lift_from_grasp",
                        result == GraspResult::IK_FAILED
                            ? "retreat ik failed"
                            : "retreat failed");
            return result;
        }

        const float lift_delta = std::sqrt(
            std::pow(lift_pose.x - retreat_pose.x, 2.0f) +
            std::pow(lift_pose.y - retreat_pose.y, 2.0f) +
            std::pow(lift_pose.z - retreat_pose.z, 2.0f));
        if (lift_delta > 0.001f) {
            result = move_and_wait(lift_pose, "lift_from_grasp");
            if (result != GraspResult::SUCCESS) {
                RecordResult(result, "lift_from_grasp",
                            result == GraspResult::IK_FAILED
                                ? "lift ik failed"
                                : "lift failed");
                return result;
            }
        }
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.post_lift_settle_ms));
    return CheckGripperHolding("after_lift", true);
}

GraspResult GraspExecutor::MoveToPlace() {
    if (support_plane_.valid && arm_path_safety_) {
        std::vector<float> current_joints;
        if (!GetCurrentJoints(current_joints)) {
            RecordResult(
                GraspResult::MOVE_FAILED, "move_to_place",
                "failed to read joints for support-surface check");
            return GraspResult::MOVE_FAILED;
        }
        const ArmPathSafetyResult path_result = arm_path_safety_->CheckPath(
            current_joints, config_.place_joints, support_plane_,
            config_.support_surface_clearance_m,
            config_.path_joint_step_rad);
        if (!path_result.safe) {
            RecordResult(
                GraspResult::OUT_OF_RANGE, "move_to_place",
                "place path is unsafe: " + path_result.detail);
            return GraspResult::OUT_OF_RANGE;
        }
    }
    GraspResult result = MoveToJointsCollisionSafe(config_.place_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_place", "move_joints failed");
        return result;
    }
    if (!WaitMotionDone(-1, config_.place_joint_tolerance_rad)) {
        RecordResult(GraspResult::TIMEOUT, "move_to_place", "motion timeout");
        return GraspResult::TIMEOUT;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.place_settle_ms));
    RecordResult(GraspResult::SUCCESS, "move_to_place");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::ReleaseObject() {
    if (grasp_set_position(gripper_, config_.place_release_open) != GRASP_OK) {
        RecordResult(GraspResult::MOVE_FAILED, "release_object",
                    "failed to set release opening");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.release_wait_ms));
    empty_closed_position_ = NAN;
    RecordResult(GraspResult::SUCCESS, "release_object");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CloseGripper() {
    if (grasp_set_position(gripper_, 0.0f) != GRASP_OK) {
        RecordResult(GraspResult::MOVE_FAILED, "close_gripper",
                    "failed to set closed position");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.home_gripper_close_wait_ms));
    RecordResult(GraspResult::SUCCESS, "close_gripper");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToJointsSafe(const std::vector<float>& joints,
                                            float speed_scale) {
    float old_speed = config_.move_speed;
    if (speed_scale > 0.0f) {
        config_.move_speed = speed_scale;
    }

    GraspResult result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed;
    if (result != GraspResult::SUCCESS) return result;
    if (!WaitMotionDone()) return GraspResult::TIMEOUT;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPreGraspSafe(const Pose3D& pre_grasp_pose,
                                                float speed_scale) {
    float speed = speed_scale > 0.0f ? speed_scale : config_.move_speed;
    GraspResult result = MoveToPoseWithIKJoints(pre_grasp_pose, speed);
    if (result != GraspResult::SUCCESS) return result;
    if (!WaitMotionDone()) return GraspResult::TIMEOUT;
    if (!VerifyPoseReached("move_to_pre_grasp_safe", pre_grasp_pose)) {
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::ExecuteGrasp(const Pose3D& grasp_pose,
                                        const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad) {
    GraspResult result = MoveToPreGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;

    result = OpenGripperForGrasp();
    if (result != GraspResult::SUCCESS) return result;

    result = MoveToGrasp(grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;

    result = CloseGripperAndCheck();
    if (result != GraspResult::SUCCESS) return result;

    result = LiftFromGrasp(
        pre_grasp_pose, pre_grasp_pose, grasp_yaw_rad, true);
    if (result != GraspResult::SUCCESS) return result;

    std::cout << "[GraspExecutor] Grasp successful!" << std::endl;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::ExecutePlace() {
    GraspResult result = MoveToPlace();
    if (result != GraspResult::SUCCESS) return result;
    result = ReleaseObject();
    if (result != GraspResult::SUCCESS) return result;
    result = CloseGripper();
    if (result != GraspResult::SUCCESS) return result;
    return MoveToObserve();
}

void GraspExecutor::EmergencyStop() {
    if (arm_) manip_stop(arm_);
    if (gripper_) grasp_stop(gripper_);
    RecordResult(GraspResult::MOVE_FAILED, "emergency_stop",
                "emergency stop requested");
}

bool GraspExecutor::GetCurrentPose(Pose3D& pose) {
    if (!arm_) return false;

    manip_joint_t joints;
    std::memset(&joints, 0, sizeof(joints));
    manip_pose_t mp;
    std::memset(&mp, 0, sizeof(mp));
    int ret = manip_get_state(arm_, &joints, &mp);
    if (ret != MANIP_OK) return false;

    pose.x = mp.x;
    pose.y = mp.y;
    pose.z = mp.z;
    pose.qw = mp.qw;
    pose.qx = mp.qx;
    pose.qy = mp.qy;
    pose.qz = mp.qz;
    return true;
}

void GraspExecutor::Tick(float dt_s) {
    if (arm_) manip_tick(arm_, dt_s);
    if (gripper_) grasp_tick(gripper_, dt_s);
}

// --- Private ---

GraspResult GraspExecutor::MoveToJoints(const std::vector<float>& joints) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    std::string limit_detail;
    if (!JointsWithinLimits(joints, config_.joint_limits, &limit_detail)) {
        std::cerr << "[GraspExecutor] joint command rejected: "
                    << limit_detail << std::endl;
        return GraspResult::OUT_OF_RANGE;
    }

    manip_joint_t target;
    target.count = static_cast<uint8_t>(joints.size());
    for (size_t i = 0; i < joints.size() && i < MANIP_MAX_JOINTS; i++) {
        target.joints[i] = joints[i];
    }

    int ret = manip_move_joints(arm_, &target, config_.move_speed);
    if (ret != MANIP_OK) {
        active_target_joints_.clear();
        std::cerr << "[GraspExecutor] move_joints failed: " << ret << std::endl;
        return GraspResult::MOVE_FAILED;
    }
    active_target_joints_.clear();
    active_target_joints_.reserve(target.count);
    for (uint8_t index = 0; index < target.count; ++index) {
        active_target_joints_.push_back(joints[index]);
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToJointsCoordinated(
    const std::vector<float>& joints) {
    std::cout << "[GraspExecutor] coordinated joint motion: continuous "
                "synchronized progress"
                << std::endl;
    constexpr float kObserveFinalJointToleranceRad = 0.060f;
    return ExecuteContinuousJointPath(
        {joints}, config_.move_speed, config_.move_speed,
        kObserveFinalJointToleranceRad);
}

bool GraspExecutor::GetCurrentJoints(std::vector<float>& joints) {
    if (!arm_) return false;

    constexpr int kMaximumReadAttempts = 5;
    int last_result = MANIP_ERR_CONNECT;
    for (int attempt = 0; attempt < kMaximumReadAttempts; ++attempt) {
        manip_joint_t current = {};
        last_result = manip_get_state(arm_, &current, nullptr);
        if (last_result == MANIP_OK && current.count > 0) {
            joints.clear();
            joints.reserve(current.count);
            for (uint8_t index = 0; index < current.count; ++index) {
                joints.push_back(current.joints[index]);
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    std::cerr << "[GraspExecutor] Failed to read current joints after "
            << kMaximumReadAttempts << " attempts, result="
            << last_result << std::endl;
    return false;
}

bool GraspExecutor::NeedsCollisionAvoidance(
    const std::vector<float>& current_joints,
    const std::vector<float>& target_joints) {
    const auto& ca = config_.collision_avoidance;
    if (!ca.enabled) return false;
    if (target_joints.size() < 2 || current_joints.size() < 2) return false;

    float target_j0 = target_joints[0];
    float target_j1 = target_joints[1];
    float current_j0 = current_joints[0];
    float current_j1 = current_joints[1];

    // The base-collision guard is only needed for a meaningful base sweep.
    // Small corrections can safely keep the arm in its retracted posture.
    constexpr float kMinimumBaseSweepRad = 0.15f;
    if (std::fabs(target_j0 - current_j0) <= kMinimumBaseSweepRad) {
        return false;
    }

    bool target_shoulder_danger = (target_j1 < ca.shoulder_threshold);
    bool current_shoulder_danger = (current_j1 < ca.shoulder_threshold);

    // joint0 是否在危险区内
    auto in_danger_zone = [&](float j0) -> bool {
        return j0 > ca.base_danger_min && j0 < ca.base_danger_max;
    };

    // joint0 运动路径是否穿越危险区
    // (从 a 到 b 的过程中是否经过 [danger_min, danger_max])
    auto path_crosses_danger = [&](float a, float b) -> bool {
        if (in_danger_zone(a) || in_danger_zone(b)) return true;
        // a 和 b 都在安全区，看是否在危险区两侧
        bool a_below = (a <= ca.base_danger_min);
        bool b_below = (b <= ca.base_danger_min);
        // 如果一个在左侧一个在右侧，路径必穿越危险区
        return (a_below != b_below);
    };

    // 情况A: 目标 joint1 在危险区，且 joint0 运动路径穿越或在危险区内
    if (target_shoulder_danger && path_crosses_danger(current_j0, target_j0)) {
        return true;
    }

    // 情况B: 当前 joint1 在危险区，且 joint0 运动路径穿越或在危险区内
    // (从安全区外到安全区外，但路径穿越；或当前/目标在危险区内)
    if (current_shoulder_danger && path_crosses_danger(current_j0, target_j0)) {
        return true;
    }

    return false;
}

GraspResult GraspExecutor::MoveToJointsCollisionSafe(
    const std::vector<float>& target_joints) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    const auto& ca = config_.collision_avoidance;
    if (!ca.enabled || target_joints.size() < 2) {
        // 碰撞避免未启用，直接运动
        GraspResult result = MoveToJoints(target_joints);
        return result;
    }

    // 读取当前关节角
    std::vector<float> current_joints;
    if (!GetCurrentJoints(current_joints) || current_joints.size() < 2) {
        std::cerr << "[GraspExecutor] Cannot read current joints for collision check"
                    << std::endl;
        return MoveToJoints(target_joints);
    }

    if (!NeedsCollisionAvoidance(current_joints, target_joints)) {
        // 无碰撞风险，直接运动
        return MoveToJoints(target_joints);
    }

    std::vector<float> bounded_current_joints = current_joints;
    if (!ClampJointsToLimits(
            bounded_current_joints, config_.joint_limits)) {
        std::cerr << "[GraspExecutor] Invalid hardware joint limits"
                    << std::endl;
        return GraspResult::OUT_OF_RANGE;
    }

    // === 需要碰撞避免: 分步运动 ===
    std::cout << "[GraspExecutor] COLLISION AVOIDANCE: splitting motion"
                << std::endl;
    std::cout << "  current j0=" << current_joints[0]
                << " j1=" << current_joints[1] << std::endl;
    std::cout << "  target  j0=" << target_joints[0]
                << " j1=" << target_joints[1] << std::endl;

    float current_j0 = current_joints[0];
    float current_j1 = current_joints[1];
    float target_j0 = target_joints[0];
    float target_j1 = target_joints[1];

    bool current_shoulder_danger = (current_j1 < ca.shoulder_threshold);
    bool target_shoulder_danger = (target_j1 < ca.shoulder_threshold);

    auto in_danger_zone = [&](float j0) -> bool {
        return j0 > ca.base_danger_min && j0 < ca.base_danger_max;
    };

    GraspResult result;

    if (current_shoulder_danger && !target_shoulder_danger) {
        // 情况: 当前 j1 在危险区，目标 j1 不在危险区
        // 策略: 先把 j1 抬到安全值 (保持 j0 不动)，再执行完整运动
        std::cout << "  Strategy: lift shoulder first (j1 -> "
                    << ca.shoulder_threshold << ")" << std::endl;

        std::vector<float> step1_joints = bounded_current_joints;
        step1_joints[1] = ca.shoulder_threshold;  // 抬到阈值

        std::cout << "  Step 1: lift j1 to safe threshold" << std::endl;
        result = MoveToJoints(step1_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        std::cout << "  Step 2: move all joints to target" << std::endl;
        result = MoveToJoints(target_joints);
        return result;

    } else if (!current_shoulder_danger && target_shoulder_danger) {
        // 情况: 当前 j1 安全，目标 j1 在危险区
        // 策略: 先把 j0 转到安全位置，再执行完整运动
        float safe_j0;
        if (!in_danger_zone(target_j0)) {
            safe_j0 = target_j0;  // 目标 j0 本身安全，直接用
        } else {
            // 选择最近的安全边界
            float dist_to_min = std::fabs(current_j0 - (ca.base_danger_min - ca.base_safe_margin));
            float dist_to_max = std::fabs(current_j0 - (ca.base_danger_max + ca.base_safe_margin));
            safe_j0 = (dist_to_min <= dist_to_max)
                ? (ca.base_danger_min - ca.base_safe_margin)
                : (ca.base_danger_max + ca.base_safe_margin);
        }

        std::cout << "  Strategy: rotate base first (j0 -> " << safe_j0 << ")"
                    << std::endl;

        std::vector<float> step1_joints = bounded_current_joints;
        step1_joints[0] = safe_j0;

        std::cout << "  Step 1: rotate base to safe position" << std::endl;
        result = MoveToJoints(step1_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        std::cout << "  Step 2: move all joints to target" << std::endl;
        result = MoveToJoints(target_joints);
        return result;

    } else {
        // 情况: 当前和目标 j1 都在危险区 (或其他复杂情况)
        // 策略: 先抬 j1 → 再转 j0 → 最后完整运动
        float safe_j0;
        if (!in_danger_zone(target_j0)) {
            safe_j0 = target_j0;
        } else {
            float dist_to_min = std::fabs(current_j0 - (ca.base_danger_min - ca.base_safe_margin));
            float dist_to_max = std::fabs(current_j0 - (ca.base_danger_max + ca.base_safe_margin));
            safe_j0 = (dist_to_min <= dist_to_max)
                ? (ca.base_danger_min - ca.base_safe_margin)
                : (ca.base_danger_max + ca.base_safe_margin);
        }

        std::cout << "  Strategy: lift shoulder + rotate base (complex)"
                    << std::endl;

        // Step 1: 抬 j1 到安全值
        std::vector<float> step1_joints = bounded_current_joints;
        step1_joints[1] = ca.shoulder_threshold;

        std::cout << "  Step 1: lift j1 to safe threshold" << std::endl;
        result = MoveToJoints(step1_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        // Step 2: 转 j0 到安全位置
        std::vector<float> step2_joints = step1_joints;
        step2_joints[0] = safe_j0;

        std::cout << "  Step 2: rotate base to safe position j0=" << safe_j0
                    << std::endl;
        result = MoveToJoints(step2_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        // Step 3: 完整运动到目标
        std::cout << "  Step 3: move all joints to target" << std::endl;
        result = MoveToJoints(target_joints);
        return result;
    }
}

GraspResult GraspExecutor::SolveIK(const Pose3D& pose, std::vector<float>& joints) {
    if (!arm_ || !kin_) return GraspResult::MOVE_FAILED;

    const auto ik_start = std::chrono::steady_clock::now();

    manip_pose_t target;
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    manip_joint_t solved;
    std::memset(&solved, 0, sizeof(solved));
    int ret = manip_solve_target_joints(arm_, &target, &solved);
    const auto ik_end = std::chrono::steady_clock::now();
    if (config_.performance_log_enabled) {
        const auto ik_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(ik_end - ik_start)
                .count();
        std::cout << "[Timing] component=IK elapsed_ms=" << ik_ms
                    << " mode=direct ret=" << ret << std::endl;
    }
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] solve_target_joints failed: " << ret << std::endl;
        if (ret == MANIP_ERR_PARAM) return GraspResult::IK_FAILED;
        return GraspResult::MOVE_FAILED;
    }

    joints.clear();
    joints.reserve(solved.count);
    for (uint8_t i = 0; i < solved.count; ++i) {
        joints.push_back(solved.joints[i]);
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::SolveIKSide(const Pose3D& pose,
                                        int timeout_ms,
                                        std::vector<float>& joints,
                                        std::string* detail,
                                        const std::vector<float>* path_start) {
    if (!kin_) return GraspResult::MOVE_FAILED;
    if (!arm_path_safety_ || !support_plane_.valid) {
        if (detail) *detail = "side grasp support surface is unavailable";
        return GraspResult::OUT_OF_RANGE;
    }

    kin_pose_t target = {};
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    const int joint_count = kin_get_num_joints(kin_);
    if (joint_count < 5 || joint_count > KIN_MAX_JOINTS) {
        if (detail) *detail = "side IK requires five arm joints";
        return GraspResult::IK_FAILED;
    }
    const int arm_joint_count = std::min(
        joint_count, static_cast<int>(config_.observe_joints.size()));
    if (arm_joint_count < 5) {
        if (detail) *detail = "side IK arm joint configuration is incomplete";
        return GraspResult::IK_FAILED;
    }
    std::vector<double> lower(joint_count);
    std::vector<double> upper(joint_count);
    if (kin_get_joint_limits(kin_, lower.data(), upper.data()) != KIN_OK) {
        if (detail) *detail = "failed to read IK joint limits";
        return GraspResult::IK_FAILED;
    }
    if (!ApplyJointLimits(
            config_.joint_limits, lower, upper, detail)) {
        return GraspResult::IK_FAILED;
    }

    const auto desired_y = ToolAxisY(
        pose.qw, pose.qx, pose.qy, pose.qz);
    const auto desired_z = ToolAxisZ(
        pose.qw, pose.qx, pose.qy, pose.qz);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(20, timeout_ms));

    struct SideSolution {
        bool valid = false;
        double score = std::numeric_limits<double>::max();
        double position_error_m = std::numeric_limits<double>::max();
        double approach_error_deg = 180.0;
        double opening_error_deg = 180.0;
        kin_joints_t joints{};
    } best;

    std::vector<float> reference_joints;
    if (path_start) {
        reference_joints = *path_start;
    } else {
        GetCurrentJoints(reference_joints);
    }
    const bool has_reference_joints =
        reference_joints.size() >= static_cast<size_t>(arm_joint_count);
    if (!has_reference_joints) {
        if (detail) *detail = "failed to read side grasp start joints";
        return GraspResult::MOVE_FAILED;
    }

    constexpr double kMaxPositionErrorM = 0.004;
    constexpr double kMaxApproachErrorDeg = 25.0;
    // The five arm joints cannot independently satisfy both side-grasp axes.
    // Keep the TCP precise while allowing the small wrist-axis deviation
    // observed at fixed-jaw-safe lateral offsets.
    constexpr double kMaxOpeningErrorDeg = 12.0;
    std::string safety_rejection;
    bool position_solution_found = false;
    double closest_orientation_score =
        std::numeric_limits<double>::max();
    double closest_position_error_m =
        std::numeric_limits<double>::max();
    double closest_approach_error_deg = 180.0;
    double closest_opening_error_deg = 180.0;
    std::array<double, 3> closest_approach_axis{};
    std::array<double, 3> closest_opening_axis{};

    constexpr std::array<double, 8> kFlexFractions = {
        0.005, 0.03, 0.08, 0.15, 0.25, 0.40, 0.60, 0.80};
    for (size_t attempt = 0; attempt < kFlexFractions.size(); ++attempt) {
        if (std::chrono::steady_clock::now() >= deadline) break;
        kin_joints_t seed = {};
        seed.count = static_cast<uint8_t>(joint_count);
        for (int joint = 0; joint < joint_count; ++joint) {
            const double midpoint = 0.5 * (lower[joint] + upper[joint]);
            // Try the elbow-up side-grasp branch before generic samples.
            if (attempt == 0 && joint < arm_joint_count) {
                seed.q[joint] = reference_joints[joint];
            } else if (attempt == 1 && joint == 0) {
                seed.q[joint] = std::clamp(
                    -std::atan2(
                        static_cast<double>(pose.y),
                        static_cast<double>(pose.x)),
                    lower[joint], upper[joint]);
            } else if (attempt == 1 && joint == 1) {
                seed.q[joint] = lower[joint] +
                    0.35 * (upper[joint] - lower[joint]);
            } else if (attempt == 1 && joint == 2) {
                seed.q[joint] = lower[joint] +
                    0.65 * (upper[joint] - lower[joint]);
            } else if (attempt == 1 && joint == 3) {
                seed.q[joint] = lower[joint] +
                    0.40 * (upper[joint] - lower[joint]);
            } else if (attempt == 1 && joint == 4) {
                seed.q[joint] = std::clamp(
                    0.0, lower[joint], upper[joint]);
            } else if (attempt == 2) {
                seed.q[joint] = std::clamp(
                    0.0, lower[joint], upper[joint]);
            } else if (joint <
                        static_cast<int>(config_.observe_joints.size())) {
                seed.q[joint] = config_.observe_joints[joint];
            } else {
                seed.q[joint] = midpoint;
            }
            if (attempt >= 2 && (joint == 1 || joint == 2)) {
                const double fraction = attempt % 2 == 0 ? 0.35 : 0.65;
                seed.q[joint] = lower[joint] +
                    fraction * (upper[joint] - lower[joint]);
            }
        }
        if (attempt > 2) {
            seed.q[3] = lower[3] + kFlexFractions[attempt] *
                (upper[3] - lower[3]);
            seed.q[4] = 0.5 * (lower[4] + upper[4]);
        }
        for (int joint = 0; joint < joint_count; ++joint) {
            seed.q[joint] = std::clamp(
                seed.q[joint], lower[joint], upper[joint]);
        }

        kin_ik_params_t parameters = {};
        parameters.epsilon = 1e-3;
        parameters.position_weight = 1.0;
        parameters.timeout_s = 0.025;
        kin_joints_t solved = {};
        const kin_joints_t* seed_pointer = attempt == 2 ? nullptr : &seed;
        const kin_ik_params_t* parameter_pointer =
            attempt == 2 ? nullptr : &parameters;
        if (kin_inverse(
                kin_, &target, seed_pointer, parameter_pointer,
                &solved) != KIN_OK) {
            continue;
        }
        position_solution_found = true;

        for (int roll_step = 0; roll_step <= 32; ++roll_step) {
            kin_joints_t rolled = solved;
            rolled.q[4] = lower[4] +
                static_cast<double>(roll_step) / 32.0 *
                (upper[4] - lower[4]);
            bool within_limits = true;
            for (int joint = 0; joint < arm_joint_count; ++joint) {
                if (rolled.q[joint] < lower[joint] ||
                    rolled.q[joint] > upper[joint]) {
                    within_limits = false;
                    break;
                }
            }
            if (!within_limits) continue;
            kin_pose_t achieved = {};
            if (kin_forward(kin_, &rolled, &achieved) != KIN_OK) continue;

            const double dx = achieved.x - pose.x;
            const double dy = achieved.y - pose.y;
            const double dz = achieved.z - pose.z;
            const double position_error = std::sqrt(
                dx * dx + dy * dy + dz * dz);
            const auto achieved_z = ToolAxisZ(
                achieved.qw, achieved.qx, achieved.qy, achieved.qz);
            const auto achieved_y = ToolAxisY(
                achieved.qw, achieved.qx, achieved.qy, achieved.qz);
            const double approach_error = AxisAngleDegrees(
                achieved_z, desired_z);
            const double opening_error = AxisAngleDegrees(
                achieved_y, desired_y);
            const double orientation_score =
                position_error * 1000.0 + approach_error + opening_error;
            if (orientation_score < closest_orientation_score) {
                closest_orientation_score = orientation_score;
                closest_position_error_m = position_error;
                closest_approach_error_deg = approach_error;
                closest_opening_error_deg = opening_error;
                closest_approach_axis = achieved_z;
                closest_opening_axis = achieved_y;
            }
            if (position_error > kMaxPositionErrorM ||
                approach_error > kMaxApproachErrorDeg ||
                opening_error > kMaxOpeningErrorDeg) {
                continue;
            }
            double joint_travel = 0.0;
            std::vector<float> rolled_joints;
            rolled_joints.reserve(arm_joint_count);
            for (int joint = 0; joint < arm_joint_count; ++joint) {
                rolled_joints.push_back(static_cast<float>(rolled.q[joint]));
                joint_travel += std::fabs(
                    rolled.q[joint] - reference_joints[joint]);
            }
            const double score = position_error * 1000.0 +
                approach_error + 0.25 * opening_error +
                0.5 * joint_travel;
            if (score >= best.score) continue;

            const ArmPathSafetyResult path_result =
                arm_path_safety_->CheckPath(
                    reference_joints, rolled_joints, support_plane_,
                    config_.support_surface_clearance_m,
                    config_.path_joint_step_rad);
            if (!path_result.safe) {
                safety_rejection = path_result.detail;
                continue;
            }

            best.valid = true;
            best.score = score;
            best.position_error_m = position_error;
            best.approach_error_deg = approach_error;
            best.opening_error_deg = opening_error;
            best.joints = rolled;
        }
        // Consecutive Cartesian waypoints use the previous valid solution
        // as their first seed. Once that branch satisfies pose and path
        // constraints, keep it instead of searching unrelated IK branches.
        if (attempt == 0 && best.valid) break;
    }

    if (!best.valid || best.position_error_m > kMaxPositionErrorM ||
        best.approach_error_deg > kMaxApproachErrorDeg ||
        best.opening_error_deg > kMaxOpeningErrorDeg) {
        if (detail) {
            if (!best.valid) {
                if (!safety_rejection.empty()) {
                    *detail = "side IK path is unsafe: " +
                        safety_rejection;
                } else if (position_solution_found) {
                    *detail = "side IK orientation mismatch: position=" +
                        std::to_string(closest_position_error_m * 1000.0) +
                        "mm approach=" +
                        std::to_string(closest_approach_error_deg) +
                        "deg opening=" +
                        std::to_string(closest_opening_error_deg) +
                        "deg approach_axis=[" +
                        std::to_string(closest_approach_axis[0]) + "," +
                        std::to_string(closest_approach_axis[1]) + "," +
                        std::to_string(closest_approach_axis[2]) +
                        "] opening_axis=[" +
                        std::to_string(closest_opening_axis[0]) + "," +
                        std::to_string(closest_opening_axis[1]) + "," +
                        std::to_string(closest_opening_axis[2]) + "]";
                } else {
                    *detail = "side IK did not converge";
                }
            } else {
                *detail = "side IK orientation error: approach=" +
                    std::to_string(best.approach_error_deg) +
                    "deg opening=" +
                    std::to_string(best.opening_error_deg) + "deg";
            }
        }
        return GraspResult::IK_FAILED;
    }

    joints.clear();
    joints.reserve(arm_joint_count);
    for (int joint = 0; joint < arm_joint_count; ++joint) {
        joints.push_back(static_cast<float>(best.joints.q[joint]));
    }
    if (config_.performance_log_enabled) {
        std::cout << "[Timing] component=IK mode=side"
                    << " position_error_mm="
                    << best.position_error_m * 1000.0
                    << " approach_error_deg=" << best.approach_error_deg
                    << " opening_error_deg=" << best.opening_error_deg
                    << " result=success" << std::endl;
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::SolveIKFast(const Pose3D& pose,
                                        bool use_top_constraints,
                                        int timeout_ms,
                                        std::vector<float>& joints) {
    if (!kin_) return GraspResult::MOVE_FAILED;

    kin_pose_t target = {};
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    const int joint_count = kin_get_num_joints(kin_);
    if (joint_count <= 0) return GraspResult::IK_FAILED;
    std::vector<double> lower(joint_count);
    std::vector<double> upper(joint_count);
    kin_get_joint_limits(kin_, lower.data(), upper.data());
    if (!ApplyJointLimits(config_.joint_limits, lower, upper)) {
        return GraspResult::IK_FAILED;
    }

    const int attempt_count = use_top_constraints ? 8 : 2;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(10, timeout_ms));
    for (int attempt = 0; attempt < attempt_count; ++attempt) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        const int attempts_left = attempt_count - attempt;
        const float attempt_timeout_s = std::clamp(
            std::chrono::duration<float>(deadline - now).count() /
                static_cast<float>(attempts_left),
            0.005f, 0.025f);
        kin_joints_t seed = {};
        seed.count = static_cast<uint8_t>(joint_count);
        for (int joint = 0; joint < joint_count; ++joint) {
            if (attempt == 0 && joint <
                static_cast<int>(config_.observe_joints.size())) {
                seed.q[joint] = config_.observe_joints[joint];
            } else if (attempt == 1) {
                seed.q[joint] = 0.5 * (lower[joint] + upper[joint]);
            } else {
                const double fraction = std::fmod(
                    0.5 + 0.61803398875 *
                        static_cast<double>(attempt + joint * 3),
                    1.0);
                seed.q[joint] = lower[joint] +
                    fraction * (upper[joint] - lower[joint]);
            }
        }
        if (use_top_constraints && attempt > 0) {
            constexpr float constraint_fractions[] = {
                0.50f, 0.20f, 0.80f, 0.35f, 0.65f, 0.10f, 0.90f};
            for (const JointConstraint& constraint :
                config_.joint_constraints) {
                if (constraint.joint_index < 0 ||
                    constraint.joint_index >= joint_count) {
                    continue;
                }
                const float fraction = constraint_fractions[attempt - 1];
                seed.q[constraint.joint_index] =
                    constraint.min_rad + fraction *
                    (constraint.max_rad - constraint.min_rad);
            }
        }
        for (int joint = 0; joint < joint_count; ++joint) {
            seed.q[joint] = std::clamp(
                seed.q[joint], lower[joint], upper[joint]);
        }

        kin_ik_params_t parameters = {};
        parameters.epsilon = 1e-3;
        parameters.position_weight = 1.0;
        parameters.timeout_s = attempt_timeout_s;

        kin_joints_t solved = {};
        if (kin_inverse(kin_, &target, &seed, &parameters, &solved) != KIN_OK) {
            continue;
        }

        bool constraints_valid = true;
        for (int joint = 0; joint < solved.count; ++joint) {
            if (solved.q[joint] < lower[joint] ||
                solved.q[joint] > upper[joint]) {
                constraints_valid = false;
                break;
            }
        }
        if (use_top_constraints) {
            for (const JointConstraint& constraint : config_.joint_constraints) {
                if (constraint.joint_index >= 0 &&
                    constraint.joint_index < solved.count) {
                    const float value = static_cast<float>(
                        solved.q[constraint.joint_index]);
                    if (value < constraint.min_rad ||
                        value > constraint.max_rad) {
                        constraints_valid = false;
                        break;
                    }
                }
            }
        }
        if (!constraints_valid) continue;

        joints.clear();
        joints.reserve(solved.count);
        for (int joint = 0; joint < solved.count; ++joint) {
            joints.push_back(static_cast<float>(solved.q[joint]));
        }
        return GraspResult::SUCCESS;
    }
    return GraspResult::IK_FAILED;
}

GraspResult GraspExecutor::ValidateGraspPoses(
    const Pose3D& pre_grasp_pose,
    const Pose3D& grasp_pose,
    const Pose3D& retreat_pose,
    const Pose3D& lift_pose,
    float entry_clearance_z_m,
    float grasp_yaw_rad,
    bool use_top_constraints,
    int timeout_ms,
    std::string* detail) {
    validated_side_staging_joints_.clear();
    validated_side_sweep_joints_.clear();
    validated_side_entry_pose_ = Pose3D{};
    validated_side_entry_joints_.clear();
    validated_side_poses_.clear();
    validated_side_joint_path_.clear();
    validated_side_path_index_ = 0;
    if (!use_top_constraints) {
        const auto validation_deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(std::max(30, timeout_ms));
        const auto remaining_budget_ms = [&validation_deadline]() {
            return static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    validation_deadline -
                    std::chrono::steady_clock::now()).count());
        };
        if (!std::isfinite(entry_clearance_z_m) ||
            entry_clearance_z_m < pre_grasp_pose.z) {
            if (detail) {
                *detail = "side entry clearance height is invalid";
            }
            return GraspResult::OUT_OF_RANGE;
        }
        Pose3D elevated_pre_grasp_pose = pre_grasp_pose;
        elevated_pre_grasp_pose.z = entry_clearance_z_m;
        std::vector<Pose3D> side_path = {elevated_pre_grasp_pose};
        const std::vector<Pose3D> descent_path =
            BuildSideCartesianPath(
                elevated_pre_grasp_pose, pre_grasp_pose, 0.025f);
        side_path.insert(
            side_path.end(), descent_path.begin(), descent_path.end());
        const std::vector<Pose3D> approach_path =
            BuildSideCartesianPath(
                pre_grasp_pose, grasp_pose, 0.030f);
        side_path.insert(
            side_path.end(), approach_path.begin(), approach_path.end());
        const std::vector<Pose3D> vertical_lift_path =
            BuildSideCartesianPath(
                grasp_pose, retreat_pose, 0.025f);
        side_path.insert(
            side_path.end(),
            vertical_lift_path.begin(), vertical_lift_path.end());
        const std::vector<Pose3D> lift_path = BuildSideLiftPath(
            retreat_pose, lift_pose);
        side_path.insert(
            side_path.end(), lift_path.begin(), lift_path.end());
        std::vector<float> current_joints;
        if (!GetCurrentJoints(current_joints)) {
            if (detail) *detail = "failed to read side-ready start joints";
            return GraspResult::MOVE_FAILED;
        }
        const ArmPathSafetyResult ready_path =
            arm_path_safety_->CheckPath(
                current_joints, config_.side_ready_joints,
                support_plane_, config_.support_surface_clearance_m,
                config_.path_joint_step_rad);
        if (!ready_path.safe) {
            if (detail) {
                *detail = "side-ready path is unsafe: " +
                    ready_path.detail;
            }
            return GraspResult::OUT_OF_RANGE;
        }
        std::vector<float> staging_joints;
        std::vector<float> sweep_joints;
        int remaining_ms = remaining_budget_ms();
        if (remaining_ms <= 0) {
            if (detail) *detail = "side path planning deadline exceeded";
            return GraspResult::TIMEOUT;
        }
        const GraspResult sweep_result = PlanSideJoint0Sweep(
            pre_grasp_pose, entry_clearance_z_m,
            remaining_ms,
            staging_joints, sweep_joints, detail);
        if (sweep_result != GraspResult::SUCCESS) return sweep_result;

        remaining_ms = remaining_budget_ms();
        if (remaining_ms <= 0) {
            if (detail) *detail = "side path planning deadline exceeded";
            return GraspResult::TIMEOUT;
        }
        std::vector<std::vector<float>> joint_path;
        const GraspResult result = PlanSideJointPath(
            side_path, remaining_ms, joint_path, detail,
            &sweep_joints);
        if (result == GraspResult::SUCCESS) {
            std::vector<float> segment_start = sweep_joints;
            for (size_t index = 0; index < joint_path.size(); ++index) {
                float maximum_joint_delta = 0.0f;
                for (size_t joint = 0;
                    joint < segment_start.size() &&
                    joint < joint_path[index].size(); ++joint) {
                    maximum_joint_delta = std::max(
                        maximum_joint_delta,
                        std::fabs(
                            joint_path[index][joint] -
                            segment_start[joint]));
                }
                std::cout << "[GraspExecutor] side path waypoint="
                            << index
                            << " pose=[" << side_path[index].x << ","
                            << side_path[index].y << ","
                            << side_path[index].z << "]"
                            << " max_joint_delta="
                            << maximum_joint_delta << "rad"
                            << std::endl;
                segment_start = joint_path[index];
            }
            const ArmPathSafetyResult place_path =
                arm_path_safety_->CheckPath(
                    joint_path.back(), config_.place_joints,
                    support_plane_, config_.support_surface_clearance_m,
                    config_.path_joint_step_rad);
            if (!place_path.safe) {
                if (detail) {
                    *detail = "side place path is unsafe: " +
                        place_path.detail;
                }
                return GraspResult::OUT_OF_RANGE;
            }
            validated_side_staging_joints_ = std::move(staging_joints);
            validated_side_sweep_joints_ = std::move(sweep_joints);
            validated_side_entry_pose_ = side_path.front();
            validated_side_entry_joints_ = joint_path.front();
            side_path.erase(side_path.begin());
            joint_path.erase(joint_path.begin());
            validated_side_poses_ = std::move(side_path);
            validated_side_joint_path_ = std::move(joint_path);
            validated_side_path_index_ = 0;
        }
        return result;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(30, timeout_ms));
    // Top grasp execution uses the constrained solver. Validate with the
    // same solver so side-grasp fast-path assumptions cannot reject a pose
    // that the top-grasp execution path can execute.
    const Pose3D poses[] = {pre_grasp_pose, grasp_pose};
    const char* names[] = {"pre-grasp", "grasp"};

    constexpr size_t kPoseCount = sizeof(poses) / sizeof(poses[0]);
    for (size_t index = 0; index < kPoseCount; ++index) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            if (detail) *detail = "IK validation deadline exceeded";
            return GraspResult::TIMEOUT;
        }
        std::vector<float> joints;
        const GraspResult result = SolveIKConstrained(
            poses[index], joints);
        if (result != GraspResult::SUCCESS) {
            if (detail && detail->empty()) {
                *detail = std::string(names[index]) + " IK failed";
            }
            return result;
        }
        if (!std::isnan(grasp_yaw_rad) && joints.size() >= 5) {
            const float scale = config_.wrist_yaw_scale;
            if (std::fabs(scale) < 1e-6f) {
                if (detail) *detail = "invalid wrist yaw scale";
                return GraspResult::IK_FAILED;
            }
            const int joint_count = kin_get_num_joints(kin_);
            std::vector<double> lower(joint_count);
            std::vector<double> upper(joint_count);
            kin_get_joint_limits(kin_, lower.data(), upper.data());
            if (!ApplyJointLimits(config_.joint_limits, lower, upper)) {
                if (detail) *detail = "invalid hardware joint limits";
                return GraspResult::IK_FAILED;
            }
            float raw_wrist = 0.0f;
            float resolved_wrist = 0.0f;
            if (joint_count <= 4 ||
                !ResolveFixedJawWristYaw(
                    grasp_yaw_rad, joints[0], scale,
                    static_cast<float>(lower[4]),
                    static_cast<float>(upper[4]),
                    raw_wrist, resolved_wrist)) {
                if (detail) {
                    *detail = std::string(names[index]) +
                        " wrist yaw exceeds joint limit";
                }
                return GraspResult::IK_FAILED;
            }
        }
    }

    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::SolveIKConstrained(const Pose3D& pose,
                                                std::vector<float>& joints) {
    if (!kin_) return GraspResult::MOVE_FAILED;

    const auto ik_start = std::chrono::steady_clock::now();

    const auto& constraints = config_.joint_constraints;

    // 如果没有约束，退回普通 IK
    if (constraints.empty()) {
        return SolveIK(pose, joints);
    }

    kin_pose_t ik_target;
    ik_target.x  = pose.x;
    ik_target.y  = pose.y;
    ik_target.z  = pose.z;
    ik_target.qw = pose.qw;
    ik_target.qx = pose.qx;
    ik_target.qy = pose.qy;
    ik_target.qz = pose.qz;

    kin_ik_params_t ik_params = {};
    ik_params.epsilon = 1e-3;
    ik_params.position_weight = 1.0;
    ik_params.timeout_s = 0.1;

    int n_joints = kin_get_num_joints(kin_);
    std::vector<double> lower(n_joints), upper(n_joints);
    kin_get_joint_limits(kin_, lower.data(), upper.data());
    if (!ApplyJointLimits(config_.joint_limits, lower, upper)) {
        return GraspResult::IK_FAILED;
    }

    std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    bool found_valid = false;

    auto check_constraints = [&](const kin_joints_t& q) -> bool {
        for (int joint = 0; joint < q.count; ++joint) {
            if (q.q[joint] < lower[joint] || q.q[joint] > upper[joint]) {
                return false;
            }
        }
        for (const auto& c : constraints) {
            if (c.joint_index >= 0 && c.joint_index < q.count) {
                float val = static_cast<float>(q.q[c.joint_index]);
                if (val < c.min_rad || val > c.max_rad) return false;
            }
        }
        return true;
    };

    const int max_trials = config_.ik_max_trials;

    for (int trial = 0; trial < max_trials; ++trial) {
        kin_joints_t q_seed;
        q_seed.count = static_cast<uint8_t>(n_joints);

        if (trial == 0) {
            // 第一次用 observe_joints 作为种子
            auto& obs = config_.observe_joints;
            for (int j = 0; j < n_joints; ++j) {
                q_seed.q[j] = (j < static_cast<int>(obs.size())) ? obs[j] : 0.0;
            }
        } else {
            // 随机种子
            for (int j = 0; j < n_joints; ++j) {
                std::uniform_real_distribution<double> dist(lower[j], upper[j]);
                q_seed.q[j] = dist(rng);
            }
            // 对有约束的关节，强制种子在约束范围内
            for (const auto& c : constraints) {
                if (c.joint_index >= 0 && c.joint_index < n_joints) {
                    std::uniform_real_distribution<double> cdist(c.min_rad, c.max_rad);
                    q_seed.q[c.joint_index] = cdist(rng);
                }
            }
        }
        for (int joint = 0; joint < n_joints; ++joint) {
            q_seed.q[joint] = std::clamp(
                q_seed.q[joint], lower[joint], upper[joint]);
        }

        kin_joints_t q_result;
        int ik_ret = kin_inverse(kin_, &ik_target, &q_seed, &ik_params, &q_result);
        if (ik_ret != KIN_OK) continue;

        if (check_constraints(q_result)) {
            joints.clear();
            for (int j = 0; j < q_result.count; ++j) {
                joints.push_back(static_cast<float>(q_result.q[j]));
            }
            found_valid = true;
            if (config_.performance_log_enabled) {
                const auto ik_end = std::chrono::steady_clock::now();
                const auto ik_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        ik_end - ik_start)
                        .count();
                std::cout << "[Timing] component=IK elapsed_ms=" << ik_ms
                            << " mode=constrained trials=" << (trial + 1)
                            << " fallback=0 result=success" << std::endl;
            }
            break;
        }
    }

    if (!found_valid) {
        if (config_.performance_log_enabled) {
            const auto ik_end = std::chrono::steady_clock::now();
            const auto ik_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    ik_end - ik_start)
                    .count();
            std::cout << "[Timing] component=IK elapsed_ms=" << ik_ms
                        << " mode=constrained trials=" << max_trials
                        << " fallback=0 result=failed" << std::endl;
        }
        std::cerr << "[GraspExecutor] IK failed: no solution in "
                    << max_trials << " trials" << std::endl;
        return GraspResult::IK_FAILED;
    }

    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPoseWithIKJoints(const Pose3D& pose, float speed) {
    std::vector<float> joints;
    GraspResult result = SolveIK(pose, joints);
    if (result != GraspResult::SUCCESS) return result;

    std::cout << "[GraspExecutor] safe IK joints(rad): [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i];
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 通过碰撞安全路径执行
    float old_speed = config_.move_speed;
    config_.move_speed = speed;
    GraspResult move_result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed;
    if (move_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] safe move_joints failed" << std::endl;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPoseSide(const Pose3D& pose, float speed) {
    if (validated_side_path_index_ == 0 &&
        !validated_side_entry_joints_.empty()) {
        std::vector<float> current_joints;
        if (!GetCurrentJoints(current_joints)) {
            return GraspResult::MOVE_FAILED;
        }
        const ArmPathSafetyResult entry_path = arm_path_safety_->CheckPath(
            current_joints, validated_side_entry_joints_, support_plane_,
            config_.support_surface_clearance_m,
            config_.path_joint_step_rad);
        if (!entry_path.safe) {
            std::cerr << "[GraspExecutor] side grasp entry rejected: "
                        << entry_path.detail << std::endl;
            return GraspResult::OUT_OF_RANGE;
        }

        std::cout << "[GraspExecutor] moving from joint0 sweep to precise "
                    "side entry"
                    << std::endl;
        const GraspResult entry_result = ExecuteContinuousJointPath(
            {validated_side_entry_joints_}, speed, speed,
            config_.side_waypoint_joint_tolerance_rad,
            kSideMotionCompletionTimeoutMs);
        if (entry_result != GraspResult::SUCCESS) return entry_result;
        validated_side_entry_joints_.clear();
        if (validated_side_path_index_ == 0 &&
            !validated_side_poses_.empty() &&
            PosesMatch(
                validated_side_entry_pose_, validated_side_poses_.front())) {
            validated_side_path_index_ = 1;
        }
    }

    std::vector<std::vector<float>> joint_path;
    if (!TakeValidatedSidePath({pose}, joint_path)) {
        Pose3D current_pose{};
        if (!GetCurrentPose(current_pose)) {
            RecordResult(
                GraspResult::MOVE_FAILED, "move_to_side_pose",
                "failed to read current pose for side path replanning");
            return GraspResult::MOVE_FAILED;
        }
        const std::vector<Pose3D> poses = BuildSideCartesianPath(
            current_pose, pose, 0.030f);
        std::string detail;
        const GraspResult result = PlanSideJointPath(
            poses, 350, joint_path, &detail);
        if (result != GraspResult::SUCCESS) {
            std::cerr << "[GraspExecutor] side path planning failed: "
                    << detail << std::endl;
            RecordResult(result, "move_to_side_pose", detail);
            return result;
        }
    }
    const GraspResult result = ExecuteContinuousJointPath(
        joint_path, speed, speed,
        config_.side_waypoint_joint_tolerance_rad,
        kSideMotionCompletionTimeoutMs);
    if (result != GraspResult::SUCCESS) {
        RecordResult(
            result, "move_to_side_pose",
            "continuous side path execution failed");
        return result;
    }
    return CorrectSidePose(pose, speed, "move_to_side_pose");
}

GraspResult GraspExecutor::PlanSideJoint0Sweep(
    const Pose3D& pre_grasp_pose,
    float entry_clearance_z_m,
    int timeout_ms,
    std::vector<float>& staging_joints,
    std::vector<float>& sweep_joints,
    std::string* detail) {
    // The chassis aligns to a distance window because its minimum reliable
    // pulse is larger than a few millimetres. Allow the validated Cartesian
    // entry path to absorb the remaining offset instead of requesting an
    // unstable base micro-adjustment.
    constexpr float kMaximumPlanarErrorM = 0.050f;
    if (!kin_ || !arm_path_safety_) {
        if (detail) *detail = "side joint0 sweep is not initialized";
        return GraspResult::MOVE_FAILED;
    }
    if (config_.side_ready_joints.size() < 5) {
        if (detail) *detail = "side-ready joints must contain five joints";
        return GraspResult::IK_FAILED;
    }

    kin_joints_t ready_joints = {};
    ready_joints.count =
        static_cast<uint8_t>(config_.side_ready_joints.size());
    for (size_t index = 0;
        index < config_.side_ready_joints.size() &&
        index < KIN_MAX_JOINTS; ++index) {
        ready_joints.q[index] = config_.side_ready_joints[index];
    }
    kin_pose_t ready_pose = {};
    if (kin_forward(kin_, &ready_joints, &ready_pose) != KIN_OK) {
        if (detail) *detail = "side-ready FK failed";
        return GraspResult::IK_FAILED;
    }

    Pose3D staging_pose{};
    staging_pose.x = static_cast<float>(ready_pose.x);
    staging_pose.y = static_cast<float>(ready_pose.y);
    staging_pose.z = std::max(
        static_cast<float>(ready_pose.z), entry_clearance_z_m);
    staging_pose.qw = static_cast<float>(ready_pose.qw);
    staging_pose.qx = static_cast<float>(ready_pose.qx);
    staging_pose.qy = static_cast<float>(ready_pose.qy);
    staging_pose.qz = static_cast<float>(ready_pose.qz);
    const GraspResult staging_result = SolveIKSide(
        staging_pose, std::max(20, timeout_ms),
        staging_joints, detail,
        &config_.side_ready_joints);
    if (staging_result != GraspResult::SUCCESS) {
        if (detail && !detail->empty()) {
            *detail = "side staging lift failed: " + *detail;
        }
        return staging_result;
    }

    sweep_joints = staging_joints;
    sweep_joints[0] = -std::atan2(pre_grasp_pose.y, pre_grasp_pose.x);
    std::string limit_detail;
    if (!JointsWithinLimits(
            sweep_joints, config_.joint_limits, &limit_detail)) {
        if (detail) {
            *detail = "side joint0 sweep exceeds limits: " + limit_detail;
        }
        return GraspResult::OUT_OF_RANGE;
    }

    const ArmPathSafetyResult path_result = arm_path_safety_->CheckPath(
        staging_joints, sweep_joints, support_plane_,
        config_.support_surface_clearance_m,
        config_.path_joint_step_rad);
    if (!path_result.safe) {
        if (detail) {
            *detail = "side joint0 sweep is unsafe: " + path_result.detail;
        }
        return GraspResult::OUT_OF_RANGE;
    }

    kin_joints_t target = {};
    target.count = static_cast<uint8_t>(sweep_joints.size());
    for (size_t index = 0;
        index < sweep_joints.size() && index < KIN_MAX_JOINTS; ++index) {
        target.q[index] = sweep_joints[index];
    }
    kin_pose_t achieved = {};
    if (kin_forward(kin_, &target, &achieved) != KIN_OK) {
        if (detail) *detail = "side joint0 sweep FK failed";
        return GraspResult::IK_FAILED;
    }
    const float dx = static_cast<float>(achieved.x) - pre_grasp_pose.x;
    const float dy = static_cast<float>(achieved.y) - pre_grasp_pose.y;
    const float planar_error = std::sqrt(dx * dx + dy * dy);
    if (planar_error > kMaximumPlanarErrorM) {
        if (detail) {
            *detail = "side joint0 sweep cannot reach pre-grasp radius: " +
                std::to_string(planar_error * 1000.0f) +
                "mm planar error; realign the mobile base";
        }
        return GraspResult::OUT_OF_RANGE;
    }

    std::cout << "[GraspExecutor] side joint0 sweep planned: joint0="
                << staging_joints[0] << " -> " << sweep_joints[0]
                << " staging_z=" << staging_pose.z
                << " target_xy=[" << pre_grasp_pose.x << ","
                << pre_grasp_pose.y << "] achieved_xy=[" << achieved.x << ","
                << achieved.y << "] planar_error=" << planar_error << "m"
                << std::endl;
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

std::vector<Pose3D> GraspExecutor::BuildSideLiftPath(
    const Pose3D& retreat_pose, const Pose3D& lift_pose) const {
    constexpr float kMaximumLiftStepM = 0.04f;
    Pose3D clearance_pose = lift_pose;
    clearance_pose.z = std::clamp(
        clearance_pose.z, retreat_pose.z,
        retreat_pose.z + config_.side_lift_clearance_m);
    const float dx = clearance_pose.x - retreat_pose.x;
    const float dy = clearance_pose.y - retreat_pose.y;
    const float dz = clearance_pose.z - retreat_pose.z;
    const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int step_count = std::max(
        1, static_cast<int>(std::ceil(distance / kMaximumLiftStepM)));
    std::vector<Pose3D> path;
    path.reserve(step_count);
    for (int step = 1; step <= step_count; ++step) {
        const float ratio = static_cast<float>(step) /
            static_cast<float>(step_count);
        Pose3D pose = retreat_pose;
        pose.x += ratio * dx;
        pose.y += ratio * dy;
        pose.z += ratio * dz;
        path.push_back(pose);
    }
    return path;
}

std::vector<Pose3D> GraspExecutor::BuildSideCartesianPath(
    const Pose3D& start_pose,
    const Pose3D& end_pose,
    float maximum_step_m) const {
    const float dx = end_pose.x - start_pose.x;
    const float dy = end_pose.y - start_pose.y;
    const float dz = end_pose.z - start_pose.z;
    const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int step_count = std::max(
        1, static_cast<int>(std::ceil(
            distance / std::max(maximum_step_m, 0.001f))));
    std::vector<Pose3D> path;
    path.reserve(step_count);
    for (int step = 1; step <= step_count; ++step) {
        const float ratio =
            static_cast<float>(step) / static_cast<float>(step_count);
        Pose3D pose = start_pose;
        pose.x += ratio * dx;
        pose.y += ratio * dy;
        pose.z += ratio * dz;
        path.push_back(pose);
    }
    return path;
}

GraspResult GraspExecutor::PlanSideJointPath(
    const std::vector<Pose3D>& poses,
    int timeout_ms,
    std::vector<std::vector<float>>& joint_path,
    std::string* detail,
    const std::vector<float>* start_override) {
    joint_path.clear();
    if (poses.empty()) {
        if (detail) *detail = "side path contains no waypoints";
        return GraspResult::IK_FAILED;
    }
    if (!support_plane_.valid) {
        if (detail) *detail = "side grasp support surface is unavailable";
        return GraspResult::OUT_OF_RANGE;
    }

    std::vector<float> start_joints;
    if (start_override) {
        start_joints = *start_override;
    } else if (!GetCurrentJoints(start_joints)) {
        if (detail) *detail = "failed to read side grasp start joints";
        return GraspResult::MOVE_FAILED;
    }

    const int path_timeout_ms = std::max(20, timeout_ms);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(path_timeout_ms);
    for (size_t index = 0; index < poses.size(); ++index) {
        size_t cached_index = index;
        for (size_t previous = 0; previous < index; ++previous) {
            if (PosesMatch(poses[index], poses[previous])) {
                cached_index = previous;
                break;
            }
        }
        if (cached_index < index) {
            const std::vector<float> cached_joints =
                joint_path[cached_index];
            const ArmPathSafetyResult path_result =
                arm_path_safety_->CheckPath(
                    start_joints, cached_joints, support_plane_,
                    config_.support_surface_clearance_m,
                    config_.path_joint_step_rad);
            if (!path_result.safe) {
                if (detail) {
                    *detail = "side waypoint " +
                        std::to_string(index + 1) + "/" +
                        std::to_string(poses.size()) + ": " +
                        path_result.detail;
                }
                return GraspResult::OUT_OF_RANGE;
            }
            joint_path.push_back(cached_joints);
            start_joints = cached_joints;
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            if (detail) *detail = "side path planning deadline exceeded";
            return GraspResult::TIMEOUT;
        }
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now)
                .count());
        const int waypoint_timeout_ms = std::max(
            20, remaining_ms /
                static_cast<int>(poses.size() - index));
        std::vector<float> solved_joints;
        std::string waypoint_detail;
        const GraspResult result = SolveIKSide(
            poses[index], waypoint_timeout_ms, solved_joints,
            &waypoint_detail, &start_joints);
        if (result != GraspResult::SUCCESS) {
            if (detail) {
                *detail = "side waypoint " + std::to_string(index + 1) +
                    "/" + std::to_string(poses.size()) + ": " +
                    waypoint_detail;
            }
            return result;
        }
        joint_path.push_back(solved_joints);
        start_joints = std::move(solved_joints);
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

bool GraspExecutor::TakeValidatedSidePath(
    const std::vector<Pose3D>& poses,
    std::vector<std::vector<float>>& joint_path) {
    joint_path.clear();
    if (poses.empty() ||
        validated_side_poses_.size() !=
            validated_side_joint_path_.size()) {
        return false;
    }

    size_t consumed_count = poses.size();
    if (poses.size() == 1) {
        size_t match_index = validated_side_path_index_;
        while (match_index < validated_side_poses_.size() &&
            !PosesMatch(
                poses.front(), validated_side_poses_[match_index])) {
            ++match_index;
        }
        if (match_index == validated_side_poses_.size()) {
            return false;
        }
        consumed_count =
            match_index - validated_side_path_index_ + 1;
    } else {
        if (validated_side_path_index_ + poses.size() >
            validated_side_poses_.size()) {
            return false;
        }
        for (size_t index = 0; index < poses.size(); ++index) {
            if (!PosesMatch(
                    poses[index],
                    validated_side_poses_[
                        validated_side_path_index_ + index])) {
                return false;
            }
        }
    }

    std::vector<float> start_joints;
    if (!GetCurrentJoints(start_joints)) return false;
    for (size_t index = 0; index < consumed_count; ++index) {
        const std::vector<float>& target_joints =
            validated_side_joint_path_[validated_side_path_index_ + index];
        const ArmPathSafetyResult path_result = arm_path_safety_->CheckPath(
            start_joints, target_joints, support_plane_,
            config_.support_surface_clearance_m,
            config_.path_joint_step_rad);
        if (!path_result.safe) {
            std::cerr << "[GraspExecutor] cached side path rejected: "
                    << path_result.detail << std::endl;
            return false;
        }
        joint_path.push_back(target_joints);
        start_joints = target_joints;
    }
    validated_side_path_index_ += consumed_count;
    return true;
}

GraspResult GraspExecutor::ExecuteContinuousJointPath(
    const std::vector<std::vector<float>>& joint_path,
    float first_speed,
    float remaining_speed,
    float final_tolerance_rad,
    int completion_timeout_ms) {
    if (joint_path.empty()) return GraspResult::SUCCESS;

    std::vector<float> segment_start;
    if (!GetCurrentJoints(segment_start)) {
        return GraspResult::MOVE_FAILED;
    }
    const std::vector<float> measured_start = segment_start;
    if (!ClampJointsToLimits(segment_start, config_.joint_limits)) {
        return GraspResult::OUT_OF_RANGE;
    }
    if (segment_start != measured_start) {
        std::cout << "[GraspExecutor] clamped measured joint state to "
                    "hardware limits before interpolation"
                    << std::endl;
    }
    const float original_speed = config_.move_speed;
    constexpr int kStreamIntervalMs = 50;
    constexpr float kStreamVelocityScale = 0.8f;
    for (size_t index = 0; index < joint_path.size(); ++index) {
        const std::vector<float>& segment_target = joint_path[index];
        if (segment_target.size() != segment_start.size()) {
            config_.move_speed = original_speed;
            return GraspResult::MOVE_FAILED;
        }
        const float segment_speed =
            index == 0 ? first_speed : remaining_speed;
        config_.move_speed = segment_speed;
        float maximum_delta = 0.0f;
        for (size_t joint = 0; joint < segment_target.size(); ++joint) {
            maximum_delta = std::max(
                maximum_delta,
                std::fabs(segment_target[joint] - segment_start[joint]));
        }
        const float maximum_step = std::max(
            0.01f,
            segment_speed *
                (static_cast<float>(kStreamIntervalMs) / 1000.0f) *
                kStreamVelocityScale);
        const int step_count = std::max(
            1, static_cast<int>(std::ceil(maximum_delta / maximum_step)));
        for (int step = 1; step <= step_count; ++step) {
            const float ratio =
                static_cast<float>(step) / static_cast<float>(step_count);
            std::vector<float> command = segment_start;
            for (size_t joint = 0; joint < command.size(); ++joint) {
                command[joint] += ratio *
                    (segment_target[joint] - segment_start[joint]);
            }
            const GraspResult result = MoveToJoints(command);
            if (result != GraspResult::SUCCESS) {
                config_.move_speed = original_speed;
                return result;
            }
            const bool final_command =
                index + 1 == joint_path.size() && step == step_count;
            if (!final_command) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(kStreamIntervalMs));
            }
        }
        segment_start = segment_target;
    }
    if (!WaitMotionDone(
            completion_timeout_ms, final_tolerance_rad)) {
        config_.move_speed = original_speed;
        return GraspResult::TIMEOUT;
    }
    config_.move_speed = original_speed;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CorrectSidePose(
    const Pose3D& pose, float speed, const char* action) {
    constexpr int kMaximumCorrectionAttempts = 2;
    constexpr float kCorrectionToleranceRatio = 0.8f;
    const float correction_tolerance =
        config_.side_pose_position_tolerance * kCorrectionToleranceRatio;
    for (int attempt = 0; attempt <= kMaximumCorrectionAttempts; ++attempt) {
        Pose3D actual_pose{};
        if (!GetCurrentPose(actual_pose)) return GraspResult::MOVE_FAILED;
        const float error = PosePositionError(actual_pose, pose);
        std::cout << "[GraspExecutor] " << action
                    << " correction=" << attempt << "/"
                    << kMaximumCorrectionAttempts
                    << " target=[" << pose.x << "," << pose.y << ","
                    << pose.z << "]"
                    << " actual=[" << actual_pose.x << ","
                    << actual_pose.y << "," << actual_pose.z << "]"
                    << " position_error=" << error << "m"
                    << " correction_tolerance=" << correction_tolerance
                    << " final_tolerance="
                    << config_.side_pose_position_tolerance
                    << "m" << std::endl;
        if (error <= correction_tolerance) {
            return GraspResult::SUCCESS;
        }
        if (attempt == kMaximumCorrectionAttempts) {
            if (error <= config_.side_pose_position_tolerance) {
                std::cout << "[GraspExecutor] " << action
                            << " accepted within final tolerance after "
                            << kMaximumCorrectionAttempts
                            << " corrections" << std::endl;
                return GraspResult::SUCCESS;
            }
            break;
        }

        constexpr float kMaximumCorrectionPerAxisM = 0.015f;
        Pose3D correction_pose = pose;
        correction_pose.x += std::clamp(
            pose.x - actual_pose.x,
            -kMaximumCorrectionPerAxisM, kMaximumCorrectionPerAxisM);
        correction_pose.y += std::clamp(
            pose.y - actual_pose.y,
            -kMaximumCorrectionPerAxisM, kMaximumCorrectionPerAxisM);
        correction_pose.z += std::clamp(
            pose.z - actual_pose.z,
            -kMaximumCorrectionPerAxisM, kMaximumCorrectionPerAxisM);
        std::cout << "[GraspExecutor] " << action
                    << " compensated_target=[" << correction_pose.x << ","
                    << correction_pose.y << "," << correction_pose.z << "]"
                    << std::endl;

        std::vector<std::vector<float>> correction_path;
        std::string detail;
        const GraspResult plan_result = PlanSideJointPath(
            {correction_pose}, 300, correction_path, &detail);
        if (plan_result != GraspResult::SUCCESS) {
            std::cerr << "[GraspExecutor] " << action
                        << " correction path rejected: " << detail
                        << std::endl;
            return plan_result;
        }
        const float correction_speed = std::min(speed, 0.3f);
        const GraspResult move_result = ExecuteContinuousJointPath(
            correction_path, correction_speed, correction_speed,
            config_.side_waypoint_joint_tolerance_rad,
            kSideMotionCompletionTimeoutMs);
        if (move_result != GraspResult::SUCCESS) return move_result;
    }
    std::cerr << "[GraspExecutor] " << action
                << " correction did not reach target" << std::endl;
    return GraspResult::MOVE_FAILED;
}

GraspResult GraspExecutor::MoveToSidePreGrasp(const Pose3D& pose,
                                                float speed) {
    std::vector<float> current_joints;
    if (!GetCurrentJoints(current_joints)) return GraspResult::MOVE_FAILED;
    const ArmPathSafetyResult ready_path = arm_path_safety_->CheckPath(
        current_joints, config_.side_ready_joints, support_plane_,
        config_.support_surface_clearance_m,
        config_.path_joint_step_rad);
    if (!ready_path.safe) {
        std::cerr << "[GraspExecutor] side-ready path rejected: "
                    << ready_path.detail << std::endl;
        return GraspResult::OUT_OF_RANGE;
    }
    constexpr float kSideReadyToleranceRad = 0.08f;
    bool at_side_observation =
        current_joints.size() == config_.side_ready_joints.size();
    if (at_side_observation) {
        for (size_t index = 0; index < current_joints.size(); ++index) {
            if (std::fabs(
                    current_joints[index] -
                    config_.side_ready_joints[index]) >
                kSideReadyToleranceRad) {
                at_side_observation = false;
                break;
            }
        }
    }
    if (!at_side_observation) {
        std::cout << "[GraspExecutor] restoring side observation pose before "
                    "the joint0 sweep"
                    << std::endl;
        const GraspResult ready_result = MoveToSideObserve();
        if (ready_result != GraspResult::SUCCESS) return ready_result;
    }

    std::vector<float> staging_joints =
        validated_side_staging_joints_;
    std::vector<float> sweep_joints = validated_side_sweep_joints_;
    if (staging_joints.empty() || sweep_joints.empty()) {
        std::string detail;
        const GraspResult plan_result = PlanSideJoint0Sweep(
            pose, pose.z + 0.050f,
            300,
            staging_joints, sweep_joints, &detail);
        if (plan_result != GraspResult::SUCCESS) {
            std::cerr << "[GraspExecutor] side joint0 sweep rejected: "
                    << detail << std::endl;
            return plan_result;
        }
    }
    const ArmPathSafetyResult staging_path = arm_path_safety_->CheckPath(
        config_.side_ready_joints, staging_joints, support_plane_,
        config_.support_surface_clearance_m,
        config_.path_joint_step_rad);
    if (!staging_path.safe) {
        std::cerr << "[GraspExecutor] side staging lift rejected: "
                    << staging_path.detail << std::endl;
        return GraspResult::OUT_OF_RANGE;
    }
    const ArmPathSafetyResult sweep_path = arm_path_safety_->CheckPath(
        staging_joints, sweep_joints, support_plane_,
        config_.support_surface_clearance_m,
        config_.path_joint_step_rad);
    if (!sweep_path.safe) {
        std::cerr << "[GraspExecutor] side joint0 sweep rejected: "
                    << sweep_path.detail << std::endl;
        return GraspResult::OUT_OF_RANGE;
    }

    std::cout << "[GraspExecutor] side pre-grasp entry: lift clear of "
                "object, then sweep joint0 "
                << staging_joints[0] << " -> " << sweep_joints[0]
                << "; joint1-joint4 fixed during sweep" << std::endl;
    GraspResult result = ExecuteContinuousJointPath(
        {staging_joints, sweep_joints},
        std::min(speed, config_.line_speed),
        std::min(speed, config_.line_speed),
        config_.side_waypoint_joint_tolerance_rad,
        kSideMotionCompletionTimeoutMs);
    if (result != GraspResult::SUCCESS) return result;

    if (validated_side_entry_joints_.empty()) {
        RecordResult(
            GraspResult::MOVE_FAILED, "move_to_side_pre_grasp",
            "validated elevated side entry is unavailable");
        return GraspResult::MOVE_FAILED;
    }
    const Pose3D elevated_pre_grasp_pose = validated_side_entry_pose_;
    std::vector<float> entry_start_joints;
    if (!GetCurrentJoints(entry_start_joints)) {
        return GraspResult::MOVE_FAILED;
    }
    const ArmPathSafetyResult entry_path_result =
        arm_path_safety_->CheckPath(
            entry_start_joints, validated_side_entry_joints_,
            support_plane_, config_.support_surface_clearance_m,
            config_.path_joint_step_rad);
    if (!entry_path_result.safe) {
        std::cerr << "[GraspExecutor] side elevated entry rejected: "
                    << entry_path_result.detail << std::endl;
        return GraspResult::OUT_OF_RANGE;
    }
    std::cout << "[GraspExecutor] side safe pre-grasp: moving above the "
                "target; descent starts only after the gripper opens"
                << std::endl;
    result = ExecuteContinuousJointPath(
        {validated_side_entry_joints_}, speed, speed,
        config_.side_waypoint_joint_tolerance_rad,
        kSideMotionCompletionTimeoutMs);
    if (result != GraspResult::SUCCESS) return result;
    validated_side_entry_joints_.clear();
    return CorrectSidePose(
        elevated_pre_grasp_pose, speed, "move_to_side_safe_pre_grasp");
}

GraspResult GraspExecutor::MoveToSideLift(const Pose3D& retreat_pose,
                                            const Pose3D& lift_pose,
                                            float speed) {
    constexpr float kLoadedJointToleranceRad = 0.080f;
    constexpr float kMaximumLiftHeightShortfallM = 0.010f;
    constexpr float kMaximumLiftPlanarErrorM = 0.035f;
    std::vector<std::vector<float>> joint_path;
    if (!TakeValidatedSidePath({lift_pose}, joint_path)) {
        Pose3D current_pose{};
        if (!GetCurrentPose(current_pose)) {
            return GraspResult::MOVE_FAILED;
        }
        std::vector<Pose3D> poses = BuildSideCartesianPath(
            current_pose, retreat_pose, 0.025f);
        const std::vector<Pose3D> lift_path = BuildSideLiftPath(
            retreat_pose, lift_pose);
        poses.insert(poses.end(), lift_path.begin(), lift_path.end());
        std::string detail;
        const GraspResult plan_result = PlanSideJointPath(
            poses, 500, joint_path, &detail);
        if (plan_result != GraspResult::SUCCESS) {
            std::cerr << "[GraspExecutor] side lift path rejected: "
                    << detail << std::endl;
            return plan_result;
        }
    }
    const GraspResult move_result = ExecuteContinuousJointPath(
        joint_path, speed, speed,
        kLoadedJointToleranceRad,
        kSideMotionCompletionTimeoutMs);
    if (move_result != GraspResult::SUCCESS &&
        move_result != GraspResult::TIMEOUT) {
        return move_result;
    }

    Pose3D actual_pose{};
    if (!GetCurrentPose(actual_pose)) return move_result;
    const float planar_error = std::hypot(
        actual_pose.x - lift_pose.x,
        actual_pose.y - lift_pose.y);
    const float minimum_safe_z =
        retreat_pose.z - kMaximumLiftHeightShortfallM;
    const bool safely_lifted =
        actual_pose.z >= minimum_safe_z &&
        planar_error <= kMaximumLiftPlanarErrorM;
    std::cout << "[GraspExecutor] side lift verification: target=["
                << lift_pose.x << "," << lift_pose.y << ","
                << lift_pose.z << "] actual=[" << actual_pose.x << ","
                << actual_pose.y << "," << actual_pose.z << "]"
                << " minimum_safe_z=" << minimum_safe_z
                << " planar_error=" << planar_error
                << " result=" << (safely_lifted ? "SAFE" : "UNSAFE")
                << std::endl;
    if (safely_lifted) {
        if (move_result == GraspResult::TIMEOUT) {
            std::cout << "[GraspExecutor] side lift accepted from measured "
                        "safe pose despite loaded joint settling timeout"
                        << std::endl;
        }
        return GraspResult::SUCCESS;
    }
    if (move_result == GraspResult::TIMEOUT) return move_result;
    return CorrectSidePose(lift_pose, speed, "lift_from_grasp");
}

GraspResult GraspExecutor::MoveToPoseConstrained(const Pose3D& pose, float speed) {
    std::vector<float> joints;
    GraspResult result = SolveIKConstrained(pose, joints);
    if (result != GraspResult::SUCCESS) return result;

    std::cout << "[GraspExecutor] constrained IK joints(rad): [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i];
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 通过碰撞安全路径执行
    float old_speed2 = config_.move_speed;
    config_.move_speed = speed;
    GraspResult move_result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed2;
    if (move_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] constrained move_joints failed" << std::endl;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPoseWithYaw(const Pose3D& pose, float speed,
                                            float yaw_rad) {
    std::vector<float> joints;
    const GraspResult result = SolveIKConstrained(pose, joints);
    if (result != GraspResult::SUCCESS) {
        diagnostics_.wrist_yaw = WristYawDiagnostics{};
        RecordResult(result, "move_to_pose_with_yaw",
                    "ik failed before yaw");
        return result;
    }

    if (joints.size() >= 5) {
        const int joint_count = kin_get_num_joints(kin_);
        std::vector<double> lower(joint_count);
        std::vector<double> upper(joint_count);
        kin_get_joint_limits(kin_, lower.data(), upper.data());
        if (!ApplyJointLimits(config_.joint_limits, lower, upper)) {
            return GraspResult::IK_FAILED;
        }

        const float joint0 = joints[0];
        const float scale = config_.wrist_yaw_scale;
        float joint5_raw = 0.0f;
        float joint5 = 0.0f;
        if (joint_count <= 4 ||
            !ResolveFixedJawWristYaw(
                yaw_rad, joint0, scale,
                static_cast<float>(lower[4]),
                static_cast<float>(upper[4]),
                joint5_raw, joint5)) {
            diagnostics_.wrist_yaw = WristYawDiagnostics{};
            RecordResult(GraspResult::IK_FAILED, "move_to_pose_with_yaw",
                        "invalid wrist yaw mapping");
            return GraspResult::IK_FAILED;
        }
        joints[4] = joint5;

        diagnostics_.wrist_yaw.valid = true;
        diagnostics_.wrist_yaw.target_yaw = yaw_rad;
        diagnostics_.wrist_yaw.joint0 = joint0;
        diagnostics_.wrist_yaw.scale = scale;
        diagnostics_.wrist_yaw.joint5_raw = joint5_raw;
        diagnostics_.wrist_yaw.joint5_limited = joint5;
        diagnostics_.wrist_yaw.joint5_min = static_cast<float>(lower[4]);
        diagnostics_.wrist_yaw.joint5_max = static_cast<float>(upper[4]);

        std::cout << "[GraspExecutor] wrist_yaw: target=" << yaw_rad
                    << " rad, joint0=" << joint0
                    << ", raw joint5=" << joint5_raw
                    << ", limited joint5=" << joint5
                    << " (limit=[" << lower[4] << ", " << upper[4] << "])"
                    << std::endl;
    }

    std::cout << "[GraspExecutor] IK+yaw joints(rad): [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i];
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 通过碰撞安全路径执行
    float old_speed3 = config_.move_speed;
    config_.move_speed = speed;
    GraspResult move_result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed3;
    if (move_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] move_joints (yaw) failed" << std::endl;
        RecordResult(move_result, "move_to_pose_with_yaw",
                    "move_joints failed after yaw override");
        return move_result;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_pose_with_yaw");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPose(const Pose3D& pose, float speed) {
    if (!arm_) return GraspResult::MOVE_FAILED;
    active_target_joints_.clear();

    manip_pose_t target;
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    int ret = manip_move_target(arm_, &target, speed);
    if (ret == MANIP_ERR_NOSYS) {
        // 如果 move_target 不支持，尝试 move_line
        ret = manip_move_line(arm_, &target, speed);
    }
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] move_target failed: " << ret << std::endl;
        if (ret == MANIP_ERR_PARAM) return GraspResult::IK_FAILED;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveLinear(const Pose3D& pose, float speed) {
    if (!arm_) return GraspResult::MOVE_FAILED;
    active_target_joints_.clear();

    manip_pose_t target;
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    int ret = manip_move_line(arm_, &target, speed);
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] move_line failed: " << ret << std::endl;
        if (ret == MANIP_ERR_PARAM) return GraspResult::IK_FAILED;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

bool GraspExecutor::WaitMotionDone(
    int timeout_ms, float target_tolerance_rad) {
    if (!arm_) return false;

    if (timeout_ms <= 0) {
        timeout_ms = wait_motion_timeout_ms_;
    }

    constexpr float kStableThreshold = 0.01f;
    constexpr int kStableCount = 10;
    constexpr int kPollIntervalMs = 50;

    manip_joint_t prev_joints;
    std::memset(&prev_joints, 0, sizeof(prev_joints));
    manip_get_state(arm_, &prev_joints, nullptr);

    int stable_counter = 0;
    manip_joint_t current_joints = prev_joints;
    float current_target_error = 0.0f;
    auto start = std::chrono::steady_clock::now();

    // 先等一小段时间让运动开始
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        int elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                .count();

        if (elapsed_ms >= timeout_ms) {
            std::cerr << "[GraspExecutor] Motion timeout ("
                        << timeout_ms << "ms)";
            if (!active_target_joints_.empty()) {
                std::cerr << "; target_error=" << current_target_error
                            << "rad target=[";
                for (size_t i = 0; i < active_target_joints_.size(); ++i) {
                    if (i > 0) std::cerr << ",";
                    std::cerr << active_target_joints_[i];
                }
                std::cerr << "] actual=[";
                for (int i = 0; i < current_joints.count; ++i) {
                    if (i > 0) std::cerr << ",";
                    std::cerr << current_joints.joints[i];
                }
                std::cerr << "]";
            }
            std::cerr << std::endl;
            return false;
        }

        Tick(static_cast<float>(kPollIntervalMs) / 1000.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));

        std::memset(&current_joints, 0, sizeof(current_joints));
        manip_get_state(arm_, &current_joints, nullptr);

        // 检查所有关节角变化是否小于阈值
        float max_diff = 0.0f;
        for (int i = 0;
            i < current_joints.count && i < prev_joints.count;
            ++i) {
            float diff = std::fabs(
                current_joints.joints[i] - prev_joints.joints[i]);
            if (diff > max_diff) max_diff = diff;
        }

        float max_target_error = 0.0f;
        const bool has_joint_target =
            !active_target_joints_.empty() &&
            active_target_joints_.size() == current_joints.count;
        if (has_joint_target) {
            for (size_t i = 0; i < active_target_joints_.size(); ++i) {
                float error = std::fabs(
                    current_joints.joints[i] - active_target_joints_[i]);
                if (error > static_cast<float>(M_PI)) {
                    error = static_cast<float>(2.0 * M_PI) - error;
                }
                max_target_error = std::max(max_target_error, error);
            }
        }
        current_target_error = max_target_error;

        if (max_diff < kStableThreshold) {
            stable_counter++;
            if (stable_counter >= kStableCount &&
                (!has_joint_target ||
                    max_target_error <= target_tolerance_rad)) {
                if (has_joint_target && max_target_error > 0.060f) {
                    std::cout << "[GraspExecutor] Joint waypoint accepted "
                                << "with target_error=" << max_target_error
                                << "rad; Cartesian correction remains active"
                                << std::endl;
                }
                return true;
            }
        } else {
            stable_counter = 0;
        }

        prev_joints = current_joints;
    }
}

bool GraspExecutor::VerifyPoseReached(const char* action,
    const Pose3D& target_pose) {
    Pose3D actual_pose{};
    if (!GetCurrentPose(actual_pose)) {
        std::cerr << "[GraspExecutor] " << action
                    << " pose verification failed: cannot read FK pose"
                    << std::endl;
        return false;
    }

    const float dx = actual_pose.x - target_pose.x;
    const float dy = actual_pose.y - target_pose.y;
    const float dz = actual_pose.z - target_pose.z;
    const float position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

    std::cout << "[GraspExecutor] " << action
                << " target_pose=[" << target_pose.x << ", "
                << target_pose.y << ", " << target_pose.z << "]"
                << " actual_pose=[" << actual_pose.x << ", "
                << actual_pose.y << ", " << actual_pose.z << "]"
                << " position_error=" << position_error << " m"
                << " tolerance=" << config_.pose_position_tolerance << " m"
                << std::endl;

    if (position_error > config_.pose_position_tolerance) {
        std::cerr << "[GraspExecutor] " << action
                    << " target not reached" << std::endl;
        return false;
    }
    return true;
}

}  // namespace perceptive_grasp
