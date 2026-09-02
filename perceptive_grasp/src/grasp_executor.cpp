/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_executor.cpp
    * @brief 抓取执行模块实现 - 机械臂 + 夹爪协调控制
    */

#include "grasp_executor.h"
#include "motion_completion.h"

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
#include <sstream>
#include <thread>

namespace perceptive_grasp {

namespace {

constexpr int kSideMotionCompletionTimeoutMs = 5000;
constexpr float kSideStagingJointToleranceRad = 0.080f;
constexpr float kSideCorrectionJointToleranceRad = 0.080f;
constexpr float kTopGraspJointToleranceRad = 0.020f;
constexpr float kTopLiftJointToleranceRad = 0.060f;
constexpr float kHomeJointToleranceRad = 0.060f;

bool ResolveFixedJawWristYaw(float target_yaw,
                            float joint0,
                            float scale,
                            float lower,
                            float upper,
                            float& raw_wrist,
                            float& resolved_wrist) {
    if (std::fabs(scale) < 1e-6f || lower > upper) return false;

    // The linksee gripper has a fixed jaw. A yaw shifted by pi preserves the
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

float MinimumJointMargin(
    const std::vector<std::vector<float>>& path,
    const std::vector<JointConstraint>& limits,
    const std::vector<JointConstraint>& constraints) {
    float minimum_margin = std::numeric_limits<float>::infinity();
    bool found = false;
    const auto include_limits = [&path, &minimum_margin, &found](
        const std::vector<JointConstraint>& active_limits) {
        for (const std::vector<float>& joints : path) {
            for (const JointConstraint& limit : active_limits) {
                if (limit.joint_index < 0 ||
                    limit.joint_index >=
                        static_cast<int>(joints.size())) {
                    continue;
                }
                const float value =
                    joints[static_cast<size_t>(limit.joint_index)];
                minimum_margin = std::min(
                    minimum_margin,
                    std::min(
                        value - limit.min_rad,
                        limit.max_rad - value));
                found = true;
            }
        }
    };
    include_limits(limits);
    include_limits(constraints);
    return found ? minimum_margin : NAN;
}

bool JointsWithinLimits(const std::vector<float>& joints,
                        const std::vector<JointConstraint>& limits,
                        std::string* detail = nullptr,
                        float tolerance_rad = 0.0f) {
    for (const JointConstraint& limit : limits) {
        if (limit.joint_index < 0 ||
            limit.joint_index >= static_cast<int>(joints.size())) {
            if (detail) *detail = "hardware joint limit index is invalid";
            return false;
        }
        const float value = joints[limit.joint_index];
        if (value < limit.min_rad - tolerance_rad ||
            value > limit.max_rad + tolerance_rad) {
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

bool JointCommandMovesTowardLimits(
    const std::vector<float>& current,
    const std::vector<float>& command,
    const std::vector<JointConstraint>& limits,
    float measured_tolerance_rad,
    std::string* detail) {
    if (current.size() != command.size()) {
        if (detail) *detail = "current and commanded joint counts differ";
        return false;
    }
    for (const JointConstraint& limit : limits) {
        if (limit.joint_index < 0 ||
            limit.joint_index >= static_cast<int>(command.size())) {
            if (detail) *detail = "hardware joint limit index is invalid";
            return false;
        }
        const size_t index = static_cast<size_t>(limit.joint_index);
        const float measured = current[index];
        const float target = command[index];
        if (target < limit.min_rad) {
            if (measured < limit.min_rad &&
                measured >= limit.min_rad - measured_tolerance_rad &&
                target >= measured) {
                continue;
            }
        } else if (target > limit.max_rad) {
            if (measured > limit.max_rad &&
                measured <= limit.max_rad + measured_tolerance_rad &&
                target <= measured) {
                continue;
            }
        } else {
            continue;
        }
        if (detail) {
            *detail = "joint " + std::to_string(limit.joint_index) +
                " command " + std::to_string(target) +
                " does not correct measured value " +
                std::to_string(measured) + " toward hardware range [" +
                std::to_string(limit.min_rad) + "," +
                std::to_string(limit.max_rad) + "]";
        }
        return false;
    }
    if (detail) detail->clear();
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
    validated_top_pre_grasp_pose_ = Pose3D{};
    validated_top_pre_grasp_joints_.clear();
    validated_top_poses_.clear();
    validated_top_joint_path_.clear();
    validated_top_path_index_ = 0;
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
        const std::string detail = diagnostics_.last_detail.empty()
            ? "move_joints failed"
            : diagnostics_.last_detail;
        RecordResult(result, "move_to_observe", detail);
        return result;
    }
    if (!use_coordinated_path) {
        const GraspResult wait_result = WaitMotionDone();
        if (wait_result != GraspResult::SUCCESS) {
            RecordResult(
                wait_result, "move_to_observe",
                last_motion_wait_detail_);
            return wait_result;
        }
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

    // An empty side grasp leaves the tool at the contact waypoint. Consume
    // the validated retreat/lift suffix before planning a normal-clearance
    // return to side observation; a direct path starting at contact is
    // correctly rejected by the regular support-surface clearance check.
    if (validated_side_path_index_ > 0 &&
        validated_side_path_index_ < validated_side_joint_path_.size()) {
        std::vector<std::vector<float>> retreat_path(
            validated_side_joint_path_.begin() +
                static_cast<std::ptrdiff_t>(validated_side_path_index_),
            validated_side_joint_path_.end());
        std::cout << "[GraspExecutor] side observation retry: retreating "
            << "from contact along " << retreat_path.size()
            << " validated waypoint(s)" << std::endl;
        const GraspResult retreat_result = ExecuteContinuousJointPath(
            retreat_path, config_.line_speed, config_.line_speed,
            kSideObserveFinalToleranceRad, -1, true);
        if (retreat_result != GraspResult::SUCCESS) {
            const std::string detail = diagnostics_.last_detail.empty()
                ? "validated side contact retreat failed"
                : diagnostics_.last_detail;
            RecordResult(
                retreat_result, "move_to_side_observe", detail);
            return retreat_result;
        }
        validated_side_path_index_ = validated_side_joint_path_.size();
    }

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
        const std::string detail = diagnostics_.last_detail.empty()
            ? "coordinated side-observation motion failed"
            : diagnostics_.last_detail;
        RecordResult(
            result, "move_to_side_observe", detail);
        return result;
    }

    CaptureEmptyClosedPosition();
    RecordResult(GraspResult::SUCCESS, "move_to_side_observe");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToHome() {
    std::vector<float> current_joints;
    const float home_completion_tolerance_rad =
        EffectiveMotionTargetToleranceRad(kHomeJointToleranceRad);
    bool already_at_home =
        GetCurrentJoints(current_joints) &&
        current_joints.size() == config_.home_joints.size();
    if (already_at_home) {
        for (size_t index = 0; index < current_joints.size(); ++index) {
            if (std::fabs(
                    current_joints[index] - config_.home_joints[index]) >
                home_completion_tolerance_rad) {
                already_at_home = false;
                break;
            }
        }
    }
    if (already_at_home) {
        RecordResult(
            GraspResult::SUCCESS, "move_to_home",
            "already within home tolerance");
        return GraspResult::SUCCESS;
    }

    GraspResult result = MoveToJointsCollisionSafe(
        config_.home_joints, true);
    if (result != GraspResult::SUCCESS) {
        const std::string detail = diagnostics_.last_detail.empty()
            ? "move_joints failed"
            : diagnostics_.last_detail;
        RecordResult(result, "move_to_home", detail);
        return result;
    }
    const GraspResult wait_result = WaitMotionDone(
        -1, kHomeJointToleranceRad);
    if (wait_result != GraspResult::SUCCESS) {
        RecordResult(
            wait_result, "move_to_home", last_motion_wait_detail_);
        return wait_result;
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
    if (use_top_constraints) {
        if (validated_top_pre_grasp_joints_.empty() ||
            !PosesMatch(
                pre_grasp_pose, validated_top_pre_grasp_pose_)) {
            RecordResult(
                GraspResult::MOVE_FAILED, "move_to_pre_grasp",
                "validated top pre-grasp path is unavailable");
            return GraspResult::MOVE_FAILED;
        }
        const float original_speed = config_.move_speed;
        result = MoveToJointsCollisionSafe(
            validated_top_pre_grasp_joints_);
        config_.move_speed = original_speed;
    } else {
        result = MoveToSidePreGrasp(pre_grasp_pose, config_.move_speed);
        if (result == GraspResult::SUCCESS) {
            RecordResult(GraspResult::SUCCESS, "move_to_pre_grasp");
            return GraspResult::SUCCESS;
        }
    }
    if (result != GraspResult::SUCCESS) {
        const std::string detail = diagnostics_.last_detail.empty()
            ? (result == GraspResult::IK_FAILED
                ? "ik failed"
                : "move command failed")
            : diagnostics_.last_detail;
        RecordResult(result, "move_to_pre_grasp", detail);
        return result;
    }
    constexpr float kMaximumTopPreGraspHeightShortfallM = 0.010f;
    const auto top_pre_grasp_pose_acceptance = [
        this, use_top_constraints, pre_grasp_pose]() {
        if (!use_top_constraints) return false;
        Pose3D actual_pose{};
        if (!GetCurrentPose(actual_pose)) return false;
        const float position_error =
            PosePositionError(actual_pose, pre_grasp_pose);
        const bool safely_at_pre_grasp =
            position_error <= config_.pose_position_tolerance &&
            actual_pose.z >= pre_grasp_pose.z -
                kMaximumTopPreGraspHeightShortfallM;
        std::cout
            << "[GraspExecutor] top pre-grasp settled-pose verification: "
            << "target=[" << pre_grasp_pose.x << ","
            << pre_grasp_pose.y << "," << pre_grasp_pose.z
            << "] actual=[" << actual_pose.x << ","
            << actual_pose.y << "," << actual_pose.z << "]"
            << " position_error=" << position_error
            << " minimum_safe_z="
            << pre_grasp_pose.z -
                kMaximumTopPreGraspHeightShortfallM
            << " result="
            << (safely_at_pre_grasp ? "SAFE" : "UNSAFE")
            << std::endl;
        return safely_at_pre_grasp;
    };
    const GraspResult wait_result = WaitMotionDone(
        -1, 0.060f, top_pre_grasp_pose_acceptance);
    if (wait_result != GraspResult::SUCCESS) {
        RecordResult(
            wait_result, "move_to_pre_grasp",
            last_motion_wait_detail_);
        return wait_result;
    }
    if (!VerifyPoseReached("move_to_pre_grasp", pre_grasp_pose)) {
        RecordResult(GraspResult::MOVE_FAILED, "move_to_pre_grasp",
                    "pose verification failed");
        return GraspResult::MOVE_FAILED;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_pre_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToGrasp(const Pose3D& grasp_pose,
                                        float grasp_yaw_rad,
                                        bool use_top_constraints) {
    if (use_top_constraints) {
        return MoveAlongValidatedTopPath(
            grasp_pose, config_.line_speed, "move_to_grasp",
            kTopGraspJointToleranceRad);
    }

    bool has_yaw = !std::isnan(grasp_yaw_rad);
    GraspResult result;
    if (has_yaw) {
        result = MoveToPoseWithYaw(
            grasp_pose, config_.line_speed, grasp_yaw_rad);
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
    const GraspResult wait_result = WaitMotionDone(
        -1, kTopGraspJointToleranceRad);
    if (wait_result != GraspResult::SUCCESS) {
        RecordResult(
            wait_result, "move_to_grasp", last_motion_wait_detail_);
        return wait_result;
    }
    if (!VerifyPoseReached("move_to_grasp", grasp_pose)) {
        RecordResult(GraspResult::MOVE_FAILED, "move_to_grasp",
                    "pose verification failed");
        return GraspResult::MOVE_FAILED;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::LiftFromGrasp(const Pose3D& retreat_pose,
                                        const Pose3D& lift_pose,
                                        float grasp_yaw_rad,
                                        bool use_top_constraints) {
    const auto move_and_wait = [this, grasp_yaw_rad](
                                    const Pose3D& pose,
                                    const char* action) -> GraspResult {
        const bool has_yaw = !std::isnan(grasp_yaw_rad);
        GraspResult result;
        if (has_yaw) {
            result = MoveToPoseWithYaw(pose, config_.line_speed,
                                        grasp_yaw_rad);
        } else {
            result = MoveToPoseSide(pose, config_.line_speed);
        }
        if (result != GraspResult::SUCCESS) return result;
        const GraspResult wait_result = WaitMotionDone();
        if (wait_result != GraspResult::SUCCESS) return wait_result;
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
    } else if (use_top_constraints) {
        result = MoveAlongValidatedTopPath(
            retreat_pose, config_.line_speed, "retreat_from_grasp",
            kTopLiftJointToleranceRad, true);
        if (result != GraspResult::SUCCESS) {
            return result;
        }
        const float lift_delta = std::sqrt(
            std::pow(lift_pose.x - retreat_pose.x, 2.0f) +
            std::pow(lift_pose.y - retreat_pose.y, 2.0f) +
            std::pow(lift_pose.z - retreat_pose.z, 2.0f));
        if (lift_delta > 0.001f) {
            result = MoveAlongValidatedTopPath(
                lift_pose, config_.line_speed, "lift_from_grasp",
                kTopLiftJointToleranceRad);
            if (result != GraspResult::SUCCESS) return result;
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
    std::vector<float> current_joints;
    const bool have_current_joints = GetCurrentJoints(current_joints) &&
        current_joints.size() >= config_.place_joints.size();
    if (have_current_joints) {
        float maximum_error = 0.0f;
        for (size_t joint = 0; joint < config_.place_joints.size(); ++joint) {
            float error = std::fabs(
                current_joints[joint] - config_.place_joints[joint]);
            if (joint == 4 && error > static_cast<float>(M_PI)) {
                error = static_cast<float>(2.0 * M_PI) - error;
            }
            maximum_error = std::max(maximum_error, error);
        }
        std::cout << "[GraspExecutor] place precheck: target=[";
        for (size_t joint = 0; joint < config_.place_joints.size(); ++joint) {
            if (joint > 0) std::cout << ",";
            std::cout << config_.place_joints[joint];
        }
        std::cout << "] actual=[";
        for (size_t joint = 0; joint < config_.place_joints.size(); ++joint) {
            if (joint > 0) std::cout << ",";
            std::cout << current_joints[joint];
        }
        std::cout << "] maximum_joint_error_rad=" << maximum_error
            << std::endl;
        if (maximum_error <= config_.place_joint_tolerance_rad) {
            std::cout << "[GraspExecutor] already at place pose: "
                << "maximum_joint_error_rad=" << maximum_error
                << " tolerance_rad="
                << config_.place_joint_tolerance_rad << std::endl;
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.timing.place_settle_ms));
            RecordResult(
                GraspResult::SUCCESS, "move_to_place",
                "current joints already satisfy place tolerance");
            return GraspResult::SUCCESS;
        }
    } else {
        std::cout << "[GraspExecutor] place precheck: current joint state "
            << "unavailable; validating a normal place move"
            << std::endl;
    }

    GraspResult result = MoveToJointsCollisionSafe(config_.place_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_place", "move_joints failed");
        return result;
    }
    const GraspResult wait_result = WaitMotionDone(
        -1, config_.place_joint_tolerance_rad);
    if (wait_result != GraspResult::SUCCESS) {
        RecordResult(
            wait_result, "move_to_place", last_motion_wait_detail_);
        return wait_result;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.place_settle_ms));
    RecordResult(GraspResult::SUCCESS, "move_to_place");
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
    const GraspResult wait_result = WaitMotionDone();
    if (wait_result != GraspResult::SUCCESS) return wait_result;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPreGraspSafe(const Pose3D& pre_grasp_pose,
                                                float speed_scale) {
    float speed = speed_scale > 0.0f ? speed_scale : config_.move_speed;
    GraspResult result = MoveToPoseWithIKJoints(pre_grasp_pose, speed);
    if (result != GraspResult::SUCCESS) return result;
    const GraspResult wait_result = WaitMotionDone();
    if (wait_result != GraspResult::SUCCESS) return wait_result;
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

// --- Private ---

GraspResult GraspExecutor::MoveToJoints(const std::vector<float>& joints) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    std::string limit_detail;
    if (!JointsWithinLimits(joints, config_.joint_limits, &limit_detail)) {
        std::vector<float> current_joints;
        constexpr float kMeasuredJointLimitToleranceRad = 0.05f;
        if (!GetCurrentJoints(current_joints) ||
            !JointCommandMovesTowardLimits(
                current_joints, joints, config_.joint_limits,
                kMeasuredJointLimitToleranceRad, &limit_detail)) {
            std::cerr << "[GraspExecutor] joint command rejected: "
                        << limit_detail << std::endl;
            RecordResult(
                GraspResult::OUT_OF_RANGE, "move_to_joints",
                limit_detail);
            return GraspResult::OUT_OF_RANGE;
        }
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

bool GraspExecutor::NeedsCollisionAvoidance(
    const std::vector<float>& current_joints,
    const std::vector<float>& target_joints) const {
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

GraspResult GraspExecutor::ValidateJointPathSafety(
    const std::vector<float>& start_joints,
    const std::vector<float>& target_joints,
    std::string* detail,
    bool allow_contact_retreat,
    bool allow_missing_support_surface) const {
    if (!arm_path_safety_) {
        if (detail) *detail = "arm path safety is unavailable";
        return GraspResult::MOVE_FAILED;
    }
    if (!support_plane_.valid && !allow_missing_support_surface) {
        if (detail) *detail = "support surface is unavailable";
        return GraspResult::OUT_OF_RANGE;
    }

    std::string limit_detail;
    constexpr float kMeasuredJointLimitToleranceRad = 0.05f;
    if (!JointsWithinLimits(
            start_joints, config_.joint_limits, &limit_detail,
            kMeasuredJointLimitToleranceRad) &&
        !JointCommandMovesTowardLimits(
            start_joints, target_joints, config_.joint_limits,
            kMeasuredJointLimitToleranceRad, &limit_detail)) {
        if (detail) {
            *detail = "current joint state cannot recover safely: " +
                limit_detail;
        }
        return GraspResult::OUT_OF_RANGE;
    }
    if (!JointsWithinLimits(
            target_joints, config_.joint_limits, &limit_detail)) {
        if (detail) *detail = "target joint state is unsafe: " + limit_detail;
        return GraspResult::OUT_OF_RANGE;
    }
    if (NeedsCollisionAvoidance(start_joints, target_joints)) {
        if (detail) {
            *detail =
                "direct path enters the configured body-collision region";
        }
        return GraspResult::OUT_OF_RANGE;
    }

    constexpr float kMaximumContactPenetrationM = 0.005f;
    constexpr float kMaximumRetreatClearanceRegressionM = 0.0015f;
    const ArmPathSafetyResult path_result =
        !support_plane_.valid && allow_missing_support_surface
        ? arm_path_safety_->CheckSelfCollisionPath(
            start_joints, target_joints,
            config_.path_joint_step_rad)
        : allow_contact_retreat
        ? arm_path_safety_->CheckContactRetreatPath(
            start_joints, target_joints, support_plane_,
            config_.support_surface_clearance_m,
            config_.path_joint_step_rad,
            kMaximumContactPenetrationM,
            kMaximumRetreatClearanceRegressionM)
        : arm_path_safety_->CheckPath(
            start_joints, target_joints, support_plane_,
            config_.support_surface_clearance_m,
            config_.path_joint_step_rad);
    if (!path_result.safe) {
        if (detail) *detail = path_result.detail;
        return GraspResult::OUT_OF_RANGE;
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::BuildCollisionSafeJointPath(
    const std::vector<float>& start_joints,
    const std::vector<float>& target_joints,
    std::vector<std::vector<float>>& path,
    std::string* detail,
    bool allow_missing_support_surface) const {
    path.clear();
    const auto& ca = config_.collision_avoidance;
    if (!ca.enabled || target_joints.size() < 2 ||
        !NeedsCollisionAvoidance(start_joints, target_joints)) {
        path.push_back(target_joints);
    } else {
        std::vector<float> bounded_current_joints = start_joints;
        if (!ClampJointsToLimits(
                bounded_current_joints, config_.joint_limits)) {
            if (detail) *detail = "configured joint limits are invalid";
            return GraspResult::OUT_OF_RANGE;
        }

        const float current_j1 = start_joints[1];
        const float target_j0 = target_joints[0];
        const float target_j1 = target_joints[1];
        const bool current_shoulder_danger =
            current_j1 < ca.shoulder_threshold;
        const bool target_shoulder_danger =
            target_j1 < ca.shoulder_threshold;

        if (current_shoulder_danger && !target_shoulder_danger) {
            std::vector<float> lift_shoulder = bounded_current_joints;
            lift_shoulder[1] = ca.shoulder_threshold;
            path.push_back(std::move(lift_shoulder));
        } else if (!current_shoulder_danger && target_shoulder_danger) {
            std::vector<float> rotate_base = bounded_current_joints;
            rotate_base[0] = target_j0;
            path.push_back(std::move(rotate_base));
        } else {
            std::vector<float> lift_shoulder = bounded_current_joints;
            lift_shoulder[1] = ca.shoulder_threshold;
            path.push_back(lift_shoulder);
            lift_shoulder[0] = target_j0;
            path.push_back(std::move(lift_shoulder));
        }
        path.push_back(target_joints);
    }

    std::vector<float> segment_start = start_joints;
    for (size_t index = 0; index < path.size(); ++index) {
        std::string segment_detail;
        const GraspResult validation = ValidateJointPathSafety(
            segment_start, path[index], &segment_detail,
            false, allow_missing_support_surface);
        if (validation != GraspResult::SUCCESS) {
            if (detail) {
                *detail = "joint path segment " +
                    std::to_string(index + 1) + "/" +
                    std::to_string(path.size()) + ": " +
                    segment_detail;
            }
            return validation;
        }
        segment_start = path[index];
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToJointsCollisionSafe(
    const std::vector<float>& target_joints,
    bool allow_missing_support_surface) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    std::vector<float> current_joints;
    if (!GetCurrentJoints(current_joints)) {
        std::cerr << "[GraspExecutor] joint path rejected: current joint "
            "state is unavailable"
            << std::endl;
        return GraspResult::MOVE_FAILED;
    }

    std::vector<std::vector<float>> path;
    std::string detail;
    const GraspResult path_result = BuildCollisionSafeJointPath(
        current_joints, target_joints, path, &detail,
        allow_missing_support_surface);
    if (path_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] joint path rejected: "
            << detail << std::endl;
        RecordResult(path_result, "move_to_joints", detail);
        return path_result;
    }
    active_motion_timeout_ms_ = EstimateJointMotionTimeoutMs(
        current_joints, target_joints, config_.move_speed, 3000, 30000);

    for (size_t index = 0; index < path.size(); ++index) {
        const GraspResult result = MoveToJoints(path[index]);
        if (result != GraspResult::SUCCESS) return result;
        if (index + 1 < path.size()) {
            const GraspResult wait_result = WaitMotionDone();
            if (wait_result != GraspResult::SUCCESS) return wait_result;
        }
    }
    return GraspResult::SUCCESS;
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

    constexpr double kMaxPositionErrorM = 0.006;
    constexpr double kMaxApproachErrorDeg = 30.0;
    // The five arm joints cannot independently satisfy both side-grasp axes.
    // Keep the TCP precise while allowing the small wrist-axis deviation
    // observed at fixed-jaw-safe lateral offsets.
    constexpr double kMaxOpeningErrorDeg = 15.0;
    constexpr double kOrientationBoundarySlackDeg = 0.5;
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
                approach_error >
                    kMaxApproachErrorDeg + kOrientationBoundarySlackDeg ||
                opening_error >
                    kMaxOpeningErrorDeg + kOrientationBoundarySlackDeg) {
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
        best.approach_error_deg >
            kMaxApproachErrorDeg + kOrientationBoundarySlackDeg ||
        best.opening_error_deg >
            kMaxOpeningErrorDeg + kOrientationBoundarySlackDeg) {
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
    diagnostics_.validation_min_joint_margin_rad = NAN;
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
            std::vector<std::vector<float>> validation_path = {
                staging_joints, sweep_joints};
            validation_path.insert(
                validation_path.end(),
                joint_path.begin(), joint_path.end());
            diagnostics_.validation_min_joint_margin_rad =
                MinimumJointMargin(
                    validation_path, config_.joint_limits,
                    config_.joint_constraints);
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
    constexpr float kTopVerticalToleranceM = 0.002f;
    const auto planar_distance = [](const Pose3D& lhs, const Pose3D& rhs) {
        return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
    };
    if (planar_distance(pre_grasp_pose, grasp_pose) >
            kTopVerticalToleranceM ||
        planar_distance(grasp_pose, retreat_pose) >
            kTopVerticalToleranceM ||
        planar_distance(retreat_pose, lift_pose) >
            kTopVerticalToleranceM) {
        if (detail) {
            *detail =
                "top grasp descent and lift must remain vertical";
        }
        return GraspResult::OUT_OF_RANGE;
    }

    std::vector<Pose3D> top_poses = {pre_grasp_pose};
    const auto append_path = [&top_poses](
        const std::vector<Pose3D>& path) {
        top_poses.insert(top_poses.end(), path.begin(), path.end());
    };
    constexpr float kTopCartesianStepM = 0.010f;
    append_path(BuildSideCartesianPath(
        pre_grasp_pose, grasp_pose, kTopCartesianStepM));
    append_path(BuildSideCartesianPath(
        grasp_pose, retreat_pose, kTopCartesianStepM));
    append_path(BuildSideCartesianPath(
        retreat_pose, lift_pose, kTopCartesianStepM));

    std::vector<std::vector<float>> top_joint_path;
    const GraspResult result = PlanTopJointPath(
        top_poses, grasp_yaw_rad, timeout_ms,
        top_joint_path, detail);
    if (result != GraspResult::SUCCESS) return result;
    diagnostics_.validation_min_joint_margin_rad =
        MinimumJointMargin(
            top_joint_path, config_.joint_limits,
            config_.joint_constraints);

    validated_top_pre_grasp_pose_ = top_poses.front();
    validated_top_pre_grasp_joints_ = top_joint_path.front();
    top_poses.erase(top_poses.begin());
    top_joint_path.erase(top_joint_path.begin());
    validated_top_poses_ = std::move(top_poses);
    validated_top_joint_path_ = std::move(top_joint_path);
    validated_top_path_index_ = 0;
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::SolveIKConstrained(
    const Pose3D& pose,
    std::vector<float>& joints,
    const std::vector<float>* seed_joints,
    int timeout_ms,
    std::function<bool(std::vector<float>&)> candidate_validator,
    float position_weight,
    float ik_epsilon) {
    if (!kin_) return GraspResult::MOVE_FAILED;

    const auto ik_start = std::chrono::steady_clock::now();

    const auto& constraints = config_.joint_constraints;

    // 如果没有约束，退回普通 IK
    if (constraints.empty()) {
        const GraspResult result = SolveIK(pose, joints);
        if (result == GraspResult::SUCCESS &&
            candidate_validator &&
            !candidate_validator(joints)) {
            return GraspResult::OUT_OF_RANGE;
        }
        return result;
    }

    kin_pose_t ik_target;
    ik_target.x  = pose.x;
    ik_target.y  = pose.y;
    ik_target.z  = pose.z;
    ik_target.qw = pose.qw;
    ik_target.qx = pose.qx;
    ik_target.qy = pose.qy;
    ik_target.qz = pose.qz;

    const int n_joints = kin_get_num_joints(kin_);
    if (n_joints <= 0 || n_joints > KIN_MAX_JOINTS) {
        return GraspResult::IK_FAILED;
    }
    const int arm_joint_count = std::min(
        n_joints, static_cast<int>(config_.observe_joints.size()));
    if (arm_joint_count <= 0) return GraspResult::IK_FAILED;
    std::vector<double> lower(n_joints), upper(n_joints);
    if (kin_get_joint_limits(kin_, lower.data(), upper.data()) != KIN_OK) {
        return GraspResult::IK_FAILED;
    }
    if (!ApplyJointLimits(config_.joint_limits, lower, upper)) {
        return GraspResult::IK_FAILED;
    }

    // Keep fallback seeds deterministic so identical geometry cannot select
    // different IK branches across runs.
    std::mt19937 rng(0x5A17u);
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
    const int effective_timeout_ms = timeout_ms > 0
        ? timeout_ms
        : std::max(100, max_trials * 100);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(effective_timeout_ms);
    int attempted_trials = 0;

    for (int trial = 0; trial < max_trials; ++trial) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        attempted_trials++;
        kin_joints_t q_seed;
        q_seed.count = static_cast<uint8_t>(n_joints);

        if (trial <= 1) {
            const bool use_path_seed =
                trial == 0 && seed_joints &&
                seed_joints->size() >=
                    static_cast<size_t>(arm_joint_count);
            const std::vector<float>& seed = use_path_seed
                ? *seed_joints
                : config_.observe_joints;
            for (int j = 0; j < n_joints; ++j) {
                q_seed.q[j] =
                    j < static_cast<int>(seed.size()) ? seed[j] : 0.0;
            }
        } else if (trial == 2) {
            for (int j = 0; j < n_joints; ++j) {
                q_seed.q[j] = 0.5 * (lower[j] + upper[j]);
            }
            if (n_joints > 1) {
                q_seed.q[1] = std::clamp(
                    static_cast<double>(
                        config_.collision_avoidance.shoulder_threshold +
                        config_.collision_avoidance.base_safe_margin),
                    lower[1], upper[1]);
            }
        } else {
            // Deterministic fallback samples within the configured limits.
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

        kin_ik_params_t ik_params = {};
        ik_params.epsilon = ik_epsilon;
        ik_params.position_weight = position_weight;
        ik_params.timeout_s = std::clamp(
            std::chrono::duration<float>(deadline - now).count(),
            0.005f, 0.1f);

        kin_joints_t q_result;
        int ik_ret = kin_inverse(kin_, &ik_target, &q_seed, &ik_params, &q_result);
        if (ik_ret != KIN_OK) continue;

        if (check_constraints(q_result)) {
            std::vector<float> candidate;
            candidate.reserve(arm_joint_count);
            for (int j = 0; j < arm_joint_count; ++j) {
                candidate.push_back(static_cast<float>(q_result.q[j]));
            }
            if (candidate_validator &&
                !candidate_validator(candidate)) {
                continue;
            }
            joints = std::move(candidate);
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
                        << " mode=constrained trials=" << attempted_trials
                        << " fallback=0 result=failed" << std::endl;
        }
        std::cerr << "[GraspExecutor] IK failed: no solution in "
                    << attempted_trials << " trials" << std::endl;
        return GraspResult::IK_FAILED;
    }

    return GraspResult::SUCCESS;
}

bool GraspExecutor::ApplyWristYaw(
    float grasp_yaw_rad,
    std::vector<float>& joints,
    std::string* detail,
    bool record_diagnostics) {
    if (std::isnan(grasp_yaw_rad)) {
        if (detail) detail->clear();
        return true;
    }
    const int joint_count = kin_get_num_joints(kin_);
    if (joint_count <= 4 || joints.size() <= 4) {
        if (detail) *detail = "wrist yaw requires five arm joints";
        return false;
    }
    std::vector<double> lower(joint_count);
    std::vector<double> upper(joint_count);
    if (kin_get_joint_limits(kin_, lower.data(), upper.data()) != KIN_OK ||
        !ApplyJointLimits(config_.joint_limits, lower, upper)) {
        if (detail) *detail = "failed to read wrist joint limits";
        return false;
    }

    const float joint0 = joints[0];
    float raw_wrist = 0.0f;
    float resolved_wrist = 0.0f;
    if (!ResolveFixedJawWristYaw(
            grasp_yaw_rad, joint0, config_.wrist_yaw_scale,
            static_cast<float>(lower[4]),
            static_cast<float>(upper[4]),
            raw_wrist, resolved_wrist)) {
        if (detail) *detail = "invalid wrist yaw mapping";
        return false;
    }
    joints[4] = resolved_wrist;
    if (record_diagnostics) {
        diagnostics_.wrist_yaw.valid = true;
        diagnostics_.wrist_yaw.target_yaw = grasp_yaw_rad;
        diagnostics_.wrist_yaw.joint0 = joint0;
        diagnostics_.wrist_yaw.scale = config_.wrist_yaw_scale;
        diagnostics_.wrist_yaw.joint5_raw = raw_wrist;
        diagnostics_.wrist_yaw.joint5_limited = resolved_wrist;
        diagnostics_.wrist_yaw.joint5_min =
            static_cast<float>(lower[4]);
        diagnostics_.wrist_yaw.joint5_max =
            static_cast<float>(upper[4]);
    }
    if (detail) detail->clear();
    return true;
}

GraspResult GraspExecutor::SolveTopIKWithWristYaw(
    const Pose3D& pose,
    float grasp_yaw_rad,
    std::vector<float>& joints,
    const std::vector<float>* seed_joints,
    int timeout_ms,
    std::function<bool(std::vector<float>&)> candidate_validator,
    std::string* detail) {
    if (!kin_) return GraspResult::MOVE_FAILED;

    Pose3D oriented_pose = pose;
    if (!std::isnan(grasp_yaw_rad)) {
        // Expected tool axes after rotating the top-down grasp about base Z.
        const float half_yaw = 0.5f * grasp_yaw_rad;
        const float c = std::cos(half_yaw);
        const float s = std::sin(half_yaw);
        oriented_pose.qw = c * pose.qw - s * pose.qz;
        oriented_pose.qx = c * pose.qx - s * pose.qy;
        oriented_pose.qy = c * pose.qy + s * pose.qx;
        oriented_pose.qz = c * pose.qz + s * pose.qw;
    }

    // The SO101 has five arm DOF; with a yaw-constrained offset TCP, lateral
    // position and jaw yaw cannot both be exact. Bound total error below the
    // banana half-width while keeping the safety-critical vertical component
    // substantially tighter.
    constexpr double kMaximumTcpPositionErrorM = 0.020;
    constexpr double kMaximumTcpVerticalErrorM = 0.008;
    constexpr double kVerticalResidualWeight =
        kMaximumTcpPositionErrorM / kMaximumTcpVerticalErrorM;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(20, timeout_ms));

    const int joint_count = kin_get_num_joints(kin_);
    if (joint_count < 5 || joint_count > KIN_MAX_JOINTS) {
        if (detail) *detail = "top yaw IK requires five arm joints";
        return GraspResult::IK_FAILED;
    }
    std::vector<double> lower(joint_count);
    std::vector<double> upper(joint_count);
    if (kin_get_joint_limits(
            kin_, lower.data(), upper.data()) != KIN_OK ||
        !ApplyJointLimits(config_.joint_limits, lower, upper, detail)) {
        return GraspResult::IK_FAILED;
    }

    const auto forward = [this](
        const std::vector<float>& candidate,
        kin_pose_t& achieved) {
        kin_joints_t arm_joints = {};
        arm_joints.count = static_cast<uint8_t>(candidate.size());
        for (size_t joint = 0; joint < candidate.size(); ++joint) {
            arm_joints.q[joint] = candidate[joint];
        }
        return kin_forward(kin_, &arm_joints, &achieved) == KIN_OK;
    };
    const auto position_error = [&pose](const kin_pose_t& achieved) {
        const double dx = pose.x - achieved.x;
        const double dy = pose.y - achieved.y;
        const double dz = pose.z - achieved.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    const auto acceptance_score = [&pose, &position_error](
        const kin_pose_t& achieved) {
        return std::max(
            position_error(achieved) / kMaximumTcpPositionErrorM,
            std::fabs(achieved.z - pose.z) /
                kMaximumTcpVerticalErrorM);
    };
    const auto constraints_valid = [this, &lower, &upper](
        const std::vector<float>& candidate) {
        for (size_t joint = 0;
            joint < candidate.size() && joint < lower.size(); ++joint) {
            if (candidate[joint] < lower[joint] ||
                candidate[joint] > upper[joint]) {
                return false;
            }
        }
        for (const JointConstraint& constraint :
            config_.joint_constraints) {
            if (constraint.joint_index >= 0 &&
                constraint.joint_index <
                    static_cast<int>(candidate.size())) {
                const float value = candidate[constraint.joint_index];
                if (value < constraint.min_rad ||
                    value > constraint.max_rad) {
                    return false;
                }
            }
        }
        return true;
    };
    const auto wrist_yaw_error = [this, grasp_yaw_rad](
        const std::vector<float>& candidate) {
        std::vector<float> mapped = candidate;
        std::string ignored;
        if (!ApplyWristYaw(
                grasp_yaw_rad, mapped, &ignored, false)) {
            return std::numeric_limits<double>::infinity();
        }
        return std::fabs(
            static_cast<double>(candidate[4] - mapped[4]));
    };

    std::vector<float> candidate;
    bool use_seed_directly = false;
    // Cartesian top paths are sampled every 10 mm. Refine nearby waypoints
    // locally from the previous joint solution so the full-pose IK cannot
    // switch wrist branches at an orientation singularity. The first,
    // distant pre-grasp waypoint still uses the global constrained solver.
    constexpr double kMaximumLocalSeedDistanceM = 0.025;
    if (seed_joints && seed_joints->size() >= 5) {
        candidate = *seed_joints;
        std::string yaw_detail;
        if (ApplyWristYaw(
                grasp_yaw_rad, candidate, &yaw_detail, false)) {
            kin_pose_t seed_pose = {};
            use_seed_directly = forward(candidate, seed_pose) &&
                position_error(seed_pose) <=
                    kMaximumLocalSeedDistanceM;
        }
    }
    if (!use_seed_directly) {
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        const std::vector<float> path_seed = candidate;
        const GraspResult ik_result = SolveIKConstrained(
            pose, candidate,
            path_seed.empty() ? nullptr : &path_seed,
            std::max(1, remaining_ms));
        if (ik_result != GraspResult::SUCCESS) return ik_result;
        std::string yaw_detail;
        if (!ApplyWristYaw(
                grasp_yaw_rad, candidate, &yaw_detail, false)) {
            if (detail) *detail = yaw_detail;
            return GraspResult::IK_FAILED;
        }
    }

    // Refine all five joints on one continuous branch. Keep wrist yaw near the
    // requested jaw direction while allowing a small coupled correction that
    // restores the fifth degree of freedom needed by an offset TCP.
    kin_pose_t achieved = {};
    double current_error_m = std::numeric_limits<double>::infinity();
    constexpr double kFiniteDifferenceRad = 0.002;
    constexpr double kDamping = 1e-5;
    constexpr double kMaximumStepRad = 0.15;
    constexpr double kMaximumWristYawErrorRad = 0.18;
    for (int iteration = 0; iteration < 30; ++iteration) {
        if (std::chrono::steady_clock::now() >= deadline) {
            if (detail) *detail = "fixed-yaw top IK timed out";
            return GraspResult::TIMEOUT;
        }
        if (!forward(candidate, achieved)) {
            if (detail) *detail = "FK failed during fixed-yaw top IK";
            return GraspResult::IK_FAILED;
        }
        current_error_m = position_error(achieved);
        if (current_error_m <= kMaximumTcpPositionErrorM &&
            std::fabs(achieved.z - pose.z) <=
                kMaximumTcpVerticalErrorM) {
            break;
        }

        double jacobian[3][5] = {};
        for (int joint = 0; joint < 5; ++joint) {
            std::vector<float> plus = candidate;
            std::vector<float> minus = candidate;
            plus[joint] = std::min(
                plus[joint] + static_cast<float>(kFiniteDifferenceRad),
                static_cast<float>(upper[joint]));
            minus[joint] = std::max(
                minus[joint] - static_cast<float>(kFiniteDifferenceRad),
                static_cast<float>(lower[joint]));
            kin_pose_t plus_pose = {};
            kin_pose_t minus_pose = {};
            const double denominator = plus[joint] - minus[joint];
            if (denominator <= 1e-8 ||
                !forward(plus, plus_pose) ||
                !forward(minus, minus_pose)) {
                continue;
            }
            jacobian[0][joint] =
                (plus_pose.x - minus_pose.x) / denominator;
            jacobian[1][joint] =
                (plus_pose.y - minus_pose.y) / denominator;
            jacobian[2][joint] =
                (plus_pose.z - minus_pose.z) / denominator;
        }
        double augmented[3][4] = {};
        const double residual[3] = {
            pose.x - achieved.x,
            pose.y - achieved.y,
            (pose.z - achieved.z) * kVerticalResidualWeight,
        };
        for (int joint = 0; joint < 5; ++joint) {
            jacobian[2][joint] *= kVerticalResidualWeight;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                for (int joint = 0; joint < 5; ++joint) {
                    augmented[row][column] +=
                        jacobian[row][joint] *
                        jacobian[column][joint];
                }
            }
            augmented[row][row] += kDamping;
            augmented[row][3] = residual[row];
        }
        bool invertible = true;
        for (int pivot = 0; pivot < 3; ++pivot) {
            int best_row = pivot;
            for (int row = pivot + 1; row < 3; ++row) {
                if (std::fabs(augmented[row][pivot]) >
                    std::fabs(augmented[best_row][pivot])) {
                    best_row = row;
                }
            }
            if (std::fabs(augmented[best_row][pivot]) < 1e-10) {
                invertible = false;
                break;
            }
            if (best_row != pivot) {
                for (int column = pivot; column < 4; ++column) {
                    std::swap(
                        augmented[pivot][column],
                        augmented[best_row][column]);
                }
            }
            const double divisor = augmented[pivot][pivot];
            for (int column = pivot; column < 4; ++column) {
                augmented[pivot][column] /= divisor;
            }
            for (int row = 0; row < 3; ++row) {
                if (row == pivot) continue;
                const double factor = augmented[row][pivot];
                for (int column = pivot; column < 4; ++column) {
                    augmented[row][column] -=
                        factor * augmented[pivot][column];
                }
            }
        }
        if (!invertible) {
            if (detail) *detail = "fixed-yaw top IK Jacobian is singular";
            return GraspResult::IK_FAILED;
        }
        double delta[5] = {};
        double delta_norm = 0.0;
        for (int joint = 0; joint < 5; ++joint) {
            for (int axis = 0; axis < 3; ++axis) {
                delta[joint] +=
                    jacobian[axis][joint] * augmented[axis][3];
            }
            delta_norm += delta[joint] * delta[joint];
        }
        delta_norm = std::sqrt(delta_norm);
        const double delta_scale = delta_norm > kMaximumStepRad
            ? kMaximumStepRad / delta_norm
            : 1.0;
        bool improved = false;
        std::vector<float> best_candidate = candidate;
        double best_score = acceptance_score(achieved);
        for (double line_scale : {1.0, 0.5, 0.25, 0.125}) {
            std::vector<float> trial = candidate;
            for (int joint = 0; joint < 5; ++joint) {
                trial[joint] = std::clamp(
                    trial[joint] + static_cast<float>(
                        line_scale * delta_scale * delta[joint]),
                    static_cast<float>(lower[joint]),
                    static_cast<float>(upper[joint]));
            }
            if (!constraints_valid(trial) ||
                wrist_yaw_error(trial) > kMaximumWristYawErrorRad) {
                continue;
            }
            kin_pose_t trial_pose = {};
            if (!forward(trial, trial_pose)) continue;
            const double trial_score = acceptance_score(trial_pose);
            if (trial_score < best_score) {
                best_score = trial_score;
                best_candidate = std::move(trial);
                improved = true;
            }
        }
        if (!improved) {
            if (detail) {
                std::ostringstream stalled;
                stalled
                    << "fixed-yaw top IK stalled; position="
                    << current_error_m * 1000.0
                    << "mm vertical="
                    << std::fabs(achieved.z - pose.z) * 1000.0
                    << "mm wrist_yaw_error="
                    << wrist_yaw_error(candidate) << "rad joints=[";
                for (size_t joint = 0; joint < candidate.size(); ++joint) {
                    if (joint > 0) stalled << ",";
                    stalled << candidate[joint];
                }
                stalled << "]";
                *detail = stalled.str();
            }
            return GraspResult::IK_FAILED;
        }
        candidate = std::move(best_candidate);
    }
    if (!forward(candidate, achieved)) {
        if (detail) *detail = "FK failed after fixed-yaw top IK";
        return GraspResult::IK_FAILED;
    }
    const double position_error_m = position_error(achieved);
    const double vertical_error_m = std::fabs(achieved.z - pose.z);
    const double approach_error_deg = AxisAngleDegrees(
        ToolAxisZ(achieved.qw, achieved.qx, achieved.qy, achieved.qz),
        ToolAxisZ(
            oriented_pose.qw, oriented_pose.qx,
            oriented_pose.qy, oriented_pose.qz));
    const double opening_error_deg = AxisAngleDegrees(
        ToolAxisY(achieved.qw, achieved.qx, achieved.qy, achieved.qz),
        ToolAxisY(
            oriented_pose.qw, oriented_pose.qx,
            oriented_pose.qy, oriented_pose.qz));
    const double final_wrist_yaw_error_rad = wrist_yaw_error(candidate);
    if (position_error_m > kMaximumTcpPositionErrorM ||
        vertical_error_m > kMaximumTcpVerticalErrorM ||
        final_wrist_yaw_error_rad > kMaximumWristYawErrorRad) {
        if (detail) {
            *detail = "fixed-yaw top IK verification failed: position=" +
                std::to_string(position_error_m * 1000.0) +
                "mm vertical=" +
                std::to_string(vertical_error_m * 1000.0) +
                "mm approach=" + std::to_string(approach_error_deg) +
                "deg opening=" + std::to_string(opening_error_deg) +
                "deg wrist_yaw_error=" +
                std::to_string(final_wrist_yaw_error_rad) + "rad";
        }
        return GraspResult::IK_FAILED;
    }
    if (candidate_validator && !candidate_validator(candidate)) {
        if (detail) *detail = "fixed-yaw top IK path is unsafe";
        return GraspResult::OUT_OF_RANGE;
    }
    joints = std::move(candidate);
    if (config_.performance_log_enabled) {
        std::cout << "[Timing] component=TOP_FIXED_YAW_IK"
            << " position_error_mm=" << position_error_m * 1000.0
            << " approach_error_deg=" << approach_error_deg
            << " opening_error_deg=" << opening_error_deg
            << " wrist_yaw_error_rad=" << final_wrist_yaw_error_rad
            << " result=success" << std::endl;
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::PlanTopJointPathLegacy(
    const std::vector<Pose3D>& poses,
    float grasp_yaw_rad,
    int timeout_ms,
    std::vector<std::vector<float>>& joint_path,
    std::string* detail,
    const std::vector<float>* start_override) {
    joint_path.clear();
    if (poses.empty()) {
        if (detail) *detail = "top path contains no waypoints";
        return GraspResult::IK_FAILED;
    }
    if (!arm_path_safety_ || !support_plane_.valid) {
        if (detail) *detail = "top grasp support surface is unavailable";
        return GraspResult::OUT_OF_RANGE;
    }

    std::vector<float> start_joints;
    if (start_override) {
        start_joints = *start_override;
    } else if (!GetCurrentJoints(start_joints)) {
        if (detail) *detail = "failed to read top grasp start joints";
        return GraspResult::MOVE_FAILED;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(30, timeout_ms));
    constexpr float kMaximumWaypointJointDeltaRad = 0.45f;
    for (size_t index = 0; index < poses.size(); ++index) {
        if (std::chrono::steady_clock::now() >= deadline) {
            if (detail) *detail = "top path planning deadline exceeded";
            return GraspResult::TIMEOUT;
        }

        std::vector<float> solved_joints;
        bool reused_waypoint = false;
        for (size_t previous = index; previous > 0; --previous) {
            const size_t previous_index = previous - 1;
            if (PosesMatch(poses[index], poses[previous_index])) {
                solved_joints = joint_path[previous_index];
                reused_waypoint = true;
                break;
            }
        }
        if (!reused_waypoint) {
            std::string candidate_rejection;
            const auto candidate_validator =
                [&](std::vector<float>& candidate) {
                    std::string yaw_detail;
                    if (!ApplyWristYaw(
                            grasp_yaw_rad, candidate,
                            &yaw_detail, false)) {
                        candidate_rejection = yaw_detail;
                        return false;
                    }
                    float maximum_joint_delta = 0.0f;
                    for (size_t joint = 0;
                        joint < start_joints.size() &&
                        joint < candidate.size(); ++joint) {
                        maximum_joint_delta = std::max(
                            maximum_joint_delta,
                            std::fabs(
                                candidate[joint] -
                                start_joints[joint]));
                    }
                    if (index > 0 &&
                        maximum_joint_delta >
                            kMaximumWaypointJointDeltaRad) {
                        candidate_rejection = "discontinuous IK branch";
                        return false;
                    }
                    std::vector<std::vector<float>> candidate_path;
                    const GraspResult path_result = index == 0
                        ? BuildCollisionSafeJointPath(
                            start_joints, candidate,
                            candidate_path, &candidate_rejection)
                        : ValidateJointPathSafety(
                            start_joints, candidate,
                            &candidate_rejection);
                    return path_result == GraspResult::SUCCESS;
                };
            const int remaining_ms = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now()).count());
            const GraspResult ik_result = SolveIKConstrained(
                poses[index], solved_joints, &start_joints,
                std::max(1, remaining_ms), candidate_validator);
            if (ik_result != GraspResult::SUCCESS) {
                if (detail) {
                    *detail =
                        "top waypoint " + std::to_string(index + 1) +
                        "/" + std::to_string(poses.size()) + ": " +
                        (candidate_rejection.empty()
                            ? "IK failed"
                            : candidate_rejection);
                }
                return ik_result;
            }
        }
        std::string yaw_detail;
        if (!ApplyWristYaw(
                grasp_yaw_rad, solved_joints, &yaw_detail, false)) {
            if (detail) {
                *detail = "top waypoint " + std::to_string(index + 1) +
                    "/" + std::to_string(poses.size()) + ": " +
                    yaw_detail;
            }
            return GraspResult::IK_FAILED;
        }

        float maximum_joint_delta = 0.0f;
        for (size_t joint = 0;
            joint < start_joints.size() && joint < solved_joints.size();
            ++joint) {
            maximum_joint_delta = std::max(
                maximum_joint_delta,
                std::fabs(solved_joints[joint] - start_joints[joint]));
        }
        if (index > 0 &&
            maximum_joint_delta > kMaximumWaypointJointDeltaRad) {
            if (detail) {
                *detail = "top waypoint " + std::to_string(index + 1) +
                    "/" + std::to_string(poses.size()) +
                    ": discontinuous IK branch";
            }
            return GraspResult::IK_FAILED;
        }

        std::string path_detail;
        std::vector<std::vector<float>> collision_safe_path;
        const GraspResult path_result = index == 0
            ? BuildCollisionSafeJointPath(
                start_joints, solved_joints,
                collision_safe_path, &path_detail)
            : ValidateJointPathSafety(
                start_joints, solved_joints, &path_detail);
        if (path_result != GraspResult::SUCCESS) {
            if (detail) {
                *detail = "top waypoint " + std::to_string(index + 1) +
                    "/" + std::to_string(poses.size()) + ": " +
                    path_detail;
            }
            return path_result;
        }
        joint_path.push_back(solved_joints);
        start_joints = std::move(solved_joints);
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::PlanTopJointPath(
    const std::vector<Pose3D>& poses,
    float grasp_yaw_rad,
    int timeout_ms,
    std::vector<std::vector<float>>& joint_path,
    std::string* detail,
    const std::vector<float>* start_override) {
    if (config_.legacy_top_ik) {
        return PlanTopJointPathLegacy(
            poses, grasp_yaw_rad, timeout_ms, joint_path,
            detail, start_override);
    }
    joint_path.clear();
    if (poses.empty()) {
        if (detail) *detail = "top path contains no waypoints";
        return GraspResult::IK_FAILED;
    }
    if (!arm_path_safety_ || !support_plane_.valid) {
        if (detail) *detail = "top grasp support surface is unavailable";
        return GraspResult::OUT_OF_RANGE;
    }

    std::vector<float> start_joints;
    if (start_override) {
        start_joints = *start_override;
    } else if (!GetCurrentJoints(start_joints)) {
        if (detail) *detail = "failed to read top grasp start joints";
        return GraspResult::MOVE_FAILED;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(30, timeout_ms));
    constexpr float kMaximumWaypointJointDeltaRad = 0.45f;
    std::vector<float> grasp_branch_seed;
    if (std::isnan(grasp_yaw_rad) && poses.size() > 1) {
        const auto lowest_pose = std::min_element(
            poses.begin(), poses.end(),
            [](const Pose3D& lhs, const Pose3D& rhs) {
                return lhs.z < rhs.z;
            });
        const int anchor_budget_ms = std::max(
            20, timeout_ms / 3);
        if (lowest_pose != poses.end() &&
            SolveIKConstrained(
                *lowest_pose, grasp_branch_seed, nullptr,
                anchor_budget_ms) == GraspResult::SUCCESS) {
            std::cout << "[GraspExecutor] top path seeded from exact "
                "lowest-pose IK branch at waypoint="
                << (std::distance(poses.begin(), lowest_pose) + 1)
                << "/" << poses.size() << std::endl;
        } else {
            grasp_branch_seed.clear();
        }
    }
    for (size_t index = 0; index < poses.size(); ++index) {
        if (std::chrono::steady_clock::now() >= deadline) {
            if (detail) *detail = "top path planning deadline exceeded";
            return GraspResult::TIMEOUT;
        }

        std::vector<float> solved_joints;
        bool reused_waypoint = false;
        for (size_t previous = index; previous > 0; --previous) {
            const size_t previous_index = previous - 1;
            if (PosesMatch(poses[index], poses[previous_index])) {
                const auto& cached_joints = joint_path[previous_index];
                float maximum_cached_delta = 0.0f;
                for (size_t joint = 0;
                    joint < start_joints.size() &&
                    joint < cached_joints.size(); ++joint) {
                    maximum_cached_delta = std::max(
                        maximum_cached_delta,
                        std::fabs(
                            cached_joints[joint] - start_joints[joint]));
                }
                if (index == 0 ||
                    maximum_cached_delta <=
                        kMaximumWaypointJointDeltaRad) {
                    solved_joints = cached_joints;
                    reused_waypoint = true;
                    break;
                }
            }
        }
        if (!reused_waypoint) {
            std::string candidate_rejection;
            const auto candidate_validator =
                [&](std::vector<float>& candidate) {
                    float maximum_joint_delta = 0.0f;
                    size_t maximum_delta_joint = 0;
                    for (size_t joint = 0;
                        joint < start_joints.size() &&
                        joint < candidate.size(); ++joint) {
                        const float delta = std::fabs(
                            candidate[joint] - start_joints[joint]);
                        if (delta > maximum_joint_delta) {
                            maximum_joint_delta = delta;
                            maximum_delta_joint = joint;
                        }
                    }
                    if (index > 0 &&
                        maximum_joint_delta >
                            kMaximumWaypointJointDeltaRad) {
                        candidate_rejection =
                            "discontinuous IK branch: joint=" +
                            std::to_string(maximum_delta_joint) +
                            " delta_rad=" +
                            std::to_string(maximum_joint_delta) +
                            " from=" +
                            std::to_string(
                                start_joints[maximum_delta_joint]) +
                            " to=" +
                            std::to_string(candidate[maximum_delta_joint]);
                        return false;
                    }
                    std::vector<std::vector<float>> candidate_path;
                    const GraspResult path_result = index == 0
                        ? BuildCollisionSafeJointPath(
                            start_joints, candidate,
                            candidate_path, &candidate_rejection)
                        : ValidateJointPathSafety(
                            start_joints, candidate,
                            &candidate_rejection);
                    return path_result == GraspResult::SUCCESS;
                };
            const int remaining_ms = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now()).count());
            std::string compensation_detail;
            const std::vector<float>* waypoint_seed =
                index == 0 && !grasp_branch_seed.empty()
                ? &grasp_branch_seed
                : &start_joints;
            const GraspResult ik_result =
                SolveTopIKWithWristYaw(
                    poses[index], grasp_yaw_rad, solved_joints,
                    waypoint_seed, std::max(1, remaining_ms),
                    candidate_validator, &compensation_detail);
            if (ik_result != GraspResult::SUCCESS) {
                if (detail) {
                    const bool generic_path_rejection =
                        compensation_detail ==
                            "fixed-yaw top IK path is unsafe";
                    *detail =
                        "top waypoint " + std::to_string(index + 1) +
                        "/" + std::to_string(poses.size()) + ": " +
                        (generic_path_rejection &&
                            !candidate_rejection.empty()
                            ? candidate_rejection
                            : !compensation_detail.empty()
                            ? compensation_detail
                            : candidate_rejection.empty()
                            ? "IK failed"
                            : candidate_rejection);
                }
                return ik_result;
            }
        }
        float maximum_joint_delta = 0.0f;
        size_t maximum_delta_joint = 0;
        for (size_t joint = 0;
            joint < start_joints.size() && joint < solved_joints.size();
            ++joint) {
            const float delta =
                std::fabs(solved_joints[joint] - start_joints[joint]);
            if (delta > maximum_joint_delta) {
                maximum_joint_delta = delta;
                maximum_delta_joint = joint;
            }
        }
        if (index > 0 &&
            maximum_joint_delta > kMaximumWaypointJointDeltaRad) {
            if (detail) {
                *detail = "top waypoint " + std::to_string(index + 1) +
                    "/" + std::to_string(poses.size()) +
                    ": discontinuous IK branch: joint=" +
                    std::to_string(maximum_delta_joint) +
                    " delta_rad=" +
                    std::to_string(maximum_joint_delta) +
                    " from=" +
                    std::to_string(start_joints[maximum_delta_joint]) +
                    " to=" +
                    std::to_string(solved_joints[maximum_delta_joint]);
            }
            return GraspResult::IK_FAILED;
        }

        std::string path_detail;
        std::vector<std::vector<float>> collision_safe_path;
        const GraspResult path_result = index == 0
            ? BuildCollisionSafeJointPath(
                start_joints, solved_joints,
                collision_safe_path, &path_detail)
            : ValidateJointPathSafety(
                start_joints, solved_joints, &path_detail);
        if (path_result != GraspResult::SUCCESS) {
            if (detail) {
                *detail = "top waypoint " + std::to_string(index + 1) +
                    "/" + std::to_string(poses.size()) + ": " +
                    path_detail;
            }
            return path_result;
        }
        joint_path.push_back(solved_joints);
        start_joints = std::move(solved_joints);
    }
    if (detail) detail->clear();
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveAlongValidatedTopPath(
    const Pose3D& target_pose,
    float speed,
    const char* action,
    float final_tolerance_rad,
    bool allow_contact_retreat) {
    if (validated_top_poses_.size() !=
        validated_top_joint_path_.size()) {
        RecordResult(
            GraspResult::MOVE_FAILED, action,
            "validated top path is unavailable");
        return GraspResult::MOVE_FAILED;
    }

    size_t target_index = validated_top_path_index_;
    while (target_index < validated_top_poses_.size() &&
        !PosesMatch(target_pose, validated_top_poses_[target_index])) {
        ++target_index;
    }
    if (target_index == validated_top_poses_.size()) {
        RecordResult(
            GraspResult::MOVE_FAILED, action,
            "target is not part of the validated top path");
        return GraspResult::MOVE_FAILED;
    }

    std::vector<float> start_joints;
    if (!GetCurrentJoints(start_joints)) {
        RecordResult(
            GraspResult::MOVE_FAILED, action,
            "current joint state is unavailable");
        return GraspResult::MOVE_FAILED;
    }
    std::vector<std::vector<float>> execution_path;
    for (size_t index = validated_top_path_index_;
        index <= target_index; ++index) {
        std::string detail;
        const GraspResult validation = ValidateJointPathSafety(
            start_joints, validated_top_joint_path_[index], &detail,
            allow_contact_retreat);
        if (validation != GraspResult::SUCCESS) {
            RecordResult(validation, action, detail);
            return validation;
        }
        execution_path.push_back(validated_top_joint_path_[index]);
        start_joints = validated_top_joint_path_[index];
    }
    if (allow_contact_retreat) {
        const ArmPathSafetyResult target_safety =
            arm_path_safety_->CheckConfiguration(
                start_joints, support_plane_,
                config_.support_surface_clearance_m);
        if (!target_safety.safe) {
            RecordResult(
                GraspResult::OUT_OF_RANGE, action,
                "contact retreat did not reach normal clearance: " +
                    target_safety.detail);
            return GraspResult::OUT_OF_RANGE;
        }
    }

    const GraspResult move_result = ExecuteContinuousJointPath(
        execution_path, speed, speed, final_tolerance_rad, -1,
        allow_contact_retreat,
        [this, action, target_pose]() {
            return VerifyPoseReached(action, target_pose);
        });
    if (move_result != GraspResult::SUCCESS) {
        if (move_result == GraspResult::TIMEOUT &&
            VerifyPoseReached(action, target_pose)) {
            std::cout << "[GraspExecutor] " << action
                    << " accepted from measured pose despite joint "
                        "settling timeout"
                    << std::endl;
        } else {
            std::string detail =
                "validated Cartesian path execution failed";
            if (!last_motion_wait_detail_.empty()) {
                detail += "; " + last_motion_wait_detail_;
            }
            RecordResult(
                move_result, action, detail);
            return move_result;
        }
    } else if (!VerifyPoseReached(action, target_pose)) {
        std::string detail =
            "linear target pose was not reached";
        RecordResult(
            GraspResult::MOVE_FAILED, action,
            detail);
        return GraspResult::MOVE_FAILED;
    }
    validated_top_path_index_ = target_index + 1;
    RecordResult(GraspResult::SUCCESS, action);
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

GraspResult GraspExecutor::CorrectSidePose(
    const Pose3D& pose, float speed, const char* action) {
    constexpr int kMaximumCorrectionAttempts = 2;
    constexpr float kCorrectionToleranceRatio = 0.8f;
    const float correction_tolerance =
        config_.side_pose_position_tolerance * kCorrectionToleranceRatio;
    for (int attempt = 0; attempt <= kMaximumCorrectionAttempts; ++attempt) {
        Pose3D actual_pose{};
        if (!GetCurrentPose(actual_pose)) {
            RecordResult(
                GraspResult::MOVE_FAILED, action,
                "failed to read the current pose during side correction");
            return GraspResult::MOVE_FAILED;
        }
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
            RecordResult(
                plan_result, action,
                "side correction " + std::to_string(attempt + 1) +
                    " planning failed: " + detail);
            return plan_result;
        }
        const float correction_speed = std::min(speed, 0.3f);
        const GraspResult move_result = ExecuteContinuousJointPath(
            correction_path, correction_speed, correction_speed,
            kSideCorrectionJointToleranceRad,
            kSideMotionCompletionTimeoutMs);
        if (move_result != GraspResult::SUCCESS) {
            RecordResult(
                move_result, action,
                "side correction " + std::to_string(attempt + 1) +
                    " motion failed; target_position_error=" +
                    std::to_string(error) + "m");
            return move_result;
        }
    }
    std::cerr << "[GraspExecutor] " << action
                << " correction did not reach target" << std::endl;
    RecordResult(
        GraspResult::MOVE_FAILED, action,
        "side correction exhausted; target position tolerance=" +
            std::to_string(config_.side_pose_position_tolerance) + "m");
    return GraspResult::MOVE_FAILED;
}

GraspResult GraspExecutor::MoveToSidePreGrasp(const Pose3D& pose,
                                                float speed) {
    constexpr float kMaximumEntryHeightShortfallM = 0.010f;
    constexpr float kMaximumEntryPlanarErrorM = 0.030f;
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
        kSideStagingJointToleranceRad,
        kSideMotionCompletionTimeoutMs);
    if (result != GraspResult::SUCCESS) {
        RecordResult(
            result, "move_to_side_pre_grasp",
            result == GraspResult::TIMEOUT
                ? "side staging and joint0 sweep completion timeout"
                : "side staging or joint0 sweep failed");
        return result;
    }

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
    if (result != GraspResult::SUCCESS) {
        if (result == GraspResult::TIMEOUT) {
            Pose3D actual_pose{};
            if (GetCurrentPose(actual_pose)) {
                const float planar_error = std::hypot(
                    actual_pose.x - elevated_pre_grasp_pose.x,
                    actual_pose.y - elevated_pre_grasp_pose.y);
                const float minimum_safe_z =
                    elevated_pre_grasp_pose.z -
                    kMaximumEntryHeightShortfallM;
                const bool safely_at_entry =
                    actual_pose.z >= minimum_safe_z &&
                    planar_error <= kMaximumEntryPlanarErrorM;
                std::cout
                    << "[GraspExecutor] side elevated entry verification: "
                    << "target=[" << elevated_pre_grasp_pose.x << ","
                    << elevated_pre_grasp_pose.y << ","
                    << elevated_pre_grasp_pose.z << "] actual=["
                    << actual_pose.x << "," << actual_pose.y << ","
                    << actual_pose.z << "] minimum_safe_z="
                    << minimum_safe_z
                    << " planar_error=" << planar_error
                    << " result="
                    << (safely_at_entry ? "SAFE" : "UNSAFE")
                    << std::endl;
                if (safely_at_entry) {
                    std::cout
                        << "[GraspExecutor] side elevated entry accepted "
                        << "for Cartesian correction despite joint "
                        << "settling timeout"
                        << std::endl;
                    result = GraspResult::SUCCESS;
                }
            }
        }
    }
    if (result != GraspResult::SUCCESS) {
        std::string detail =
            result == GraspResult::TIMEOUT
                ? "elevated side entry completion timeout"
                : "elevated side entry motion failed";
        if (!last_motion_wait_detail_.empty()) {
            detail += "; " + last_motion_wait_detail_;
        }
        RecordResult(
            result, "move_to_side_pre_grasp", detail);
        return result;
    }
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

    std::string yaw_detail;
    if (!ApplyWristYaw(yaw_rad, joints, &yaw_detail, true)) {
        diagnostics_.wrist_yaw = WristYawDiagnostics{};
        RecordResult(
            GraspResult::IK_FAILED, "move_to_pose_with_yaw", yaw_detail);
        return GraspResult::IK_FAILED;
    }
    std::cout << "[GraspExecutor] wrist_yaw: target=" << yaw_rad
        << " rad, joint0=" << diagnostics_.wrist_yaw.joint0
        << ", raw joint5=" << diagnostics_.wrist_yaw.joint5_raw
        << ", limited joint5="
        << diagnostics_.wrist_yaw.joint5_limited
        << " (limit=[" << diagnostics_.wrist_yaw.joint5_min
        << ", " << diagnostics_.wrist_yaw.joint5_max << "])"
        << std::endl;

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

}  // namespace perceptive_grasp
