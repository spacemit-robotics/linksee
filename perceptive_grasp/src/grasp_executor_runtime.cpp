/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_executor_runtime.cpp
 * @brief Manipulator state access and continuous motion execution runtime
 */

#include "grasp_executor.h"

extern "C" {
#include "grasp.h"
#include "manipulator.h"
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

namespace perceptive_grasp {

void GraspExecutor::EmergencyStop() {
    if (arm_) manip_stop(arm_);
    if (gripper_) grasp_stop(gripper_);
    RecordResult(
        GraspResult::MOVE_FAILED, "emergency_stop",
        "emergency stop requested");
}

bool GraspExecutor::GetCurrentPose(Pose3D& pose) {
    if (!arm_) return false;

    manip_joint_t joints;
    std::memset(&joints, 0, sizeof(joints));
    manip_pose_t manipulator_pose;
    std::memset(&manipulator_pose, 0, sizeof(manipulator_pose));
    const int result =
        manip_get_state(arm_, &joints, &manipulator_pose);
    if (result != MANIP_OK) return false;

    pose.x = manipulator_pose.x;
    pose.y = manipulator_pose.y;
    pose.z = manipulator_pose.z;
    pose.qw = manipulator_pose.qw;
    pose.qx = manipulator_pose.qx;
    pose.qy = manipulator_pose.qy;
    pose.qz = manipulator_pose.qz;
    return true;
}

void GraspExecutor::Tick(float dt_s) {
    if (arm_) manip_tick(arm_, dt_s);
    if (gripper_) grasp_tick(gripper_, dt_s);
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

GraspResult GraspExecutor::ExecuteContinuousJointPath(
    const std::vector<std::vector<float>>& joint_path,
    float first_speed,
    float remaining_speed,
    float final_tolerance_rad,
    int completion_timeout_ms,
    bool allow_contact_retreat,
    std::function<bool()> settled_acceptance) {
    if (joint_path.empty()) return GraspResult::SUCCESS;

    std::vector<float> segment_start;
    if (!GetCurrentJoints(segment_start)) {
        return GraspResult::MOVE_FAILED;
    }
    active_motion_timeout_ms_ = EstimateJointMotionTimeoutMs(
        segment_start, joint_path.back(),
        std::min(first_speed, remaining_speed), 3000, 30000);
    std::vector<float> validation_start = segment_start;
    for (size_t index = 0; index < joint_path.size(); ++index) {
        std::string detail;
        const GraspResult validation = ValidateJointPathSafety(
            validation_start, joint_path[index], &detail,
            allow_contact_retreat);
        if (validation != GraspResult::SUCCESS) {
            std::cerr << "[GraspExecutor] continuous path segment "
                    << index + 1 << "/" << joint_path.size()
                    << " rejected: " << detail << std::endl;
            RecordResult(
                validation, "execute_continuous_joint_path", detail);
            return validation;
        }
        validation_start = joint_path[index];
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
            1, static_cast<int>(
                std::ceil(maximum_delta / maximum_step)));
        for (int step = 1; step <= step_count; ++step) {
            const float ratio =
                static_cast<float>(step) /
                static_cast<float>(step_count);
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
    const GraspResult wait_result = WaitMotionDone(
        completion_timeout_ms, final_tolerance_rad,
        std::move(settled_acceptance));
    config_.move_speed = original_speed;
    return wait_result;
}

GraspResult GraspExecutor::WaitMotionDone(
    int timeout_ms,
    float target_tolerance_rad,
    std::function<bool()> settled_acceptance) {
    last_motion_wait_detail_.clear();
    if (!arm_) {
        last_motion_wait_detail_ =
            "motion_state=COMMUNICATION_FAILED: manipulator is unavailable";
        return GraspResult::MOVE_FAILED;
    }

    if (timeout_ms <= 0) {
        timeout_ms = std::max(
            wait_motion_timeout_ms_, active_motion_timeout_ms_);
    } else {
        timeout_ms = std::max(timeout_ms, active_motion_timeout_ms_);
    }

    constexpr float kStableThreshold = 0.01f;
    constexpr int kStableCount = 10;
    constexpr int kPollIntervalMs = 50;
    const float effective_target_tolerance_rad =
        EffectiveMotionTargetToleranceRad(target_tolerance_rad);

    manip_joint_t previous_joints = {};
    const int initial_read_result =
        manip_get_state(arm_, &previous_joints, nullptr);

    int stable_counter = 0;
    manip_joint_t current_joints = previous_joints;
    float current_target_error = 0.0f;
    MotionCompletionEvidence evidence;
    evidence.has_target =
        !active_target_joints_.empty() &&
        initial_read_result == MANIP_OK &&
        active_target_joints_.size() == previous_joints.count;
    evidence.successful_reads =
        initial_read_result == MANIP_OK ? 1 : 0;
    evidence.consecutive_read_failures =
        initial_read_result == MANIP_OK ? 0 : 1;
    bool has_initial_target_error = false;
    std::deque<float> recent_joint_deltas;
    const auto start = std::chrono::steady_clock::now();
    int effective_timeout_ms = timeout_ms;
    int timeout_extension_count = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while (true) {
        const auto elapsed =
            std::chrono::steady_clock::now() - start;
        const int elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                elapsed).count();

        if (elapsed_ms >= effective_timeout_ms) {
            evidence.stable_samples = stable_counter;
            evidence.target_error_rad = current_target_error;
            evidence.recent_joint_delta_rad = 0.0f;
            for (float delta : recent_joint_deltas) {
                evidence.recent_joint_delta_rad = std::max(
                    evidence.recent_joint_delta_rad, delta);
            }
            const MotionCompletionState state =
                ClassifyMotionCompletion(
                    evidence, target_tolerance_rad,
                    kStableThreshold, kStableCount);
            const int extension_ms =
                RecommendedMotionTimeoutExtensionMs(
                    state, timeout_extension_count, timeout_ms);
            if (extension_ms > 0) {
                effective_timeout_ms += extension_ms;
                ++timeout_extension_count;
                std::cout
                    << "[GraspExecutor] motion still progressing; "
                    << "extend completion wait by " << extension_ms
                    << "ms" << std::endl;
                continue;
            }
            last_motion_wait_detail_ = DescribeMotionCompletion(
                state, evidence, target_tolerance_rad,
                effective_timeout_ms);
            std::cerr << "[GraspExecutor] "
                    << last_motion_wait_detail_;
            if (!active_target_joints_.empty()) {
                std::cerr << " target=[";
                for (size_t index = 0;
                    index < active_target_joints_.size(); ++index) {
                    if (index > 0) std::cerr << ",";
                    std::cerr << active_target_joints_[index];
                }
                std::cerr << "] actual=[";
                for (int index = 0;
                    index < current_joints.count; ++index) {
                    if (index > 0) std::cerr << ",";
                    std::cerr << current_joints.joints[index];
                }
                std::cerr << "]";
            }
            std::cerr << std::endl;
            return state == MotionCompletionState::COMMUNICATION_FAILED
                ? GraspResult::MOVE_FAILED
                : GraspResult::TIMEOUT;
        }

        Tick(static_cast<float>(kPollIntervalMs) / 1000.0f);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kPollIntervalMs));

        std::memset(&current_joints, 0, sizeof(current_joints));
        const int read_result =
            manip_get_state(arm_, &current_joints, nullptr);
        if (read_result != MANIP_OK || current_joints.count == 0) {
            ++evidence.consecutive_read_failures;
            stable_counter = 0;
            continue;
        }
        ++evidence.successful_reads;
        evidence.consecutive_read_failures = 0;

        float maximum_delta = 0.0f;
        for (int index = 0;
            index < current_joints.count &&
                index < previous_joints.count;
            ++index) {
            const float delta = std::fabs(
                current_joints.joints[index] -
                previous_joints.joints[index]);
            maximum_delta = std::max(maximum_delta, delta);
        }
        recent_joint_deltas.push_back(maximum_delta);
        constexpr size_t kRecentMotionSampleCount = 10;
        if (recent_joint_deltas.size() > kRecentMotionSampleCount) {
            recent_joint_deltas.pop_front();
        }

        float maximum_target_error = 0.0f;
        const bool has_joint_target =
            !active_target_joints_.empty() &&
            active_target_joints_.size() == current_joints.count;
        if (has_joint_target) {
            for (size_t index = 0;
                index < active_target_joints_.size(); ++index) {
                float error = std::fabs(
                    current_joints.joints[index] -
                    active_target_joints_[index]);
                if (error > static_cast<float>(M_PI)) {
                    error =
                        static_cast<float>(2.0 * M_PI) - error;
                }
                maximum_target_error =
                    std::max(maximum_target_error, error);
            }
        }
        current_target_error = maximum_target_error;
        evidence.has_target = has_joint_target;
        if (has_joint_target) {
            if (!has_initial_target_error) {
                evidence.initial_target_error_rad =
                    maximum_target_error;
                evidence.best_target_error_rad =
                    maximum_target_error;
                has_initial_target_error = true;
            } else {
                evidence.best_target_error_rad = std::min(
                    evidence.best_target_error_rad,
                    maximum_target_error);
            }
        }

        if (maximum_delta < kStableThreshold) {
            stable_counter++;
            if (stable_counter == kStableCount &&
                has_joint_target &&
                maximum_target_error >
                    effective_target_tolerance_rad &&
                settled_acceptance &&
                settled_acceptance()) {
                std::cout
                    << "[GraspExecutor] Settled joint target accepted "
                    << "from measured Cartesian pose: target_error="
                    << maximum_target_error
                    << "rad effective_tolerance="
                    << effective_target_tolerance_rad
                    << "rad"
                    << std::endl;
                last_motion_wait_detail_ =
                    "motion_state=REACHED_BY_CARTESIAN_POSE";
                return GraspResult::SUCCESS;
            }
            if (stable_counter >= kStableCount &&
                (!has_joint_target ||
                maximum_target_error <=
                    effective_target_tolerance_rad)) {
                if (has_joint_target &&
                    maximum_target_error > target_tolerance_rad) {
                    std::cout
                        << "[GraspExecutor] Joint target accepted with "
                        << "settled boundary slack: target_error="
                        << maximum_target_error
                        << "rad configured_tolerance="
                        << target_tolerance_rad
                        << "rad effective_tolerance="
                        << effective_target_tolerance_rad
                        << "rad"
                        << std::endl;
                } else if (has_joint_target &&
                    maximum_target_error > 0.060f) {
                    std::cout
                        << "[GraspExecutor] Joint waypoint accepted "
                        << "with target_error="
                        << maximum_target_error
                        << "rad; Cartesian correction remains active"
                        << std::endl;
                }
                last_motion_wait_detail_ = "motion_state=REACHED";
                return GraspResult::SUCCESS;
            }
        } else {
            stable_counter = 0;
        }

        previous_joints = current_joints;
    }
}

bool GraspExecutor::VerifyPoseReached(
    const char* action, const Pose3D& target_pose) {
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
    const float position_error =
        std::sqrt(dx * dx + dy * dy + dz * dz);

    std::cout << "[GraspExecutor] " << action
            << " target_pose=[" << target_pose.x << ", "
            << target_pose.y << ", " << target_pose.z << "]"
            << " actual_pose=[" << actual_pose.x << ", "
            << actual_pose.y << ", " << actual_pose.z << "]"
            << " position_error=" << position_error << " m"
            << " tolerance="
            << config_.pose_position_tolerance << " m"
            << std::endl;

    if (position_error > config_.pose_position_tolerance) {
        std::cerr << "[GraspExecutor] " << action
                << " target not reached" << std::endl;
        return false;
    }
    return true;
}

}  // namespace perceptive_grasp
