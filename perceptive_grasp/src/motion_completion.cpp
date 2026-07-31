/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_completion.cpp
 * @brief Joint-motion timeout estimation and completion classification.
 */

#include "motion_completion.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace perceptive_grasp {

float EffectiveMotionTargetToleranceRad(float configured_tolerance_rad) {
    constexpr float kSettledBoundarySlackRad = 0.020f;
    return std::max(0.0f, configured_tolerance_rad) +
        kSettledBoundarySlackRad;
}

int EstimateJointMotionTimeoutMs(
    const std::vector<float>& start_joints,
    const std::vector<float>& target_joints,
    float speed_ratio,
    int minimum_timeout_ms,
    int maximum_timeout_ms) {
    if (minimum_timeout_ms <= 0 || maximum_timeout_ms < minimum_timeout_ms ||
        start_joints.empty() ||
        start_joints.size() != target_joints.size()) {
        return std::max(minimum_timeout_ms, maximum_timeout_ms);
    }

    float maximum_delta_rad = 0.0f;
    for (size_t index = 0; index < start_joints.size(); ++index) {
        maximum_delta_rad = std::max(
            maximum_delta_rad,
            std::fabs(target_joints[index] - start_joints[index]));
    }

    // SO101 speed is a ratio rather than a calibrated angular velocity.
    // This conservative model leaves time for acceleration and final settling.
    constexpr float kNominalJointVelocityRadPerSecond = 0.45f;
    constexpr float kMotionSafetyFactor = 1.8f;
    constexpr int kSettlingMarginMs = 1500;
    const float effective_speed = std::clamp(speed_ratio, 0.10f, 1.0f);
    const float expected_seconds =
        maximum_delta_rad /
        (kNominalJointVelocityRadPerSecond * effective_speed);
    const int estimated_ms = static_cast<int>(std::ceil(
        expected_seconds * 1000.0f * kMotionSafetyFactor)) +
        kSettlingMarginMs;
    return std::clamp(
        estimated_ms, minimum_timeout_ms, maximum_timeout_ms);
}

MotionCompletionState ClassifyMotionCompletion(
    const MotionCompletionEvidence& evidence,
    float target_tolerance_rad,
    float stable_threshold_rad,
    int required_stable_samples) {
    if (evidence.successful_reads == 0 ||
        evidence.consecutive_read_failures >= 3) {
        return MotionCompletionState::COMMUNICATION_FAILED;
    }
    if (evidence.stable_samples >= required_stable_samples) {
        if (!evidence.has_target ||
            evidence.target_error_rad <=
                EffectiveMotionTargetToleranceRad(
                    target_tolerance_rad)) {
            return MotionCompletionState::REACHED;
        }
        return MotionCompletionState::SETTLED_OUTSIDE_TOLERANCE;
    }
    if (evidence.recent_joint_delta_rad >= stable_threshold_rad) {
        return MotionCompletionState::STILL_MOVING;
    }
    constexpr float kMinimumProgressRad = 0.015f;
    if (evidence.has_target &&
        evidence.initial_target_error_rad -
            evidence.best_target_error_rad >= kMinimumProgressRad) {
        return MotionCompletionState::STILL_MOVING;
    }
    return MotionCompletionState::STALLED;
}

int RecommendedMotionTimeoutExtensionMs(
    MotionCompletionState state,
    int extension_count,
    int timeout_ms) {
    if (state != MotionCompletionState::STILL_MOVING ||
        extension_count > 0 || timeout_ms <= 0) {
        return 0;
    }
    constexpr int kMinimumExtensionMs = 2000;
    constexpr int kMaximumExtensionMs = 5000;
    return std::clamp(
        timeout_ms / 2, kMinimumExtensionMs, kMaximumExtensionMs);
}

const char* MotionCompletionStateName(MotionCompletionState state) {
    switch (state) {
        case MotionCompletionState::REACHED:
            return "REACHED";
        case MotionCompletionState::COMMUNICATION_FAILED:
            return "COMMUNICATION_FAILED";
        case MotionCompletionState::STILL_MOVING:
            return "STILL_MOVING";
        case MotionCompletionState::STALLED:
            return "STALLED";
        case MotionCompletionState::SETTLED_OUTSIDE_TOLERANCE:
            return "SETTLED_OUTSIDE_TOLERANCE";
    }
    return "UNKNOWN";
}

std::string DescribeMotionCompletion(
    MotionCompletionState state,
    const MotionCompletionEvidence& evidence,
    float target_tolerance_rad,
    int timeout_ms) {
    std::ostringstream output;
    output << "motion_state=" << MotionCompletionStateName(state)
        << " timeout_ms=" << timeout_ms
        << " target_error_rad=" << evidence.target_error_rad
        << " tolerance_rad=" << target_tolerance_rad
        << " effective_tolerance_rad="
        << EffectiveMotionTargetToleranceRad(target_tolerance_rad)
        << " recent_joint_delta_rad="
        << evidence.recent_joint_delta_rad
        << " stable_samples=" << evidence.stable_samples
        << " successful_reads=" << evidence.successful_reads
        << " consecutive_read_failures="
        << evidence.consecutive_read_failures;
    return output.str();
}

}  // namespace perceptive_grasp
