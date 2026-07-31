/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_completion.h
 * @brief Joint-motion timeout estimation and completion classification.
 */

#ifndef MOTION_COMPLETION_H
#define MOTION_COMPLETION_H

#include <string>
#include <vector>

namespace perceptive_grasp {

enum class MotionCompletionState {
    REACHED,
    COMMUNICATION_FAILED,
    STILL_MOVING,
    STALLED,
    SETTLED_OUTSIDE_TOLERANCE,
};

struct MotionCompletionEvidence {
    bool has_target = false;
    int successful_reads = 0;
    int consecutive_read_failures = 0;
    int stable_samples = 0;
    float target_error_rad = 0.0f;
    float initial_target_error_rad = 0.0f;
    float best_target_error_rad = 0.0f;
    float recent_joint_delta_rad = 0.0f;
};

int EstimateJointMotionTimeoutMs(
    const std::vector<float>& start_joints,
    const std::vector<float>& target_joints,
    float speed_ratio,
    int minimum_timeout_ms,
    int maximum_timeout_ms);

float EffectiveMotionTargetToleranceRad(float configured_tolerance_rad);

MotionCompletionState ClassifyMotionCompletion(
    const MotionCompletionEvidence& evidence,
    float target_tolerance_rad,
    float stable_threshold_rad,
    int required_stable_samples);

int RecommendedMotionTimeoutExtensionMs(
    MotionCompletionState state,
    int extension_count,
    int timeout_ms);

const char* MotionCompletionStateName(MotionCompletionState state);

std::string DescribeMotionCompletion(
    MotionCompletionState state,
    const MotionCompletionEvidence& evidence,
    float target_tolerance_rad,
    int timeout_ms);

}  // namespace perceptive_grasp

#endif  // MOTION_COMPLETION_H
