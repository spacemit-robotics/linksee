/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_completion_test.cpp
 * @brief Tests joint-motion timeout and completion classification.
 */

#include "motion_completion.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using perceptive_grasp::ClassifyMotionCompletion;
using perceptive_grasp::DescribeMotionCompletion;
using perceptive_grasp::EffectiveMotionTargetToleranceRad;
using perceptive_grasp::EstimateJointMotionTimeoutMs;
using perceptive_grasp::MotionCompletionEvidence;
using perceptive_grasp::MotionCompletionState;
using perceptive_grasp::RecommendedMotionTimeoutExtensionMs;

namespace {

bool ExpectState(const char* name,
    const MotionCompletionEvidence& evidence,
    MotionCompletionState expected) {
    const MotionCompletionState actual = ClassifyMotionCompletion(
        evidence, 0.06f, 0.01f, 10);
    if (actual == expected) return true;
    std::cerr << name << " returned "
        << DescribeMotionCompletion(actual, evidence, 0.06f, 5000)
        << std::endl;
    return false;
}

}  // namespace

int main() {
    const std::vector<float> start{0.0f, 0.0f};
    const int short_timeout = EstimateJointMotionTimeoutMs(
        start, {0.05f, 0.02f}, 1.0f, 3000, 20000);
    const int long_timeout = EstimateJointMotionTimeoutMs(
        start, {1.5f, 0.2f}, 0.5f, 3000, 20000);
    if (short_timeout != 3000 || long_timeout <= short_timeout ||
        long_timeout > 20000) {
        std::cerr << "motion timeout estimate is not distance-aware"
            << std::endl;
        return 1;
    }

    MotionCompletionEvidence reached;
    reached.has_target = true;
    reached.successful_reads = 20;
    reached.stable_samples = 10;
    reached.target_error_rad = 0.02f;
    if (!ExpectState(
            "reached", reached, MotionCompletionState::REACHED)) {
        return 1;
    }

    MotionCompletionEvidence communication_failed;
    communication_failed.consecutive_read_failures = 3;
    if (!ExpectState(
            "communication_failed", communication_failed,
            MotionCompletionState::COMMUNICATION_FAILED)) {
        return 1;
    }

    MotionCompletionEvidence moving;
    moving.has_target = true;
    moving.successful_reads = 20;
    moving.target_error_rad = 0.20f;
    moving.initial_target_error_rad = 1.0f;
    moving.best_target_error_rad = 0.20f;
    moving.recent_joint_delta_rad = 0.02f;
    if (!ExpectState(
            "moving", moving, MotionCompletionState::STILL_MOVING)) {
        return 1;
    }

    MotionCompletionEvidence stalled;
    stalled.has_target = true;
    stalled.successful_reads = 20;
    stalled.target_error_rad = 0.40f;
    stalled.initial_target_error_rad = 0.40f;
    stalled.best_target_error_rad = 0.40f;
    if (!ExpectState(
            "stalled", stalled, MotionCompletionState::STALLED)) {
        return 1;
    }

    MotionCompletionEvidence settled = stalled;
    settled.stable_samples = 10;
    if (!ExpectState(
            "settled", settled,
            MotionCompletionState::SETTLED_OUTSIDE_TOLERANCE)) {
        return 1;
    }

    MotionCompletionEvidence settled_boundary = reached;
    settled_boundary.target_error_rad = 0.079f;
    if (!ExpectState(
            "settled_boundary", settled_boundary,
            MotionCompletionState::REACHED) ||
        std::fabs(
            EffectiveMotionTargetToleranceRad(0.06f) - 0.080f) >
            1e-6f) {
        std::cerr << "settled boundary slack is incorrect" << std::endl;
        return 1;
    }

    MotionCompletionEvidence settled_beyond_boundary = reached;
    settled_beyond_boundary.target_error_rad = 0.081f;
    if (!ExpectState(
            "settled_beyond_boundary", settled_beyond_boundary,
            MotionCompletionState::SETTLED_OUTSIDE_TOLERANCE)) {
        return 1;
    }

    if (RecommendedMotionTimeoutExtensionMs(
            MotionCompletionState::STILL_MOVING, 0, 5000) != 2500 ||
        RecommendedMotionTimeoutExtensionMs(
            MotionCompletionState::STILL_MOVING, 0, 20000) != 5000 ||
        RecommendedMotionTimeoutExtensionMs(
            MotionCompletionState::STILL_MOVING, 1, 5000) != 0 ||
        RecommendedMotionTimeoutExtensionMs(
            MotionCompletionState::STALLED, 0, 5000) != 0) {
        std::cerr << "motion timeout extension policy is unsafe"
            << std::endl;
        return 1;
    }
    return 0;
}
