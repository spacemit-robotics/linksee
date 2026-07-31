/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file gripper_holding.h
 * @brief Gripper baseline estimation and sustained holding evidence.
 */

#ifndef GRIPPER_HOLDING_H
#define GRIPPER_HOLDING_H

#include <cstddef>
#include <limits>
#include <vector>

namespace perceptive_grasp {

struct GripperFeedbackSample {
    float position = std::numeric_limits<float>::quiet_NaN();
    float load = std::numeric_limits<float>::quiet_NaN();
    bool state_holding = false;
    bool state_empty = false;
};

struct GripperBaseline {
    bool valid = false;
    size_t sample_count = 0;
    float position_median = std::numeric_limits<float>::quiet_NaN();
    float position_mad = std::numeric_limits<float>::quiet_NaN();
    float load_median = std::numeric_limits<float>::quiet_NaN();
    float load_mad = std::numeric_limits<float>::quiet_NaN();
};

enum class GripperHoldingResult {
    HOLDING,
    EMPTY,
    INCONCLUSIVE,
};

struct GripperHoldingConfig {
    float minimum_load_threshold = 100.0f;
    float opening_margin = 0.03f;
    int required_consecutive_samples = 3;
};

struct GripperHoldingEvidence {
    GripperHoldingResult result = GripperHoldingResult::INCONCLUSIVE;
    int opening_count = 0;
    int load_count = 0;
    int state_holding_count = 0;
    int contact_count = 0;
    int empty_count = 0;
    int sample_count = 0;
    float minimum_object_position =
        std::numeric_limits<float>::quiet_NaN();
    float effective_load_threshold = 100.0f;
    float position = std::numeric_limits<float>::quiet_NaN();
    float load = std::numeric_limits<float>::quiet_NaN();
};

GripperBaseline EstimateGripperBaseline(
    const std::vector<GripperFeedbackSample>& samples);

GripperHoldingEvidence EvaluateGripperHolding(
    const std::vector<GripperFeedbackSample>& samples,
    const GripperBaseline& baseline,
    const GripperHoldingConfig& config);

const char* GripperHoldingResultName(GripperHoldingResult result);

}  // namespace perceptive_grasp

#endif  // GRIPPER_HOLDING_H
