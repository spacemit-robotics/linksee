/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file gripper_holding.cpp
 * @brief Gripper baseline estimation and sustained holding evidence.
 */

#include "gripper_holding.h"

#include <algorithm>
#include <cmath>

namespace perceptive_grasp {

namespace {

constexpr float kFallbackEmptyPosition = 0.02f;
constexpr float kMinimumPositionNoiseMargin = 0.005f;
constexpr float kMinimumBaselineLoadMargin = 40.0f;
constexpr float kNoiseScale = 4.0f;
constexpr float kPartialOpeningMarginRatio = 0.5f;

float Median(std::vector<float> values) {
    if (values.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const size_t middle = values.size() / 2;
    std::nth_element(
        values.begin(), values.begin() + middle, values.end());
    const float upper = values[middle];
    if (values.size() % 2 != 0) return upper;
    std::nth_element(
        values.begin(), values.begin() + middle - 1, values.end());
    return 0.5f * (values[middle - 1] + upper);
}

float MedianAbsoluteDeviation(const std::vector<float>& values,
    float median) {
    std::vector<float> deviations;
    deviations.reserve(values.size());
    for (float value : values) {
        deviations.push_back(std::fabs(value - median));
    }
    return Median(std::move(deviations));
}

int UpdateConsecutiveCount(bool evidence, int count) {
    return evidence ? count + 1 : 0;
}

}  // namespace

GripperBaseline EstimateGripperBaseline(
    const std::vector<GripperFeedbackSample>& samples) {
    std::vector<float> positions;
    std::vector<float> loads;
    positions.reserve(samples.size());
    loads.reserve(samples.size());
    for (const GripperFeedbackSample& sample : samples) {
        if (!std::isfinite(sample.position) ||
            !std::isfinite(sample.load)) {
            continue;
        }
        positions.push_back(sample.position);
        loads.push_back(sample.load);
    }

    GripperBaseline baseline;
    baseline.sample_count = positions.size();
    if (positions.size() < 3) return baseline;

    baseline.position_median = Median(positions);
    baseline.position_mad =
        MedianAbsoluteDeviation(positions, baseline.position_median);
    baseline.load_median = Median(loads);
    baseline.load_mad =
        MedianAbsoluteDeviation(loads, baseline.load_median);
    baseline.valid =
        std::isfinite(baseline.position_median) &&
        std::isfinite(baseline.position_mad) &&
        std::isfinite(baseline.load_median) &&
        std::isfinite(baseline.load_mad);
    return baseline;
}

GripperHoldingEvidence EvaluateGripperHolding(
    const std::vector<GripperFeedbackSample>& samples,
    const GripperBaseline& baseline,
    const GripperHoldingConfig& config) {
    GripperHoldingEvidence evidence;
    const float baseline_position = baseline.valid
        ? baseline.position_median
        : kFallbackEmptyPosition;
    const float position_noise_margin = baseline.valid
        ? std::max(
            kMinimumPositionNoiseMargin,
            kNoiseScale * baseline.position_mad)
        : kMinimumPositionNoiseMargin;
    evidence.minimum_object_position =
        baseline_position +
        std::max(config.opening_margin, position_noise_margin);
    const float minimum_partial_object_position =
        baseline_position +
        std::max(
            position_noise_margin,
            kPartialOpeningMarginRatio *
                std::max(0.0f, config.opening_margin));

    const float baseline_load_threshold = baseline.valid
        ? baseline.load_median +
            std::max(
                kMinimumBaselineLoadMargin,
                kNoiseScale * baseline.load_mad)
        : config.minimum_load_threshold;
    evidence.effective_load_threshold = std::max(
        config.minimum_load_threshold, baseline_load_threshold);

    const int required = std::max(
        1, config.required_consecutive_samples);
    int opening_count = 0;
    int load_count = 0;
    int state_holding_count = 0;
    int contact_count = 0;
    int empty_count = 0;
    for (const GripperFeedbackSample& sample : samples) {
        if (!std::isfinite(sample.position) ||
            !std::isfinite(sample.load)) {
            opening_count = 0;
            load_count = 0;
            state_holding_count = 0;
            contact_count = 0;
            empty_count = 0;
            continue;
        }

        evidence.sample_count++;
        evidence.position = sample.position;
        evidence.load = sample.load;
        const bool opening =
            sample.position > evidence.minimum_object_position;
        const bool partial_opening =
            sample.position > minimum_partial_object_position;
        const bool loaded =
            sample.load >= evidence.effective_load_threshold;
        const int holding_votes =
            static_cast<int>(opening) +
            static_cast<int>(loaded) +
            static_cast<int>(sample.state_holding);
        const bool contact =
            holding_votes >= 2 || (loaded && partial_opening);
        const bool empty =
            !opening && !loaded && sample.state_empty;

        opening_count = UpdateConsecutiveCount(opening, opening_count);
        load_count = UpdateConsecutiveCount(loaded, load_count);
        state_holding_count = UpdateConsecutiveCount(
            sample.state_holding, state_holding_count);
        contact_count = UpdateConsecutiveCount(contact, contact_count);
        empty_count = UpdateConsecutiveCount(empty, empty_count);

        evidence.opening_count =
            std::max(evidence.opening_count, opening_count);
        evidence.load_count = std::max(evidence.load_count, load_count);
        evidence.state_holding_count = std::max(
            evidence.state_holding_count, state_holding_count);
        evidence.contact_count =
            std::max(evidence.contact_count, contact_count);
        evidence.empty_count =
            std::max(evidence.empty_count, empty_count);
    }

    // Classify from the latest sustained sequence, not an earlier transient.
    // The max counters above remain useful diagnostics, but must not cause an
    // old contact event to mask a later confirmed-empty close.
    if (contact_count >= required) {
        evidence.result = GripperHoldingResult::HOLDING;
    } else if (empty_count >= required) {
        evidence.result = GripperHoldingResult::EMPTY;
    }
    return evidence;
}

const char* GripperHoldingResultName(GripperHoldingResult result) {
    switch (result) {
        case GripperHoldingResult::HOLDING: return "HOLDING";
        case GripperHoldingResult::EMPTY: return "EMPTY";
        case GripperHoldingResult::INCONCLUSIVE: return "INCONCLUSIVE";
    }
    return "UNKNOWN";
}

}  // namespace perceptive_grasp
