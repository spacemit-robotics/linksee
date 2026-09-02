/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file gripper_holding_test.cpp
 * @brief Unit tests for sustained gripper holding evidence.
 */

#include <cassert>
#include <cmath>
#include <vector>

#include "gripper_holding.h"

namespace {

using perceptive_grasp::EstimateGripperBaseline;
using perceptive_grasp::EvaluateGripperHolding;
using perceptive_grasp::GripperBaseline;
using perceptive_grasp::GripperFeedbackSample;
using perceptive_grasp::GripperHoldingConfig;
using perceptive_grasp::GripperHoldingResult;

std::vector<GripperFeedbackSample> RepeatedSamples(
    float position, float load, bool holding, bool empty, int count) {
    return std::vector<GripperFeedbackSample>(
        static_cast<size_t>(count),
        GripperFeedbackSample{position, load, holding, empty});
}

}  // namespace

int main() {
    const std::vector<GripperFeedbackSample> baseline_samples = {
        {0.001f, 20.0f, false, true},
        {0.002f, 24.0f, false, true},
        {0.003f, 28.0f, false, true},
        {0.002f, 24.0f, false, true},
        {0.001f, 22.0f, false, true},
    };
    const GripperBaseline baseline =
        EstimateGripperBaseline(baseline_samples);
    assert(baseline.valid);
    assert(baseline.sample_count == baseline_samples.size());
    assert(std::fabs(baseline.position_median - 0.002f) < 1e-6f);
    assert(std::fabs(baseline.load_median - 24.0f) < 1e-6f);

    GripperHoldingConfig config;
    config.minimum_load_threshold = 100.0f;
    config.opening_margin = 0.03f;
    config.required_consecutive_samples = 3;

    const auto moving_contact = EvaluateGripperHolding(
        RepeatedSamples(0.14f, 200.0f, false, false, 3),
        baseline, config);
    assert(moving_contact.result == GripperHoldingResult::HOLDING);
    assert(moving_contact.contact_count == 3);
    assert(moving_contact.effective_load_threshold == 100.0f);

    const auto holding_state = EvaluateGripperHolding(
        RepeatedSamples(0.14f, 80.0f, true, false, 3),
        baseline, config);
    assert(
        holding_state.result == GripperHoldingResult::HOLDING);
    assert(holding_state.state_holding_count == 3);

    const auto empty = EvaluateGripperHolding(
        RepeatedSamples(0.002f, 24.0f, false, true, 3),
        baseline, config);
    assert(empty.result == GripperHoldingResult::EMPTY);
    assert(empty.empty_count == 3);

    const auto conflicting_driver_state = EvaluateGripperHolding(
        RepeatedSamples(0.14f, 200.0f, false, true, 3),
        baseline, config);
    assert(
        conflicting_driver_state.result ==
        GripperHoldingResult::HOLDING);

    std::vector<GripperFeedbackSample> transient_contact = {
        {0.14f, 200.0f, false, false},
    };
    const auto empty_samples =
        RepeatedSamples(0.002f, 24.0f, false, true, 3);
    transient_contact.insert(
        transient_contact.end(), empty_samples.begin(), empty_samples.end());
    const auto transient = EvaluateGripperHolding(
        transient_contact, baseline, config);
    assert(transient.result == GripperHoldingResult::EMPTY);
    assert(transient.contact_count == 1);

    std::vector<GripperFeedbackSample> released_after_contact =
        RepeatedSamples(0.14f, 200.0f, false, false, 3);
    released_after_contact.insert(
        released_after_contact.end(),
        empty_samples.begin(), empty_samples.end());
    const auto released = EvaluateGripperHolding(
        released_after_contact, baseline, config);
    assert(released.result == GripperHoldingResult::EMPTY);
    assert(released.contact_count == 3);
    assert(released.empty_count == 3);

    const auto high_load_closed = EvaluateGripperHolding(
        RepeatedSamples(0.002f, 200.0f, false, true, 3),
        baseline, config);
    assert(
        high_load_closed.result ==
        GripperHoldingResult::INCONCLUSIVE);

    GripperBaseline cup_baseline;
    cup_baseline.valid = true;
    cup_baseline.sample_count = 10;
    cup_baseline.position_median = 0.0118302f;
    cup_baseline.position_mad = 0.0f;
    cup_baseline.load_median = 80.0f;
    cup_baseline.load_mad = 0.0f;
    GripperHoldingConfig after_lift_config = config;
    after_lift_config.opening_margin = 0.020f;
    const auto held_cup_after_lift = EvaluateGripperHolding(
        RepeatedSamples(
            0.0299235f, 184.0f, false, true, 3),
        cup_baseline, after_lift_config);
    assert(
        held_cup_after_lift.result ==
        GripperHoldingResult::HOLDING);
    assert(held_cup_after_lift.opening_count == 0);
    assert(held_cup_after_lift.load_count == 3);
    assert(held_cup_after_lift.contact_count == 3);

    const auto high_load_with_only_noise = EvaluateGripperHolding(
        RepeatedSamples(
            0.0158302f, 184.0f, false, true, 3),
        cup_baseline, after_lift_config);
    assert(
        high_load_with_only_noise.result ==
        GripperHoldingResult::INCONCLUSIVE);

    const std::vector<GripperFeedbackSample> noisy_baseline_samples = {
        {0.001f, 80.0f, false, true},
        {0.002f, 90.0f, false, true},
        {0.003f, 100.0f, false, true},
        {0.002f, 90.0f, false, true},
        {0.001f, 85.0f, false, true},
    };
    const auto noisy_baseline =
        EstimateGripperBaseline(noisy_baseline_samples);
    const auto adaptive_load = EvaluateGripperHolding(
        RepeatedSamples(0.14f, 120.0f, false, false, 3),
        noisy_baseline, config);
    assert(adaptive_load.effective_load_threshold > 100.0f);
    assert(
        adaptive_load.result ==
        GripperHoldingResult::INCONCLUSIVE);

    return 0;
}
