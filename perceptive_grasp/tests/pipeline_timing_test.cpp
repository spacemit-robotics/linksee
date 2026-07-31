/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_timing_test.cpp
 * @brief Regression tests for pipeline stage timing semantics.
 */

#include "pipeline_timing.h"

#include <chrono>
#include <iostream>

namespace {

using perceptive_grasp::PipelineState;
using perceptive_grasp::PipelineTiming;
using perceptive_grasp::PipelineTimingTransition;

PipelineTiming::TimePoint AtMilliseconds(int milliseconds) {
    return PipelineTiming::TimePoint{} +
        std::chrono::milliseconds(milliseconds);
}

bool CheckStage(
    const PipelineTimingTransition& transition,
    int sequence,
    PipelineState state,
    std::int64_t elapsed_ms,
    const char* result) {
    if (!transition.completed_stage.has_value()) return false;
    const auto& stage = *transition.completed_stage;
    return stage.sequence == sequence &&
        stage.state == state &&
        stage.elapsed_ms == elapsed_ms &&
        stage.result == result;
}

}  // namespace

int main() {
    PipelineTiming timing;
    timing.Begin(AtMilliseconds(100));

    PipelineTimingTransition transition = timing.Transition(
        PipelineState::IDLE,
        PipelineState::DETECTING,
        PipelineState::DETECTING,
        false,
        AtMilliseconds(110));
    if (!transition.started_stage.has_value() ||
        transition.started_stage->sequence != 1 ||
        transition.started_stage->state != PipelineState::DETECTING) {
        std::cerr << "detecting stage did not start" << std::endl;
        return 1;
    }

    transition = timing.Transition(
        PipelineState::DETECTING,
        PipelineState::PLANNING,
        PipelineState::PLANNING,
        false,
        AtMilliseconds(160));
    if (!CheckStage(
            transition, 1, PipelineState::DETECTING,
            50, "SUCCESS") ||
        !transition.started_stage.has_value() ||
        transition.started_stage->sequence != 2) {
        std::cerr << "normal stage transition changed" << std::endl;
        return 1;
    }

    transition = timing.Transition(
        PipelineState::PLANNING,
        PipelineState::RECOVERING,
        PipelineState::ERROR,
        false,
        AtMilliseconds(190));
    if (!CheckStage(
            transition, 2, PipelineState::PLANNING,
            30, "FAILED") ||
        !transition.started_stage.has_value() ||
        transition.started_stage->state != PipelineState::RECOVERING) {
        std::cerr << "failure recovery timing changed" << std::endl;
        return 1;
    }

    transition = timing.Transition(
        PipelineState::RECOVERING,
        PipelineState::ERROR,
        PipelineState::ERROR,
        true,
        AtMilliseconds(240));
    if (!CheckStage(
            transition, 3, PipelineState::RECOVERING,
            50, "SUCCESS") ||
        transition.started_stage.has_value()) {
        std::cerr << "successful recovery timing changed" << std::endl;
        return 1;
    }
    if (timing.ElapsedMs(AtMilliseconds(250)) != 150 ||
        timing.Stages().size() != 3) {
        std::cerr << "task summary timing changed" << std::endl;
        return 1;
    }

    timing.Finish();
    if (timing.Active()) {
        std::cerr << "finished task remains active" << std::endl;
        return 1;
    }

    timing.Begin(AtMilliseconds(300));
    timing.Transition(
        PipelineState::IDLE,
        PipelineState::OBSERVING,
        PipelineState::OBSERVING,
        false,
        AtMilliseconds(310));
    transition = timing.Transition(
        PipelineState::OBSERVING,
        PipelineState::IDLE,
        PipelineState::IDLE,
        false,
        AtMilliseconds(345));
    if (!CheckStage(
            transition, 1, PipelineState::OBSERVING,
            35, "CANCELLED")) {
        std::cerr << "cancelled stage timing changed" << std::endl;
        return 1;
    }

    std::cout << "pipeline_timing_test passed" << std::endl;
    return 0;
}
