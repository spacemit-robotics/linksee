/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_timing.cpp
 * @brief Task and stage timing for the perceptive grasp state machine.
 */

#include "pipeline_timing.h"

namespace perceptive_grasp {

void PipelineTiming::Begin(TimePoint now) {
    task_active_ = true;
    stage_active_ = false;
    stage_sequence_ = 0;
    stages_.clear();
    task_started_at_ = now;
}

PipelineTimingTransition PipelineTiming::Transition(
    PipelineState previous_state,
    PipelineState new_state,
    PipelineState requested_state,
    bool recovery_succeeded,
    TimePoint now) {
    PipelineTimingTransition transition;
    if (!task_active_) return transition;

    if (stage_active_ &&
        IsPipelineTaskStage(previous_state) &&
        previous_state != new_state) {
        const bool recovery_completed =
            previous_state == PipelineState::RECOVERING &&
            requested_state == PipelineState::ERROR &&
            recovery_succeeded;
        const char* result = recovery_completed
            ? "SUCCESS"
            : (requested_state == PipelineState::ERROR
                ? "FAILED"
                : (new_state == PipelineState::IDLE
                    ? "CANCELLED"
                    : "SUCCESS"));
        PipelineStageTiming completed{
            stage_sequence_,
            previous_state,
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - stage_started_at_)
                .count(),
            result,
        };
        stages_.push_back(completed);
        transition.completed_stage = std::move(completed);
        stage_active_ = false;
    }

    if (IsPipelineTaskStage(new_state) &&
        previous_state != new_state) {
        ++stage_sequence_;
        stage_started_at_ = now;
        stage_active_ = true;
        transition.started_stage = PipelineStageStart{
            stage_sequence_, new_state};
    }
    return transition;
}

std::int64_t PipelineTiming::ElapsedMs(TimePoint now) const {
    if (!task_active_) return 0;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now - task_started_at_)
        .count();
}

void PipelineTiming::Finish() {
    task_active_ = false;
    stage_active_ = false;
}

}  // namespace perceptive_grasp
