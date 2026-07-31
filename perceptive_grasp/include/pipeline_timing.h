/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_timing.h
 * @brief Task and stage timing for the perceptive grasp state machine.
 */

#ifndef PIPELINE_TIMING_H
#define PIPELINE_TIMING_H

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "pipeline_state.h"

namespace perceptive_grasp {

/** Completed state-machine stage recorded for the task summary. */
struct PipelineStageTiming {
    int sequence = 0;
    PipelineState state = PipelineState::IDLE;
    std::int64_t elapsed_ms = 0;
    std::string result;
};

/** Newly started state-machine stage. */
struct PipelineStageStart {
    int sequence = 0;
    PipelineState state = PipelineState::IDLE;
};

/** Timing events produced by one state transition. */
struct PipelineTimingTransition {
    std::optional<PipelineStageTiming> completed_stage;
    std::optional<PipelineStageStart> started_stage;
};

/**
 * @brief Record task and stage timing independently from hardware execution.
 *
 * Call Begin() once per task, Transition() whenever the pipeline state
 * changes, and Finish() after emitting the terminal task summary.
 */
class PipelineTiming {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    void Begin(TimePoint now = Clock::now());

    PipelineTimingTransition Transition(
        PipelineState previous_state,
        PipelineState new_state,
        PipelineState requested_state,
        bool recovery_succeeded,
        TimePoint now = Clock::now());

    std::int64_t ElapsedMs(TimePoint now = Clock::now()) const;
    void Finish();

    bool Active() const { return task_active_; }
    const std::vector<PipelineStageTiming>& Stages() const {
        return stages_;
    }

private:
    bool task_active_ = false;
    bool stage_active_ = false;
    int stage_sequence_ = 0;
    TimePoint task_started_at_{};
    TimePoint stage_started_at_{};
    std::vector<PipelineStageTiming> stages_;
};

}  // namespace perceptive_grasp

#endif  // PIPELINE_TIMING_H
