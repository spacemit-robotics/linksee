/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_state.h
 * @brief Shared state definitions for the perceptive grasp pipeline.
 */

#ifndef PIPELINE_STATE_H
#define PIPELINE_STATE_H

namespace perceptive_grasp {

/** Execution states of one perceptive grasp task. */
enum class PipelineState {
    IDLE,
    OBSERVING,
    DETECTING,
    PLANNING,
    BASE_ALIGNING,
    APPROACHING,
    GRASPING,
    LIFTING,
    PLACING,
    HOMING,
    RECOVERING,
    DONE,
    ERROR,
};

/** Return the stable log name for a pipeline state. */
const char* PipelineStateName(PipelineState state);

/** Return whether the state terminates one task. */
bool IsTerminalPipelineState(PipelineState state);

/** Return whether the state contributes a timed task stage. */
bool IsPipelineTaskStage(PipelineState state);

}  // namespace perceptive_grasp

#endif  // PIPELINE_STATE_H
