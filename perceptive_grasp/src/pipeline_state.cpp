/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_state.cpp
 * @brief Pipeline state names and classification helpers.
 */

#include "pipeline_state.h"

namespace perceptive_grasp {

const char* PipelineStateName(PipelineState state) {
    switch (state) {
        case PipelineState::IDLE: return "IDLE";
        case PipelineState::OBSERVING: return "OBSERVING";
        case PipelineState::DETECTING: return "DETECTING";
        case PipelineState::PLANNING: return "PLANNING";
        case PipelineState::BASE_ALIGNING: return "BASE_ALIGNING";
        case PipelineState::APPROACHING: return "APPROACHING";
        case PipelineState::GRASPING: return "GRASPING";
        case PipelineState::LIFTING: return "LIFTING";
        case PipelineState::PLACING: return "PLACING";
        case PipelineState::HOMING: return "HOMING";
        case PipelineState::RECOVERING: return "RECOVERING";
        case PipelineState::DONE: return "DONE";
        case PipelineState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

bool IsTerminalPipelineState(PipelineState state) {
    return state == PipelineState::DONE || state == PipelineState::ERROR;
}

bool IsPipelineTaskStage(PipelineState state) {
    return state != PipelineState::IDLE &&
        !IsTerminalPipelineState(state);
}

}  // namespace perceptive_grasp
