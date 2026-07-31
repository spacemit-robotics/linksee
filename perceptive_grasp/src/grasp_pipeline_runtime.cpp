/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_pipeline_runtime.cpp
 * @brief Pipeline asynchronous action and state timing runtime
 */

#include "grasp_pipeline.h"

#include <cerrno>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <unistd.h>

namespace perceptive_grasp {

namespace {

void WriteStructuredLine(const std::string& text) {
    const std::string line = text + "\n";
    ssize_t written;
    do {
        written = write(STDOUT_FILENO, line.data(), line.size());
    } while (written < 0 && errno == EINTR);
    if (written < 0) {
        std::cout << line << std::flush;
    }
}

const char* GraspResultName(GraspResult result) {
    switch (result) {
        case GraspResult::SUCCESS: return "SUCCESS";
        case GraspResult::EMPTY: return "EMPTY";
        case GraspResult::IK_FAILED: return "IK_FAILED";
        case GraspResult::OUT_OF_RANGE: return "OUT_OF_RANGE";
        case GraspResult::MOVE_FAILED: return "MOVE_FAILED";
        case GraspResult::TIMEOUT: return "TIMEOUT";
    }
    return "UNKNOWN";
}

std::int64_t ProcessCpuMillis() {
    timespec value{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0) {
        return 0;
    }
    return static_cast<std::int64_t>(value.tv_sec) * 1000 +
        static_cast<std::int64_t>(value.tv_nsec) / 1000000;
}

}  // namespace

bool GraspPipeline::StartAction(PipelineState owner, const std::string& name,
                                std::function<GraspResult()> fn) {
    if (action_.active) {
        return false;
    }
    action_.active = true;
    action_.cancelling = false;
    action_.owner = owner;
    action_.name = name;
    action_.started_at = std::chrono::steady_clock::now();
    action_.started_cpu_ms = ProcessCpuMillis();
    action_.future = std::async(
        std::launch::async, [name, fn = std::move(fn)]() {
            try {
                return fn();
            } catch (const std::exception& e) {
                std::cerr << "[Pipeline] Async action exception (" << name
                        << "): " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "[Pipeline] Async action unknown exception ("
                        << name << ")" << std::endl;
            }
            return GraspResult::MOVE_FAILED;
        });
    std::ostringstream action_log;
    action_log << "[Action] START stage=" << PipelineStateName(owner)
            << " name=" << name;
    WriteStructuredLine(action_log.str());
    return true;
}

std::optional<GraspResult> GraspPipeline::PollAction(
    PipelineState owner, bool accept_any_owner) {
    if (!action_.active) return std::nullopt;
    if (!accept_any_owner && action_.owner != owner) return std::nullopt;

    if (action_.future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
        return std::nullopt;
    }

    const GraspResult result = action_.future.get();
    const auto action_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - action_.started_at)
            .count();
    std::ostringstream action_log;
    action_log << "[Action] END stage=" << PipelineStateName(action_.owner)
            << " name=" << action_.name
            << " elapsed_ms=" << action_ms;
    if (config_.performance_log_enabled) {
        action_log << " cpu_ms="
                << ProcessCpuMillis() - action_.started_cpu_ms;
    }
    action_log << " result=" << GraspResultName(result);
    WriteStructuredLine(action_log.str());
    action_.active = false;
    action_.cancelling = false;
    action_.name.clear();
    action_.owner = PipelineState::IDLE;
    return result;
}

void GraspPipeline::ClearAction() {
    if (!action_.active) return;
    if (action_.future.valid()) {
        action_.future.wait();
    }
    action_.active = false;
    action_.cancelling = false;
    action_.name.clear();
    action_.owner = PipelineState::IDLE;
}

void GraspPipeline::BeginTaskTiming() {
    failure_recovery_active_ = false;
    failure_recovery_succeeded_ = false;
    pending_failure_message_.clear();
    task_timing_.Begin();
}

void GraspPipeline::PrintTaskSummary(PipelineState terminal_state,
                                    const std::string& message) {
    const auto total_ms = task_timing_.ElapsedMs();
    const char* result = terminal_state == PipelineState::DONE
        ? "SUCCESS"
        : (terminal_state == PipelineState::ERROR ? "FAILED" : "CANCELLED");

    std::cout << "\n========== PIPELINE SUMMARY ==========" << std::endl;
    std::cout << "result=" << result
            << " target="
            << (target_label_.empty() ? "auto" : target_label_)
            << " initialization_ms=" << initialization_elapsed_ms_
            << " task_ms=" << total_ms
            << " end_to_end_ms=" << initialization_elapsed_ms_ + total_ms
            << " base_align_attempts=" << base_align_attempts_
            << std::endl;
    for (const auto& timing : task_timing_.Stages()) {
        std::cout << "  [" << std::setw(2) << std::setfill('0')
                << timing.sequence << std::setfill(' ') << "] "
                << PipelineStateName(timing.state)
                << " elapsed_ms=" << timing.elapsed_ms
                << " result=" << timing.result << std::endl;
    }
    if (!message.empty()) {
        std::cout << "message=" << message << std::endl;
    }
    std::cout << "======================================" << std::endl;

    task_timing_.Finish();
}

void GraspPipeline::SetState(PipelineState new_state,
                            const std::string& msg) {
    const PipelineState requested_state = new_state;
    const PipelineState previous_state = state_.load();
    std::string state_message = msg;
    const bool start_failure_recovery =
        requested_state == PipelineState::ERROR &&
        task_timing_.Active() && executor_ && !config_.plan_only &&
        !failure_recovery_active_;
    if (start_failure_recovery) {
        pending_failure_message_ =
            msg.empty() ? last_status_message_ : msg;
        failure_recovery_active_ = true;
        failure_recovery_succeeded_ = false;
        new_state = PipelineState::RECOVERING;
        state_message =
            config_.auto_loop && !object_may_be_held_.load()
            ? "Task failed; returning to observation position"
            : "Task failed; returning to home position";
    }

    if (!state_message.empty()) {
        last_status_message_ = state_message;
    }

    const PipelineTimingTransition timing_transition =
        task_timing_.Transition(
            previous_state, new_state, requested_state,
            failure_recovery_succeeded_);
    if (timing_transition.completed_stage.has_value()) {
        const PipelineStageTiming& timing =
            *timing_transition.completed_stage;
        std::ostringstream stage_log;
        stage_log << "[Stage " << timing.sequence << "] END   "
                << PipelineStateName(timing.state)
                << " elapsed_ms=" << timing.elapsed_ms
                << " result=" << timing.result;
        WriteStructuredLine(stage_log.str());
    }

    state_.store(new_state);

    if (timing_transition.started_stage.has_value()) {
        const PipelineStageStart& stage =
            *timing_transition.started_stage;
        std::ostringstream stage_log;
        stage_log << "\n[Stage " << stage.sequence << "] START "
                << PipelineStateName(stage.state);
        if (!state_message.empty()) {
            stage_log << " | " << state_message;
        }
        WriteStructuredLine(stage_log.str());
    } else if (!task_timing_.Active() && !state_message.empty()) {
        std::ostringstream pipeline_log;
        pipeline_log << "[Pipeline] " << PipelineStateName(new_state)
                    << " | " << state_message;
        WriteStructuredLine(pipeline_log.str());
    }

    if (IsTerminalPipelineState(new_state)) {
        SaveTaskResultDebug(
            new_state,
            state_message.empty() ? last_status_message_ : state_message);
    }

    if (task_timing_.Active() &&
        (IsTerminalPipelineState(new_state) ||
        new_state == PipelineState::IDLE)) {
        PrintTaskSummary(
            new_state,
            state_message.empty() ? last_status_message_ : state_message);
    }

    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
        callback_(new_state, state_message);
    }
}

}  // namespace perceptive_grasp
