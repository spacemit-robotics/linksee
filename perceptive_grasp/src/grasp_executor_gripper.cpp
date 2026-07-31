/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_executor_gripper.cpp
 * @brief Gripper control, baseline calibration, and holding verification
 */

#include "grasp_executor.h"

extern "C" {
#include "grasp.h"
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace perceptive_grasp {

namespace {

const char* GraspStateName(grasp_state_t state) {
    switch (state) {
        case GRASP_STATE_IDLE: return "IDLE";
        case GRASP_STATE_MOVING: return "MOVING";
        case GRASP_STATE_HOLDING: return "HOLDING";
        case GRASP_STATE_EMPTY: return "EMPTY";
        case GRASP_STATE_ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

}  // namespace

void GraspExecutor::CaptureEmptyClosedPosition() {
    if (gripper_baseline_.valid) return;

    const int sample_count = std::max(
        6, config_.timing.grasp_check_count);
    const int interval_ms = std::max(
        20, config_.timing.grasp_check_interval_ms);
    std::vector<GripperFeedbackSample> samples;
    samples.reserve(static_cast<size_t>(sample_count));
    for (int index = 0; index < sample_count; ++index) {
        const float interval_s =
            static_cast<float>(interval_ms) / 1000.0f;
        grasp_tick(gripper_, interval_s);
        const grasp_state_t state = grasp_get_state(gripper_);
        float position = NAN;
        float load = NAN;
        if (grasp_get_feedback(
                gripper_, &position, &load) == GRASP_OK &&
            std::isfinite(position) && std::isfinite(load)) {
            if (state == GRASP_STATE_HOLDING ||
                load >= config_.gripper_hold_load_threshold) {
                std::cout << "[GraspExecutor] empty gripper baseline "
                            "skipped: feedback indicates a possible held "
                            "object" << std::endl;
                return;
            }
            samples.push_back(
                GripperFeedbackSample{
                    position, load, false,
                    state == GRASP_STATE_EMPTY ||
                        state == GRASP_STATE_IDLE});
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(interval_ms));
    }

    gripper_baseline_ = EstimateGripperBaseline(samples);
    if (!gripper_baseline_.valid) {
        std::cout << "[GraspExecutor] empty gripper baseline unavailable: "
                    "insufficient valid samples" << std::endl;
        return;
    }
    std::cout << "[GraspExecutor] empty gripper baseline: samples="
            << gripper_baseline_.sample_count
            << ", position_median="
            << gripper_baseline_.position_median
            << ", position_mad=" << gripper_baseline_.position_mad
            << ", load_median=" << gripper_baseline_.load_median
            << ", load_mad=" << gripper_baseline_.load_mad
            << std::endl;
}

GraspResult GraspExecutor::OpenGripperForGrasp(float minimum_opening) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.pre_grasp_settle_ms));
    float opening = config_.gripper_open;
    if (std::isfinite(minimum_opening)) {
        opening = std::max(
            opening, std::clamp(minimum_opening, 0.0f, 1.0f));
    }
    std::cout << "[GraspExecutor] opening gripper: configured="
            << config_.gripper_open << " requested=" << minimum_opening
            << " command=" << opening << std::endl;
    if (grasp_set_position(gripper_, opening) != GRASP_OK) {
        RecordResult(
            GraspResult::MOVE_FAILED, "open_gripper_for_grasp",
            "failed to set grasp opening");
        return GraspResult::MOVE_FAILED;
    }
    if (!WaitGripperOpening(opening)) {
        RecordResult(
            GraspResult::TIMEOUT, "open_gripper_for_grasp",
            "gripper did not reach requested opening");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "open_gripper_for_grasp");
    return GraspResult::SUCCESS;
}

bool GraspExecutor::WaitGripperOpening(float target_position) {
    constexpr int kPollIntervalMs = 50;
    constexpr float kPositionTolerance = 0.03f;
    const int timeout_ms = std::max(
        config_.gripper_timeout_ms,
        config_.timing.gripper_open_wait_ms);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    float position = NAN;
    float load = NAN;
    while (std::chrono::steady_clock::now() < deadline) {
        grasp_tick(
            gripper_, static_cast<float>(kPollIntervalMs) / 1000.0f);
        const grasp_state_t state = grasp_get_state(gripper_);
        if (state == GRASP_STATE_ERROR) return false;
        if (grasp_get_feedback(gripper_, &position, &load) == GRASP_OK &&
            std::isfinite(position) &&
            position >= target_position - kPositionTolerance) {
            std::cout << "[GraspExecutor] gripper opening reached: target="
                    << target_position << " position=" << position
                    << " load=" << load << std::endl;
            return true;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kPollIntervalMs));
    }

    std::cerr << "[GraspExecutor] gripper opening timeout: target="
            << target_position << " position=" << position
            << " load=" << load << std::endl;
    return false;
}

GraspResult GraspExecutor::CloseGripperAndCheck() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.grasp_settle_ms));
    if (grasp_execute(
            gripper_, GRASP_CMD_GRAB,
            config_.gripper_effort) != GRASP_OK) {
        RecordResult(
            GraspResult::MOVE_FAILED, "close_gripper_and_check",
            "failed to start gripper close");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.gripper_close_wait_ms));

    return CheckGripperHolding("after_close", false);
}

GraspResult GraspExecutor::CheckGripperHolding(
    const char* phase, bool after_lift) {
    grasp_state_t state = GRASP_STATE_ERROR;
    int performed_checks = 0;
    std::vector<GripperFeedbackSample> samples;

    const int required_holding =
        std::max(2, config_.timing.grasp_check_count / 3);
    const int maximum_checks = std::max(
        config_.timing.grasp_check_count,
        config_.gripper_timeout_ms /
            std::max(1, config_.timing.grasp_check_interval_ms));
    constexpr float kAfterLiftOpeningHysteresis = 0.010f;
    const float position_margin = after_lift
        ? std::max(
            0.0f,
            config_.gripper_empty_position_margin -
                kAfterLiftOpeningHysteresis)
        : config_.gripper_empty_position_margin;
    GripperHoldingConfig holding_config;
    holding_config.minimum_load_threshold =
        config_.gripper_hold_load_threshold;
    holding_config.opening_margin = position_margin;
    holding_config.required_consecutive_samples = required_holding;
    GripperHoldingEvidence evidence;
    for (int index = 0; index < maximum_checks; ++index) {
        const float interval_s =
            static_cast<float>(config_.timing.grasp_check_interval_ms) /
            1000.0f;
        grasp_tick(gripper_, interval_s);
        performed_checks++;
        state = grasp_get_state(gripper_);

        float current_position = NAN;
        float current_load = NAN;
        grasp_get_feedback(
            gripper_, &current_position, &current_load);
        samples.push_back(
            GripperFeedbackSample{
                current_position,
                current_load,
                state == GRASP_STATE_HOLDING,
                state == GRASP_STATE_EMPTY ||
                    state == GRASP_STATE_IDLE});
        evidence = EvaluateGripperHolding(
            samples, gripper_baseline_, holding_config);

        std::this_thread::sleep_for(std::chrono::milliseconds(
            config_.timing.grasp_check_interval_ms));
        if (evidence.result == GripperHoldingResult::HOLDING) {
            break;
        }
        if (performed_checks >= config_.timing.grasp_check_count &&
            evidence.result == GripperHoldingResult::EMPTY) {
            break;
        }
    }

    diagnostics_.gripper_check.phase = phase;
    diagnostics_.gripper_check.state = GraspStateName(state);
    diagnostics_.gripper_check.decision =
        GripperHoldingResultName(evidence.result);
    diagnostics_.gripper_check.holding_count =
        evidence.state_holding_count;
    diagnostics_.gripper_check.load_holding_count =
        evidence.load_count;
    diagnostics_.gripper_check.opening_count =
        evidence.opening_count;
    diagnostics_.gripper_check.contact_count =
        evidence.contact_count;
    diagnostics_.gripper_check.empty_count =
        evidence.empty_count;
    diagnostics_.gripper_check.check_count = performed_checks;
    diagnostics_.gripper_check.baseline_sample_count =
        static_cast<int>(gripper_baseline_.sample_count);
    diagnostics_.gripper_check.load_threshold =
        evidence.effective_load_threshold;
    diagnostics_.gripper_check.empty_closed_position =
        gripper_baseline_.position_median;
    diagnostics_.gripper_check.empty_closed_position_mad =
        gripper_baseline_.position_mad;
    diagnostics_.gripper_check.empty_closed_load =
        gripper_baseline_.load_median;
    diagnostics_.gripper_check.empty_closed_load_mad =
        gripper_baseline_.load_mad;
    diagnostics_.gripper_check.min_object_position =
        evidence.minimum_object_position;
    diagnostics_.gripper_check.position = evidence.position;
    diagnostics_.gripper_check.load = evidence.load;

    std::cout << "[GraspExecutor] grasp check: phase=" << phase
            << ", state=" << GraspStateName(state)
            << ", decision="
            << GripperHoldingResultName(evidence.result)
            << ", state_holding=" << evidence.state_holding_count << "/"
            << performed_checks
            << ", opening=" << evidence.opening_count << "/"
            << performed_checks
            << ", load=" << evidence.load_count << "/"
            << performed_checks
            << ", contact=" << evidence.contact_count << "/"
            << performed_checks
            << ", empty=" << evidence.empty_count << "/"
            << performed_checks
            << ", effective_load_threshold="
            << evidence.effective_load_threshold
            << ", min_object_position="
            << evidence.minimum_object_position
            << ", position=" << evidence.position
            << ", load=" << evidence.load << std::endl;

    if (evidence.result == GripperHoldingResult::HOLDING) {
        const char* action = after_lift
            ? "verify_grasp_after_lift"
            : "close_gripper_and_check";
        RecordResult(
            GraspResult::SUCCESS, action,
            "baseline-relative opening and sustained contact confirmed");
        return GraspResult::SUCCESS;
    }
    if (evidence.result == GripperHoldingResult::EMPTY) {
        std::cout << "[GraspExecutor] Grasp empty - nothing grabbed"
                << std::endl;
        RecordResult(
            GraspResult::EMPTY,
            after_lift
                ? "verify_grasp_after_lift"
                : "close_gripper_and_check",
            "gripper closed without object");
        return GraspResult::EMPTY;
    }
    if (after_lift) {
        RecordResult(
            GraspResult::TIMEOUT, "verify_grasp_after_lift",
            "holding evidence was inconclusive after lift; preserving "
            "possible-object state for safe recovery");
        return GraspResult::TIMEOUT;
    }
    if (state == GRASP_STATE_MOVING) {
        std::cerr << "[GraspExecutor] Gripper still moving after close check"
                << std::endl;
        RecordResult(
            GraspResult::TIMEOUT, "close_gripper_and_check",
            "gripper still moving after close check");
        return GraspResult::TIMEOUT;
    }
    std::cerr << "[GraspExecutor] Gripper error during close check"
            << std::endl;
    RecordResult(
        GraspResult::MOVE_FAILED, "close_gripper_and_check",
        "gripper error during close check");
    return GraspResult::MOVE_FAILED;
}

GraspResult GraspExecutor::ReleaseObject() {
    if (grasp_set_position(
            gripper_, config_.place_release_open) != GRASP_OK) {
        RecordResult(
            GraspResult::MOVE_FAILED, "release_object",
            "failed to set release opening");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.release_wait_ms));
    gripper_baseline_ = GripperBaseline{};
    RecordResult(GraspResult::SUCCESS, "release_object");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CloseGripper() {
    if (grasp_set_position(gripper_, 0.0f) != GRASP_OK) {
        RecordResult(
            GraspResult::MOVE_FAILED, "close_gripper",
            "failed to set closed position");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.home_gripper_close_wait_ms));
    RecordResult(GraspResult::SUCCESS, "close_gripper");
    return GraspResult::SUCCESS;
}

}  // namespace perceptive_grasp
