/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_executor_gripper.cpp
 * @brief Gripper control, baseline calibration, and holding verification
 */

#include "grasp_executor.h"
#include "executor_device.h"
#include "manipulation/device.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace perceptive_grasp {

namespace {

enum class GripperState { IDLE, MOVING, HOLDING, EMPTY, ERROR };

const char* GraspStateName(GripperState state) {
    switch (state) {
        case GripperState::IDLE: return "IDLE";
        case GripperState::MOVING: return "MOVING";
        case GripperState::HOLDING: return "HOLDING";
        case GripperState::EMPTY: return "EMPTY";
        case GripperState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

}  // namespace

bool GraspExecutor::SetGripperPosition(float position) {
    if (!device_) return false;
    try {
        device_->FollowGripper(position, config_.gripper_command_duration_s);
        return true;
    } catch (const std::exception& error) {
        RecordResult(GraspResult::MOVE_FAILED, "gripper_command", error.what());
        return false;
    }
}

bool GraspExecutor::ReadGripperFeedback(GripperFeedbackSample& sample, bool& moving) {
    if (!device_) return false;
    try {
        const auto feedback = device_->ReadGripper();
        sample.position = static_cast<float>(feedback.positions[0]);
        // Preserve the established holding thresholds while using the public signed load unit.
        constexpr float kPerMille = 1000.0f;
        sample.load = static_cast<float>(std::abs(feedback.loads[0])) * kPerMille;
        moving = feedback.moving || feedback.command_active;
        sample.state_holding = !moving && feedback.holding == true;
        sample.state_empty = !moving && feedback.holding == false;
        return true;
    } catch (const std::exception& error) {
        RecordResult(GraspResult::MOVE_FAILED, "gripper_feedback", error.what());
        return false;
    }
}

void GraspExecutor::CaptureEmptyClosedPosition() {
    if (gripper_baseline_.valid) return;

    const int sample_count = std::max(
        6, config_.timing.grasp_check_count);
    const int interval_ms = std::max(
        20, config_.timing.grasp_check_interval_ms);
    std::vector<GripperFeedbackSample> samples;
    samples.reserve(static_cast<size_t>(sample_count));
    for (int index = 0; index < sample_count; ++index) {
        GripperFeedbackSample sample;
        bool moving = false;
        if (!ReadGripperFeedback(sample, moving)) return;
        if (std::isfinite(sample.position) && std::isfinite(sample.load)) {
            if (sample.state_holding || sample.load >= config_.gripper_hold_load_threshold) {
                std::cout << "[GraspExecutor] empty gripper baseline "
                            "skipped: feedback indicates a possible held "
                            "object" << std::endl;
                return;
            }
            if (!moving) samples.push_back(sample);
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
    if (!SetGripperPosition(opening)) {
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
        GripperFeedbackSample sample;
        bool moving = false;
        if (!ReadGripperFeedback(sample, moving)) return false;
        position = sample.position;
        load = sample.load;
        if (!moving && std::isfinite(position) &&
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
    if (device_) device_->Fail();
    return false;
}

GraspResult GraspExecutor::CloseGripperAndCheck() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.grasp_settle_ms));
    if (!SetGripperPosition(0.0f)) {
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
    GripperState state = GripperState::ERROR;
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
        performed_checks++;
        GripperFeedbackSample sample;
        bool moving = false;
        if (!ReadGripperFeedback(sample, moving)) return GraspResult::MOVE_FAILED;
        state = moving ? GripperState::MOVING :
            sample.state_holding ? GripperState::HOLDING : GripperState::EMPTY;
        // Motion samples cannot establish holding or empty evidence.
        if (moving) sample.load = NAN;
        samples.push_back(sample);
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
    const bool driver_empty_at_calibrated_position =
        (state == GripperState::EMPTY || state == GripperState::IDLE) &&
        std::isfinite(evidence.position) &&
        std::isfinite(evidence.load) &&
        evidence.position <= evidence.minimum_object_position &&
        evidence.load < evidence.effective_load_threshold;
    const bool confirmed_empty =
        evidence.result == GripperHoldingResult::EMPTY ||
        driver_empty_at_calibrated_position;
    diagnostics_.gripper_check.decision = confirmed_empty
        ? "EMPTY"
        : GripperHoldingResultName(evidence.result);
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
            << diagnostics_.gripper_check.decision
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

    if (confirmed_empty) {
        std::cout << "[GraspExecutor] Grasp empty - nothing grabbed"
                << std::endl;
        RecordResult(
            GraspResult::EMPTY,
            after_lift
                ? "verify_grasp_after_lift"
                : "close_gripper_and_check",
            "gripper reached the calibrated empty position");
        return GraspResult::EMPTY;
    }
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
    const char* action = after_lift
        ? "verify_grasp_after_lift"
        : "close_gripper_and_check";
    std::cerr << "[GraspExecutor] Holding evidence remained inconclusive"
            << std::endl;
    RecordResult(
        GraspResult::TIMEOUT, action,
        "holding evidence was inconclusive; preserving possible-object state for safe recovery");
    return GraspResult::TIMEOUT;
}

GraspResult GraspExecutor::ReleaseObject() {
    if (!SetGripperPosition(config_.place_release_open)) {
        RecordResult(
            GraspResult::MOVE_FAILED, "release_object",
            "failed to set release opening");
        return GraspResult::MOVE_FAILED;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.release_wait_ms));
    if (!WaitGripperOpening(config_.place_release_open)) {
        RecordResult(GraspResult::TIMEOUT, "release_object", "release opening not confirmed");
        return GraspResult::TIMEOUT;
    }
    gripper_baseline_ = GripperBaseline{};
    RecordResult(GraspResult::SUCCESS, "release_object");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CloseGripper() {
    if (!SetGripperPosition(0.0f)) {
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
