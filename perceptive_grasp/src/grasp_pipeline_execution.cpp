/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_pipeline_execution.cpp
 * @brief Pipeline arm execution and recovery state handlers
 */

#include "grasp_pipeline.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

namespace perceptive_grasp {

void GraspPipeline::HandleApproaching() {
    if (!action_.active) {
        const char* prompt = grasp_strategy_ == GraspStrategy::SIDE
            ? "即将移动到目标上方安全预抓取位 (safe_pre_grasp)"
            : "即将移动到预抓取位 (pre_grasp)";
        if (!WaitForConfirm(prompt)) return;
        StartAction(PipelineState::APPROACHING, "move_to_pre_grasp", [this]() {
            return executor_->MoveToPreGrasp(
                pre_grasp_pose_, grasp_yaw_rad_,
                grasp_strategy_ == GraspStrategy::TOP);
        });
        return;
    }

    auto result = PollAction(PipelineState::APPROACHING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        const char* debug_name = grasp_strategy_ == GraspStrategy::SIDE
            ? "safe_pre_grasp"
            : "pre_grasp";
        if (config_.step_mode &&
            (!FlushCameraAfterMotion("pre-grasp motion") ||
            !SaveStepCameraDebug(debug_name))) {
            SetState(PipelineState::ERROR,
                    "Failed to capture pre-grasp camera verification");
            return;
        }
        if (grasp_strategy_ == GraspStrategy::SIDE) {
            const float dx = grasp_pose_.x - pre_grasp_pose_.x;
            const float dy = grasp_pose_.y - pre_grasp_pose_.y;
            const float dz = grasp_pose_.z - pre_grasp_pose_.z;
            std::ostringstream message;
            message << "At safe pre-grasp above target; open gripper, "
                    << "descend, then advance distance="
                    << std::sqrt(dx * dx + dy * dy + dz * dz)
                    << "m dx=" << dx
                    << "m dy=" << dy
                    << "m dz=" << dz << "m";
            SetState(PipelineState::GRASPING, message.str());
        } else {
            SetState(
                PipelineState::GRASPING,
                "At pre-grasp, executing grasp...");
        }
    } else {
        if (RetryRecoverableMotion("Pre-grasp motion", *result)) return;
        SetState(PipelineState::ERROR,
                ResultMessage("Pre-grasp move failed", *result));
    }
}

void GraspPipeline::HandleGrasping() {
    if (!action_.active) {
        if (!WaitForConfirm(
                "即将张开夹爪、下降、水平进给并闭合抓取")) {
            return;
        }
        StartAction(
            PipelineState::GRASPING, "open_move_close_gripper", [this]() {
                std::cout << "[GraspExecutor] grasp phase=open_gripper"
                        << std::endl;
                auto result = executor_->OpenGripperForGrasp(grasp_opening_);
                if (result != GraspResult::SUCCESS) return result;
                object_may_be_held_.store(false);
                std::cout << "[GraspExecutor] grasp phase=descend_to_target"
                        << std::endl;
                result = executor_->MoveToGrasp(
                    grasp_pose_, grasp_yaw_rad_,
                    grasp_strategy_ == GraspStrategy::TOP);
                if (result != GraspResult::SUCCESS) return result;
                if (config_.step_mode &&
                    (!FlushCameraAfterMotion("grasp motion") ||
                    !SaveStepCameraDebug("grasp_pose"))) {
                    return GraspResult::MOVE_FAILED;
                }
                // Closing has started, so recovery must assume that an
                // object may be held until EMPTY or release is confirmed.
                object_may_be_held_.store(true);
                std::cout << "[GraspExecutor] grasp phase=close_gripper"
                        << std::endl;
                return executor_->CloseGripperAndCheck();
            });
        return;
    }

    auto result = PollAction(PipelineState::GRASPING);
    if (!result.has_value()) return;

    switch (*result) {
        case GraspResult::SUCCESS:
            object_may_be_held_.store(true);
            SetState(
                PipelineState::LIFTING,
                "Gripper closed; lifting to verify object holding...");
            break;

        case GraspResult::EMPTY:
            object_may_be_held_.store(false);
            retry_count_++;
            if (retry_count_ < config_.max_retries) {
                std::cout << "[Pipeline] Retry " << retry_count_ << "/"
                        << config_.max_retries << std::endl;
                stable_count_ = 0;
                // The target may have moved, and the arm can occlude the
                // forward stereo camera. Re-observe before retrying.
                SetState(PipelineState::OBSERVING,
                        "Retry: retracting arm for re-detection");
            } else {
                SetState(PipelineState::ERROR,
                        "Grasp empty; max retries reached");
            }
            break;

        case GraspResult::IK_FAILED:
        case GraspResult::OUT_OF_RANGE:
            if (RetryRecoverableMotion("Grasp motion", *result)) break;
            SetState(PipelineState::ERROR,
                    ResultMessage("Grasp move failed", *result));
            break;

        case GraspResult::TIMEOUT:
            SetState(PipelineState::ERROR,
                    ResultMessage("Grasp action timeout", *result));
            break;

        case GraspResult::MOVE_FAILED:
            SetState(PipelineState::ERROR,
                    ResultMessage("Grasp action failed", *result));
            break;

        default:
            SetState(PipelineState::ERROR,
                    ResultMessage("Grasp failed", *result));
            break;
    }
}

void GraspPipeline::HandleLifting() {
    if (!action_.active) {
        if (!WaitForConfirm("抓取成功，即将抬起到预抓取位")) return;
        StartAction(PipelineState::LIFTING, "lift_from_grasp", [this]() {
            return executor_->LiftFromGrasp(
                retreat_pose_, lift_pose_, grasp_yaw_rad_,
                grasp_strategy_ == GraspStrategy::TOP);
        });
        return;
    }

    auto result = PollAction(PipelineState::LIFTING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        SetState(PipelineState::PLACING,
                "Lift completed and object holding confirmed; placing...");
    } else if (*result == GraspResult::EMPTY) {
        object_may_be_held_.store(false);
        retry_count_++;
        if (retry_count_ < config_.max_retries) {
            stable_count_ = 0;
            SetState(PipelineState::OBSERVING,
                    "Object lost after lift; retracting for re-detection");
        } else {
            SetState(PipelineState::ERROR,
                    "Object not held after lift; max retries reached");
        }
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Lift failed", *result));
    }
}

void GraspPipeline::HandlePlacing() {
    if (!action_.active) {
        if (!WaitForConfirm("抓取成功，即将移动到放置位")) return;
        StartAction(PipelineState::PLACING, "place_and_release", [this]() {
            auto result = executor_->MoveToPlace();
            if (result == GraspResult::SUCCESS) {
                result = executor_->ReleaseObject();
                if (result == GraspResult::SUCCESS) {
                    object_may_be_held_.store(false);
                }
            }
            if (result == GraspResult::SUCCESS) {
                result = executor_->CloseGripper();
            }
            return result;
        });
        return;
    }

    auto result = PollAction(PipelineState::PLACING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        const bool return_to_observe =
            config_.voice.enabled || config_.auto_loop;
        const char* target_pose =
            return_to_observe ? "observe position" : "home position";
        SetState(PipelineState::HOMING,
                std::string("Object released, returning to ") +
                    target_pose + "...");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Place failed", *result));
    }
}

void GraspPipeline::HandleHoming() {
    const bool return_to_observe =
        config_.voice.enabled || config_.auto_loop;
    if (!action_.active) {
        if (!WaitForConfirm(return_to_observe ? "即将回到观察位"
                                            : "即将回到 Home 位")) {
            return;
        }
        const std::string action_name = return_to_observe
            ? "move_to_observe_after_place"
            : "move_to_home_after_place";
        StartAction(PipelineState::HOMING, action_name,
                    [this, return_to_observe]() {
            if (!return_to_observe) {
                return executor_->MoveToHome();
            }
            if (observation_strategy_selected_ &&
                grasp_strategy_ == GraspStrategy::SIDE) {
                return executor_->MoveToSideObserve();
            }
            return executor_->MoveToObserve();
        });
        return;
    }

    auto result = PollAction(PipelineState::HOMING);
    if (!result.has_value()) return;
    if (*result == GraspResult::SUCCESS) {
        SetState(PipelineState::DONE, "Task completed!");
    } else {
        SetState(PipelineState::ERROR,
                ResultMessage("Final move failed", *result));
    }
}

void GraspPipeline::HandleRecovering() {
    const bool carrying_object = object_may_be_held_.load();
    const bool return_to_observe = config_.auto_loop && !carrying_object;
    const bool use_side_observation =
        observation_strategy_selected_ &&
        grasp_strategy_ == GraspStrategy::SIDE;
    const char* recovery_target =
        carrying_object
        ? "place position and then home position"
        : (return_to_observe ? "observation position" : "home position");
    if (!action_.active) {
        const char* action_name = carrying_object
            ? "place_possible_object_after_failure"
            : (return_to_observe
                ? "return_observe_after_failure"
                : "return_home_after_failure");
        if (!StartAction(
                PipelineState::RECOVERING, action_name,
                [this, carrying_object, return_to_observe,
                use_side_observation]() {
                    if (carrying_object) {
                        return PlacePossibleObjectAndReturnHome();
                    }
                    if (!return_to_observe) {
                        return executor_->MoveToHome();
                    }
                    return use_side_observation
                        ? executor_->MoveToSideObserve()
                        : executor_->MoveToObserve();
                })) {
            failure_recovery_succeeded_ = false;
            const std::string message =
                pending_failure_message_ +
                "; failed to start recovery action for " +
                recovery_target;
            if (return_to_observe) {
                observation_strategy_selected_ = false;
            }
            if (object_may_be_held_.load()) {
                shutdown_requested_.store(true);
            }
            SetState(PipelineState::ERROR, message);
            failure_recovery_active_ = false;
            pending_failure_message_.clear();
        }
        return;
    }

    auto result = PollAction(PipelineState::RECOVERING);
    if (!result.has_value()) return;

    failure_recovery_succeeded_ = *result == GraspResult::SUCCESS;
    std::string message = pending_failure_message_;
    if (failure_recovery_succeeded_) {
        message += "; returned to ";
        message += recovery_target;
    } else {
        if (return_to_observe) {
            observation_strategy_selected_ = false;
        }
        message += "; " +
            ResultMessage(
                return_to_observe
                    ? "Observation recovery failed"
                    : "Home recovery failed",
                *result);
    }

    if (object_may_be_held_.load()) {
        message +=
            "; automatic restart disabled because the gripper may hold an "
            "object";
        shutdown_requested_.store(true);
    }
    SetState(PipelineState::ERROR, message);
    failure_recovery_active_ = false;
    failure_recovery_succeeded_ = false;
    pending_failure_message_.clear();
}

}  // namespace perceptive_grasp
