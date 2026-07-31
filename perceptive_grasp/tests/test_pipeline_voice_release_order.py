#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for voice stop feedback without torque release."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"
PIPELINE_EXECUTION_CPP = ROOT / "src" / "grasp_pipeline_execution.cpp"
PIPELINE_RUNTIME_CPP = ROOT / "src" / "grasp_pipeline_runtime.cpp"


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError(f"unterminated function: {signature}")


class PipelineVoiceReleaseOrderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            PIPELINE_CPP.read_text(encoding="utf-8")
            + PIPELINE_EXECUTION_CPP.read_text(encoding="utf-8")
            + PIPELINE_RUNTIME_CPP.read_text(encoding="utf-8")
        )

    def test_voice_cancel_queues_release_without_immediate_torque_drop(self):
        body = _function_body(
            self.source,
            "bool GraspPipeline::TriggerVoiceCommand",
        )
        cancel_branch = body[:body.index("auto target = parser.ParseTarget")]
        self.assertIn("cancel_requested_.store(true)", cancel_branch)
        self.assertNotIn("EmergencyStop()", cancel_branch)

    def test_cancel_does_not_release_torque(self):
        body = _function_body(self.source, "void GraspPipeline::SpinOnce")
        cancel_match = re.search(
            r"if \(cancel_requested_\.exchange\(false\)\) "
            r"\{(?P<body>.*?)\n    \}",
            body,
            re.DOTALL,
        )
        self.assertIsNotNone(cancel_match, "missing cancel branch")
        cancel_body = cancel_match.group("body")
        self.assertIn("Cancelling; keeping observe pose", cancel_body)
        self.assertIn("return_to_observe_pending_", cancel_body)
        self.assertIn("return_to_home_pending_", cancel_body)
        self.assertNotIn("executor_->EmergencyStop()", cancel_body)

    def test_cancel_while_holding_places_before_returning_home(self):
        body = _function_body(self.source, "void GraspPipeline::SpinOnce")
        cancel_match = re.search(
            r"if \(cancel_requested_\.exchange\(false\)\) "
            r"\{(?P<body>.*?)\n    \}",
            body,
            re.DOTALL,
        )
        self.assertIsNotNone(cancel_match, "missing cancel branch")
        cancel_body = cancel_match.group("body")
        self.assertIn("object_may_be_held_.load()", cancel_body)
        self.assertIn("place_possible_object_pending_ = true", cancel_body)
        self.assertIn("placing it before returning home", cancel_body)

    def test_stop_does_not_release_torque(self):
        body = _function_body(self.source, "void GraspPipeline::Stop")
        stopped_index = body.find('"Stopped"')
        self.assertNotIn("executor_->EmergencyStop()", body)
        self.assertGreaterEqual(stopped_index, 0)

    def test_voice_error_does_not_release_torque(self):
        body = _function_body(self.source, "void GraspPipeline::SpinOnce")
        match = re.search(
            r"case PipelineState::ERROR:(?P<body>.*?)\n\s*break;",
            body,
            re.DOTALL,
        )
        self.assertIsNotNone(match, "missing ERROR state handler")
        terminal_body = match.group("body")
        idle_index = terminal_body.find("Voice: waiting for next command")
        self.assertNotIn("executor_->EmergencyStop()", terminal_body)
        self.assertNotIn("return_to_observe_pending_", terminal_body)
        self.assertGreaterEqual(idle_index, 0)

    def test_voice_success_does_not_schedule_extra_observe_return(self):
        body = _function_body(self.source, "void GraspPipeline::SpinOnce")
        match = re.search(
            r"case PipelineState::DONE:(?P<body>.*?)\n\s*break;",
            body,
            re.DOTALL,
        )
        self.assertIsNotNone(match, "missing DONE state handler")
        done_body = match.group("body")
        self.assertNotIn("return_to_observe_pending_", done_body)
        self.assertIn("Voice: waiting for next command", done_body)

    def test_cancelled_action_returns_to_observe(self):
        body = _function_body(self.source, "void GraspPipeline::HandleIdle")
        self.assertIn("return_to_observe_after_cancel", body)
        self.assertIn("executor_->MoveToObserve()", body)
        self.assertIn("Cancelled; returning to observe position", body)

    def test_successful_place_returns_home_or_observe_by_mode(self):
        body = _function_body(self.source, "void GraspPipeline::HandleHoming")
        self.assertIn("move_to_observe_after_place", body)
        self.assertIn("move_to_home_after_place", body)
        self.assertIn("config_.voice.enabled", body)
        self.assertIn("executor_->MoveToObserve()", body)
        self.assertIn("executor_->MoveToHome()", body)

    def test_home_voice_command_returns_home_then_exits(self):
        trigger_body = _function_body(
            self.source,
            "bool GraspPipeline::TriggerVoiceCommand",
        )
        run_body = _function_body(self.source, "void GraspPipeline::Run")
        idle_body = _function_body(self.source, "void GraspPipeline::HandleIdle")
        self.assertIn("parser.IsHomeCommand(command_text)", trigger_body)
        self.assertIn("PendingVoiceCommand::Type::HOME", trigger_body)
        self.assertIn("return_to_home_on_command", idle_body)
        self.assertIn("executor_->MoveToHome()", idle_body)
        self.assertIn("shutdown_requested_", idle_body)
        self.assertIn("Home position reached; exiting", idle_body)
        self.assertIn("shutdown_requested_", run_body)

    def test_graceful_shutdown_waits_for_action_then_returns_home(self):
        request_body = _function_body(
            self.source,
            "void GraspPipeline::RequestGracefulShutdown",
        )
        spin_body = _function_body(self.source, "void GraspPipeline::SpinOnce")
        idle_body = _function_body(self.source, "void GraspPipeline::HandleIdle")
        run_body = _function_body(self.source, "void GraspPipeline::Run")

        self.assertIn("graceful_shutdown_requested_.exchange(true)",
                      request_body)
        self.assertIn("cancel_requested_.store(true)", request_body)
        self.assertNotIn("executor_->EmergencyStop()", request_body)
        self.assertIn("action_.cancelling = true", spin_body)
        self.assertIn("return_to_home_pending_", spin_body)
        self.assertIn("place_possible_object_pending_", spin_body)
        self.assertIn("return_to_home_on_command", idle_body)
        self.assertIn("place_possible_object_after_cancel", idle_body)
        self.assertIn("executor_->MoveToHome()", idle_body)
        self.assertIn("external_shutdown_requested()", run_body)
        self.assertIn("shutdown_requested_", run_body)

    def test_plan_only_shutdown_never_sends_recovery_motion(self):
        request_body = _function_body(
            self.source,
            "void GraspPipeline::RequestGracefulShutdown",
        )
        spin_body = _function_body(self.source, "void GraspPipeline::SpinOnce")
        idle_body = _function_body(self.source, "void GraspPipeline::HandleIdle")

        self.assertIn("config_.plan_only", request_body)
        self.assertIn("plan-only mode without motion", request_body)
        self.assertIn("if (config_.plan_only)", spin_body)
        self.assertIn(
            "Plan-only shutdown complete; exiting without motion",
            spin_body,
        )
        self.assertIn("if (config_.plan_only)", idle_body)
        home_pending = idle_body.find("if (return_to_home_pending_)")
        plan_only_guard = idle_body.find("if (config_.plan_only)", home_pending)
        return_home_action = idle_body.find(
            "return_to_home_on_command", plan_only_guard)
        self.assertGreaterEqual(home_pending, 0)
        self.assertGreaterEqual(plan_only_guard, 0)
        self.assertGreater(return_home_action, plan_only_guard)

    def test_home_failure_ends_with_error(self):
        idle_body = _function_body(self.source, "void GraspPipeline::HandleIdle")
        self.assertIn("*result == GraspResult::SUCCESS", idle_body)
        self.assertIn("Home return failed during shutdown", idle_body)
        self.assertIn("SetState(\n                        PipelineState::ERROR",
                      idle_body)

    def test_task_failure_returns_to_safe_pose_before_terminal_error(self):
        set_state_body = _function_body(
            self.source,
            "void GraspPipeline::SetState",
        )
        recovery_body = _function_body(
            self.source,
            "void GraspPipeline::HandleRecovering",
        )
        self.assertIn("requested_state == PipelineState::ERROR", set_state_body)
        self.assertIn("new_state = PipelineState::RECOVERING", set_state_body)
        self.assertIn("!config_.plan_only", set_state_body)
        self.assertIn("failure_recovery_active_", set_state_body)
        self.assertIn("config_.auto_loop", recovery_body)
        self.assertIn("executor_->MoveToHome()", recovery_body)
        self.assertIn("executor_->MoveToSideObserve()", recovery_body)
        self.assertIn("executor_->MoveToObserve()", recovery_body)
        self.assertIn("return_home_after_failure", recovery_body)
        self.assertIn("return_observe_after_failure", recovery_body)
        self.assertIn(
            "observation_strategy_selected_ = false", recovery_body)
        self.assertIn("SetState(PipelineState::ERROR", recovery_body)
        self.assertNotIn("executor_->EmergencyStop()", recovery_body)

    def test_held_object_failure_places_then_returns_home(self):
        grasp_body = _function_body(
            self.source,
            "void GraspPipeline::HandleGrasping",
        )
        place_body = _function_body(
            self.source,
            "void GraspPipeline::HandlePlacing",
        )
        recovery_body = _function_body(
            self.source,
            "void GraspPipeline::HandleRecovering",
        )
        safe_place_body = _function_body(
            self.source,
            "GraspResult GraspPipeline::PlacePossibleObjectAndReturnHome",
        )
        spin_body = _function_body(self.source, "void GraspPipeline::SpinOnce")

        self.assertIn("object_may_be_held_.store(true)", grasp_body)
        self.assertIn("object_may_be_held_.store(false)", grasp_body)
        self.assertIn("object_may_be_held_.store(false)", place_body)
        self.assertIn("const bool carrying_object", recovery_body)
        self.assertIn(
            "config_.auto_loop && !carrying_object", recovery_body)
        self.assertIn("PlacePossibleObjectAndReturnHome()", recovery_body)
        place_index = safe_place_body.find("executor_->MoveToPlace()")
        release_index = safe_place_body.find("executor_->ReleaseObject()")
        empty_index = safe_place_body.find(
            "object_may_be_held_.store(false)")
        close_index = safe_place_body.find("executor_->CloseGripper()")
        home_index = safe_place_body.find("executor_->MoveToHome()")
        self.assertLess(place_index, release_index)
        self.assertLess(release_index, empty_index)
        self.assertLess(empty_index, close_index)
        self.assertLess(close_index, home_index)
        self.assertIn("shutdown_requested_.store(true)", recovery_body)
        self.assertIn("if (!object_may_be_held_.load())", spin_body)

    def test_step_input_eof_aborts_and_returns_home(self):
        confirm_body = _function_body(
            self.source,
            "bool GraspPipeline::WaitForConfirm",
        )
        eof_branch = confirm_body[:confirm_body.index("if (input.empty())")]
        self.assertIn("if (!std::getline(std::cin, input))", eof_branch)
        self.assertIn("RequestGracefulShutdown()", eof_branch)
        self.assertIn("return false", eof_branch)


if __name__ == "__main__":
    unittest.main()
