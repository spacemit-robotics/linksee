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
        cls.source = PIPELINE_CPP.read_text(encoding="utf-8")

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
        self.assertNotIn("executor_->EmergencyStop()", cancel_body)

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
        self.assertIn("return_to_observe_pending_", terminal_body)
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

    def test_successful_place_returns_directly_to_observe(self):
        body = _function_body(self.source, "void GraspPipeline::HandleHoming")
        self.assertIn("move_to_observe_after_place", body)
        self.assertIn("executor_->MoveToObserve()", body)
        self.assertNotIn("executor_->MoveToHome()", body)

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


if __name__ == "__main__":
    unittest.main()
