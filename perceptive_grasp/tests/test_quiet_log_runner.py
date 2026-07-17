#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for the structured stdout filter launcher."""

import importlib.util
import inspect
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_perceptive_grasp.py"
SPEC = importlib.util.spec_from_file_location("run_perceptive_grasp", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class QuietLogRunnerTest(unittest.TestCase):
    def test_keeps_structured_pipeline_lines(self):
        lines = [
            "[Init] START pipeline\n",
            "[Stage 2] START DETECTING\n",
            "[Action] END stage=GRASPING result=SUCCESS\n",
            "[Timing] stage=DETECTING elapsed_ms=145\n",
            "========== PIPELINE SUMMARY ==========\n",
            "result=SUCCESS target=carrot task_ms=100\n",
            "  [01] OBSERVING elapsed_ms=50 result=SUCCESS\n",
            "message=Task completed!\n",
            "VOICE_STATUS\tstate=DONE\n",
        ]
        self.assertTrue(all(MODULE.should_keep_line(line) for line in lines))

    def test_hides_module_debug_lines(self):
        lines = [
            "[OpenCL] cache input dmabuf fd=28\n",
            "[Feetech] Factory: motor_id=1\n",
            "[KIN-Pinocchio] URDF loaded\n",
            "[GraspExecutor] IK+yaw joints(rad): []\n",
            "[CHASSIS-UART-DIFF] RX thread started\n",
        ]
        self.assertFalse(any(MODULE.should_keep_line(line) for line in lines))

    def test_hides_intermediate_detection_misses(self):
        self.assertFalse(MODULE.should_keep_line(
            "[Timing] stage=DETECTING elapsed_ms=149 result=NOT_FOUND\n"))
        self.assertTrue(MODULE.should_keep_line(
            "[Timing] stage=DETECTING elapsed_ms=145 result=FOUND\n"))

    def test_preserves_step_prompts_and_ready_state(self):
        self.assertTrue(MODULE.should_keep_line("[Step] continue?"))
        self.assertTrue(MODULE.should_keep_line("[Pipeline] IDLE | Ready\n"))
        self.assertTrue(MODULE.should_keep_line(
            "Usage: ./perceptive_grasp_core [options]\n"))

    def test_extracts_status_event_from_interleaved_module_log(self):
        line = (
            b"[CHASSIS-UART-DIFF] RX thread started"
            b"VOICE_STATUS\tstate=IDLE;message=Ready\n"
        )
        with tempfile.TemporaryFile() as output:
            with mock.patch.object(MODULE.sys.stdout, "fileno",
                                   return_value=output.fileno()):
                MODULE._write_if_kept(line)
            output.seek(0)
            self.assertEqual(
                output.read(),
                b"VOICE_STATUS\tstate=IDLE;message=Ready\n",
            )

    def test_debug_mode_is_removed_before_starting_core(self):
        command, step_mode, debug_mode = MODULE._parse_command([
            "--binary", "/tmp/perceptive_grasp_core",
            "--debug", "--config", "config.yaml",
        ])
        self.assertEqual(command, [
            "/tmp/perceptive_grasp_core", "--config", "config.yaml",
        ])
        self.assertFalse(step_mode)
        self.assertTrue(debug_mode)

    def test_help_does_not_enable_debug_mode(self):
        _command, _step_mode, debug_mode = MODULE._parse_command([
            "--binary", "/tmp/perceptive_grasp_core", "--help",
        ])
        self.assertFalse(debug_mode)

    def test_normal_mode_filters_stdout_and_stderr_together(self):
        source = inspect.getsource(MODULE.main)
        self.assertIn("stderr=subprocess.STDOUT", source)


if __name__ == "__main__":
    unittest.main()
