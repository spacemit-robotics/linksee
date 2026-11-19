#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for the structured stdout filter launcher."""

import importlib.util
import inspect
from pathlib import Path
import shutil
import subprocess
import sys
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
            "[Loop] START iteration=4 target=banana\n",
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
            "[Pipeline] IDLE | Home position reached; exiting\n"))
        self.assertTrue(MODULE.should_keep_line(
            "[Pipeline] Failed to init stereo camera backend: unavailable\n"))
        self.assertTrue(MODULE.should_keep_line(
            "[StereoCamera] camera.type=remote_mujoco requested, but disabled\n"))
        self.assertTrue(MODULE.should_keep_line(
            "[MockDetector] Running in dummy mode (no detection)\n"))
        self.assertTrue(MODULE.should_keep_line(
            "[Main] Graceful shutdown requested\n"))
        self.assertTrue(MODULE.should_keep_line(
            "Usage: ./perceptive_grasp_core [options]\n"))
        self.assertTrue(MODULE.should_keep_line(
            "Error loading config: unsupported camera.type: remote_mujoco\n"))
        self.assertTrue(MODULE.should_keep_line(
            "Failed to start ./perceptive_grasp_core: permission denied\n"))
        self.assertTrue(MODULE.should_keep_line(
            "Mode: torque untouched (arm stays as found)\n"))
        self.assertTrue(MODULE.should_keep_line(
            "Mode: plan-only (no motion commands)\n"))

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

    def test_simulation_server_uses_sibling_binary(self):
        with mock.patch.object(
                MODULE, "_default_simulation_server_binary",
                return_value="/tmp/mujoco_grasp_sim_server"):
            command = MODULE._parse_simulation_server_command([
                "--serve-simulation", "--config", "simulation.yaml",
                "--viewer",
            ])
        self.assertEqual(command, [
            "/tmp/mujoco_grasp_sim_server",
            "--config", "simulation.yaml", "--viewer",
        ])

    def test_normal_pipeline_does_not_select_simulation_server(self):
        self.assertIsNone(MODULE._parse_simulation_server_command([
            "--config", "hardware.yaml", "--target", "cup",
        ]))

    def test_voice_control_uses_sibling_bridge_and_same_launcher(self):
        with mock.patch.object(
                MODULE, "_default_voice_bridge",
                return_value="/tmp/local_voice_bridge.py"), \
             mock.patch.object(
                 MODULE, "_current_launcher",
                 return_value="/tmp/perceptive_grasp"):
            command = MODULE._parse_voice_control_command([
                "--voice-control",
                "--config", "config/remote.yaml",
                "--remote-host", "10.0.91.182",
            ])
        self.assertEqual(command, [
            MODULE.sys.executable,
            "/tmp/local_voice_bridge.py",
            "--binary", "/tmp/perceptive_grasp",
            "--config", "config/remote.yaml",
            "--remote-host", "10.0.91.182",
        ])

    def test_voice_control_is_not_selected_for_normal_pipeline(self):
        self.assertIsNone(MODULE._parse_voice_control_command([
            "--config", "config/hardware.yaml",
        ]))

    def test_installed_launcher_finds_voice_bridge_from_any_directory(self):
        with tempfile.TemporaryDirectory(prefix="grasp install ") as directory:
            root = Path(directory)
            launcher = root / "prefix/bin/perceptive_grasp"
            bridge = root / "prefix/share/perceptive_grasp/scripts/local_voice_bridge.py"
            launcher.parent.mkdir(parents=True)
            bridge.parent.mkdir(parents=True)
            shutil.copyfile(SCRIPT, launcher)
            bridge.write_text("print('installed voice bridge')\n", encoding="utf-8")
            # A stale script in the caller's directory must not replace the
            # bridge shipped with this installation.
            stale = root / "scripts/local_voice_bridge.py"
            stale.parent.mkdir()
            stale.write_text("print('stale voice bridge')\n", encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(launcher), "--voice-control", "--help"],
                cwd=root, capture_output=True, text=True, timeout=10, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), "installed voice bridge")

    def test_source_launcher_preserves_sibling_voice_bridge(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            launcher = root / "scripts/run_perceptive_grasp.py"
            launcher.parent.mkdir()
            shutil.copyfile(SCRIPT, launcher)
            (launcher.parent / "local_voice_bridge.py").write_text(
                "print('source voice bridge')\n", encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(launcher), "--voice-control", "--help"],
                cwd=root, capture_output=True, text=True, timeout=10, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), "source voice bridge")

    def test_config_validation_result_reaches_launcher_stdout(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "validation_output.py"
            fixture.write_text(
                "print('[SDK] internal setup')\nprint('[Config] valid')\n",
                encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(SCRIPT), "--binary", sys.executable, str(fixture)],
                cwd=directory, capture_output=True, text=True, timeout=10, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), "[Config] valid")

    def test_normal_mode_filters_stdout_and_stderr_together(self):
        source = inspect.getsource(MODULE.main)
        self.assertIn(
            "stderr=None if debug_mode else subprocess.STDOUT",
            source,
        )

    def test_sigint_is_not_forwarded_twice(self):
        source = inspect.getsource(MODULE.main)
        self.assertIn("signum != signal.SIGINT", source)

    def test_debug_mode_uses_managed_child_process(self):
        source = inspect.getsource(MODULE.main)
        self.assertNotIn("subprocess.call", source)
        self.assertIn("if debug_mode:\n            return process.wait()", source)


if __name__ == "__main__":
    unittest.main()
