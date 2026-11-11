#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Simulation launcher must keep hardware defaults and SDK installation untouched."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


APP = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "simulation_launcher", APP / "scripts/run_manipulation_simulation.py")
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


class SimulationLauncherTest(unittest.TestCase):
    def test_full_build_is_isolated(self):
        with tempfile.TemporaryDirectory(prefix="simulation launcher ") as temporary:
            with patch.object(launcher, "run") as run:
                launcher.main(["build", "--build-dir", temporary, "--jobs", "1"])
            commands = [[str(item) for item in call.args[0]] for call in run.call_args_list]
            self.assertEqual(len(commands), 5)
            self.assertIn(f"-DCMAKE_INSTALL_PREFIX={temporary}/prefix", commands[0])
            self.assertIn("-DMANIPULATION_ENABLE_NERO=OFF", commands[0])
            self.assertIn("-DMANIPULATION_ENABLE_SO101=OFF", commands[0])
            self.assertIn("-DENABLE_MANIPULATION_MUJOCO=ON", commands[3])
            self.assertIn("-DUSE_MOCK_EXECUTOR=ON", commands[3])
            self.assertNotIn("output/staging", " ".join(sum(commands, [])))

    def test_client_build_needs_no_engine(self):
        with patch.object(launcher, "run") as run:
            launcher.main(["build", "--client-only"])
        commands = [[str(item) for item in call.args[0]] for call in run.call_args_list]
        self.assertEqual(len(commands), 2)
        self.assertIn("-DENABLE_MUJOCO_EXECUTOR=OFF", commands[0])
        self.assertIn("-DENABLE_MANIPULATION_MUJOCO=OFF", commands[0])
        self.assertIn("-DUSE_MOCK_EXECUTOR=ON", commands[0])

    def test_grasp_uses_explicit_loopback_and_simulation_profile(self):
        with patch.object(Path, "is_file", return_value=True):
            with patch.object(launcher.os, "chdir"), patch.object(launcher.os, "execv") as execute:
                launcher.main(["grasp", "apple", "--client-only"])
        command = execute.call_args.args[1]
        self.assertIn("127.0.0.1", command)
        self.assertIn(str(APP / "config/grasp_pipeline_manipulation_remote.yaml"), command)
        self.assertTrue(command[0].endswith("/client/perceptive_grasp"))

    def test_server_does_not_bind_public_interfaces(self):
        with patch.object(Path, "is_file", return_value=True):
            with patch.object(launcher.os, "chdir"), patch.object(launcher.os, "execv") as execute:
                launcher.main(["serve", "--no-viewer"])
        command = execute.call_args.args[1]
        self.assertIn("127.0.0.1", command)
        self.assertNotIn("--viewer", command)
        self.assertNotIn("0.0.0.0", command)

    def test_invalid_numeric_settings_do_not_run(self):
        with patch.object(launcher, "run") as run:
            with self.assertRaises(ValueError):
                launcher.main(["build", "--jobs", "0"])
            with self.assertRaises(ValueError):
                launcher.main(["grasp", "apple", "--port", "0"])
            run.assert_not_called()

    def test_server_limits_software_render_workers_without_overriding_user(self):
        for supplied, expected in (({}, "2"), ({"LP_NUM_THREADS": "4"}, "4")):
            with self.subTest(environment=supplied):
                with patch.dict(launcher.os.environ, supplied, clear=True):
                    with patch.object(Path, "is_file", return_value=True):
                        with patch.object(launcher.os, "chdir"), patch.object(launcher.os, "execv"):
                            launcher.main(["serve"])
                    self.assertEqual(launcher.os.environ["LP_NUM_THREADS"], expected)

    def test_client_does_not_change_graphics_environment(self):
        with patch.dict(launcher.os.environ, {}, clear=True):
            with patch.object(Path, "is_file", return_value=True):
                with patch.object(launcher.os, "chdir"), patch.object(launcher.os, "execv"):
                    launcher.main(["grasp", "apple", "--client-only"])
            self.assertNotIn("LP_NUM_THREADS", launcher.os.environ)


if __name__ == "__main__":
    unittest.main()
