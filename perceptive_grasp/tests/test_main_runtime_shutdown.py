#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for perceptive_grasp process shutdown paths."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = ROOT / "src" / "main.cpp"


class MainRuntimeShutdownTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAIN_CPP.read_text(encoding="utf-8")

    def test_voice_stdin_exit_forces_process_after_cleanup(self):
        run_index = self.source.index("g_pipeline->Run([]()")
        return_index = self.source.index("return exit_code;", run_index)
        body = self.source[run_index:return_index]
        self.assertIn("voice_stdin", body)
        self.assertIn("CleanupRuntime(false)", body)
        self.assertIn("std::_Exit(exit_code)", body)

    def test_cleanup_runtime_does_not_stop_pipeline_twice(self):
        match = re.search(
            r"static void CleanupRuntime\(bool destroy_pipeline = true\) "
            r"\{(?P<body>.*?)\n\}",
            self.source,
            re.DOTALL,
        )

        self.assertIsNotNone(match, "missing CleanupRuntime")
        self.assertNotIn("g_pipeline->Stop()", match.group("body"))

    def test_failed_task_returns_nonzero_exit_code(self):
        self.assertIn(
            "g_pipeline->GetState() == PipelineState::ERROR",
            self.source,
        )
        self.assertIn("std::_Exit(exit_code)", self.source)

    def test_first_signal_requests_graceful_shutdown(self):
        match = re.search(
            r"static void SignalHandler\(int sig\) \{(?P<body>.*?)\n\}",
            self.source,
            re.DOTALL,
        )

        self.assertIsNotNone(match, "missing SignalHandler")
        body = match.group("body")
        self.assertNotIn("CleanupRuntime()", body)
        self.assertIn("g_shutdown_signal = sig", body)
        self.assertIn("Graceful shutdown requested", body)
        self.assertIn("return;", body)

    def test_second_signal_forces_exit(self):
        match = re.search(
            r"static void SignalHandler\(int sig\) \{(?P<body>.*?)\n\}",
            self.source,
            re.DOTALL,
        )

        self.assertIsNotNone(match, "missing SignalHandler")
        body = match.group("body")
        self.assertIn("Second signal received; forcing exit", body)
        self.assertIn("std::_Exit", body)

    def test_run_polls_signal_outside_handler(self):
        self.assertIn(
            "g_pipeline->Run([]() {\n"
            "        return g_shutdown_signal != 0;\n"
            "    });",
            self.source,
        )

    def test_status_event_uses_one_low_level_write(self):
        match = re.search(
            r"static void WriteStatusEvent\(PipelineState state, "
            r"const std::string& msg\) \{(?P<body>.*?)\n\}",
            self.source,
            re.DOTALL,
        )

        self.assertIsNotNone(match, "missing atomic status writer")
        body = match.group("body")
        self.assertIn('"VOICE_STATUS\\t" + MakeStatusEvent', body)
        self.assertIn("write(STDOUT_FILENO, line.data(), line.size())", body)


if __name__ == "__main__":
    unittest.main()
