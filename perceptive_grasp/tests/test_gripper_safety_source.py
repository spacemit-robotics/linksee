#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for gripper configuration and empty-grasp handling."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
EXECUTOR_CPP = ROOT / "src" / "grasp_executor.cpp"
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"
CONFIG_YAML = ROOT / "config" / "grasp_pipeline.yaml"


class GripperSafetySourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.executor = EXECUTOR_CPP.read_text(encoding="utf-8")
        cls.pipeline = PIPELINE_CPP.read_text(encoding="utf-8")
        cls.config = yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8"))

    def test_empty_closed_baseline_is_used(self):
        self.assertIn("CaptureEmptyClosedPosition()", self.executor)
        self.assertIn("empty_closed_position_", self.executor)
        self.assertIn("gripper_empty_position_margin", self.executor)

    def test_load_alone_cannot_report_success(self):
        self.assertNotIn("Grasp inferred from sustained load", self.executor)
        self.assertIn("state_or_baseline_confirmed", self.executor)
        self.assertIn("!std::isnan(empty_closed_position_)", self.executor)
        self.assertIn("load_holding_count >= required_holding", self.executor)
        self.assertIn("opening_indicates_object", self.executor)

    def test_grasp_is_verified_after_lift(self):
        self.assertIn('CheckGripperHolding("after_lift", true)', self.executor)
        self.assertIn("Object lost after lift", self.pipeline)

    def test_release_opening_uses_position_control(self):
        self.assertIn(
            "grasp_set_position(gripper_, config_.place_release_open)",
            self.executor,
        )

    def test_gripper_controls_are_in_release_config(self):
        grasp = self.config["grasp"]
        place = self.config["place"]
        self.assertIn("gripper_open", grasp)
        self.assertIn("gripper_effort", grasp)
        self.assertIn("gripper_empty_position_margin", grasp)
        self.assertIn("release_open", place)


if __name__ == "__main__":
    unittest.main()
