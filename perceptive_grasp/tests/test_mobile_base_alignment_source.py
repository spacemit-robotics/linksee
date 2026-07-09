#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for mobile-base-assisted grasp alignment."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PIPELINE_H = ROOT / "include" / "grasp_pipeline.h"
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"
MAIN_CPP = ROOT / "src" / "main.cpp"
CMAKE = ROOT / "CMakeLists.txt"
CONFIG = ROOT / "config" / "grasp_pipeline.yaml"


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


class MobileBaseAlignmentSourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = PIPELINE_H.read_text(encoding="utf-8")
        cls.pipeline = PIPELINE_CPP.read_text(encoding="utf-8")
        cls.main = MAIN_CPP.read_text(encoding="utf-8")
        cls.cmake = CMAKE.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")

    def test_pipeline_has_base_alignment_state_and_handler(self):
        self.assertIn("BASE_ALIGNING", self.header)
        self.assertIn("void HandleBaseAligning()", self.header)
        self.assertIn("std::unique_ptr<MobileBaseController>", self.header)

    def test_planning_schedules_base_alignment_before_arm_motion(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        align_index = body.find("PlanMobileBaseAlignment")
        approach_index = body.find('SetState(PipelineState::APPROACHING')
        self.assertGreaterEqual(align_index, 0)
        self.assertGreaterEqual(approach_index, 0)
        self.assertLess(align_index, approach_index)
        self.assertIn("PipelineState::BASE_ALIGNING", body)

    def test_max_base_alignment_attempts_continue_to_arm_planning(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        max_attempts_index = body.find("continue with arm planning")
        plan_index = body.find("planner_->PlanTopGrasp")
        error_index = body.find("Mobile base alignment failed: max attempts reached")
        self.assertGreaterEqual(max_attempts_index, 0)
        self.assertGreaterEqual(plan_index, 0)
        self.assertLess(max_attempts_index, plan_index)
        self.assertEqual(error_index, -1)

    def test_base_alignment_moves_base_then_detects_again(self):
        body = _function_body(
            self.pipeline, "void GraspPipeline::HandleBaseAligning")
        self.assertIn("mobile_base_->Execute", body)
        self.assertIn('SetState(PipelineState::DETECTING', body)
        self.assertIn("stable_count_ = 0", body)

    def test_config_and_loader_expose_mobile_base_settings(self):
        self.assertIn("mobile_base:", self.config)
        self.assertIn("cfg.mobile_base.enabled", self.main)
        self.assertIn("target_x", self.main)
        self.assertIn("max_align_attempts", self.main)

    def test_build_includes_mobile_base_controller(self):
        self.assertIn("src/mobile_base_controller.cpp", self.cmake)
        self.assertIn("chassis", self.cmake)


if __name__ == "__main__":
    unittest.main()
