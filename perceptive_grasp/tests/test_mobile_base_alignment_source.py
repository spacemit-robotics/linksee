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

    def test_max_base_alignment_attempts_stop_before_arm_planning(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        max_attempts_index = body.find("Base alignment failed: max attempts")
        plan_index = body.find("planner_->PlanTopGrasp")
        self.assertGreaterEqual(max_attempts_index, 0)
        self.assertGreaterEqual(plan_index, 0)
        self.assertLess(max_attempts_index, plan_index)

    def test_alignment_has_progress_and_travel_safety_guards(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn("MeasureMobileBaseAlignmentProgress", body)
        self.assertIn("RequiredMobileBaseAlignmentProgress", body)
        self.assertIn("required_progress", body)
        self.assertIn("max_total_travel_m", body)
        self.assertIn("visual progress", body)
        self.assertIn("check depth and motion", body)

    def test_planning_prefers_foreground_mask_depth(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn("ForegroundDepthFromMask", body)
        self.assertIn("mask_foreground_q25", body)

    def test_base_alignment_moves_base_then_detects_again(self):
        body = _function_body(
            self.pipeline, "void GraspPipeline::HandleBaseAligning")
        self.assertIn("mobile_base_->Execute", body)
        self.assertIn('SetState(PipelineState::DETECTING', body)
        self.assertIn("stable_count_ = 0", body)
        self.assertIn('FlushCameraAfterMotion("base motion")', body)

    def test_config_and_loader_expose_mobile_base_settings(self):
        self.assertIn("mobile_base:", self.config)
        self.assertIn("target_x: 0.275", self.config)
        self.assertIn("x_tolerance:", self.config)
        self.assertIn("y_tolerance: 0.15", self.config)
        self.assertIn("cfg.mobile_base.enabled", self.main)
        self.assertIn("target_x", self.main)
        self.assertIn("max_align_attempts", self.main)
        self.assertIn("min_progress_m", self.main)
        self.assertIn("min_progress_ratio", self.main)
        self.assertIn("min_progress_floor_m", self.main)
        self.assertIn("max_total_travel_m", self.main)

    def test_build_includes_mobile_base_controller(self):
        self.assertIn("src/mobile_base_controller.cpp", self.cmake)
        self.assertIn("chassis", self.cmake)


if __name__ == "__main__":
    unittest.main()
