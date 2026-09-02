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
MOBILE_BASE_CPP = ROOT / "src" / "mobile_base_controller.cpp"
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
        cls.mobile_base = MOBILE_BASE_CPP.read_text(encoding="utf-8")
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

    def test_max_base_alignment_attempts_fall_back_to_arm_validation(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        max_attempts_index = body.find(
            "reached the alignment attempt")
        plan_index = body.find("executor_->ValidateGraspPoses")
        self.assertGreaterEqual(max_attempts_index, 0)
        self.assertGreaterEqual(plan_index, 0)
        self.assertLess(max_attempts_index, plan_index)
        self.assertIn(
            "continuing with workspace, IK and path validation", body)

    def test_alignment_has_progress_and_travel_safety_guards(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        visual = _function_body(
            self.pipeline,
            "bool GraspPipeline::ValidateMobileBaseVisualProgress",
        )
        self.assertIn("ValidateMobileBaseVisualProgress", body)
        self.assertIn("MeasureMobileBaseAlignmentProgress", visual)
        self.assertIn("RequiredMobileBaseAlignmentProgress", visual)
        self.assertIn("required_progress", visual)
        self.assertIn("visual motion confirmed", visual)
        self.assertIn("max_visual_regression_m", visual)
        self.assertIn("visual progress regressed", visual)
        self.assertIn(
            "visual feedback did not confirm", visual
        )
        self.assertIn("SetState(PipelineState::ERROR", visual)
        self.assertIn("base_alignment_soft_stopped", body)
        self.assertIn("max_total_travel_m", body)
        self.assertIn("base_align_travel_m_", body)
        self.assertIn("visual progress", visual)
        self.assertIn("last_base_motion_odometry_confirmed_", visual)
        self.assertIn("odometry already confirmed", visual)
        self.assertIn("progress >= -maximum_regression", visual)
        self.assertIn(
            "arm safety validation remains required", body)
        self.assertIn("ValidateBaseAlignmentCommandTransition", body)

    def test_top_planning_uses_same_visual_progress_gate(self):
        body = _function_body(
            self.pipeline, "void GraspPipeline::HandleTopPlanning"
        )
        self.assertIn(
            "if (!ValidateMobileBaseVisualProgress(alignment_point)) "
            "return;",
            body,
        )

    def test_unconfirmed_odometry_falls_back_to_visual_confirmation(self):
        execute = _function_body(
            self.mobile_base,
            "GraspResult MobileBaseController::Execute")
        self.assertIn(
            "continuing with visual confirmation", execute)
        unconfirmed_index = execute.find(
            "report.odometry_available && !report.motion_confirmed")
        return_index = execute.find(
            "return GraspResult::SUCCESS", unconfirmed_index)
        self.assertGreaterEqual(unconfirmed_index, 0)
        self.assertGreater(return_index, unconfirmed_index)

    def test_planning_prefers_foreground_mask_depth(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn("foreground_depth_mm", body)
        self.assertIn("geometry_foreground_cluster", body)

    def test_side_grasp_aligns_closed_gripper_sweep_distance(self):
        body = _function_body(self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn("side_pregrasp_min_x_m", body)
        self.assertIn("kSidePreGraspAlignmentWindowM", body)
        self.assertIn("minimum_pregrasp_x", body)
        self.assertIn("maximum_pregrasp_x", body)
        self.assertIn("preferred_pregrasp_x", body)
        self.assertIn("desired_range=[", body)
        self.assertIn("required_shift", body)
        self.assertIn("std::clamp", body)
        self.assertIn("config_.planner.workspace.x_max", body)
        self.assertIn(
            "alignment_config.x_tolerance =\n"
            "                0.5f * kSidePreGraspAlignmentWindowM",
            body,
        )
        self.assertIn(
            "alignment_config.x_hysteresis =\n"
            "                kSidePreGraspUpperHysteresisM",
            body,
        )
        self.assertNotIn(
            "alignment_config.x_tolerance = std::min",
            body,
        )
        self.assertNotIn(
            "config_.mobile_base.target_x +",
            body,
        )
        self.assertIn("alignment_config.target_x = base_point[0]", body)
        self.assertIn("preserving current base distance", body)

    def test_side_alignment_stops_after_minimum_pulse_overshoot(self):
        body = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn("side_pregrasp_alignment_active", body)
        self.assertIn(
            "IsMobileBaseDirectionReversal(",
            body,
        )
        self.assertIn(
            "minimum chassis pulse crossed the side alignment target",
            body,
        )
        self.assertIn(
            "validating ",
            body,
        )
        self.assertIn(
            "arm reachability at the current position",
            body,
        )
        self.assertIn(
            "workspace and IK validation ",
            body,
        )
        self.assertIn(
            "remain required",
            body,
        )

    def test_top_grasp_alignment_uses_candidate_grasp_point(self):
        body = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn("candidate.strategy == GraspStrategy::TOP", body)
        self.assertIn(
            "alignment_point[0] = candidate.grasp_pose.x", body)
        self.assertIn(
            "alignment_point[1] = candidate.grasp_pose.y", body)
        self.assertIn(
            "alignment_config, alignment_point, base_align_attempts_", body)
        self.assertIn(
            "ValidateMobileBaseVisualProgress(alignment_point)", body)

    def test_base_alignment_moves_base_then_detects_again(self):
        body = _function_body(
            self.pipeline, "void GraspPipeline::HandleBaseAligning")
        self.assertIn("mobile_base_->Execute", body)
        self.assertIn("mobile_base_->LastMotionReport()", body)
        self.assertIn("motion_report.odometry_available", body)
        self.assertIn("motion_report.translation_m", body)
        self.assertIn("motion_report.motion_confirmed", body)
        self.assertIn("last_base_motion_odometry_confirmed_", body)
        self.assertIn('SetState(PipelineState::DETECTING', body)
        self.assertIn("stable_count_ = 0", body)
        self.assertIn('FlushCameraAfterMotion("base motion")', body)

    def test_config_and_loader_expose_mobile_base_settings(self):
        self.assertIn("mobile_base:", self.config)
        self.assertIn("target_x: 0.275", self.config)
        self.assertIn("x_tolerance: 0.035", self.config)
        self.assertIn("y_tolerance: 0.15", self.config)
        self.assertIn("min_cmd_duration_ms: 350", self.config)
        self.assertIn("cfg.mobile_base.enabled", self.main)
        self.assertIn("target_x", self.main)
        self.assertIn("x_hysteresis", self.main)
        self.assertIn("max_align_attempts", self.main)
        self.assertIn("min_progress_m", self.main)
        self.assertIn("min_progress_ratio", self.main)
        self.assertIn("min_progress_floor_m", self.main)
        self.assertIn("max_visual_regression_m", self.main)
        self.assertIn("max_total_travel_m", self.main)
        self.assertIn("odom_min_translation_m", self.main)
        self.assertIn("odom_min_rotation_rad", self.main)
        self.assertIn("odom_min_command_ratio", self.main)
        self.assertIn("max_direction_reversals", self.main)

    def test_build_includes_mobile_base_controller(self):
        self.assertIn("src/mobile_base_controller.cpp", self.cmake)
        self.assertIn("chassis", self.cmake)


if __name__ == "__main__":
    unittest.main()
