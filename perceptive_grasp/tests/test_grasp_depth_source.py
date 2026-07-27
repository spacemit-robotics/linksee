#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Regression checks for grasp-depth pixel handling."""

from pathlib import Path
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


class GraspDepthSourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pipeline = PIPELINE_CPP.read_text(encoding="utf-8")

    def test_geometry_reference_pixel_is_clamped_to_depth_image(self):
        self.assertIn("int ClampPixel", self.pipeline)
        self.assertIn("current_depth_.cols", self.pipeline)
        self.assertIn("current_depth_.rows", self.pipeline)
        self.assertIn("Geometry reference pixel", self.pipeline)

    def test_depth_uses_geometry_then_optional_target_center(self):
        self.assertIn("foreground_depth_mm", self.pipeline)
        self.assertIn("geometry_foreground_cluster", self.pipeline)
        self.assertIn("MedianDepthAtPixel", self.pipeline)
        self.assertIn("geometry_silhouette", self.pipeline)
        self.assertNotIn("Target depth invalid at mask and center",
                         self.pipeline)

    def test_top_grasp_uses_mask_pixel_depth_planning(self):
        self.assertIn(
            "bool GraspPipeline::BuildMaskTopGrasp", self.pipeline)
        self.assertIn("ComputeGraspPixel(", self.pipeline)
        self.assertIn("ForegroundDepthFromMask(", self.pipeline)
        self.assertIn("planner_->PlanTopGrasp(", self.pipeline)
        self.assertIn("config_.top_grasp_point_x_ratio", self.pipeline)
        self.assertIn(
            "Top-grasp yaw source=2d_mask", self.pipeline)
        self.assertNotIn(
            "Top-grasp endpoint selection:", self.pipeline)
        self.assertNotIn("farther_from_arm_base", self.pipeline)
        self.assertNotIn("opposite_radius_sq", self.pipeline)
        self.assertNotIn("Top-grasp yaw source=3d_geometry", self.pipeline)
        self.assertIn(
            "preferred_candidate->strategy == GraspStrategy::TOP",
            self.pipeline,
        )

    def test_selected_top_strategy_bypasses_3d_candidate_pipeline(self):
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        top_planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandleTopPlanning",
        )
        self.assertIn(
            "observation_strategy_selected_ &&\n"
            "        grasp_strategy_ == GraspStrategy::TOP",
            planning,
        )
        self.assertIn(
            'current_target_.label_name != "cup"',
            planning,
        )
        self.assertIn(
            'current_target_.label_name != "bottle"',
            planning,
        )
        self.assertIn("HandleTopPlanning()", planning)
        self.assertIn("BuildMaskTopGrasp(", top_planning)
        self.assertIn("PlanMobileBaseAlignment(", top_planning)
        self.assertIn(
            "Mobile base alignment target center:", top_planning)
        self.assertIn(
            "config_.mobile_base, alignment_point", top_planning)
        self.assertIn("PipelineState::APPROACHING", top_planning)
        self.assertNotIn("geometry_planner_->Plan", top_planning)
        self.assertIn("if (config_.plan_only)", top_planning)
        self.assertIn("executor_->ValidateGraspPoses", top_planning)
        self.assertIn(
            "config_.geometry.planning_timeout_ms", top_planning)
        self.assertIn(
            "Top-grasp plan validation failed", top_planning)
        self.assertNotIn("PERCEPTION_PLANNING", top_planning)

    def test_post_motion_geometry_gets_one_bounded_refresh(self):
        self.assertIn("kMotionGeometryMaxRefreshes = 1", self.pipeline)
        self.assertIn("kGeometryRefreshAttempt = 6", self.pipeline)
        self.assertIn("motion_geometry_refresh_count_", self.pipeline)
        self.assertIn('"geometry estimation retry"', self.pipeline)
        self.assertIn(
            '"Refreshed camera after transient post-motion "',
            self.pipeline,
        )
        self.assertIn('"3D geometry"', self.pipeline)
        self.assertIn('"geometry confirmation retry"', self.pipeline)
        self.assertIn(
            "Refreshing transient 3D geometry after motion", self.pipeline)
        self.assertNotIn(
            "motion_geometry_expected_dimensions_", self.pipeline)


if __name__ == "__main__":
    unittest.main()
