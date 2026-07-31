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
        top_grasp = _function_body(
            self.pipeline, "bool GraspPipeline::BuildMaskTopGrasp")
        self.assertIn(
            "bool GraspPipeline::BuildMaskTopGrasp", self.pipeline)
        self.assertIn("ComputeGraspPixel(", self.pipeline)
        self.assertIn("SampleMaskedDepthNearPixel(", top_grasp)
        self.assertLess(
            top_grasp.find("SampleMaskedDepthNearPixel("),
            top_grasp.find("ForegroundDepthFromMask("),
        )
        self.assertIn("depth_sample.x", top_grasp)
        self.assertIn("depth_sample.y", top_grasp)
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

    def test_selected_top_uses_2d_candidate_with_3d_support_plane(self):
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
        self.assertNotIn('current_target_.label_name != "cup"', planning)
        self.assertNotIn('current_target_.label_name != "bottle"', planning)
        self.assertIn("HandleTopPlanning()", planning)
        self.assertIn("BuildMaskTopGrasp(", top_planning)
        self.assertIn("PlanMobileBaseAlignment(", top_planning)
        self.assertIn(
            "Mobile base alignment target center:", top_planning)
        self.assertIn(
            "config_.mobile_base, alignment_point", top_planning)
        self.assertIn("PipelineState::APPROACHING", top_planning)
        self.assertIn("geometry_planner_->Plan", top_planning)
        self.assertIn("ResolveTopSupportPlane(", top_planning)
        self.assertIn("executor_->SetSupportPlane", top_planning)
        resolve_support = _function_body(
            self.pipeline,
            "bool GraspPipeline::ResolveTopSupportPlane",
        )
        self.assertIn("BuildSupportPlane(", resolve_support)
        self.assertIn("last_top_support_plane_valid_", resolve_support)
        self.assertIn("BuildWorkspaceSupportPlane(", resolve_support)
        self.assertNotIn(
            "RetryTransientTopPlanning", resolve_support)
        initial_top_index = planning.find(
            "if (!observation_strategy_selected_ &&\n"
            "        explicit_top_strategy)")
        support_index = planning.find(
            "executor_->SetSupportPlane(observation_support_plane)",
            initial_top_index,
        )
        observe_index = planning.find(
            "PipelineState::OBSERVING",
            initial_top_index,
        )
        self.assertGreaterEqual(initial_top_index, 0)
        self.assertGreaterEqual(support_index, 0)
        self.assertGreaterEqual(observe_index, 0)
        self.assertLess(support_index, observe_index)
        self.assertIn("if (config_.plan_only)", top_planning)
        self.assertIn("executor_->ValidateGraspPoses", top_planning)
        self.assertIn(
            "config_.geometry.planning_timeout_ms", top_planning)
        self.assertIn(
            "Top-grasp plan validation failed", top_planning)
        self.assertNotIn("PERCEPTION_PLANNING", top_planning)
        validation_index = top_planning.find(
            "executor_->ValidateGraspPoses")
        plan_only_index = top_planning.find("if (config_.plan_only)")
        approach_index = top_planning.find(
            "PipelineState::APPROACHING")
        self.assertLess(validation_index, plan_only_index)
        self.assertLess(validation_index, approach_index)

    def test_top_grasp_uses_geometry_depth_for_local_depth_holes(self):
        build = _function_body(
            self.pipeline,
            "bool GraspPipeline::BuildMaskTopGrasp",
        )
        top_planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandleTopPlanning",
        )
        self.assertIn("fallback_depth_mm", build)
        self.assertIn("intended_grasp_x", build)
        self.assertIn(
            "source=geometry_foreground_cluster",
            build,
        )
        self.assertIn(
            "safety_geometry.foreground_depth_mm",
            top_planning,
        )

    def test_top_grasp_uses_mask_or_support_plane_for_sparse_depth(self):
        build = _function_body(
            self.pipeline,
            "bool GraspPipeline::BuildMaskTopGrasp",
        )
        estimate = _function_body(
            self.pipeline,
            "bool GraspPipeline::EstimateSupportPlaneDepth",
        )
        top_planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandleTopPlanning",
        )
        self.assertIn("ForegroundDepthFromMask(", build)
        self.assertIn("EstimateSupportPlaneDepth(", build)
        self.assertIn("source=target_mask_foreground", build)
        self.assertIn("source=support_plane_intersection", build)
        self.assertIn("camera_->Deproject", estimate)
        self.assertIn("planner_->CameraToBase", estimate)
        self.assertIn("&support_plane", top_planning)

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

    def test_top_grasp_retries_transient_depth_with_a_fixed_limit(self):
        top_planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandleTopPlanning",
        )
        retry = _function_body(
            self.pipeline,
            "bool GraspPipeline::RetryTransientTopPlanning",
        )
        self.assertIn("RetryTransientTopPlanning(error)", top_planning)
        self.assertIn("kMaxGeometryAttempts", retry)
        self.assertNotIn("config_.auto_loop", retry)
        self.assertIn("PipelineState::DETECTING", retry)
        self.assertIn('"top-grasp depth recovery"', retry)
        self.assertIn("FlushCameraAfterMotion(", retry)
        self.assertNotIn("PipelineState::OBSERVING", retry)

    def test_top_base_alignment_uses_stable_mask_depth(self):
        top_planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandleTopPlanning",
        )
        confirmation = _function_body(
            self.pipeline,
            "bool GraspPipeline::ConfirmTopAlignmentPoint",
        )
        self.assertIn("ForegroundDepthFromMask(", top_planning)
        self.assertIn("ConfirmTopAlignmentPoint(", top_planning)
        self.assertIn(
            "kTopAlignmentMaximumFrameDeltaM", confirmation)
        self.assertIn("top_alignment_reference_valid_", confirmation)
        self.assertIn("PipelineState::DETECTING", confirmation)

    def test_low_profile_target_can_select_top_before_observation(self):
        planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandlePlanning",
        )
        fallback = _function_body(
            self.pipeline,
            "bool GraspPipeline::BuildLowProfileTopFallback",
        )
        self.assertIn("BuildLowProfileTopFallback(", planning)
        self.assertIn(
            "!observation_strategy_selected_", planning)
        self.assertIn(
            "geometry_retry_count_ >= kTopGeometryRecoveryAttempt",
            planning,
        )
        self.assertIn("ResolveTopSupportPlane(", fallback)
        self.assertIn("BuildMaskTopGrasp(", fallback)
        self.assertIn("geometry.height_m = std::max(", fallback)
        self.assertIn("kMaximumBelowSupportPlaneM", fallback)
        self.assertNotIn(
            "signed_table_distance < minimum_height_m", fallback)
        self.assertIn("failed_geometry.foreground_depth_mm", fallback)
        self.assertIn("base_align_attempts_ > 0", fallback)
        self.assertIn("last_valid_geometry_available_", fallback)
        self.assertIn("last_valid_strategy_available_", fallback)
        self.assertIn(
            "last_valid_strategy_ == GraspStrategy::TOP",
            fallback,
        )
        self.assertIn(
            "confirmed_top_after_base_motion ? &support_plane : nullptr",
            fallback,
        )
        self.assertIn(
            "fallback_depth_plane,\n"
            "            confirmed_top_after_base_motion",
            fallback,
        )
        self.assertIn(
            "config_.geometry.side_min_height_m",
            fallback,
        )
        self.assertIn(
            "config_.geometry.object_min_height_m",
            fallback,
        )
        self.assertNotIn("label_name", fallback)

    def test_base_motion_preserves_confirmed_top_geometry_for_recovery(self):
        planning = _function_body(
            self.pipeline,
            "void GraspPipeline::HandlePlanning",
        )
        preferred_index = planning.find(
            "GraspCandidate* preferred_candidate = nullptr")
        cache_index = planning.find(
            "last_valid_strategy_ = preferred_candidate->strategy",
            preferred_index,
        )
        base_index = planning.find(
            "base_alignment_command_ = PlanMobileBaseAlignment",
            cache_index,
        )
        self.assertGreaterEqual(preferred_index, 0)
        self.assertGreater(cache_index, preferred_index)
        self.assertGreater(base_index, cache_index)


if __name__ == "__main__":
    unittest.main()
