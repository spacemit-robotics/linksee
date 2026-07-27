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
EXECUTOR_HEADER = ROOT / "include" / "grasp_executor.h"
GEOMETRY_HEADER = ROOT / "include" / "grasp_geometry.h"
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"
GEOMETRY_CPP = ROOT / "src" / "grasp_geometry.cpp"
MAIN_CPP = ROOT / "src" / "main.cpp"
DEBUG_LOCALIZE_CPP = ROOT / "tools" / "debug_localize.cpp"
CONFIG_YAML = ROOT / "config" / "grasp_pipeline.yaml"


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


class GripperSafetySourceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.executor = EXECUTOR_CPP.read_text(encoding="utf-8")
        cls.header = EXECUTOR_HEADER.read_text(encoding="utf-8")
        cls.geometry_header = GEOMETRY_HEADER.read_text(encoding="utf-8")
        cls.pipeline = PIPELINE_CPP.read_text(encoding="utf-8")
        cls.geometry = GEOMETRY_CPP.read_text(encoding="utf-8")
        cls.main = MAIN_CPP.read_text(encoding="utf-8")
        cls.debug_localize = DEBUG_LOCALIZE_CPP.read_text(encoding="utf-8")
        cls.config = yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8"))

    def test_empty_closed_baseline_is_used(self):
        self.assertIn("CaptureEmptyClosedPosition()", self.executor)
        self.assertIn("empty_closed_position_", self.executor)
        self.assertIn("gripper_empty_position_margin", self.executor)
        baseline = _function_body(
            self.executor,
            "void GraspExecutor::CaptureEmptyClosedPosition",
        )
        release = _function_body(
            self.executor, "GraspResult GraspExecutor::ReleaseObject")
        self.assertIn("std::isfinite(empty_closed_position_)", baseline)
        self.assertIn("gripper_hold_load_threshold", baseline)
        self.assertIn("empty_closed_position_ = NAN", release)

    def test_grasp_confirmation_uses_sustained_contact_feedback(self):
        self.assertNotIn("Grasp inferred from sustained load", self.executor)
        self.assertNotIn("state_or_baseline_confirmed", self.executor)
        self.assertIn("load_holding_count >= required_holding", self.executor)
        self.assertIn("opening_indicates_object", self.executor)
        self.assertIn(
            "if (cur_position > min_object_position &&",
            self.executor,
        )
        self.assertNotIn(
            "if (state == GRASP_STATE_HOLDING &&\n"
            "                cur_position > min_object_position",
            self.executor,
        )
        self.assertIn("maximum_checks", self.executor)
        self.assertIn("kAfterLiftOpeningHysteresis", self.executor)
        self.assertIn(
            "kAfterLiftOpeningHysteresis = 0.010f",
            self.executor,
        )
        self.assertIn(
            "int gripper_close_wait_ms = 1000",
            self.header,
        )

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
        top = grasp["top"]
        place = self.config["place"]
        self.assertIn("gripper_open", top)
        self.assertIn("gripper_effort", grasp)
        self.assertIn("gripper_empty_position_margin", grasp)
        self.assertIn("release_open", place)

    def test_top_and_side_config_blocks_are_loaded_independently(self):
        self.assertIn(
            'const YAML::Node top = g["top"]',
            self.main,
        )
        self.assertIn('if (auto side = g["side"])', self.main)
        self.assertIn('top["grasp_point_x_ratio"]', self.main)
        self.assertIn('side["gripper_offset_m"]', self.main)
        self.assertNotIn('g["top"] ? g["top"] : g', self.main)
        self.assertNotIn('geometry_node["side_', self.main)
        self.assertNotIn('geometry["side_', self.debug_localize)

    def test_removed_side_config_aliases_are_not_parsed(self):
        self.assertNotIn(
            "side_pregrasp_clearance_m",
            self.geometry_header,
        )
        self.assertNotIn("side_pregrasp_clearance_m", self.main)
        self.assertNotIn(
            "side_pregrasp_clearance_m",
            self.debug_localize,
        )
        self.assertNotIn('geometry["strategy"]', self.debug_localize)

    def test_top_grasp_uses_constrained_yaw_execution_path(self):
        self.assertNotIn("validated_top_pre_grasp_joints_", self.executor)
        self.assertNotIn("validated_top_grasp_joints_", self.executor)
        self.assertNotIn("SolveTopIKWithYaw", self.executor)

    def test_predefined_joint_poses_are_checked_against_limits(self):
        self.assertIn("validate_joint_pose", self.main)
        self.assertIn(
            '"manipulator.home_joints", cfg.executor.home_joints',
            self.main,
        )
        self.assertIn(
            '"manipulator.observe_joints", cfg.executor.observe_joints',
            self.main,
        )
        self.assertIn(
            '"manipulator.side_ready_joints", '
            "cfg.executor.side_ready_joints",
            self.main,
        )
        self.assertIn(
            '"place.place_joints", cfg.executor.place_joints',
            self.main,
        )
        self.assertIn("is outside configured range", self.main)

    def test_fixed_jaw_yaw_uses_configured_rotation_direction(self):
        resolver = _function_body(
            self.executor, "bool ResolveFixedJawWristYaw")
        self.assertIn(
            "raw_wrist = (target_yaw - joint0) / scale", resolver)
        self.assertIn(
            "resolved_wrist = std::clamp(raw_wrist, lower, upper)",
            resolver,
        )
        self.assertNotIn("equivalent_yaw", resolver)
        self.assertNotIn("M_PI", resolver)
        self.assertIn(
            "ResolveFixedJawWristYaw(", self.executor)
        self.assertNotIn(
            "ResolveEquivalentWristYaw(", self.executor)

    def test_side_grasp_stages_open_after_pre_grasp(self):
        move_to_pre_grasp = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToPreGrasp")
        close_index = move_to_pre_grasp.find("CloseGripper()")
        side_move_index = move_to_pre_grasp.find("MoveToSidePreGrasp")
        self.assertGreaterEqual(close_index, 0)
        self.assertGreaterEqual(side_move_index, 0)
        self.assertLess(close_index, side_move_index)

        pre_grasp = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToSidePreGrasp")
        ready_index = pre_grasp.find("side_ready_joints")
        approach_index = pre_grasp.find("PlanSideJoint0Sweep")
        self.assertGreaterEqual(ready_index, 0)
        self.assertGreaterEqual(approach_index, 0)
        self.assertLess(ready_index, approach_index)
        self.assertIn(
            "{staging_joints, sweep_joints}",
            pre_grasp,
        )
        self.assertIn("ExecuteContinuousJointPath(", pre_grasp)
        self.assertNotIn("TakeValidatedSidePath", pre_grasp)
        self.assertIn(
            '"move_to_side_safe_pre_grasp"',
            pre_grasp,
        )

        sweep = _function_body(
            self.executor, "GraspResult GraspExecutor::PlanSideJoint0Sweep")
        self.assertIn("staging_pose.z = std::max(", sweep)
        self.assertIn("SolveIKSide(", sweep)
        self.assertIn("sweep_joints = staging_joints", sweep)
        self.assertIn("sweep_joints[0] =", sweep)
        self.assertNotIn("sweep_joints[1] =", sweep)
        self.assertNotIn("sweep_joints[2] =", sweep)
        self.assertNotIn("sweep_joints[3] =", sweep)
        self.assertNotIn("sweep_joints[4] =", sweep)

        side_pose = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToPoseSide")
        self.assertNotIn("move_to_side_grasp_entry", side_pose)
        self.assertIn(
            "{validated_side_entry_joints_}",
            pre_grasp,
        )

        grasping = _function_body(
            self.pipeline, "void GraspPipeline::HandleGrasping")
        open_index = grasping.find("OpenGripperForGrasp")
        move_index = grasping.find("MoveToGrasp")
        close_index = grasping.find("CloseGripperAndCheck")
        self.assertGreaterEqual(open_index, 0)
        self.assertLess(open_index, move_index)
        self.assertLess(move_index, close_index)
        self.assertIn('SaveStepCameraDebug("grasp_pose")', grasping)
        self.assertIn(
            "At safe pre-grasp above target",
            self.pipeline,
        )

    def test_observe_motion_coordinates_elbow_and_wrist(self):
        observe = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToObserve")
        self.assertIn("starts_near_home", observe)
        self.assertIn("kHomeJointToleranceRad", observe)
        coordinated_index = observe.find("MoveToJointsCoordinated")
        collision_safe_index = observe.find("MoveToJointsCollisionSafe")
        self.assertGreaterEqual(coordinated_index, 0)
        self.assertGreaterEqual(collision_safe_index, 0)
        self.assertLess(coordinated_index, collision_safe_index)

        coordinated = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveToJointsCoordinated",
        )
        normalized = " ".join(coordinated.replace('"', "").split())
        self.assertIn("continuous synchronized progress", normalized)
        self.assertIn("ExecuteContinuousJointPath", coordinated)
        self.assertIn("kObserveFinalJointToleranceRad", coordinated)

    def test_side_observation_uses_continuous_fourth_joint_lead(self):
        side_observe = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveToSideObserve",
        )
        self.assertIn("constexpr size_t kFourthJointIndex = 3",
                      side_observe)
        self.assertIn("kOtherJointLeadProgress", side_observe)
        self.assertIn("kFourthJointLeadProgress", side_observe)
        self.assertIn("build_lead_path", side_observe)
        self.assertIn("ExecuteContinuousJointPath", side_observe)
        self.assertIn("config_.home_joints", side_observe)
        self.assertNotIn("MoveToJoints(fourth_joint_first)", side_observe)
        self.assertNotIn("WaitMotionDone()", side_observe)

    def test_recoverable_motion_replans_before_terminal_failure(self):
        retry = _function_body(
            self.pipeline,
            "bool GraspPipeline::RetryRecoverableMotion",
        )
        approaching = _function_body(
            self.pipeline, "void GraspPipeline::HandleApproaching")
        grasping = _function_body(
            self.pipeline, "void GraspPipeline::HandleGrasping")
        self.assertIn("GraspResult::OUT_OF_RANGE", retry)
        self.assertIn("GraspResult::IK_FAILED", retry)
        self.assertIn("PipelineState::OBSERVING", retry)
        self.assertIn("RetryRecoverableMotion", approaching)
        self.assertIn("RetryRecoverableMotion", grasping)
        self.assertNotIn("target out of workspace", self.pipeline)

    def test_workspace_filter_runs_after_mobile_base_alignment(self):
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        alignment_index = planning.find("PlanMobileBaseAlignment")
        workspace_index = planning.find(
            "grasp pose remains outside workspace after base alignment")
        ik_index = planning.find("executor_->ValidateGraspPoses")
        self.assertGreaterEqual(alignment_index, 0)
        self.assertGreaterEqual(workspace_index, 0)
        self.assertGreaterEqual(ik_index, 0)
        self.assertLess(alignment_index, workspace_index)
        self.assertLess(workspace_index, ik_index)
        self.assertIn(
            "base_point, grasp_pose, pre_grasp_pose, false",
            self.pipeline,
        )
        self.assertNotIn(
            'Reject(candidate, "side grasp pose is outside workspace")',
            self.geometry,
        )
        self.assertNotIn(
            'Reject(top, "top grasp pose is outside workspace")',
            self.geometry,
        )

    def test_detection_and_strategy_selection_precede_observation_motion(self):
        trigger = _function_body(
            self.pipeline,
            "bool GraspPipeline::TriggerGrasp(const std::string& target_label)",
        )
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        observing = _function_body(
            self.pipeline, "void GraspPipeline::HandleObserving")

        self.assertIn("SetState(PipelineState::DETECTING", trigger)
        self.assertNotIn("SetState(PipelineState::OBSERVING", trigger)
        strategy_index = planning.find(
            "observation_strategy_selected_ = true")
        observe_index = planning.find("PipelineState::OBSERVING")
        base_alignment_index = planning.find("PlanMobileBaseAlignment")
        self.assertGreaterEqual(strategy_index, 0)
        self.assertGreaterEqual(observe_index, 0)
        self.assertGreaterEqual(base_alignment_index, 0)
        self.assertLess(strategy_index, observe_index)
        self.assertLess(observe_index, base_alignment_index)
        self.assertIn("matches_observation_strategy", planning)
        self.assertIn("executor_->MoveToSideObserve()", observing)
        self.assertIn("executor_->MoveToObserve()", observing)

    def test_auto_loop_restarts_success_and_failure_with_target_and_timing(self):
        spin_once = _function_body(
            self.pipeline, "void GraspPipeline::SpinOnce")
        restart = _function_body(
            self.pipeline, "void GraspPipeline::RestartAutoLoop")
        placing = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlacing")
        detecting = _function_body(
            self.pipeline, "void GraspPipeline::HandleDetecting")

        self.assertIn('RestartAutoLoop("success")', spin_once)
        self.assertIn('RestartAutoLoop("failure")', spin_once)
        loop_index = placing.find("if (config_.auto_loop)")
        homing_index = placing.find("PipelineState::HOMING")
        self.assertEqual(loop_index, -1)
        self.assertGreaterEqual(homing_index, 0)
        self.assertIn("config_.auto_loop", placing)
        self.assertIn("if (config_.auto_loop)", detecting)
        self.assertIn("staying in DETECTING without returning home", detecting)
        target_index = restart.find(
            "const std::string target = auto_loop_target_label_")
        strategy_index = restart.find(
            "const GraspStrategy grasp_strategy = grasp_strategy_")
        reset_index = restart.find("ResetTaskState()")
        restore_index = restart.find("target_label_ = target")
        restore_strategy_index = restart.find(
            "grasp_strategy_ = grasp_strategy")
        timing_index = restart.find("BeginTaskTiming()")
        detecting_index = restart.find("PipelineState::DETECTING")
        self.assertLess(target_index, reset_index)
        self.assertLess(strategy_index, reset_index)
        self.assertLess(reset_index, restore_index)
        self.assertLess(reset_index, restore_strategy_index)
        self.assertLess(restore_index, timing_index)
        self.assertLess(restore_strategy_index, timing_index)
        self.assertLess(timing_index, detecting_index)
        self.assertIn("[Loop] START iteration=", restart)

    def test_auto_loop_returns_to_matching_observation_pose(self):
        homing = _function_body(
            self.pipeline, "void GraspPipeline::HandleHoming")
        self.assertIn(
            "config_.voice.enabled || config_.auto_loop", homing)
        self.assertIn("executor_->MoveToSideObserve()", homing)
        self.assertIn("executor_->MoveToObserve()", homing)
        self.assertIn("executor_->MoveToHome()", homing)

    def test_low_profile_top_recovery_keeps_side_geometry_strict(self):
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")

        self.assertIn("kTopGeometryRecoveryAttempt", self.pipeline)
        self.assertIn(
            "grasp_strategy_ == GraspStrategy::TOP", planning)
        self.assertIn("last_valid_geometry_available_", planning)
        self.assertIn("BuildMaskTopGrasp(", planning)
        self.assertIn(
            "Recovered low-profile top-grasp geometry", planning)
        self.assertNotIn(
            "grasp_strategy_ == GraspStrategy::SIDE &&\n"
            "            last_valid_geometry_available_",
            planning,
        )
        self.assertIn(
            "candidate_geometry.table.inlier_count >\n"
            "                result.geometry.table.inlier_count",
            self.geometry,
        )

    def test_side_approach_uses_cartesian_waypoints(self):
        validate = _function_body(
            self.executor, "GraspResult GraspExecutor::ValidateGraspPoses")
        self.assertIn(
            "elevated_pre_grasp_pose.z = entry_clearance_z_m",
            validate,
        )
        self.assertIn(
            "elevated_pre_grasp_pose, pre_grasp_pose",
            validate,
        )
        self.assertIn(
            "BuildSideCartesianPath(\n"
            "                pre_grasp_pose, grasp_pose",
            validate,
        )
        self.assertIn(
            "BuildSideCartesianPath(\n"
            "                grasp_pose, retreat_pose",
            validate,
        )
        self.assertIn(
            "const Pose3D poses[] = {pre_grasp_pose, grasp_pose}",
            validate,
        )
        self.assertIn("SolveIKConstrained(", validate)
        self.assertNotIn("SolveIKFast(", validate)

        cached_path = _function_body(
            self.executor, "bool GraspExecutor::TakeValidatedSidePath")
        self.assertIn("match_index", cached_path)
        self.assertIn("consumed_count", cached_path)

        move_side = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToPoseSide")
        self.assertIn("validated_side_path_index_ = 1", move_side)
        self.assertIn(
            "BuildSideCartesianPath(\n"
            "            current_pose, pose",
            move_side,
        )

    def test_side_waypoints_are_streamed_without_intermediate_stops(self):
        execute = _function_body(
            self.executor,
            "GraspResult GraspExecutor::ExecuteContinuousJointPath",
        )
        self.assertIn("kStreamIntervalMs", execute)
        self.assertIn("kStreamVelocityScale", execute)
        self.assertIn("ClampJointsToLimits", execute)
        self.assertEqual(execute.count("WaitMotionDone("), 1)

        lift = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToSideLift")
        self.assertIn("TakeValidatedSidePath({lift_pose}", lift)
        self.assertIn("current_pose, retreat_pose", lift)
        self.assertIn("kLoadedJointToleranceRad = 0.080f", lift)
        self.assertIn("minimum_safe_z", lift)
        self.assertIn("planar_error", lift)
        self.assertIn("safely_lifted", lift)
        self.assertIn(
            "despite loaded joint settling timeout",
            lift,
        )

    def test_side_grasp_safety_controls_are_configurable(self):
        top = self.config["grasp"]["top"]
        side = self.config["grasp"]["side"]
        workspace = self.config["grasp"]["workspace"]
        manipulator = self.config["manipulator"]
        self.assertEqual(side["approach_distance_m"], 0.03)
        self.assertIn("pregrasp_min_x_m", side)
        self.assertIn("gripper_offset_m", side)
        self.assertIn("grasp_point_x_ratio", top)
        self.assertEqual(side["initial_lift_m"], 0.05)
        self.assertLess(workspace["z_min"], workspace["z_max"])
        self.assertGreaterEqual(workspace["z_min"], -0.05)
        self.assertNotEqual(
            manipulator["observe_joints"],
            manipulator["side_ready_joints"],
        )
        self.assertEqual(len(manipulator["side_ready_joints"]), 5)
        self.assertEqual(len(manipulator["joint_limits"]), 5)

    def test_collision_avoidance_bounds_gravity_drift(self):
        self.assertIn("ClampJointsToLimits", self.executor)
        self.assertIn("bounded_current_joints", self.executor)
        self.assertIn(
            "std::vector<float> step1_joints = bounded_current_joints",
            self.executor,
        )

    def test_small_base_correction_does_not_expand_to_collision_path(self):
        self.assertIn("kMinimumBaseSweepRad", self.executor)
        self.assertIn(
            "std::fabs(target_j0 - current_j0) <= kMinimumBaseSweepRad",
            self.executor,
        )

    def test_side_grasp_waypoints_keep_cartesian_correction(self):
        self.assertIn(
            "config_.side_waypoint_joint_tolerance_rad",
            self.executor,
        )
        self.assertIn("Cartesian correction remains active", self.executor)
        self.assertIn("CorrectSidePose(pose", self.executor)

    def test_side_joint0_sweep_keeps_narrow_planar_error_bound(self):
        sweep = _function_body(
            self.executor, "GraspResult GraspExecutor::PlanSideJoint0Sweep")
        self.assertIn("kMaximumPlanarErrorM = 0.050f", sweep)
        self.assertIn("planar_error > kMaximumPlanarErrorM", sweep)
        self.assertIn("arm_path_safety_->CheckPath", sweep)

    def test_side_ik_keeps_bounded_orientation_error(self):
        solve = _function_body(
            self.executor, "GraspResult GraspExecutor::SolveIKSide")
        self.assertIn("kMaxPositionErrorM = 0.004", solve)
        self.assertIn("kMaxApproachErrorDeg = 25.0", solve)
        self.assertIn("kMaxOpeningErrorDeg = 12.0", solve)
        self.assertIn("arm_path_safety_->CheckPath", solve)

    def test_side_pose_correction_avoids_tolerance_edge_jitter(self):
        correction = _function_body(
            self.executor, "GraspResult GraspExecutor::CorrectSidePose")
        self.assertIn("kCorrectionToleranceRatio", correction)
        self.assertIn("error <= correction_tolerance", correction)
        self.assertIn(
            "error <= config_.side_pose_position_tolerance", correction)
        self.assertNotIn(
            "config_.pose_position_tolerance", correction)
        self.assertIn("pose.x - actual_pose.x", correction)
        self.assertIn("pose.y - actual_pose.y", correction)
        self.assertIn("pose.z - actual_pose.z", correction)
        self.assertIn("{correction_pose}", correction)

    def test_top_and_side_pose_tolerances_are_isolated(self):
        self.assertIn(
            "float pose_position_tolerance = 0.03f", self.header)
        self.assertIn(
            "float side_pose_position_tolerance = 0.005f", self.header)
        verify = _function_body(
            self.executor, "bool GraspExecutor::VerifyPoseReached")
        self.assertIn("config_.pose_position_tolerance", verify)
        correction = _function_body(
            self.executor, "GraspResult GraspExecutor::CorrectSidePose")
        self.assertIn("config_.side_pose_position_tolerance", correction)

        move_to_grasp = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToGrasp")
        side_move_index = move_to_grasp.find("MoveToPoseSide")
        record_index = move_to_grasp.find(
            'RecordResult(GraspResult::SUCCESS, "move_to_grasp")',
            side_move_index,
        )
        self.assertGreaterEqual(side_move_index, 0)
        self.assertGreater(record_index, side_move_index)

    def test_place_uses_independent_joint_tolerance(self):
        self.assertIn(
            "float place_joint_tolerance_rad = 0.080f", self.header)
        move_to_place = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToPlace")
        self.assertIn(
            "WaitMotionDone(-1, config_.place_joint_tolerance_rad)",
            move_to_place)


if __name__ == "__main__":
    unittest.main()
