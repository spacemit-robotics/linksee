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
EXECUTOR_GRIPPER_CPP = ROOT / "src" / "grasp_executor_gripper.cpp"
EXECUTOR_RUNTIME_CPP = ROOT / "src" / "grasp_executor_runtime.cpp"
EXECUTOR_HEADER = ROOT / "include" / "grasp_executor.h"
GEOMETRY_HEADER = ROOT / "include" / "grasp_geometry.h"
PIPELINE_CPP = ROOT / "src" / "grasp_pipeline.cpp"
PIPELINE_EXECUTION_CPP = ROOT / "src" / "grasp_pipeline_execution.cpp"
PIPELINE_DEBUG_WRITER_CPP = ROOT / "src" / "pipeline_debug_writer.cpp"
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
        cls.executor = (
            EXECUTOR_CPP.read_text(encoding="utf-8")
            + EXECUTOR_GRIPPER_CPP.read_text(encoding="utf-8")
            + EXECUTOR_RUNTIME_CPP.read_text(encoding="utf-8")
        )
        cls.header = EXECUTOR_HEADER.read_text(encoding="utf-8")
        cls.geometry_header = GEOMETRY_HEADER.read_text(encoding="utf-8")
        cls.pipeline = (
            PIPELINE_CPP.read_text(encoding="utf-8")
            + PIPELINE_EXECUTION_CPP.read_text(encoding="utf-8")
        )
        cls.pipeline_debug_writer = PIPELINE_DEBUG_WRITER_CPP.read_text(
            encoding="utf-8"
        )
        cls.geometry = GEOMETRY_CPP.read_text(encoding="utf-8")
        cls.main = MAIN_CPP.read_text(encoding="utf-8")
        cls.debug_localize = DEBUG_LOCALIZE_CPP.read_text(encoding="utf-8")
        cls.config = yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8"))

    def test_empty_closed_baseline_is_used(self):
        self.assertIn("CaptureEmptyClosedPosition()", self.executor)
        self.assertIn("gripper_baseline_", self.executor)
        self.assertIn("gripper_empty_position_margin", self.executor)
        baseline = _function_body(
            self.executor,
            "void GraspExecutor::CaptureEmptyClosedPosition",
        )
        release = _function_body(
            self.executor, "GraspResult GraspExecutor::ReleaseObject")
        self.assertIn("std::vector<GripperFeedbackSample>", baseline)
        self.assertIn("EstimateGripperBaseline(samples)", baseline)
        self.assertIn("position_median", baseline)
        self.assertIn("position_mad", baseline)
        self.assertIn("load_median", baseline)
        self.assertIn("load_mad", baseline)
        self.assertIn("gripper_hold_load_threshold", baseline)
        self.assertIn(
            "gripper_baseline_ = GripperBaseline{}", release)

    def test_grasp_confirmation_uses_sustained_contact_feedback(self):
        self.assertNotIn("Grasp inferred from sustained load", self.executor)
        self.assertNotIn("state_or_baseline_confirmed", self.executor)
        self.assertIn("EvaluateGripperHolding(", self.executor)
        self.assertIn("GripperHoldingConfig holding_config", self.executor)
        self.assertIn(
            "GripperHoldingResult::HOLDING", self.executor)
        self.assertIn(
            "state == GRASP_STATE_HOLDING",
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
        self.assertIn(
            "possible-object state for safe recovery",
            self.executor,
        )
        self.assertNotIn(
            'GraspResult::EMPTY, "verify_grasp_after_lift",\n'
            '            "holding evidence was inconclusive after lift"',
            self.executor,
        )

    def test_gripper_diagnostics_include_baseline_and_joint_evidence(self):
        writer = self.pipeline_debug_writer
        self.assertIn('\\"decision\\"', writer)
        self.assertIn('\\"opening_count\\"', writer)
        self.assertIn('\\"contact_count\\"', writer)
        self.assertIn('\\"empty_count\\"', writer)
        self.assertIn('\\"baseline_sample_count\\"', writer)
        self.assertIn('\\"empty_closed_position_mad\\"', writer)
        self.assertIn('\\"empty_closed_load\\"', writer)
        self.assertIn('\\"empty_closed_load_mad\\"', writer)

    def test_release_opening_uses_position_control(self):
        release = _function_body(
            self.executor, "GraspResult GraspExecutor::ReleaseObject")
        self.assertIn("grasp_set_position(", release)
        self.assertIn("config_.place_release_open", release)

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

    def test_side_entry_clearance_is_loaded_as_an_independent_parameter(self):
        self.assertIn("side_entry_clearance_m", self.geometry_header)
        self.assertIn('side["entry_clearance_m"]', self.main)
        self.assertIn('side["entry_clearance_m"]', self.debug_localize)
        self.assertNotIn('geometry["strategy"]', self.debug_localize)

    def test_top_grasp_uses_prevalidated_cartesian_joint_path(self):
        validate = _function_body(
            self.executor, "GraspResult GraspExecutor::ValidateGraspPoses")
        plan = _function_body(
            self.executor, "GraspResult GraspExecutor::PlanTopJointPath")
        solve = _function_body(
            self.executor, "GraspResult GraspExecutor::SolveIKConstrained")
        execute = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveAlongValidatedTopPath",
        )
        self.assertIn("top grasp descent and lift must remain vertical",
                      validate)
        self.assertIn("kTopCartesianStepM", validate)
        self.assertIn("PlanTopJointPath(", validate)
        self.assertIn("validated_top_joint_path_", validate)
        self.assertIn("SolveIKConstrained(", plan)
        self.assertIn("ApplyWristYaw(", plan)
        self.assertIn("BuildCollisionSafeJointPath(", plan)
        self.assertIn("ValidateJointPathSafety(", plan)
        self.assertIn("const int arm_joint_count", solve)
        self.assertIn("j < arm_joint_count", solve)
        self.assertIn("ExecuteContinuousJointPath(", execute)
        self.assertIn("validated_top_path_index_", execute)
        self.assertIn(
            "accepted from measured pose despite joint",
            execute,
        )
        self.assertIn(
            "settling timeout",
            execute,
        )
        self.assertIn(
            "move_result == GraspResult::TIMEOUT",
            execute,
        )
        self.assertIn("VerifyPoseReached(action, target_pose)", execute)
        self.assertIn(
            "kTopGraspJointToleranceRad = 0.020f",
            self.executor,
        )
        self.assertIn("last_motion_wait_detail_", execute)
        self.assertIn(
            "kTopLiftJointToleranceRad = 0.060f",
            self.executor,
        )
        move_to_grasp = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToGrasp")
        lift = _function_body(
            self.executor, "GraspResult GraspExecutor::LiftFromGrasp")
        self.assertIn("kTopGraspJointToleranceRad", move_to_grasp)
        self.assertIn("kTopLiftJointToleranceRad", lift)

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
        self.assertNotIn("kFourthJointLeadProgress", coordinated)

    def test_home_motion_skips_only_when_current_pose_is_already_home(self):
        home = _function_body(
            self.executor, "GraspResult GraspExecutor::MoveToHome")
        current_index = home.find("GetCurrentJoints(current_joints)")
        no_op_index = home.find("already within home tolerance")
        safe_move_index = home.find("MoveToJointsCollisionSafe")
        self.assertGreaterEqual(current_index, 0)
        self.assertGreaterEqual(no_op_index, 0)
        self.assertGreaterEqual(safe_move_index, 0)
        self.assertLess(current_index, no_op_index)
        self.assertLess(no_op_index, safe_move_index)
        self.assertIn(
            "config_.home_joints, true",
            " ".join(home.split()),
        )
        self.assertIn("kHomeNoOpToleranceRad = 0.12f", home)

    def test_unused_direct_cartesian_motion_interfaces_are_removed(self):
        self.assertNotIn("GraspExecutor::MoveToPose(", self.executor)
        self.assertNotIn("GraspExecutor::MoveLinear(", self.executor)
        self.assertNotIn("manip_move_target(", self.executor)
        self.assertNotIn("manip_move_line(", self.executor)

    def test_top_ik_uses_five_arm_joint_seed_deterministically(self):
        constrained = _function_body(
            self.executor, "GraspResult GraspExecutor::SolveIKConstrained")
        self.assertIn("std::mt19937 rng(0x5A17u)", constrained)
        self.assertIn(
            "static_cast<size_t>(arm_joint_count)",
            constrained,
        )
        self.assertIn("trial <= 1", constrained)
        self.assertIn("config_.observe_joints", constrained)
        self.assertIn(
            "config_.collision_avoidance.shoulder_threshold",
            constrained,
        )
        self.assertNotIn(
            "seed_joints->size() >= static_cast<size_t>(n_joints)",
            constrained,
        )
        self.assertIn("candidate_validator(candidate)", constrained)

        top_path = _function_body(
            self.executor, "GraspResult GraspExecutor::PlanTopJointPath")
        self.assertIn("const auto candidate_validator", top_path)
        self.assertIn("BuildCollisionSafeJointPath(", top_path)
        self.assertIn("ValidateJointPathSafety(", top_path)
        self.assertIn("PosesMatch(poses[index]", top_path)
        self.assertIn(
            "solved_joints = joint_path[previous_index]",
            top_path,
        )
        validator_index = top_path.find("const auto candidate_validator")
        solve_index = top_path.find("SolveIKConstrained(")
        continuity_index = top_path.find(
            "kMaximumWaypointJointDeltaRad",
            validator_index,
        )
        self.assertGreater(continuity_index, validator_index)
        self.assertLess(continuity_index, solve_index)

    def test_body_collision_route_rotates_before_folding_shoulder(self):
        route = _function_body(
            self.executor,
            "GraspResult GraspExecutor::BuildCollisionSafeJointPath",
        )
        self.assertIn("rotate_base[0] = target_j0", route)
        self.assertIn("lift_shoulder[0] = target_j0", route)
        self.assertNotIn("safe_joint0", route)

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
        self.assertIn('"Grasp action timeout"', grasping)
        self.assertIn('"Grasp action failed"', grasping)
        self.assertNotIn('"Gripper close timeout"', grasping)
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
        base_alignment_index = planning.find("PlanMobileBaseAlignment")
        strategy_index = planning.find(
            "observation_strategy_selected_ = true",
            base_alignment_index,
        )
        observe_index = planning.find(
            "PipelineState::OBSERVING",
            strategy_index,
        )
        self.assertGreaterEqual(strategy_index, 0)
        self.assertGreaterEqual(observe_index, 0)
        self.assertGreaterEqual(base_alignment_index, 0)
        self.assertLess(base_alignment_index, strategy_index)
        self.assertLess(strategy_index, observe_index)
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
        self.assertIn("is temporarily unavailable; waiting at the", detecting)
        self.assertNotIn("Waiting for next", detecting)
        target_index = restart.find(
            "const std::string target = auto_loop_target_label_")
        reset_index = restart.find("ResetTaskState()")
        restore_index = restart.find("target_label_ = target")
        timing_index = restart.find("BeginTaskTiming()")
        detecting_index = restart.find("PipelineState::DETECTING")
        self.assertLess(target_index, reset_index)
        self.assertLess(reset_index, restore_index)
        self.assertLess(restore_index, timing_index)
        self.assertLess(timing_index, detecting_index)
        self.assertNotIn(
            "const GraspStrategy grasp_strategy = grasp_strategy_", restart)
        self.assertNotIn("grasp_strategy_ = grasp_strategy", restart)
        self.assertIn("[Loop] START iteration=", restart)
        self.assertIn("Loop: detecting stable target", restart)
        self.assertNotIn("auto_loop_rearm_required_", restart)
        self.assertNotIn("auto_loop_rearm_required_", detecting)
        self.assertNotIn("kAutoLoopRearmAbsentFrames", detecting)
        self.assertNotIn("previous target cleared", detecting)

    def test_candidate_score_records_ik_margin_after_validation(self):
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        self.assertIn(
            "diagnostics.validation_min_joint_margin_rad", planning)
        self.assertIn("candidate.ik_margin_rad", planning)
        self.assertIn("candidate.score += 0.10f", planning)

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
        resolve_top_support = _function_body(
            self.pipeline,
            "bool GraspPipeline::ResolveTopSupportPlane",
        )

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
        self.assertIn("BuildWorkspaceSupportPlane(", resolve_top_support)
        self.assertIn(
            '"3D grasp safety validation failed: support surface is invalid"',
            planning,
        )
        self.assertIn(
            "candidate_geometry.table.inlier_count >\n"
            "                result.geometry.table.inlier_count",
            self.geometry,
        )

    def test_geometry_retry_limit_ends_the_current_iteration(self):
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")
        error_index = planning.find(
            '"3D grasp geometry failed: "')
        self.assertGreaterEqual(error_index, 0)
        self.assertNotIn(
            '"No graspable 3D target; waiting at the observation "',
            planning,
        )
        self.assertNotIn(
            "config_.auto_loop &&\n"
            "            !motion_geometry_confirmation_pending_",
            planning,
        )

    def test_locked_observation_strategy_survives_geometry_noise(self):
        planning = _function_body(
            self.pipeline, "void GraspPipeline::HandlePlanning")

        unavailable_index = planning.find("if (!preferred_candidate)")
        retry_index = planning.find(
            "strategy temporarily unavailable; retaining")
        error_index = planning.find(
            "SetState(PipelineState::ERROR, message.str())",
            unavailable_index,
        )

        self.assertGreaterEqual(unavailable_index, 0)
        self.assertGreater(retry_index, unavailable_index)
        self.assertGreater(error_index, retry_index)
        self.assertIn(
            "if (geometry_retry_count_ < kMaxGeometryAttempts)",
            planning,
        )
        self.assertNotIn(
            "strategy remains unavailable; retaining the", planning)
        self.assertIn("target_track_ = TargetTrack{}", planning)
        reset_index = planning.find("geometry_retry_count_ = 0;")
        selection_index = planning.find(
            "Observation strategy selected after base ")
        self.assertGreaterEqual(reset_index, 0)
        self.assertGreater(selection_index, reset_index)

    def test_strategy_selection_waits_for_stationary_visual_target(self):
        detecting = _function_body(
            self.pipeline, "void GraspPipeline::HandleDetecting")

        stationary_index = detecting.find("AreTargetTracksStationary(")
        planning_index = detecting.find(
            "PipelineState::PLANNING", stationary_index)
        self.assertGreaterEqual(stationary_index, 0)
        self.assertGreater(planning_index, stationary_index)
        self.assertIn("target_stationary_confirmed_", detecting)
        self.assertIn("config_.detect_stable_frames", detecting)
        self.assertIn("Target moved; waiting for it to settle", detecting)
        self.assertNotIn("ConfirmStrategyGeometry(", self.pipeline)

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
        self.assertIn("kTopVerticalToleranceM", validate)
        self.assertIn("PlanTopJointPath(", validate)

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
        self.assertIn("ValidateJointPathSafety(", execute)
        self.assertNotIn("ClampJointsToLimits", execute)
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

    def test_side_staging_and_precise_entry_use_separate_tolerances(self):
        pre_grasp = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveToSidePreGrasp",
        )
        self.assertIn(
            "kSideStagingJointToleranceRad = 0.080f",
            self.executor,
        )
        staging_index = pre_grasp.find(
            "{staging_joints, sweep_joints}")
        staging_tolerance_index = pre_grasp.find(
            "kSideStagingJointToleranceRad", staging_index)
        entry_index = pre_grasp.find(
            "{validated_side_entry_joints_}")
        entry_tolerance_index = pre_grasp.find(
            "config_.side_waypoint_joint_tolerance_rad",
            entry_index,
        )
        self.assertGreaterEqual(staging_index, 0)
        self.assertGreater(staging_tolerance_index, staging_index)
        self.assertGreater(entry_index, staging_tolerance_index)
        self.assertGreater(entry_tolerance_index, entry_index)
        self.assertIn(
            "side staging and joint0 sweep completion timeout",
            pre_grasp,
        )
        self.assertIn(
            "elevated side entry completion timeout",
            pre_grasp,
        )

    def test_side_entry_timeout_requires_a_measured_safe_pose(self):
        pre_grasp = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveToSidePreGrasp",
        )
        self.assertIn(
            "kMaximumEntryHeightShortfallM = 0.010f",
            pre_grasp,
        )
        self.assertIn(
            "kMaximumEntryPlanarErrorM = 0.030f",
            pre_grasp,
        )
        self.assertIn(
            "result == GraspResult::TIMEOUT",
            pre_grasp,
        )
        self.assertIn("GetCurrentPose(actual_pose)", pre_grasp)
        self.assertIn("actual_pose.z >= minimum_safe_z", pre_grasp)
        self.assertIn(
            "planar_error <= kMaximumEntryPlanarErrorM",
            pre_grasp,
        )
        self.assertIn(
            "side elevated entry accepted",
            pre_grasp,
        )
        self.assertIn(
            "for Cartesian correction despite joint",
            pre_grasp,
        )
        correction_index = pre_grasp.find(
            "CorrectSidePose(")
        safe_acceptance_index = pre_grasp.find(
            "side elevated entry accepted")
        self.assertGreater(correction_index, safe_acceptance_index)

    def test_side_grasp_safety_controls_are_configurable(self):
        top = self.config["grasp"]["top"]
        side = self.config["grasp"]["side"]
        workspace = self.config["grasp"]["workspace"]
        manipulator = self.config["manipulator"]
        self.assertEqual(side["approach_distance_m"], 0.02)
        self.assertEqual(side["entry_clearance_m"], 0.03)
        self.assertIn("pregrasp_min_x_m", side)
        self.assertEqual(side["gripper_offset_m"], 0.01)
        self.assertEqual(side["grasp_forward_offset_m"], 0.02)
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

    def test_predefined_poses_are_inside_joint_limits(self):
        manipulator = self.config["manipulator"]
        limits = {
            item["joint"]: (item["min"], item["max"])
            for item in manipulator["joint_limits"]
        }
        poses = {
            "home_joints": manipulator["home_joints"],
            "observe_joints": manipulator["observe_joints"],
            "side_ready_joints": manipulator["side_ready_joints"],
            "place_joints": self.config["place"]["place_joints"],
        }
        for pose_name, joints in poses.items():
            self.assertEqual(len(joints), len(limits), pose_name)
            for joint, value in enumerate(joints):
                lower, upper = limits[joint]
                self.assertGreaterEqual(value, lower, f"{pose_name}[{joint}]")
                self.assertLessEqual(value, upper, f"{pose_name}[{joint}]")

    def test_collision_avoidance_bounds_gravity_drift(self):
        move = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveToJointsCollisionSafe",
        )
        build = _function_body(
            self.executor,
            "GraspResult GraspExecutor::BuildCollisionSafeJointPath",
        )
        unavailable_index = move.find(
            "if (!GetCurrentJoints(current_joints))")
        path_index = move.find("BuildCollisionSafeJointPath(")
        validation_index = build.find("ValidateJointPathSafety(")
        command_index = move.find("MoveToJoints(path[index])")
        self.assertGreaterEqual(unavailable_index, 0)
        self.assertLess(unavailable_index, path_index)
        self.assertGreaterEqual(validation_index, 0)
        self.assertGreaterEqual(command_index, 0)
        self.assertIn("ClampJointsToLimits(", build)
        self.assertNotIn("bounded_current_joints != start_joints", build)
        self.assertNotIn(
            "return MoveToJoints(target_joints)", move[:path_index])

    def test_joint_commands_only_tolerate_motion_back_into_limits(self):
        move = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveToJoints("
            "const std::vector<float>& joints)")
        correction = _function_body(
            self.executor, "bool JointCommandMovesTowardLimits")
        self.assertIn("JointCommandMovesTowardLimits(", move)
        self.assertIn(
            "kMeasuredJointLimitToleranceRad = 0.05f", move)
        self.assertIn('"move_to_joints"', move)
        self.assertIn("target >= measured", correction)
        self.assertIn("target <= measured", correction)
        self.assertIn(
            "measured >= limit.min_rad - measured_tolerance_rad",
            correction,
        )
        self.assertIn(
            "measured <= limit.max_rad + measured_tolerance_rad",
            correction,
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
        self.assertIn("kMaxPositionErrorM = 0.006", solve)
        self.assertIn("kMaxApproachErrorDeg = 30.0", solve)
        self.assertIn("kMaxOpeningErrorDeg = 15.0", solve)
        self.assertIn("kOrientationBoundarySlackDeg = 0.5", solve)
        self.assertIn(
            "kMaxOpeningErrorDeg + kOrientationBoundarySlackDeg",
            solve,
        )
        self.assertIn("arm_path_safety_->CheckPath", solve)

    def test_soft_timing_limits_do_not_reject_safe_motion(self):
        self.assertIn(
            "EffectiveMotionTargetToleranceRad",
            self.executor,
        )
        self.assertIn(
            "settled boundary slack",
            self.executor,
        )
        self.assertIn(
            'within_perception_budget ? "SUCCESS" : "OVERRUN"',
            self.pipeline,
        )
        self.assertIn(
            "continuing because workspace, IK and ",
            self.pipeline,
        )
        self.assertNotIn(
            "Perception and planning exceeded the configured budget",
            self.pipeline,
        )
        self.assertIn(
            "kMinimumCandidateValidationTimeoutMs = 750",
            self.pipeline,
        )
        self.assertNotIn(
            "planning deadline exceeded",
            self.pipeline,
        )

    def test_side_pose_correction_avoids_tolerance_edge_jitter(self):
        correction = _function_body(
            self.executor, "GraspResult GraspExecutor::CorrectSidePose")
        self.assertIn(
            "kSideCorrectionJointToleranceRad = 0.080f",
            self.executor,
        )
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
        self.assertIn(
            "kSideCorrectionJointToleranceRad", correction)
        self.assertIn("side correction ", correction)
        self.assertIn("target_position_error=", correction)

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
        self.assertRegex(
            move_to_place,
            r"WaitMotionDone\(\s*-1,\s*"
            r"config_\.place_joint_tolerance_rad\)")

    def test_top_path_accepts_settled_cartesian_target_without_timeout(self):
        move = _function_body(
            self.executor,
            "GraspResult GraspExecutor::MoveAlongValidatedTopPath",
        )
        wait = _function_body(
            self.executor,
            "GraspResult GraspExecutor::WaitMotionDone",
        )
        self.assertIn(
            "VerifyPoseReached(action, target_pose)",
            move,
        )
        self.assertIn(
            "stable_counter == kStableCount",
            wait,
        )
        self.assertIn(
            "settled_acceptance()",
            wait,
        )
        self.assertIn(
            "motion_state=REACHED_BY_CARTESIAN_POSE",
            wait,
        )


if __name__ == "__main__":
    unittest.main()
