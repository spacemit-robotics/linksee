/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_grasp_executor.cpp
 * @brief Remote MuJoCo grasp executor client.
 */

#include "remote_mujoco_grasp_executor.h"

#include <cmath>
#include <iostream>

namespace perceptive_grasp {
namespace {

static void WritePose(remote_mujoco::BufferWriter* writer,
                    const Pose3D& pose) {
    writer->WriteF32(pose.x);
    writer->WriteF32(pose.y);
    writer->WriteF32(pose.z);
    writer->WriteF32(pose.qw);
    writer->WriteF32(pose.qx);
    writer->WriteF32(pose.qy);
    writer->WriteF32(pose.qz);
}

static bool ReadPose(remote_mujoco::BufferReader* reader, Pose3D* pose) {
    return reader->ReadF32(&pose->x) &&
        reader->ReadF32(&pose->y) &&
        reader->ReadF32(&pose->z) &&
        reader->ReadF32(&pose->qw) &&
        reader->ReadF32(&pose->qx) &&
        reader->ReadF32(&pose->qy) &&
        reader->ReadF32(&pose->qz);
}

static void WriteBool(remote_mujoco::BufferWriter* writer, bool value) {
    writer->WriteU8(value ? 1 : 0);
}

}  // namespace

RemoteMujocoGraspExecutor::RemoteMujocoGraspExecutor(
    const ExecutorConfig& config)
    : GraspExecutor(config) {}

RemoteMujocoGraspExecutor::~RemoteMujocoGraspExecutor() = default;

void RemoteMujocoGraspExecutor::SetTargetLabel(
    const std::string& target_label) {
    remote_mujoco::BufferWriter writer;
    writer.WriteString(target_label);
    SendAction(
        remote_mujoco::Command::SET_TARGET_LABEL,
        writer.Data(), "set_target_label");
}

GraspResult RemoteMujocoGraspExecutor::SendAction(
    remote_mujoco::Command command,
    const std::vector<std::uint8_t>& payload,
    const char* action) {
    remote_mujoco::Response response;
    std::string error;
    if (!remote_mujoco::SendRequest(
            config_.remote_mujoco.host,
            config_.remote_mujoco.port,
            config_.remote_mujoco.timeout_ms,
            command, payload, &response, &error)) {
        diagnostics_.last_result = GraspResult::MOVE_FAILED;
        diagnostics_.last_action = action;
        diagnostics_.last_detail = error;
        return GraspResult::MOVE_FAILED;
    }
    const auto result = static_cast<GraspResult>(response.result);
    diagnostics_.last_result = result;
    diagnostics_.last_action = action;
    diagnostics_.last_detail = response.detail;
    return response.ok ? result : GraspResult::MOVE_FAILED;
}

bool RemoteMujocoGraspExecutor::Init() {
    Pose3D pose;
    if (!GetCurrentPose(pose)) {
        std::cerr << "[RemoteMujocoExecutor] init failed: "
                << diagnostics_.last_detail << std::endl;
        return false;
    }
    std::cout << "[RemoteMujocoExecutor] Initialized: "
            << config_.remote_mujoco.host << ":"
            << config_.remote_mujoco.port << std::endl;
    return true;
}

GraspResult RemoteMujocoGraspExecutor::MoveToObserve() {
    return SendAction(
        remote_mujoco::Command::MOVE_TO_OBSERVE, {}, "move_to_observe");
}

GraspResult RemoteMujocoGraspExecutor::MoveToSideObserve() {
    return SendAction(
        remote_mujoco::Command::MOVE_TO_SIDE_OBSERVE, {},
        "move_to_side_observe");
}

GraspResult RemoteMujocoGraspExecutor::MoveToHome() {
    return SendAction(
        remote_mujoco::Command::MOVE_TO_HOME, {}, "move_to_home");
}

GraspResult RemoteMujocoGraspExecutor::MoveToPreGrasp(
    const Pose3D& pre_grasp_pose, float grasp_yaw_rad,
    bool use_top_constraints) {
    remote_mujoco::BufferWriter writer;
    WritePose(&writer, pre_grasp_pose);
    writer.WriteF32(grasp_yaw_rad);
    WriteBool(&writer, use_top_constraints);
    return SendAction(
        remote_mujoco::Command::MOVE_TO_PRE_GRASP,
        writer.Data(), "move_to_pre_grasp");
}

GraspResult RemoteMujocoGraspExecutor::OpenGripperForGrasp(
    float minimum_opening) {
    remote_mujoco::BufferWriter writer;
    writer.WriteF32(minimum_opening);
    return SendAction(
        remote_mujoco::Command::OPEN_GRIPPER,
        writer.Data(), "open_gripper");
}

GraspResult RemoteMujocoGraspExecutor::MoveToGrasp(
    const Pose3D& grasp_pose, float grasp_yaw_rad,
    bool use_top_constraints) {
    remote_mujoco::BufferWriter writer;
    WritePose(&writer, grasp_pose);
    writer.WriteF32(grasp_yaw_rad);
    WriteBool(&writer, use_top_constraints);
    return SendAction(
        remote_mujoco::Command::MOVE_TO_GRASP,
        writer.Data(), "move_to_grasp");
}

GraspResult RemoteMujocoGraspExecutor::CloseGripperAndCheck() {
    return SendAction(
        remote_mujoco::Command::CLOSE_GRIPPER_AND_CHECK, {},
        "close_gripper_and_check");
}

GraspResult RemoteMujocoGraspExecutor::LiftFromGrasp(
    const Pose3D& retreat_pose, const Pose3D& lift_pose,
    float grasp_yaw_rad, bool use_top_constraints) {
    remote_mujoco::BufferWriter writer;
    WritePose(&writer, retreat_pose);
    WritePose(&writer, lift_pose);
    writer.WriteF32(grasp_yaw_rad);
    WriteBool(&writer, use_top_constraints);
    return SendAction(
        remote_mujoco::Command::LIFT_FROM_GRASP,
        writer.Data(), "lift_from_grasp");
}

GraspResult RemoteMujocoGraspExecutor::ValidateGraspPoses(
    const Pose3D& pre_grasp_pose,
    const Pose3D& grasp_pose,
    const Pose3D& retreat_pose,
    const Pose3D& lift_pose,
    float entry_clearance_z_m,
    float grasp_yaw_rad,
    bool use_top_constraints,
    int timeout_ms,
    std::string* detail) {
    remote_mujoco::BufferWriter writer;
    WritePose(&writer, pre_grasp_pose);
    WritePose(&writer, grasp_pose);
    WritePose(&writer, retreat_pose);
    WritePose(&writer, lift_pose);
    writer.WriteF32(entry_clearance_z_m);
    writer.WriteF32(grasp_yaw_rad);
    WriteBool(&writer, use_top_constraints);
    writer.WriteI32(timeout_ms);
    const GraspResult result = SendAction(
        remote_mujoco::Command::VALIDATE_GRASP_POSES,
        writer.Data(), "validate_grasp_poses");
    if (detail) *detail = diagnostics_.last_detail;
    return result;
}

void RemoteMujocoGraspExecutor::SetSupportPlane(
    const SupportPlane& support_plane) {
    support_plane_ = support_plane;
    remote_mujoco::BufferWriter writer;
    writer.WriteF32(support_plane.normal_x);
    writer.WriteF32(support_plane.normal_y);
    writer.WriteF32(support_plane.normal_z);
    writer.WriteF32(support_plane.d);
    writer.WriteU8(support_plane.valid ? 1 : 0);
    writer.WriteF32(support_plane.min_x);
    writer.WriteF32(support_plane.max_x);
    writer.WriteF32(support_plane.min_y);
    writer.WriteF32(support_plane.max_y);
    writer.WriteU8(support_plane.bounds_valid ? 1 : 0);
    SendAction(
        remote_mujoco::Command::SET_SUPPORT_PLANE,
        writer.Data(), "set_support_plane");
}

GraspResult RemoteMujocoGraspExecutor::MoveToPlace() {
    return SendAction(
        remote_mujoco::Command::MOVE_TO_PLACE, {}, "move_to_place");
}

GraspResult RemoteMujocoGraspExecutor::ReleaseObject() {
    return SendAction(
        remote_mujoco::Command::RELEASE_OBJECT, {}, "release_object");
}

GraspResult RemoteMujocoGraspExecutor::CloseGripper() {
    return SendAction(
        remote_mujoco::Command::CLOSE_GRIPPER, {}, "close_gripper");
}

GraspResult RemoteMujocoGraspExecutor::ExecuteGrasp(
    const Pose3D& grasp_pose,
    const Pose3D& pre_grasp_pose,
    float grasp_yaw_rad) {
    GraspResult result = MoveToPreGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    result = OpenGripperForGrasp();
    if (result != GraspResult::SUCCESS) return result;
    result = MoveToGrasp(grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    return CloseGripperAndCheck();
}

GraspResult RemoteMujocoGraspExecutor::ExecutePlace() {
    GraspResult result = MoveToPlace();
    if (result != GraspResult::SUCCESS) return result;
    return ReleaseObject();
}

void RemoteMujocoGraspExecutor::EmergencyStop() {
    SendAction(
        remote_mujoco::Command::EMERGENCY_STOP, {}, "emergency_stop");
}

bool RemoteMujocoGraspExecutor::GetCurrentPose(Pose3D& pose) {
    remote_mujoco::Response response;
    std::string error;
    if (!remote_mujoco::SendRequest(
            config_.remote_mujoco.host,
            config_.remote_mujoco.port,
            config_.remote_mujoco.timeout_ms,
            remote_mujoco::Command::GET_CURRENT_POSE,
            {}, &response, &error)) {
        diagnostics_.last_result = GraspResult::MOVE_FAILED;
        diagnostics_.last_action = "get_current_pose";
        diagnostics_.last_detail = error;
        return false;
    }
    if (!response.ok) {
        diagnostics_.last_result = GraspResult::MOVE_FAILED;
        diagnostics_.last_action = "get_current_pose";
        diagnostics_.last_detail = response.detail;
        return false;
    }
    remote_mujoco::BufferReader reader(response.payload);
    if (!ReadPose(&reader, &pose)) {
        diagnostics_.last_result = GraspResult::MOVE_FAILED;
        diagnostics_.last_action = "get_current_pose";
        diagnostics_.last_detail = "malformed pose response";
        return false;
    }
    diagnostics_.last_result = GraspResult::SUCCESS;
    diagnostics_.last_action = "get_current_pose";
    diagnostics_.last_detail.clear();
    return true;
}

void RemoteMujocoGraspExecutor::Tick(float dt_s) {
    remote_mujoco::BufferWriter writer;
    writer.WriteF32(dt_s);
    SendAction(remote_mujoco::Command::TICK, writer.Data(), "tick");
}

}  // namespace perceptive_grasp
