/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_grasp_executor.h
 * @brief Remote MuJoCo grasp executor client.
 */

#ifndef REMOTE_MUJOCO_GRASP_EXECUTOR_H
#define REMOTE_MUJOCO_GRASP_EXECUTOR_H

#include "grasp_executor.h"
#include "remote_mujoco_protocol.h"

namespace perceptive_grasp {

class RemoteMujocoGraspExecutor : public GraspExecutor {
public:
    explicit RemoteMujocoGraspExecutor(const ExecutorConfig& config);
    ~RemoteMujocoGraspExecutor() override;

    void SetTargetLabel(const std::string& target_label) override;
    bool Init() override;
    GraspResult MoveToObserve() override;
    GraspResult MoveToSideObserve() override;
    GraspResult MoveToHome() override;
    GraspResult MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                float grasp_yaw_rad = NAN,
                                bool use_top_constraints = true) override;
    GraspResult OpenGripperForGrasp(float minimum_opening = NAN) override;
    GraspResult MoveToGrasp(const Pose3D& grasp_pose,
                            float grasp_yaw_rad = NAN,
                            bool use_top_constraints = true) override;
    GraspResult CloseGripperAndCheck() override;
    GraspResult LiftFromGrasp(const Pose3D& retreat_pose,
                            const Pose3D& lift_pose,
                            float grasp_yaw_rad = NAN,
                            bool use_top_constraints = true) override;
    GraspResult ValidateGraspPoses(
        const Pose3D& pre_grasp_pose,
        const Pose3D& grasp_pose,
        const Pose3D& retreat_pose,
        const Pose3D& lift_pose,
        float entry_clearance_z_m,
        float grasp_yaw_rad,
        bool use_top_constraints,
        int timeout_ms,
        std::string* detail) override;
    void SetSupportPlane(const SupportPlane& support_plane) override;
    GraspResult MoveToPlace() override;
    GraspResult ReleaseObject() override;
    GraspResult CloseGripper() override;
    GraspResult ExecuteGrasp(const Pose3D& grasp_pose,
                            const Pose3D& pre_grasp_pose,
                            float grasp_yaw_rad = NAN) override;
    GraspResult ExecutePlace() override;
    void EmergencyStop() override;
    bool GetCurrentPose(Pose3D& pose) override;
    void Tick(float dt_s) override;
    ExecutorDiagnostics GetDiagnostics() const override {
        return diagnostics_;
    }

private:
    GraspResult SendAction(
        remote_mujoco::Command command,
        const std::vector<std::uint8_t>& payload,
        const char* action);

    ExecutorDiagnostics diagnostics_;
    SupportPlane support_plane_;
};

}  // namespace perceptive_grasp

#endif  // REMOTE_MUJOCO_GRASP_EXECUTOR_H
