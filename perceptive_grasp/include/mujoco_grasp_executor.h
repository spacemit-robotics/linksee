/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_grasp_executor.h
 * @brief MuJoCo-backed grasp executor for PC simulation.
 */

#ifndef MUJOCO_GRASP_EXECUTOR_H
#define MUJOCO_GRASP_EXECUTOR_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "grasp_executor.h"

namespace perceptive_grasp {

class MujocoGraspExecutor : public GraspExecutor {
public:
    explicit MujocoGraspExecutor(const ExecutorConfig& config);
    ~MujocoGraspExecutor() override;

    void SetVisualStepCallback(std::function<void()> callback);
    /** Reset the world and randomize the pickup layout explicitly. */
    void ResetScene();
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
    GraspResult ValidateGraspPoses(const Pose3D& pre_grasp_pose,
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
    ExecutorDiagnostics GetDiagnostics() const override { return diagnostics_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ExecutorDiagnostics diagnostics_;

    void RecordResult(GraspResult result, const std::string& action,
                        const std::string& detail = "");
    GraspResult MoveToJointsSim(const std::vector<float>& joints,
                                const char* action,
                                bool allow_target_contact = false);
    GraspResult MoveToPoseSim(const Pose3D& pose,
                            const char* action,
                            bool constrain_orientation = true,
                            bool allow_target_contact = false);
    GraspResult SetGripperCtrl(float ctrl, const char* action);
};

}  // namespace perceptive_grasp

#endif  // MUJOCO_GRASP_EXECUTOR_H
