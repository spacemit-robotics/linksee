/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file mock_executor.h
    * @brief X86 Standalone 模式下的机械臂/夹爪 mock
    *
    * 不连接真实硬件，只打印动作日志。
    * 用于验证 pipeline 逻辑和视觉链路。
    */

#ifndef MOCK_EXECUTOR_H
#define MOCK_EXECUTOR_H

#include "grasp_executor.h"

namespace perceptive_grasp {

/**
    * @brief Mock 执行器 (x86 standalone)
    *
    * 所有运动指令只打印日志，不连接硬件。
    * 抓取永远返回 SUCCESS (模拟成功)。
    */
class MockExecutor : public GraspExecutor {
public:
    explicit MockExecutor(const ExecutorConfig& config);
    ~MockExecutor() override = default;

    bool Init() override;
    GraspResult MoveToObserve() override;
    GraspResult MoveToHome() override;
    GraspResult MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                float grasp_yaw_rad = NAN) override;
    GraspResult OpenGripperForGrasp() override;
    GraspResult MoveToGrasp(const Pose3D& grasp_pose,
                            float grasp_yaw_rad = NAN) override;
    GraspResult CloseGripperAndCheck() override;
    GraspResult LiftFromGrasp(const Pose3D& pre_grasp_pose,
                                float grasp_yaw_rad = NAN) override;
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
    ExecutorDiagnostics diagnostics_;
};

}  // namespace perceptive_grasp

#endif  // MOCK_EXECUTOR_H
