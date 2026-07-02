/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file mock_executor.cpp
    * @brief X86 Standalone 机械臂/夹爪 mock - 只打印日志
    */

#include "mock/mock_executor.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>

namespace perceptive_grasp {

MockExecutor::MockExecutor(const ExecutorConfig& config)
    : GraspExecutor(config) {}

bool MockExecutor::Init() {
    std::cout << "[MockExecutor] Initialized (no real hardware)" << std::endl;
    std::cout << "[MockExecutor]   driver: " << config_.manip_driver << std::endl;
    std::cout << "[MockExecutor]   urdf: " << config_.urdf_path << std::endl;
    return true;
}

GraspResult MockExecutor::MoveToObserve() {
    std::cout << "[MockExecutor] MoveToObserve: close gripper + joints=[";
    for (size_t i = 0; i < config_.observe_joints.size(); i++) {
        if (i > 0) std::cout << ", ";
        printf("%.2f", config_.observe_joints[i]);
    }
    std::cout << "]" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.observe_gripper_close_wait_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::MoveToHome() {
    std::cout << "[MockExecutor] MoveToHome: joints=[";
    for (size_t i = 0; i < config_.home_joints.size(); i++) {
        if (i > 0) std::cout << ", ";
        printf("%.2f", config_.home_joints[i]);
    }
    std::cout << "]" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad) {
    printf("[MockExecutor] MoveToPreGrasp: (%.4f, %.4f, %.4f)\n",
            pre_grasp_pose.x, pre_grasp_pose.y, pre_grasp_pose.z);
    if (!std::isnan(grasp_yaw_rad)) {
        printf("[MockExecutor]   yaw: %.4f rad (%.1f°)\n",
                grasp_yaw_rad, grasp_yaw_rad * 180.0f / M_PI);
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.pre_grasp_settle_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::OpenGripperForGrasp() {
    std::cout << "[MockExecutor] OpenGripperForGrasp: open="
                << config_.gripper_open << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.gripper_open_wait_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::MoveToGrasp(const Pose3D& grasp_pose,
                                        float grasp_yaw_rad) {
    printf("[MockExecutor] MoveToGrasp: (%.4f, %.4f, %.4f)\n",
            grasp_pose.x, grasp_pose.y, grasp_pose.z);
    if (!std::isnan(grasp_yaw_rad)) {
        printf("[MockExecutor]   yaw: %.4f rad (%.1f°)\n",
                grasp_yaw_rad, grasp_yaw_rad * 180.0f / M_PI);
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.grasp_settle_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::CloseGripperAndCheck() {
    std::cout << "[MockExecutor] CloseGripperAndCheck: effort="
                << config_.gripper_effort << " -> HOLDING" << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.gripper_close_wait_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::LiftFromGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad) {
    (void)grasp_yaw_rad;
    printf("[MockExecutor] LiftFromGrasp: (%.4f, %.4f, %.4f)\n",
            pre_grasp_pose.x, pre_grasp_pose.y, pre_grasp_pose.z);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.grasp_check_count *
        config_.timing.grasp_check_interval_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::MoveToPlace() {
    std::cout << "[MockExecutor] MoveToPlace: joints=[";
    for (size_t i = 0; i < config_.place_joints.size(); i++) {
        if (i > 0) std::cout << ", ";
        printf("%.2f", config_.place_joints[i]);
    }
    std::cout << "]" << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.place_settle_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::ReleaseObject() {
    std::cout << "[MockExecutor] ReleaseObject: open="
                << config_.place_release_open << ", wait="
                << config_.timing.release_wait_ms << "ms" << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.release_wait_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::CloseGripper() {
    std::cout << "[MockExecutor] CloseGripper" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.home_gripper_close_wait_ms));
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::ExecuteGrasp(const Pose3D& grasp_pose,
                                        const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad) {
    std::cout << "[MockExecutor] === ExecuteGrasp ===" << std::endl;

    printf("[MockExecutor]   pre_grasp: (%.4f, %.4f, %.4f)\n",
            pre_grasp_pose.x, pre_grasp_pose.y, pre_grasp_pose.z);
    printf("[MockExecutor]   grasp:     (%.4f, %.4f, %.4f)\n",
            grasp_pose.x, grasp_pose.y, grasp_pose.z);

    if (!std::isnan(grasp_yaw_rad)) {
        printf("[MockExecutor]   yaw:       %.4f rad (%.1f°)\n",
                grasp_yaw_rad, grasp_yaw_rad * 180.0f / M_PI);
    }

    GraspResult result = MoveToPreGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    result = OpenGripperForGrasp();
    if (result != GraspResult::SUCCESS) return result;
    result = MoveToGrasp(grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    result = CloseGripperAndCheck();
    if (result != GraspResult::SUCCESS) return result;
    result = LiftFromGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;
    std::cout << "[MockExecutor] === Grasp SUCCESS ===" << std::endl;
    return GraspResult::SUCCESS;
}

GraspResult MockExecutor::ExecutePlace() {
    std::cout << "[MockExecutor] === ExecutePlace ===" << std::endl;
    GraspResult result = MoveToPlace();
    if (result != GraspResult::SUCCESS) return result;
    result = ReleaseObject();
    if (result != GraspResult::SUCCESS) return result;
    result = CloseGripper();
    if (result != GraspResult::SUCCESS) return result;
    result = MoveToObserve();
    if (result != GraspResult::SUCCESS) return result;
    std::cout << "[MockExecutor] === Place SUCCESS ===" << std::endl;
    return GraspResult::SUCCESS;
}

void MockExecutor::EmergencyStop() {
    std::cout << "[MockExecutor] EMERGENCY STOP!" << std::endl;
}

bool MockExecutor::GetCurrentPose(Pose3D& pose) {
    // 返回一个假的位姿
    pose = {0.1f, 0.0f, 0.15f, 1.0f, 0.0f, 0.0f, 0.0f};
    return true;
}

void MockExecutor::Tick(float dt_s) {
    (void)dt_s;
    // Mock: nothing to do
}

}  // namespace perceptive_grasp
