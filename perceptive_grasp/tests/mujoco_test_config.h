/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MUJOCO_TEST_CONFIG_H
#define MUJOCO_TEST_CONFIG_H
#include <string>
#include "grasp_executor.h"
namespace perceptive_grasp::test {
inline ExecutorConfig Configuration() {
    ExecutorConfig config;
    const std::string root = PERCEPTIVE_GRASP_SOURCE_DIR;
    config.manip_driver = "mujoco_ur5e";
    config.mujoco.device_plan = root + "/simulation/mujoco/manipulation/device.yaml";
    config.mujoco.end_effector_site = "robot_gripper_pinch";
    config.mujoco.gripper_actuator = "robot_gripper_fingers_actuator";
    config.mujoco.robot_root_body = "robot_base";
    config.mujoco.gripper_root_body = "robot_gripper_base_mount";
    config.mujoco.gravity_compensation = false;
    config.mujoco.arm_stiffness_scale = 1;
    config.mujoco.gripper_open_ctrl = 0;
    config.mujoco.gripper_close_ctrl = 255;
    config.mujoco.joint_names = {"robot_shoulder_pan_joint", "robot_shoulder_lift_joint",
            "robot_elbow_joint", "robot_wrist_1_joint", "robot_wrist_2_joint",
            "robot_wrist_3_joint"};
    config.mujoco.actuator_names = {"robot_shoulder_pan", "robot_shoulder_lift", "robot_elbow",
            "robot_wrist_1", "robot_wrist_2", "robot_wrist_3"};
    config.home_joints = {-1.5708f, -1.5708f, 1.5708f, -1.5708f, -1.5708f, 0};
    config.observe_joints = config.home_joints;
    config.side_ready_joints = {0.2f, -0.9f, 1.3f, -1.8f, -1.57f, 0};
    config.place_joints = {-1.2f, -1.0f, 1.4f, -1.9f, -1.57f, 0};
    config.move_speed = 0.6f;
    config.mujoco.ik_iterations = 300;
    config.mujoco.ik_damping = 0.010f;
    config.mujoco.ik_step_scale = 0.80f;
    config.mujoco.ik_position_tolerance_m = 0.006f;
    config.mujoco.cartesian_tracking_tolerance_m = 0.003f;
    config.mujoco.settle_steps = 500;
    return config;
}
}  // namespace perceptive_grasp::test
#endif
