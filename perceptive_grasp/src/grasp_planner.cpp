/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_planner.cpp
    * @brief 抓取规划模块实现 - 坐标变换 + 顶抓姿态生成
    */

#include "grasp_planner.h"

#include <cmath>
#include <iostream>

namespace perceptive_grasp {

GraspPlanner::GraspPlanner(const GraspPlannerConfig& config) : config_(config) {
    ComputeTransformMatrix();
    std::cout << "[GraspPlanner] Camera mount: front-facing (horizontal)"
                << std::endl;
    std::cout << "[GraspPlanner] T_base_camera translation: ["
                << config_.t_base_camera[0] << ", " << config_.t_base_camera[1]
                << ", " << config_.t_base_camera[2] << "]" << std::endl;
    std::cout << "[GraspPlanner] T_base_camera rotation (RPY): ["
                << config_.r_base_camera[0] << ", " << config_.r_base_camera[1]
                << ", " << config_.r_base_camera[2] << "]" << std::endl;
}

void GraspPlanner::ComputeTransformMatrix() {
    // 从 RPY + translation 构建 4x4 齐次变换矩阵
    float roll = config_.r_base_camera[0];
    float pitch = config_.r_base_camera[1];
    float yaw = config_.r_base_camera[2];

    float cr = std::cos(roll), sr = std::sin(roll);
    float cp = std::cos(pitch), sp = std::sin(pitch);
    float cy = std::cos(yaw), sy = std::sin(yaw);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // 行主序 4x4
    T_base_camera_[0] = cy * cp;
    T_base_camera_[1] = cy * sp * sr - sy * cr;
    T_base_camera_[2] = cy * sp * cr + sy * sr;
    T_base_camera_[3] = config_.t_base_camera[0];

    T_base_camera_[4] = sy * cp;
    T_base_camera_[5] = sy * sp * sr + cy * cr;
    T_base_camera_[6] = sy * sp * cr - cy * sr;
    T_base_camera_[7] = config_.t_base_camera[1];

    T_base_camera_[8] = -sp;
    T_base_camera_[9] = cp * sr;
    T_base_camera_[10] = cp * cr;
    T_base_camera_[11] = config_.t_base_camera[2];

    T_base_camera_[12] = 0.0f;
    T_base_camera_[13] = 0.0f;
    T_base_camera_[14] = 0.0f;
    T_base_camera_[15] = 1.0f;
}

void GraspPlanner::CameraToBase(const float cam_point[3],
                                float base_point[3]) const {
    // P_base = T_base_camera * P_camera
    base_point[0] = T_base_camera_[0] * cam_point[0] +
                    T_base_camera_[1] * cam_point[1] +
                    T_base_camera_[2] * cam_point[2] + T_base_camera_[3];
    base_point[1] = T_base_camera_[4] * cam_point[0] +
                    T_base_camera_[5] * cam_point[1] +
                    T_base_camera_[6] * cam_point[2] + T_base_camera_[7];
    base_point[2] = T_base_camera_[8] * cam_point[0] +
                    T_base_camera_[9] * cam_point[1] +
                    T_base_camera_[10] * cam_point[2] + T_base_camera_[11];
}

bool GraspPlanner::PlanTopGrasp(const float base_point[3], Pose3D& grasp_pose,
                                Pose3D& pre_grasp_pose) const {
    // 检查工作空间
    if (!InWorkspace(base_point[0], base_point[1], base_point[2])) {
        std::cerr << "[GraspPlanner] Target out of workspace: ("
                    << base_point[0] << ", " << base_point[1] << ", "
                    << base_point[2] << ")" << std::endl;
        return false;
    }

    // z 方向裁剪到 z_min (深度噪声可能导致 z 偏低)
    float target_z = base_point[2];
    if (target_z < config_.workspace.z_min) {
        std::cout << "[GraspPlanner] Clamping z from " << target_z
                    << " to z_min=" << config_.workspace.z_min << std::endl;
        target_z = config_.workspace.z_min;
    }

    // 顶抓姿态: 末端 Z 轴朝下 (即绕 X 轴旋转 180°)
    // 四元数: qw=0, qx=1, qy=0, qz=0 (绕X轴180°)
    // 对于 SO101，实际的顶抓朝向取决于 URDF 中 tool frame 的定义
    // 这里用 qw=0, qx=0, qy=1, qz=0 (绕Y轴180°，即Z朝下)
    float top_down_qw = 0.0f;
    float top_down_qx = 0.0f;
    float top_down_qy = 1.0f;
    float top_down_qz = 0.0f;

    // 抓取位姿: 目标位置 - 深度偏移（下探到物体表面以下）
    // 注: 夹爪方向偏移在 pipeline 中根据 grasp_yaw 动态计算
    grasp_pose.x = base_point[0];
    grasp_pose.y = base_point[1];
    grasp_pose.z = target_z - config_.grasp_depth;
    // 限制 grasp_z 不低于工作空间下限（防止撞桌面）
    if (grasp_pose.z < config_.workspace.z_min) {
        grasp_pose.z = config_.workspace.z_min;
    }
    grasp_pose.qw = top_down_qw;
    grasp_pose.qx = top_down_qx;
    grasp_pose.qy = top_down_qy;
    grasp_pose.qz = top_down_qz;

    // 预抓取位姿: 目标上方
    pre_grasp_pose.x = base_point[0];
    pre_grasp_pose.y = base_point[1];
    pre_grasp_pose.z = target_z + config_.approach_height;
    pre_grasp_pose.qw = top_down_qw;
    pre_grasp_pose.qx = top_down_qx;
    pre_grasp_pose.qy = top_down_qy;
    pre_grasp_pose.qz = top_down_qz;

    return true;
}

bool GraspPlanner::InWorkspace(float x, float y, float z) const {
    // z 方向不在此处拒绝，由 PlanTopGrasp 裁剪到 z_min
    return x >= config_.workspace.x_min && x <= config_.workspace.x_max &&
            y >= config_.workspace.y_min && y <= config_.workspace.y_max &&
            z <= config_.workspace.z_max;
}

void GraspPlanner::UpdateCalibration(const std::array<float, 3>& translation,
                                    const std::array<float, 3>& rotation) {
    config_.t_base_camera = translation;
    config_.r_base_camera = rotation;
    ComputeTransformMatrix();
    std::cout << "[GraspPlanner] Calibration updated: T=["
                << translation[0] << ", " << translation[1] << ", "
                << translation[2] << "]" << std::endl;
}

}  // namespace perceptive_grasp
