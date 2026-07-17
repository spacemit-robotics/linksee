/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_planner.h
    * @brief 抓取规划模块 - 3D 定位 + 顶抓姿态生成
    */

#ifndef GRASP_PLANNER_H
#define GRASP_PLANNER_H

#include <array>
#include <string>

#include <opencv2/core.hpp>

namespace perceptive_grasp {

/** 3D 空间中的位姿 */
struct Pose3D {
    float x, y, z;          // 位置 (米)
    float qw, qx, qy, qz;  // 姿态四元数
};

/** 工作空间限制 */
struct WorkspaceLimits {
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;
};

struct GraspPlannerConfig {
    // 手眼标定: 相机→基座变换
    std::array<float, 3> t_base_camera = {0.07f, 0.0f, 0.295f};
    std::array<float, 3> r_base_camera = {0.0f, 0.0f, 0.0f};  // RPY

    // 顶抓参数
    float approach_height = 0.10f;  // 预抓取高度 (米)
    float grasp_depth = 0.01f;      // 抓取深度 (米)

    // 夹爪固定爪方向偏移 (米)
    // SO101 左爪固定、右爪运动，抓取点需要沿夹爪方向往固定爪侧偏移
    // 该偏移会根据 grasp_yaw 自动投影到 x/y 方向
    // 正值 = 往固定爪侧偏移
    float gripper_offset = 0.0f;

    // 工作空间安全限制
    WorkspaceLimits workspace = {
        0.0f, 0.5f,    // x
        -0.3f, 0.3f,   // y
        0.0f, 0.20f    // z
    };
};

/**
    * @brief 抓取规划器
    *
    * 负责:
    * 1. 将像素+深度转换为相机坐标系 3D 点
    * 2. 通过手眼标定变换到机械臂基坐标系
    * 3. 生成顶抓姿态 (末端朝下)
    * 4. 检查工作空间安全限制
    */
class GraspPlanner {
public:
    explicit GraspPlanner(const GraspPlannerConfig& config);
    ~GraspPlanner() = default;

    /**
    * @brief 将相机坐标系 3D 点转换到机械臂基坐标系
    * @param cam_point 相机坐标系点 [x, y, z] (米)
    * @param[out] base_point 基坐标系点 [x, y, z] (米)
    */
    void CameraToBase(const float cam_point[3], float base_point[3]) const;

    /**
    * @brief 生成顶抓姿态
    * @param base_point 目标在基坐标系中的位置 [x, y, z]
    * @param[out] grasp_pose 抓取位姿
    * @param[out] pre_grasp_pose 预抓取位姿 (上方)
    * @return true 在工作空间内，规划成功
    */
    bool PlanTopGrasp(const float base_point[3],
                        Pose3D& grasp_pose,
                        Pose3D& pre_grasp_pose) const;

    /**
    * @brief 检查点是否在工作空间内
    */
    bool InWorkspace(float x, float y, float z) const;

    /**
    * @brief 更新手眼标定参数 (运行时可调)
    */
    void UpdateCalibration(const std::array<float, 3>& translation,
                            const std::array<float, 3>& rotation);

private:
    GraspPlannerConfig config_;

    // 手眼变换矩阵 (4x4, 行主序)
    float T_base_camera_[16];

    void ComputeTransformMatrix();
};

}  // namespace perceptive_grasp

#endif  // GRASP_PLANNER_H
