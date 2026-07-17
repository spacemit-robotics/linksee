/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file orientation_estimator.h
    * @brief 物体方向估计 - 从检测 mask/bbox 计算夹爪旋转角
    */

#ifndef ORIENTATION_ESTIMATOR_H
#define ORIENTATION_ESTIMATOR_H

#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "target_detector.h"

namespace perceptive_grasp {

/** 方向估计配置 */
struct OrientationConfig {
    // 长宽比阈值: 超过此值才认为物体有明显方向性
    // 低于此值的物体 (如圆形/正方形) 不需要对齐
    float aspect_ratio_threshold = 1.2f;

    // 相机安装旋转补偿 (弧度)
    // 从图像坐标系角度到基座坐标系角度的偏移
    // 对于前视相机: 图像 X 轴 ≈ 基座 Y 轴 (左右), 图像 Y 轴 ≈ 基座 X 轴 (前后)
    float camera_yaw_offset = 1.57f;
};

/**
    * @brief 从检测目标估计夹爪旋转角 (wrist yaw)
    *
    * 策略:
    * 1. 如果有 mask，用 cv::minAreaRect 求最小外接矩形方向
    * 2. 如果没有 mask，用 bbox 长宽比判断方向
    * 3. 如果物体接近圆形/正方形 (长宽比 < 阈值)，返回 NAN (不需要对齐)
    *
    * 返回值:
    * - 有效角度 (弧度): 夹爪应旋转到的 wrist yaw 角度
    * - NAN: 物体无明显方向性，不需要覆盖 joint5
    *
    * @param target 检测目标 (含 mask 和 bbox)
    * @param config 方向估计配置
    * @return 夹爪旋转角 (弧度)，NAN 表示不需要对齐
    */
float ComputeGraspYaw(const DetectionTarget& target,
                        const OrientationConfig& config = OrientationConfig());

/**
    * @brief 从 mask 轮廓计算物体主方向角 (图像坐标系)
    *
    * 使用 cv::minAreaRect 求最小外接矩形，返回长轴方向角。
    * 角度范围: [-pi/2, pi/2]，0 表示水平方向。
    *
    * @param mask 二值 mask (CV_8UC1)
    * @param[out] aspect_ratio 长宽比 (长边/短边)
    * @return 长轴方向角 (弧度)，失败返回 0
    */
float ComputeOrientationFromMask(const cv::Mat& mask, float& aspect_ratio);

/**
    * @brief 从 bbox 计算物体主方向角 (图像坐标系)
    *
    * 简单策略: bbox 宽 > 高 → 水平 (0°)，高 > 宽 → 垂直 (90°)
    *
    * @param x1 左上角 x
    * @param y1 左上角 y
    * @param x2 右下角 x
    * @param y2 右下角 y
    * @param[out] aspect_ratio 长宽比 (长边/短边)
    * @return 方向角 (弧度)
    */
float ComputeOrientationFromBbox(float x1, float y1, float x2, float y2,
                                float& aspect_ratio);

/**
    * @brief 将图像坐标系角度转换为 wrist yaw 角度
    *
    * 考虑相机安装方向，将图像中的物体方向角映射到机械臂 joint5 角度。
    * 夹爪应垂直于物体长轴方向抓取。
    *
    * @param image_angle 图像坐标系中的物体方向角 (弧度)
    * @param camera_yaw_offset 相机安装旋转补偿 (弧度)
    * @return wrist yaw 角度 (弧度)
    */
float ImageAngleToWristYaw(float image_angle, float camera_yaw_offset = 0.0f);

/**
    * @brief 计算沿短轴方向偏移的抓取像素坐标
    *
    * SO-101 固定爪在左侧，抓取点应从物体中心沿短轴方向偏移到固定爪侧边缘。
    * 短轴方向 = 垂直于物体长轴 = image_angle + 90°
    * 偏移方向固定为 image_angle + 90°，pipeline 会令固定爪朝向该偏移侧。
    *
    * @param target 检测目标 (含 mask 和 bbox)
    * @param[out] grasp_px 抓取点 X 像素坐标
    * @param[out] grasp_py 抓取点 Y 像素坐标
    * @param offset_ratio 沿短轴偏移比例 [0~1]，0=中心，1=短轴边缘
    * @param[out] offset_dir_angle 实际偏移方向角 (图像坐标系弧度)，用于计算 yaw
    * @return true 计算成功
    */
bool ComputeGraspPixel(const DetectionTarget& target,
                        float& grasp_px, float& grasp_py,
                        float offset_ratio = 0.5f,
                        const OrientationConfig& config = OrientationConfig(),
                        float* offset_dir_angle = nullptr);

}  // namespace perceptive_grasp

#endif  // ORIENTATION_ESTIMATOR_H
