/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file orientation_estimator.cpp
    * @brief 物体方向估计实现
    */

#include "orientation_estimator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace perceptive_grasp {

namespace {

float NormalizeAnglePi(float angle) {
    while (angle > static_cast<float>(M_PI)) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle <= -static_cast<float>(M_PI)) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

}  // namespace

float ComputeOrientationFromMask(const cv::Mat& mask, float& aspect_ratio) {
    aspect_ratio = 1.0f;

    if (mask.empty()) return 0.0f;

    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0.0f;

    // 取最大轮廓
    size_t max_idx = 0;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }

    if (contours[max_idx].size() < 5) return 0.0f;

    // 最小外接矩形
    cv::RotatedRect rect = cv::minAreaRect(contours[max_idx]);

    float w = rect.size.width;
    float h = rect.size.height;

    if (w < 1.0f || h < 1.0f) return 0.0f;

    // 确保长边/短边
    float long_side = std::max(w, h);
    float short_side = std::min(w, h);
    aspect_ratio = long_side / short_side;

    // minAreaRect 的 angle 定义:
    // OpenCV 4.x: angle 是矩形的 width 边与水平轴的夹角, 范围 [-90, 0)
    // 我们需要长轴方向角
    float angle_deg = rect.angle;

    // 如果 width < height，长轴是 height 方向，需要加 90°
    if (w < h) {
        angle_deg += 90.0f;
    }

    // 转换为弧度，范围 [-pi/2, pi/2]
    float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

    // 归一化到 [-pi/2, pi/2]
    while (angle_rad > static_cast<float>(M_PI) / 2.0f)
        angle_rad -= static_cast<float>(M_PI);
    while (angle_rad < -static_cast<float>(M_PI) / 2.0f)
        angle_rad += static_cast<float>(M_PI);

    return angle_rad;
}

float ComputeOrientationFromBbox(float x1, float y1, float x2, float y2,
                                float& aspect_ratio) {
    float w = x2 - x1;
    float h = y2 - y1;

    if (w < 1.0f || h < 1.0f) {
        aspect_ratio = 1.0f;
        return 0.0f;
    }

    float long_side = std::max(w, h);
    float short_side = std::min(w, h);
    aspect_ratio = long_side / short_side;

    // bbox 只能给出 0° (水平) 或 90° (垂直) 的粗略方向
    if (h > w) {
        // 物体竖直方向
        return static_cast<float>(M_PI) / 2.0f;
    } else {
        // 物体水平方向
        return 0.0f;
    }
}

float ImageAngleToWristYaw(float image_angle, float camera_yaw_offset) {
    // SO-101 joint5 定义:
    //   joint5 = 0: 活动爪向 +Y 方向闭合 (夹爪平行Y轴)
    //   joint5 = pi/2: 活动爪向 -X 方向闭合 (夹爪垂直Y轴)
    //   joint5 = pi: 活动爪向 -Y 方向闭合
    //   范围: [0, pi]
    //
    // 夹爪应垂直于物体长轴方向抓取 (从短轴两侧夹住)
    // 映射关系 (camera_yaw_offset = pi/2):
    //   image_angle=0 (水平) -> 物体沿base X -> 夹爪从Y方向夹 -> joint5=pi/2
    //   image_angle=pi/2 (垂直) -> 物体沿base Y -> 夹爪从X方向夹 -> joint5=0
    //   image_angle=-50° -> joint5 ≈ 2.2
    // 公式: yaw = camera_yaw_offset - image_angle

    float yaw = camera_yaw_offset - image_angle;

    // 归一化到 [0, pi] (利用 180° 对称性: 抓哪头都行)
    while (yaw > static_cast<float>(M_PI))
        yaw -= static_cast<float>(M_PI);
    while (yaw < 0.0f)
        yaw += static_cast<float>(M_PI);

    return yaw;
}

float ComputeGraspYaw(const DetectionTarget& target,
                        const OrientationConfig& config) {
    float aspect_ratio = 1.0f;
    float image_angle = 0.0f;

    // 优先使用 mask (更精确)
    if (!target.mask.empty()) {
        image_angle = ComputeOrientationFromMask(target.mask, aspect_ratio);
        std::cout << "[Orientation] From mask: angle=" << image_angle * 180.0f / M_PI
                    << "°, aspect_ratio=" << aspect_ratio << std::endl;
    } else {
        // Fallback 到 bbox
        image_angle = ComputeOrientationFromBbox(
            target.x1, target.y1, target.x2, target.y2, aspect_ratio);
        std::cout << "[Orientation] From bbox: angle=" << image_angle * 180.0f / M_PI
                    << "°, aspect_ratio=" << aspect_ratio << std::endl;
    }

    // 如果物体接近圆形/正方形，不需要对齐
    if (aspect_ratio < config.aspect_ratio_threshold) {
        std::cout << "[Orientation] Object is nearly symmetric (ratio="
                    << aspect_ratio << " < " << config.aspect_ratio_threshold
                    << "), no yaw alignment needed" << std::endl;
        return NAN;
    }

    // 转换为 wrist yaw
    float wrist_yaw = ImageAngleToWristYaw(image_angle, config.camera_yaw_offset);

    std::cout << "[Orientation] Computed wrist_yaw=" << wrist_yaw
                << " rad (" << wrist_yaw * 180.0f / M_PI << "°)" << std::endl;

    return wrist_yaw;
}

bool ComputeGraspPixel(const DetectionTarget& target,
                        float& grasp_px, float& grasp_py,
                        float offset_ratio,
                        const OrientationConfig& config,
                        float* offset_dir_angle) {
    // 物体中心
    float cx = target.center.x;
    float cy = target.center.y;

    float aspect_ratio = 1.0f;
    float image_angle = 0.0f;
    float short_half = 0.0f;  // 短轴半长 (像素)

    if (!target.mask.empty()) {
        // 从 mask 获取精确的最小外接矩形
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(target.mask, contours, cv::RETR_EXTERNAL,
                        cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // 取最大轮廓
            size_t max_idx = 0;
            double max_area = 0;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }

            if (contours[max_idx].size() >= 5) {
                cv::RotatedRect rect = cv::minAreaRect(contours[max_idx]);
                float w = rect.size.width;
                float h = rect.size.height;

                // 确定长轴/短轴
                float long_side = std::max(w, h);
                float short_side = std::min(w, h);
                aspect_ratio = (short_side > 0) ? long_side / short_side : 1.0f;
                short_half = short_side / 2.0f;

                // 长轴方向角
                float angle_deg = rect.angle;
                if (w < h) angle_deg += 90.0f;
                image_angle = angle_deg * static_cast<float>(M_PI) / 180.0f;
                while (image_angle > static_cast<float>(M_PI) / 2.0f)
                    image_angle -= static_cast<float>(M_PI);
                while (image_angle < -static_cast<float>(M_PI) / 2.0f)
                    image_angle += static_cast<float>(M_PI);

                // 使用 mask 的中心 (可能比 bbox 中心更准)
                cx = rect.center.x;
                cy = rect.center.y;
            }
        }
    } else {
        // 从 bbox 估算
        float bw = target.x2 - target.x1;
        float bh = target.y2 - target.y1;
        float long_side = std::max(bw, bh);
        float short_side = std::min(bw, bh);
        aspect_ratio = (short_side > 0) ? long_side / short_side : 1.0f;
        short_half = short_side / 2.0f;

        if (bh > bw) {
            image_angle = static_cast<float>(M_PI) / 2.0f;
        } else {
            image_angle = 0.0f;
        }
    }

    // 如果物体接近圆形，不偏移，直接用中心
    if (aspect_ratio < config.aspect_ratio_threshold || short_half < 5.0f) {
        grasp_px = cx;
        grasp_py = cy;
        if (offset_dir_angle) *offset_dir_angle = NAN;
        std::cout << "[GraspPixel] Object nearly symmetric, using center"
                    << " (ratio=" << aspect_ratio
                    << " < threshold=" << config.aspect_ratio_threshold << ")"
                    << std::endl;
        return true;
    }

    // 短轴方向 = 长轴方向 + 90°
    float short_axis_angle = image_angle + static_cast<float>(M_PI) / 2.0f;

    // 固定选择 image_angle + 90° 这一侧作为抓取点偏移方向。
    // executor/pipeline 会令固定爪朝向该偏移侧；不要为了减少 wrist_roll
    // 行程自动翻到相反侧，否则单动爪结构会从物体另一侧闭合。
    float dir_angle = NormalizeAnglePi(short_axis_angle);
    float dir_x = std::cos(dir_angle);
    float dir_y = std::sin(dir_angle);

    // 偏移距离 = 短轴半长 × offset_ratio
    float offset_px = short_half * offset_ratio;

    grasp_px = cx + dir_x * offset_px;
    grasp_py = cy + dir_y * offset_px;

    // 输出实际偏移方向角
    if (offset_dir_angle) {
        *offset_dir_angle = dir_angle;
    }

    std::cout << "[GraspPixel] image_angle=" << image_angle * 180.0f / M_PI
                << "°, short_half=" << short_half << "px"
                << ", offset=" << offset_px << "px"
                << ", dir=[" << dir_x << "," << dir_y << "]"
                << ", dir_angle=" << dir_angle * 180.0f / M_PI << "°"
                << ", center=[" << cx << "," << cy << "]"
                << " -> grasp=[" << grasp_px << "," << grasp_py << "]"
                << std::endl;

    return true;
}

}  // namespace perceptive_grasp
