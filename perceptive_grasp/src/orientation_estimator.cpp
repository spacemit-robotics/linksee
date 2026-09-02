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
#include <limits>
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

bool FindSafeMaskInterior(
    const cv::Mat& mask, float center_x, float center_y,
    float& grasp_x, float& grasp_y,
    float* grasp_clearance = nullptr,
    float* maximum_mask_clearance = nullptr,
    cv::Point2f* deepest_interior = nullptr) {
    if (mask.empty()) return false;

    cv::Mat binary;
    cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY);
    cv::Mat distance;
    cv::distanceTransform(binary, distance, cv::DIST_L2, 5);
    double maximum_clearance = 0.0;
    cv::Point maximum_location;
    cv::minMaxLoc(
        distance, nullptr, &maximum_clearance,
        nullptr, &maximum_location);
    if (maximum_clearance < 1.0) return false;
    if (maximum_mask_clearance != nullptr) {
        *maximum_mask_clearance = static_cast<float>(maximum_clearance);
    }
    if (deepest_interior != nullptr) {
        *deepest_interior = cv::Point2f(
            static_cast<float>(maximum_location.x),
            static_cast<float>(maximum_location.y));
    }

    const float minimum_clearance = std::max(
        2.0f, 0.70f * static_cast<float>(maximum_clearance));
    float best_distance_squared = std::numeric_limits<float>::max();
    cv::Point best(-1, -1);
    for (int row = 0; row < distance.rows; ++row) {
        const float* values = distance.ptr<float>(row);
        for (int column = 0; column < distance.cols; ++column) {
            if (values[column] < minimum_clearance) continue;
            const float dx = static_cast<float>(column) - center_x;
            const float dy = static_cast<float>(row) - center_y;
            const float distance_squared = dx * dx + dy * dy;
            if (distance_squared < best_distance_squared) {
                best_distance_squared = distance_squared;
                best = cv::Point(column, row);
            }
        }
    }
    if (best.x < 0) return false;

    grasp_x = static_cast<float>(best.x);
    grasp_y = static_cast<float>(best.y);
    if (grasp_clearance != nullptr) {
        *grasp_clearance = distance.at<float>(best.y, best.x);
    }
    return true;
}

bool SnapCenterToMaskInterior(
    const cv::Mat& mask, float center_x, float center_y,
    float& grasp_x, float& grasp_y,
    float* grasp_clearance = nullptr) {
    if (mask.empty()) return false;

    const int x = std::clamp(
        static_cast<int>(std::lround(grasp_x)), 0, mask.cols - 1);
    const int y = std::clamp(
        static_cast<int>(std::lround(grasp_y)), 0, mask.rows - 1);
    if (mask.at<uint8_t>(y, x) != 0) return false;

    return FindSafeMaskInterior(
        mask, center_x, center_y, grasp_x, grasp_y, grasp_clearance);
}

bool FindStableDeepConcaveInterior(
    const cv::Mat& mask, float long_axis_angle,
    float reference_x, float reference_y,
    float& center_x, float& center_y,
    float& center_clearance) {
    if (mask.empty()) return false;
    cv::Mat binary;
    cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY);
    cv::Mat distance;
    cv::distanceTransform(binary, distance, cv::DIST_L2, 5);
    double maximum_clearance = 0.0;
    cv::minMaxLoc(distance, nullptr, &maximum_clearance);
    if (maximum_clearance < 2.0) return false;

    constexpr float kDeepRidgeRatio = 0.90f;
    const float minimum_clearance =
        kDeepRidgeRatio * static_cast<float>(maximum_clearance);
    const float axis_x = std::cos(long_axis_angle);
    const float axis_y = std::sin(long_axis_angle);
    float minimum_projection = std::numeric_limits<float>::max();
    for (int y = 0; y < distance.rows; ++y) {
        const float* row = distance.ptr<float>(y);
        for (int x = 0; x < distance.cols; ++x) {
            if (row[x] < minimum_clearance) continue;
            minimum_projection = std::min(
                minimum_projection, axis_x * x + axis_y * y);
        }
    }
    if (!std::isfinite(minimum_projection)) return false;

    // Pick the deepest ridge point within a small band at a deterministic
    // end of the global long axis. This avoids both the curved middle and
    // frame-to-frame jumps between equal distance-transform maxima.
    constexpr float kProjectionBandPx = 5.0f;
    float best_clearance = -1.0f;
    float best_reference_distance = std::numeric_limits<float>::max();
    cv::Point best(-1, -1);
    for (int y = 0; y < distance.rows; ++y) {
        const float* row = distance.ptr<float>(y);
        for (int x = 0; x < distance.cols; ++x) {
            if (row[x] < minimum_clearance ||
                axis_x * x + axis_y * y >
                    minimum_projection + kProjectionBandPx) {
                continue;
            }
            const float dx = static_cast<float>(x) - reference_x;
            const float dy = static_cast<float>(y) - reference_y;
            const float reference_distance = dx * dx + dy * dy;
            if (row[x] > best_clearance + 1e-4f ||
                (std::fabs(row[x] - best_clearance) <= 1e-4f &&
                reference_distance < best_reference_distance)) {
                best_clearance = row[x];
                best_reference_distance = reference_distance;
                best = cv::Point(x, y);
            }
        }
    }
    if (best.x < 0) return false;
    center_x = static_cast<float>(best.x);
    center_y = static_cast<float>(best.y);
    center_clearance = best_clearance;
    return true;
}

bool LocalMaskOrientation(
    const cv::Mat& mask, float center_x, float center_y,
    float center_clearance, float preferred_short_axis_angle,
    float& long_axis_angle, float& short_half) {
    if (mask.empty() || center_clearance < 1.0f) return false;

    // Keep the PCA neighbourhood local to one straight section. A wider
    // radius around a banana bend includes both arms of the crescent and
    // estimates a closing axis that wedges the object out of the gripper.
    const float radius = std::max(12.0f, 1.5f * center_clearance);
    const float radius_squared = radius * radius;
    std::vector<cv::Point2f> samples;
    for (int y = std::max(
            0, static_cast<int>(std::floor(center_y - radius)));
        y <= std::min(
            mask.rows - 1,
            static_cast<int>(std::ceil(center_y + radius))); ++y) {
        const uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = std::max(
                0, static_cast<int>(std::floor(center_x - radius)));
            x <= std::min(
                mask.cols - 1,
                static_cast<int>(std::ceil(center_x + radius))); ++x) {
            if (row[x] == 0) continue;
            const float dx = static_cast<float>(x) - center_x;
            const float dy = static_cast<float>(y) - center_y;
            if (dx * dx + dy * dy <= radius_squared) {
                samples.emplace_back(static_cast<float>(x),
                    static_cast<float>(y));
            }
        }
    }
    if (samples.size() < 20) return false;

    cv::Mat values(static_cast<int>(samples.size()), 2, CV_32F);
    for (int row = 0; row < values.rows; ++row) {
        values.at<float>(row, 0) = samples[row].x;
        values.at<float>(row, 1) = samples[row].y;
    }
    cv::PCA pca(values, cv::Mat(), cv::PCA::DATA_AS_ROW);
    const float secondary_variance = pca.eigenvalues.at<float>(1, 0);
    if (secondary_variance <= 1e-6f ||
        pca.eigenvalues.at<float>(0, 0) / secondary_variance < 1.2f) {
        return false;
    }

    long_axis_angle = std::atan2(
        pca.eigenvectors.at<float>(0, 1),
        pca.eigenvectors.at<float>(0, 0));
    float short_axis_angle =
        long_axis_angle + static_cast<float>(M_PI) / 2.0f;
    const float alignment =
        std::cos(short_axis_angle - preferred_short_axis_angle);
    if (alignment < 0.0f) {
        long_axis_angle += static_cast<float>(M_PI);
        short_axis_angle += static_cast<float>(M_PI);
    }
    long_axis_angle = NormalizeAnglePi(long_axis_angle);

    const float direction_x = std::cos(short_axis_angle);
    const float direction_y = std::sin(short_axis_angle);
    float last_inside_distance = 0.0f;
    const float maximum_distance = std::max(
        2.0f * center_clearance, center_clearance + 8.0f);
    for (float distance = 0.5f;
        distance <= maximum_distance; distance += 0.5f) {
        const int x = static_cast<int>(std::lround(
            center_x + direction_x * distance));
        const int y = static_cast<int>(std::lround(
            center_y + direction_y * distance));
        if (x < 0 || x >= mask.cols || y < 0 || y >= mask.rows ||
            mask.at<uint8_t>(y, x) == 0) {
            break;
        }
        last_inside_distance = distance;
    }
    if (last_inside_distance < 1.0f) return false;
    short_half = last_inside_distance;
    return true;
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

                if (config.safe_mask_interior) {
                    const int rect_center_x = std::clamp(
                        static_cast<int>(std::lround(cx)), 0,
                        target.mask.cols - 1);
                    const int rect_center_y = std::clamp(
                        static_cast<int>(std::lround(cy)), 0,
                        target.mask.rows - 1);
                    const bool center_outside_mask =
                        target.mask.at<uint8_t>(
                            rect_center_y, rect_center_x) == 0;

                    std::vector<cv::Point> hull;
                    cv::convexHull(contours[max_idx], hull);
                    const double hull_area = cv::contourArea(hull);
                    const float solidity = hull_area > 1.0
                        ? static_cast<float>(max_area / hull_area)
                        : 1.0f;

                    float safe_x = cx;
                    float safe_y = cy;
                    float safe_clearance = 0.0f;
                    float maximum_clearance = 0.0f;
                    cv::Point2f deepest_interior(cx, cy);
                    const bool has_safe_interior = FindSafeMaskInterior(
                        target.mask, cx, cy, safe_x, safe_y,
                        &safe_clearance, &maximum_clearance,
                        &deepest_interior);
                    const bool curved_concave_mask =
                        solidity < 0.80f && maximum_clearance >= 2.0f &&
                        short_half > 1.50f * maximum_clearance;
                    if ((center_outside_mask || curved_concave_mask) &&
                        has_safe_interior) {
                        if (curved_concave_mask &&
                            !FindStableDeepConcaveInterior(
                                target.mask, image_angle, cx, cy,
                                safe_x, safe_y, safe_clearance)) {
                            safe_x = deepest_interior.x;
                            safe_y = deepest_interior.y;
                            safe_clearance = maximum_clearance;
                        }
                        const float preferred_short_axis_angle =
                            image_angle + static_cast<float>(M_PI) / 2.0f;
                        float local_angle = image_angle;
                        float local_short_half = short_half;
                        if (LocalMaskOrientation(
                                target.mask, safe_x, safe_y,
                                safe_clearance,
                                preferred_short_axis_angle,
                                local_angle, local_short_half)) {
                            image_angle = local_angle;
                            short_half = local_short_half;
                        } else {
                            short_half = std::min(
                                short_half, safe_clearance);
                        }
                        cx = safe_x;
                        cy = safe_y;
                        std::cout
                            << "[GraspPixel] Concave mask uses local cross-section"
                            << " center=[" << cx << "," << cy << "]"
                            << " solidity=" << solidity
                            << " global_short_half=" << short_side / 2.0f
                            << "px max_clearance=" << maximum_clearance
                            << "px local_clearance=" << safe_clearance << "px"
                            << " center_outside=" << center_outside_mask
                            << std::endl;
                    }
                }
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
        if (config.safe_mask_interior && SnapCenterToMaskInterior(
                target.mask, cx, cy, grasp_px, grasp_py)) {
            std::cout << "[GraspPixel] Geometric center is outside mask; "
                        "snapped to safe interior point=["
                    << grasp_px << "," << grasp_py << "]" << std::endl;
        }
        if (offset_dir_angle) *offset_dir_angle = NAN;
        std::cout << "[GraspPixel] Object nearly symmetric, using center"
                    << " (ratio=" << aspect_ratio
                    << " < threshold=" << config.aspect_ratio_threshold << ")"
                    << std::endl;
        return true;
    }

    // 短轴方向 = 长轴方向 + 90°
    float short_axis_angle = image_angle + static_cast<float>(M_PI) / 2.0f;

    // Keep the configured fixed-jaw side. Switching to the opposite
    // short-axis endpoint changes how the asymmetric gripper closes.
    float dir_angle = NormalizeAnglePi(short_axis_angle);
    float dir_x = std::cos(dir_angle);
    float dir_y = std::sin(dir_angle);

    // 偏移距离 = 短轴半长 × 偏移比例
    float offset_px = short_half * offset_ratio;

    grasp_px = cx + dir_x * offset_px;
    grasp_py = cy + dir_y * offset_px;

    if (config.safe_mask_interior &&
        std::fabs(offset_ratio) <= 1e-6f &&
        SnapCenterToMaskInterior(
            target.mask, cx, cy, grasp_px, grasp_py)) {
        std::cout << "[GraspPixel] Geometric center is outside mask; "
                    "snapped to safe interior point=["
                << grasp_px << "," << grasp_py << "]" << std::endl;
    }

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
