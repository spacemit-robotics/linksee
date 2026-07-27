/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file orientation_estimator_test.cpp
 * @brief Tests image-space grasp orientation estimation.
 */

#include "orientation_estimator.h"

#include <cassert>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {

constexpr float kPi = static_cast<float>(CV_PI);

perceptive_grasp::DetectionTarget MakeRotatedTarget(float angle_deg) {
    constexpr int kImageSize = 400;
    const cv::Point2f center(200.0f, 200.0f);

    perceptive_grasp::DetectionTarget target{};
    target.center = center;
    target.mask = cv::Mat::zeros(kImageSize, kImageSize, CV_8UC1);
    cv::ellipse(
        target.mask, center, cv::Size(80, 25), angle_deg,
        0.0, 360.0, cv::Scalar(255), cv::FILLED);
    return target;
}

void CheckRotatedTarget(float angle_deg) {
    const perceptive_grasp::DetectionTarget target =
        MakeRotatedTarget(angle_deg);
    float grasp_x = 0.0f;
    float grasp_y = 0.0f;
    float offset_angle = NAN;
    const bool valid = perceptive_grasp::ComputeGraspPixel(
        target, grasp_x, grasp_y, 1.0f,
        perceptive_grasp::OrientationConfig{}, &offset_angle);
    assert(valid);
    assert(std::isfinite(offset_angle));

    float aspect_ratio = 0.0f;
    const float long_axis_angle =
        perceptive_grasp::ComputeOrientationFromMask(
            target.mask, aspect_ratio);
    assert(aspect_ratio > 2.5f);

    const float offset_x = grasp_x - target.center.x;
    const float offset_y = grasp_y - target.center.y;
    const float offset_length = std::hypot(offset_x, offset_y);
    assert(offset_length > 23.0f);
    assert(offset_length < 27.0f);

    const float expected_x = std::cos(long_axis_angle + kPi / 2.0f);
    const float expected_y = std::sin(long_axis_angle + kPi / 2.0f);
    const float direction_dot =
        (offset_x * expected_x + offset_y * expected_y) / offset_length;
    assert(direction_dot > 0.995f);

    float wrist_yaw = -offset_angle;
    while (wrist_yaw < 0.0f) wrist_yaw += kPi;
    while (wrist_yaw >= kPi) wrist_yaw -= kPi;
    assert(wrist_yaw >= 0.0f);
    assert(wrist_yaw < kPi);
}

}  // namespace

int main() {
    for (float angle_deg = 0.0f; angle_deg < 180.0f; angle_deg += 15.0f) {
        CheckRotatedTarget(angle_deg);
    }
    return 0;
}
