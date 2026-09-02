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

perceptive_grasp::OrientationConfig SafeInteriorConfig() {
    perceptive_grasp::OrientationConfig config;
    config.safe_mask_interior = true;
    return config;
}

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

void CheckConcaveTargetUsesMaskInterior() {
    perceptive_grasp::DetectionTarget target{};
    target.mask = cv::Mat::zeros(300, 300, CV_8UC1);
    cv::circle(
        target.mask, cv::Point(150, 150), 80,
        cv::Scalar(255), cv::FILLED);
    cv::circle(
        target.mask, cv::Point(180, 150), 58,
        cv::Scalar(0), cv::FILLED);
    target.center = cv::Point2f(150.0f, 150.0f);

    float grasp_x = 0.0f;
    float grasp_y = 0.0f;
    float offset_angle = NAN;
    const bool valid = perceptive_grasp::ComputeGraspPixel(
        target, grasp_x, grasp_y, 0.0f,
        SafeInteriorConfig(), &offset_angle);
    assert(valid);
    assert(!std::isfinite(offset_angle));
    const int x = static_cast<int>(std::lround(grasp_x));
    const int y = static_cast<int>(std::lround(grasp_y));
    assert(target.mask.at<uint8_t>(y, x) != 0);
    assert(std::hypot(grasp_x - target.center.x,
                    grasp_y - target.center.y) > 5.0f);
    cv::Mat distance;
    cv::distanceTransform(target.mask, distance, cv::DIST_L2, 5);
    double maximum_clearance = 0.0;
    cv::minMaxLoc(distance, nullptr, &maximum_clearance);
    assert(distance.at<float>(y, x) >= 0.70 * maximum_clearance);
}

void CheckElongatedConcaveTargetUsesLocalWidth() {
    perceptive_grasp::DetectionTarget target{};
    target.mask = cv::Mat::zeros(320, 400, CV_8UC1);
    cv::ellipse(
        target.mask, cv::Point(200, 160), cv::Size(125, 65), 0.0,
        35.0, 325.0, cv::Scalar(255), 34);
    target.center = cv::Point2f(200.0f, 160.0f);

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_angle = NAN;
    assert(perceptive_grasp::ComputeGraspPixel(
        target, center_x, center_y, 0.0f,
        SafeInteriorConfig(), &center_angle));
    cv::Mat center_distance;
    cv::distanceTransform(
        target.mask, center_distance, cv::DIST_L2, 5);
    double maximum_clearance = 0.0;
    cv::minMaxLoc(
        center_distance, nullptr, &maximum_clearance);
    assert(center_distance.at<float>(
        static_cast<int>(std::lround(center_y)),
        static_cast<int>(std::lround(center_x))) >=
        0.90 * maximum_clearance);

    float edge_x = 0.0f;
    float edge_y = 0.0f;
    float edge_angle = NAN;
    assert(perceptive_grasp::ComputeGraspPixel(
        target, edge_x, edge_y, 1.0f,
        SafeInteriorConfig(), &edge_angle));
    assert(std::isfinite(edge_angle));

    const int center_ix = static_cast<int>(std::lround(center_x));
    const int center_iy = static_cast<int>(std::lround(center_y));
    const int edge_ix = static_cast<int>(std::lround(edge_x));
    const int edge_iy = static_cast<int>(std::lround(edge_y));
    assert(target.mask.at<uint8_t>(center_iy, center_ix) != 0);
    assert(target.mask.at<uint8_t>(edge_iy, edge_ix) != 0);
    assert(std::hypot(edge_x - center_x, edge_y - center_y) < 30.0f);
}

void CheckConcaveTargetWithRectCenterInsideUsesLocalWidth() {
    perceptive_grasp::DetectionTarget target{};
    target.mask = cv::Mat::zeros(320, 400, CV_8UC1);
    cv::line(
        target.mask, cv::Point(90, 70), cv::Point(190, 235),
        cv::Scalar(255), 38);
    cv::line(
        target.mask, cv::Point(190, 235), cv::Point(320, 175),
        cv::Scalar(255), 38);
    // Model masks may contain a thin bridge across the concavity. This makes
    // the global rectangle center technically foreground, but not a safe
    // grasp cross-section.
    cv::line(
        target.mask, cv::Point(190, 235), cv::Point(182, 172),
        cv::Scalar(255), 7);
    target.center = cv::Point2f(182.0f, 172.0f);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        target.mask, contours, cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);
    assert(!contours.empty());
    const cv::RotatedRect rect = cv::minAreaRect(contours.front());
    const int rect_x = static_cast<int>(std::lround(rect.center.x));
    const int rect_y = static_cast<int>(std::lround(rect.center.y));
    assert(target.mask.at<uint8_t>(rect_y, rect_x) != 0);

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_angle = NAN;
    assert(perceptive_grasp::ComputeGraspPixel(
        target, center_x, center_y, 0.0f,
        SafeInteriorConfig(), &center_angle));
    assert(std::hypot(
        center_x - rect.center.x, center_y - rect.center.y) > 20.0f);

    float edge_x = 0.0f;
    float edge_y = 0.0f;
    float edge_angle = NAN;
    assert(perceptive_grasp::ComputeGraspPixel(
        target, edge_x, edge_y, 1.0f,
        SafeInteriorConfig(), &edge_angle));
    assert(std::isfinite(edge_angle));
    assert(target.mask.at<uint8_t>(
        static_cast<int>(std::lround(edge_y)),
        static_cast<int>(std::lround(edge_x))) != 0);
    assert(std::hypot(edge_x - center_x, edge_y - center_y) < 35.0f);
}

}  // namespace

int main() {
    for (float angle_deg = 0.0f; angle_deg < 180.0f; angle_deg += 15.0f) {
        CheckRotatedTarget(angle_deg);
    }
    CheckConcaveTargetUsesMaskInterior();
    CheckElongatedConcaveTargetUsesLocalWidth();
    CheckConcaveTargetWithRectCenterInsideUsesLocalWidth();
    return 0;
}
