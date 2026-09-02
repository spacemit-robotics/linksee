/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file simulation_color_detector_test.cpp
 * @brief Tests for the synthetic-scene color instance fallback.
 */

#include "target_detector.h"

#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace {

cv::Scalar HsvToBgr(int hue, int saturation, int value) {
    cv::Mat hsv(1, 1, CV_8UC3,
                cv::Scalar(hue, saturation, value));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    return bgr.at<cv::Vec3b>(0, 0);
}

}  // namespace

int main() {
    perceptive_grasp::DetectorConfig config;
    perceptive_grasp::SimulationColorTargetConfig banana;
    banana.label = "banana";
    banana.hsv_min = cv::Scalar(20, 150, 150);
    banana.hsv_max = cv::Scalar(35, 255, 255);
    banana.min_area = 100.0f;
    banana.max_area = 2000.0f;
    config.simulation_color_targets.push_back(banana);
    config.allow_color_only_fallback = true;

    cv::Mat image(180, 240, CV_8UC3, cv::Scalar(20, 20, 20));
    cv::rectangle(image, cv::Rect(40, 30, 60, 25),
                HsvToBgr(27, 230, 240), cv::FILLED);
    cv::rectangle(image, cv::Rect(20, 90, 200, 60),
                HsvToBgr(27, 230, 240), cv::FILLED);

    std::vector<perceptive_grasp::DetectionTarget> targets;
    perceptive_grasp::AppendSimulationColorTargets(image, config, &targets);
    if (targets.size() != 1 || targets[0].label_name != "banana" ||
        targets[0].mask.empty() ||
        cv::countNonZero(targets[0].mask) < 1200) {
        std::cerr << "configured color target was not detected" << std::endl;
        return 1;
    }

    perceptive_grasp::AppendSimulationColorTargets(image, config, &targets);
    if (targets.size() != 1) {
        std::cerr << "color fallback duplicated an existing target" << std::endl;
        return 1;
    }

    targets[0].mask = cv::Mat::zeros(image.size(), CV_8UC1);
    targets[0].score = 0.72f;
    config.refine_with_simulation_colors = true;
    perceptive_grasp::AppendSimulationColorTargets(image, config, &targets);
    if (targets.size() != 1 || targets[0].score != 0.72f ||
        cv::countNonZero(targets[0].mask) < 1200) {
        std::cerr << "simulation color mask did not refine yolo geometry"
            << std::endl;
        return 1;
    }

    config.simulation_color_targets.clear();
    targets.clear();
    perceptive_grasp::AppendSimulationColorTargets(image, config, &targets);
    if (!targets.empty()) {
        std::cerr << "disabled color fallback produced a target" << std::endl;
        return 1;
    }
    return 0;
}
