/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mock_detector_test.cpp
 * @brief OpenCV YOLOv8 segmentation post-processing regression test.
 */

#include "mock/mock_detector.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

namespace perceptive_grasp {

struct MockDetectorTestAccess {
    static void Postprocess(MockDetector* detector,
                            const std::vector<cv::Mat>& outputs,
                            const cv::Mat& image,
                            std::vector<DetectionTarget>* targets) {
        detector->PostprocessYOLOv8(outputs, image, *targets);
    }
};

}  // namespace perceptive_grasp

int main() {
    using perceptive_grasp::DetectionTarget;
    using perceptive_grasp::DetectorConfig;
    using perceptive_grasp::MockDetector;
    using perceptive_grasp::MockDetectorTestAccess;

    DetectorConfig config;
    config.min_confidence = 0.25f;
    config.min_area = 10.0f;
    MockDetector detector(config);

    constexpr int kClassCount = 80;
    constexpr int kMaskDimensions = 2;
    int prediction_shape[] = {1, 4 + kClassCount + kMaskDimensions, 1};
    cv::Mat prediction(3, prediction_shape, CV_32F, cv::Scalar(0));
    float* values = prediction.ptr<float>();
    values[0] = 320.0f;
    values[1] = 320.0f;
    values[2] = 320.0f;
    values[3] = 320.0f;
    values[4 + 46] = 0.95f;
    values[4 + kClassCount] = 8.0f;
    values[4 + kClassCount + 1] = -8.0f;

    int prototype_shape[] = {1, kMaskDimensions, 4, 4};
    cv::Mat prototype(4, prototype_shape, CV_32F, cv::Scalar(0));
    float* prototype_values = prototype.ptr<float>();
    std::fill(prototype_values, prototype_values + 16, 1.0f);

    const cv::Mat image = cv::Mat::zeros(64, 64, CV_8UC3);
    std::vector<DetectionTarget> targets;
    MockDetectorTestAccess::Postprocess(
        &detector, {prediction, prototype}, image, &targets);

    if (targets.size() != 1 || targets[0].label != 46 ||
        targets[0].mask.size() != image.size() ||
        targets[0].mask.type() != CV_8U ||
        cv::countNonZero(targets[0].mask) == 0) {
        std::cerr << "YOLOv8 segmentation output was not reconstructed"
                << std::endl;
        return 1;
    }
    return 0;
}
