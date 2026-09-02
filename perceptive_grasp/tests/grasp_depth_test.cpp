/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_depth_test.cpp
 * @brief Unit tests for mask-constrained grasp depth sampling.
 */

#include <cassert>

#include <opencv2/core.hpp>

#include "grasp_depth.h"

int main() {
    cv::Mat depth = cv::Mat::zeros(20, 20, CV_16UC1);
    cv::Mat mask = cv::Mat::zeros(20, 20, CV_8UC1);
    for (int y = 4; y <= 15; ++y) {
        for (int x = 4; x <= 15; ++x) {
            mask.at<uint8_t>(y, x) = 255;
            depth.at<uint16_t>(y, x) = x < 10 ? 700 : 1100;
        }
    }

    perceptive_grasp::GraspDepthSample right_sample;
    assert(perceptive_grasp::SampleMaskedDepthNearPixel(
        depth, mask, 14, 10, 4, &right_sample));
    assert(right_sample.x == 14);
    assert(right_sample.y == 10);
    assert(right_sample.depth_mm == 1100);

    depth.at<uint16_t>(10, 15) = 0;
    perceptive_grasp::GraspDepthSample snapped_sample;
    assert(perceptive_grasp::SampleMaskedDepthNearPixel(
        depth, mask, 16, 10, 4, &snapped_sample));
    assert(snapped_sample.x == 15);
    assert(snapped_sample.y == 9 || snapped_sample.y == 11);
    assert(snapped_sample.depth_mm == 1100);

    cv::Mat empty_depth = cv::Mat::zeros(20, 20, CV_16UC1);
    perceptive_grasp::GraspDepthSample invalid_sample;
    assert(!perceptive_grasp::SampleMaskedDepthNearPixel(
        empty_depth, mask, 10, 10, 4, &invalid_sample));

    cv::Mat stale_depth(20, 20, CV_16UC1, cv::Scalar(20434));
    assert(!perceptive_grasp::SampleMaskedDepthNearPixel(
        stale_depth, mask, 10, 10, 4, &invalid_sample));

    stale_depth.at<uint16_t>(10, 11) = 900;
    perceptive_grasp::GraspDepthSample recovered_sample;
    assert(perceptive_grasp::SampleMaskedDepthNearPixel(
        stale_depth, mask, 10, 10, 4, &recovered_sample));
    assert(recovered_sample.x == 11);
    assert(recovered_sample.y == 10);
    assert(recovered_sample.depth_mm == 900);

    return 0;
}
