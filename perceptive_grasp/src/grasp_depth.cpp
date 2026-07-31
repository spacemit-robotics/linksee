/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_depth.cpp
 * @brief Depth sampling helpers for mask-based grasp points.
 */

#include "grasp_depth.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {
namespace {

bool NormalizeMask(
    const cv::Mat& input_mask,
    const cv::Size& output_size,
    cv::Mat* mask) {
    if (input_mask.empty() || mask == nullptr) {
        return false;
    }

    if (input_mask.size() == output_size) {
        *mask = input_mask.clone();
    } else {
        cv::resize(input_mask, *mask, output_size, 0.0, 0.0,
            cv::INTER_NEAREST);
    }

    if (mask->channels() == 3) {
        cv::cvtColor(*mask, *mask, cv::COLOR_BGR2GRAY);
    } else if (mask->channels() == 4) {
        cv::cvtColor(*mask, *mask, cv::COLOR_BGRA2GRAY);
    } else if (mask->channels() != 1) {
        return false;
    }
    if (mask->type() != CV_8UC1) {
        mask->convertTo(*mask, CV_8UC1);
    }
    cv::threshold(*mask, *mask, 0, 255, cv::THRESH_BINARY);
    return true;
}

}  // namespace

bool SampleMaskedDepthNearPixel(const cv::Mat& depth,
                                const cv::Mat& input_mask,
                                int requested_x,
                                int requested_y,
                                int search_radius,
                                GraspDepthSample* sample) {
    if (sample == nullptr || depth.empty() || depth.type() != CV_16UC1 ||
        search_radius < 0) {
        return false;
    }

    cv::Mat mask;
    if (!NormalizeMask(input_mask, depth.size(), &mask)) {
        return false;
    }

    requested_x = std::clamp(requested_x, 0, depth.cols - 1);
    requested_y = std::clamp(requested_y, 0, depth.rows - 1);
    const int x_min = std::max(0, requested_x - search_radius);
    const int x_max = std::min(depth.cols - 1, requested_x + search_radius);
    const int y_min = std::max(0, requested_y - search_radius);
    const int y_max = std::min(depth.rows - 1, requested_y + search_radius);

    int sample_x = -1;
    int sample_y = -1;
    int nearest_distance_sq = std::numeric_limits<int>::max();
    for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
            if (mask.at<uint8_t>(y, x) == 0 ||
                depth.at<uint16_t>(y, x) == 0) {
                continue;
            }
            const int dx = x - requested_x;
            const int dy = y - requested_y;
            const int distance_sq = dx * dx + dy * dy;
            if (distance_sq < nearest_distance_sq) {
                nearest_distance_sq = distance_sq;
                sample_x = x;
                sample_y = y;
            }
        }
    }
    if (sample_x < 0 || sample_y < 0) {
        return false;
    }

    constexpr int kMedianRadius = 2;
    std::vector<uint16_t> depth_values;
    for (int y = std::max(0, sample_y - kMedianRadius);
        y <= std::min(depth.rows - 1, sample_y + kMedianRadius); ++y) {
        for (int x = std::max(0, sample_x - kMedianRadius);
            x <= std::min(depth.cols - 1, sample_x + kMedianRadius); ++x) {
            const uint16_t value = depth.at<uint16_t>(y, x);
            if (mask.at<uint8_t>(y, x) != 0 && value > 0) {
                depth_values.push_back(value);
            }
        }
    }
    if (depth_values.empty()) {
        return false;
    }

    const size_t median_index = depth_values.size() / 2;
    std::nth_element(depth_values.begin(),
        depth_values.begin() + median_index,
        depth_values.end());
    sample->x = sample_x;
    sample->y = sample_y;
    sample->depth_mm = depth_values[median_index];
    sample->sample_count = depth_values.size();
    return true;
}

}  // namespace perceptive_grasp
