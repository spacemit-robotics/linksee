/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_depth.h
 * @brief Depth sampling helpers for mask-based grasp points.
 */

#ifndef GRASP_DEPTH_H
#define GRASP_DEPTH_H

#include <cstddef>
#include <cstdint>

#include <opencv2/core.hpp>

namespace perceptive_grasp {

// Grasp planning is limited to the near-field manipulation workspace. Values
// outside this range are invalid/stale stereo samples and must never feed arm
// or mobile-base motion.
constexpr uint16_t kMinimumGraspDepthMm = 50;
constexpr uint16_t kMaximumGraspDepthMm = 2000;

inline bool IsValidGraspDepth(uint16_t depth_mm) {
    return depth_mm >= kMinimumGraspDepthMm &&
        depth_mm <= kMaximumGraspDepthMm;
}

struct GraspDepthSample {
    int x = 0;
    int y = 0;
    uint16_t depth_mm = 0;
    size_t sample_count = 0;
};

bool SampleMaskedDepthNearPixel(const cv::Mat& depth,
                                const cv::Mat& input_mask,
                                int requested_x,
                                int requested_y,
                                int search_radius,
                                GraspDepthSample* sample);

}  // namespace perceptive_grasp

#endif  // GRASP_DEPTH_H
