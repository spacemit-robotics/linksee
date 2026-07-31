/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file target_association.h
 * @brief Associate one detected object instance across perception cycles.
 */

#ifndef TARGET_ASSOCIATION_H
#define TARGET_ASSOCIATION_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "target_detector.h"

namespace perceptive_grasp {

struct TargetTrack {
    bool valid = false;
    std::string label_name;
    cv::Point2f center{};
    float width = 0.0f;
    float height = 0.0f;
    float area = 0.0f;
};

struct TargetAssociationConfig {
    float max_center_distance_ratio = 4.0f;
    float min_area_ratio = 0.20f;
    float max_area_ratio = 5.0f;
};

struct TargetAssociationResult {
    int index = -1;
    float cost = 0.0f;
    bool matched_existing_track = false;
    std::string reason;
};

TargetAssociationResult SelectTargetInstance(
    const std::vector<DetectionTarget>& candidates,
    const std::string& requested_label,
    const TargetTrack& previous,
    const TargetAssociationConfig& config = {});

TargetTrack UpdateTargetTrack(const DetectionTarget& target);

bool AreTargetTracksStationary(const TargetTrack& previous,
    const TargetTrack& current,
    std::string* reason = nullptr);

}  // namespace perceptive_grasp

#endif  // TARGET_ASSOCIATION_H
