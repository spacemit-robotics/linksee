/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file target_association.cpp
 * @brief Associate one detected object instance across perception cycles.
 */

#include "target_association.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perceptive_grasp {

namespace {

constexpr float kMinimumBoxExtent = 1.0f;
constexpr float kMinimumAssociationScale = 24.0f;

float BoxWidth(const DetectionTarget& target) {
    return std::max(kMinimumBoxExtent, target.x2 - target.x1);
}

float BoxHeight(const DetectionTarget& target) {
    return std::max(kMinimumBoxExtent, target.y2 - target.y1);
}

float BoxArea(const DetectionTarget& target) {
    if (target.area > 0.0f) return target.area;
    return BoxWidth(target) * BoxHeight(target);
}

bool MatchesLabel(const DetectionTarget& target,
    const std::string& requested_label,
    const TargetTrack& previous) {
    if (!requested_label.empty() &&
        target.label_name != requested_label) {
        return false;
    }
    return !previous.valid ||
        target.label_name == previous.label_name;
}

}  // namespace

TargetAssociationResult SelectTargetInstance(
    const std::vector<DetectionTarget>& candidates,
    const std::string& requested_label,
    const TargetTrack& previous,
    const TargetAssociationConfig& config) {
    TargetAssociationResult result;
    float best_cost = std::numeric_limits<float>::max();
    float best_score = -std::numeric_limits<float>::max();
    bool saw_matching_label = false;

    for (size_t index = 0; index < candidates.size(); ++index) {
        const DetectionTarget& candidate = candidates[index];
        if (!MatchesLabel(candidate, requested_label, previous)) continue;
        saw_matching_label = true;

        if (!previous.valid) {
            if (candidate.score > best_score) {
                best_score = candidate.score;
                result.index = static_cast<int>(index);
            }
            continue;
        }

        const float width = BoxWidth(candidate);
        const float height = BoxHeight(candidate);
        const float area = BoxArea(candidate);
        const float previous_area = std::max(
            previous.area, kMinimumBoxExtent);
        const float area_ratio = area / previous_area;
        const float center_distance = cv::norm(
            candidate.center - previous.center);
        const float association_scale = std::max(
            kMinimumAssociationScale,
            std::hypot(previous.width, previous.height));
        const float center_distance_ratio =
            center_distance / association_scale;
        if (center_distance_ratio >
                config.max_center_distance_ratio ||
            area_ratio < config.min_area_ratio ||
            area_ratio > config.max_area_ratio) {
            continue;
        }

        const float previous_aspect = previous.width /
            std::max(previous.height, kMinimumBoxExtent);
        const float aspect = width / height;
        const float area_change = std::fabs(std::log(area_ratio));
        const float aspect_change = std::fabs(std::log(
            aspect / std::max(previous_aspect, 1e-3f)));
        const float cost = center_distance_ratio +
            0.60f * area_change +
            0.30f * aspect_change -
            0.10f * std::clamp(candidate.score, 0.0f, 1.0f);
        if (cost < best_cost) {
            best_cost = cost;
            result.index = static_cast<int>(index);
        }
    }

    if (result.index >= 0) {
        result.cost = previous.valid ? best_cost : 0.0f;
        result.matched_existing_track = previous.valid;
        result.reason = previous.valid
            ? "matched tracked instance"
            : "started new target track";
    } else if (saw_matching_label && previous.valid) {
        result.reason =
            "same-label detections did not match the tracked instance";
    } else {
        result.reason = "requested target label was not detected";
    }
    return result;
}

TargetTrack UpdateTargetTrack(const DetectionTarget& target) {
    TargetTrack track;
    track.valid = true;
    track.label_name = target.label_name;
    track.center = target.center;
    track.width = BoxWidth(target);
    track.height = BoxHeight(target);
    track.area = BoxArea(target);
    return track;
}

bool AreTargetTracksStationary(const TargetTrack& previous,
    const TargetTrack& current,
    std::string* reason) {
    if (!previous.valid || !current.valid ||
        previous.label_name != current.label_name) {
        if (reason) *reason = "target track started or changed";
        return false;
    }

    constexpr float kMinimumCenterTolerancePx = 4.0f;
    constexpr float kCenterToleranceScale = 0.05f;
    constexpr float kMinimumAreaRatio = 0.80f;
    constexpr float kMaximumAreaRatio = 1.25f;
    const float box_diagonal = std::hypot(
        std::max(previous.width, kMinimumBoxExtent),
        std::max(previous.height, kMinimumBoxExtent));
    const float center_tolerance = std::max(
        kMinimumCenterTolerancePx,
        kCenterToleranceScale * box_diagonal);
    const float center_delta = cv::norm(current.center - previous.center);
    const float area_ratio = current.area /
        std::max(previous.area, kMinimumBoxExtent);
    const bool stationary =
        center_delta <= center_tolerance &&
        area_ratio >= kMinimumAreaRatio &&
        area_ratio <= kMaximumAreaRatio;

    if (reason) {
        *reason = "center_delta=" + std::to_string(center_delta) +
            "px tolerance=" + std::to_string(center_tolerance) +
            "px area_ratio=" + std::to_string(area_ratio);
    }
    return stationary;
}

}  // namespace perceptive_grasp
