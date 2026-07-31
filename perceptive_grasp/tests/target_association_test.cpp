/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "target_association.h"

#include <iostream>
#include <string>
#include <vector>

using perceptive_grasp::DetectionTarget;
using perceptive_grasp::AreTargetTracksStationary;
using perceptive_grasp::SelectTargetInstance;
using perceptive_grasp::TargetTrack;
using perceptive_grasp::UpdateTargetTrack;

namespace {

DetectionTarget MakeTarget(const std::string& label,
    float center_x,
    float center_y,
    float width,
    float height,
    float score) {
    DetectionTarget target{};
    target.label_name = label;
    target.center = cv::Point2f(center_x, center_y);
    target.x1 = center_x - width * 0.5f;
    target.y1 = center_y - height * 0.5f;
    target.x2 = center_x + width * 0.5f;
    target.y2 = center_y + height * 0.5f;
    target.area = width * height;
    target.score = score;
    return target;
}

}  // namespace

int main() {
    const DetectionTarget first =
        MakeTarget("cup", 120.0f, 200.0f, 70.0f, 110.0f, 0.82f);
    const DetectionTarget second =
        MakeTarget("cup", 430.0f, 205.0f, 75.0f, 115.0f, 0.94f);

    const auto initial = SelectTargetInstance(
        {first, second}, "cup", TargetTrack{});
    if (initial.index != 1 || initial.matched_existing_track) {
        std::cerr << "new target track did not select best confidence"
                    << std::endl;
        return 1;
    }

    const TargetTrack track = UpdateTargetTrack(first);
    const DetectionTarget moved_first =
        MakeTarget("cup", 148.0f, 196.0f, 74.0f, 108.0f, 0.78f);
    const auto reordered = SelectTargetInstance(
        {second, moved_first}, "cup", track);
    if (reordered.index != 1 || !reordered.matched_existing_track) {
        std::cerr << "same-label reorder switched target instance"
                    << std::endl;
        return 1;
    }

    const DetectionTarget unrelated =
        MakeTarget("cup", 580.0f, 50.0f, 18.0f, 20.0f, 0.99f);
    const auto rejected = SelectTargetInstance(
        {unrelated}, "cup", track);
    if (rejected.index >= 0 ||
        rejected.reason.find("tracked instance") == std::string::npos) {
        std::cerr << "unrelated same-label target was accepted" << std::endl;
        return 1;
    }

    const auto wrong_label = SelectTargetInstance(
        {first}, "banana", TargetTrack{});
    if (wrong_label.index >= 0) {
        std::cerr << "wrong label was accepted" << std::endl;
        return 1;
    }

    std::string stationarity_reason;
    const TargetTrack stable_track = UpdateTargetTrack(
        MakeTarget("cup", 122.0f, 201.0f, 72.0f, 108.0f, 0.80f));
    if (!AreTargetTracksStationary(
            track, stable_track, &stationarity_reason)) {
        std::cerr << "detector jitter was treated as target motion: "
                    << stationarity_reason << std::endl;
        return 1;
    }

    const TargetTrack moving_track = UpdateTargetTrack(
        MakeTarget("cup", 145.0f, 201.0f, 72.0f, 108.0f, 0.80f));
    if (AreTargetTracksStationary(
            track, moving_track, &stationarity_reason)) {
        std::cerr << "moving target was treated as stationary" << std::endl;
        return 1;
    }

    const TargetTrack resized_track = UpdateTargetTrack(
        MakeTarget("cup", 121.0f, 200.0f, 100.0f, 140.0f, 0.80f));
    if (AreTargetTracksStationary(
            track, resized_track, &stationarity_reason)) {
        std::cerr << "changing target silhouette was treated as stationary"
                    << std::endl;
        return 1;
    }

    std::cout << "target_association_test: PASS" << std::endl;
    return 0;
}
