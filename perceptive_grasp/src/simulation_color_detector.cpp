/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file simulation_color_detector.cpp
 * @brief Configurable instance fallback for synthetic mujoco images.
 */

#include "target_detector.h"

#include <algorithm>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {

void AppendSimulationColorTargets(
    const cv::Mat& image, const DetectorConfig& config,
    std::vector<DetectionTarget>* targets) {
    if (image.empty() || targets == nullptr ||
        config.simulation_color_targets.empty()) {
        return;
    }

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(3, 3));
    std::unordered_set<std::string> refined_labels;

    for (const auto& color_target : config.simulation_color_targets) {
        if (refined_labels.count(color_target.label) != 0) {
            continue;
        }
        auto existing = std::find_if(
            targets->begin(), targets->end(),
            [&color_target](const DetectionTarget& target) {
                return target.label_name == color_target.label;
            });
        if (existing != targets->end() &&
            !config.refine_with_simulation_colors) {
            continue;
        }
        if (existing == targets->end() &&
            !config.allow_color_only_fallback) {
            continue;
        }

        cv::Mat mask;
        cv::inRange(hsv, color_target.hsv_min, color_target.hsv_max, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                        cv::CHAIN_APPROX_SIMPLE);
        auto contour = contours.end();
        double contour_area = 0.0;
        for (auto candidate = contours.begin(); candidate != contours.end();
                ++candidate) {
            const double area = cv::contourArea(*candidate);
            const bool in_range = area >= color_target.min_area &&
                (color_target.max_area <= 0.0f ||
                    area <= color_target.max_area);
            if (in_range && area > contour_area) {
                contour = candidate;
                contour_area = area;
            }
        }
        if (contour == contours.end()) {
            continue;
        }

        cv::Mat instance_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(instance_mask, contours,
                        static_cast<int>(contour - contours.begin()),
                        cv::Scalar(255), cv::FILLED);
        const cv::Rect bounds = cv::boundingRect(*contour);

        DetectionTarget target = existing == targets->end()
            ? DetectionTarget{} : *existing;
        target.x1 = static_cast<float>(bounds.x);
        target.y1 = static_cast<float>(bounds.y);
        target.x2 = static_cast<float>(bounds.x + bounds.width);
        target.y2 = static_cast<float>(bounds.y + bounds.height);
        if (existing == targets->end()) {
            target.score = color_target.score;
            target.label = -1;
            target.label_name = color_target.label;
        }
        target.center = cv::Point2f(
            0.5f * (target.x1 + target.x2),
            0.5f * (target.y1 + target.y2));
        target.mask = std::move(instance_mask);
        target.area = static_cast<float>(bounds.area());
        if (existing == targets->end()) {
            targets->push_back(std::move(target));
        } else {
            *existing = std::move(target);
        }
        refined_labels.insert(color_target.label);
    }

    std::sort(targets->begin(), targets->end(),
            [](const DetectionTarget& lhs, const DetectionTarget& rhs) {
                return lhs.area > rhs.area;
            });
}

}  // namespace perceptive_grasp
