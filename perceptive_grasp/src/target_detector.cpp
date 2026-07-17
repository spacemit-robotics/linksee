/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file target_detector.cpp
    * @brief 目标检测模块实现
    */

#include "target_detector.h"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace perceptive_grasp {

bool TargetDetector::Init() {
    // Eagerly create the ORT session during pipeline initialization. Keeping
    // lazy loading enabled adds model loading to the first detection frame,
    // which makes the first grasp command noticeably slower.
    vision_ = VisionService::Create(config_.config_path, "", false);
    if (!vision_) {
        std::cerr << "[TargetDetector] Failed to create VisionService: "
                    << VisionService::LastCreateError() << std::endl;
        return false;
    }

    // 加载类别名称
    std::string label_file = vision_->GetConfigPathValue("label_file_path");
    if (!label_file.empty()) {
        LoadLabelNames(label_file);
    }

    std::cout << "[TargetDetector] Initialized with " << label_names_.size()
                << " labels" << std::endl;
    return true;
}

bool TargetDetector::Detect(const cv::Mat& image,
                            std::vector<DetectionTarget>& targets) {
    targets.clear();

    if (!vision_ || image.empty()) return false;

    std::vector<VisionServiceResult> results;
    VisionServiceResponse response;
    auto status = InferImageDetections(
        vision_.get(), image, &results, &response);
    if (status != VISION_SERVICE_OK) {
        std::cerr << "[TargetDetector] Inference failed: status="
                  << static_cast<int>(status)
                  << " image=" << image.cols << "x" << image.rows
                  << " type=" << image.type()
                  << " continuous=" << (image.isContinuous() ? 1 : 0);
        if (!response.error_message.empty()) {
            std::cerr << " response=\"" << response.error_message << '"';
        }
        const std::string& service_error = vision_->LastError();
        if (!service_error.empty() &&
            service_error != response.error_message) {
            std::cerr << " service=\"" << service_error << '"';
        }
        std::cerr << std::endl;
        return false;
    }

    for (const auto& r : results) {
        if (!PassFilter(r)) continue;

        DetectionTarget target;
        target.x1 = r.x1;
        target.y1 = r.y1;
        target.x2 = r.x2;
        target.y2 = r.y2;
        target.score = r.score;
        target.label = r.label;
        target.center = cv::Point2f((r.x1 + r.x2) / 2.0f,
                                    (r.y1 + r.y2) / 2.0f);
        target.area = (r.x2 - r.x1) * (r.y2 - r.y1);
        target.mask = r.mask.clone();

        if (r.label >= 0 && r.label < static_cast<int>(label_names_.size())) {
            target.label_name = label_names_[r.label];
        }

        targets.push_back(std::move(target));
    }

    // 按面积降序排列
    std::sort(targets.begin(), targets.end(),
                [](const DetectionTarget& a, const DetectionTarget& b) {
                    return a.area > b.area;
                });

    return true;
}

bool TargetDetector::DetectBest(const cv::Mat& image, DetectionTarget& target) {
    std::vector<DetectionTarget> targets;
    if (!Detect(image, targets) || targets.empty()) {
        return false;
    }
    target = targets[0];  // 最大的
    return true;
}

bool TargetDetector::DetectByName(const cv::Mat& image,
                                    const std::string& target_name,
                                    DetectionTarget& target) {
    std::vector<DetectionTarget> targets;
    if (!Detect(image, targets)) return false;

    for (const auto& t : targets) {
        if (t.label_name == target_name) {
            target = t;
            return true;
        }
    }
    return false;
}

bool TargetDetector::LoadLabelNames(const std::string& label_file) {
    std::ifstream ifs(label_file);
    if (!ifs.is_open()) {
        std::cerr << "[TargetDetector] Cannot open label file: " << label_file
                    << std::endl;
        return false;
    }

    label_names_.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        // 去除首尾空白
        auto start = line.find_first_not_of(" \t\r\n");
        auto end = line.find_last_not_of(" \t\r\n");
        if (start != std::string::npos) {
            label_names_.push_back(line.substr(start, end - start + 1));
        }
    }
    return true;
}

bool TargetDetector::PassFilter(const VisionServiceResult& result) const {
    // 置信度过滤
    if (result.score < config_.min_confidence) return false;

    // 面积过滤
    float area = (result.x2 - result.x1) * (result.y2 - result.y1);
    if (area < config_.min_area) return false;

    // 类别过滤
    if (!config_.target_labels.empty()) {
        bool found = false;
        for (int label : config_.target_labels) {
            if (result.label == label) {
                found = true;
                break;
            }
        }
        if (!found) return false;
    }

    return true;
}

}  // namespace perceptive_grasp
