/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file target_detector.h
    * @brief 目标检测模块 - 封装 VisionService 用于抓取目标识别
    */

#ifndef TARGET_DETECTOR_H
#define TARGET_DETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#ifndef MOCK_DETECTOR
#include "vision_result_adapter.h"
#endif

namespace perceptive_grasp {

struct DetectionTarget {
    float x1, y1, x2, y2;   // bbox
    float score;             // 置信度
    int label;               // COCO 类别 ID
    std::string label_name;  // 类别名称
    cv::Point2f center;      // bbox 中心像素坐标
    cv::Mat mask;            // 分割 mask (如果有)
    float area;              // bbox 面积 (像素^2)
};

struct DetectorConfig {
    std::string config_path;           // VisionService YAML 配置路径
    std::vector<int> target_labels;    // 目标类别过滤 (空=全部)
    float min_confidence = 0.5f;
    float min_area = 1000.0f;
};

/**
    * @brief 目标检测器
    *
    * 封装 VisionService，提供面向抓取场景的检测接口。
    */
class TargetDetector {
public:
    explicit TargetDetector(const DetectorConfig& config) : config_(config) {}
    virtual ~TargetDetector() = default;

#ifdef MOCK_DETECTOR
    virtual bool Init() = 0;
    virtual bool Detect(const cv::Mat& image, std::vector<DetectionTarget>& targets) = 0;
    virtual bool DetectBest(const cv::Mat& image, DetectionTarget& target) = 0;
    virtual bool DetectByName(const cv::Mat& image, const std::string& target_name,
                                DetectionTarget& target) = 0;
#else
    virtual bool Init();
    virtual bool Detect(const cv::Mat& image, std::vector<DetectionTarget>& targets);
    virtual bool DetectBest(const cv::Mat& image, DetectionTarget& target);
    virtual bool DetectByName(const cv::Mat& image, const std::string& target_name,
                                DetectionTarget& target);
#endif

protected:
    DetectorConfig config_;
    std::vector<std::string> label_names_;
    bool LoadLabelNames(const std::string& label_file);

#ifndef MOCK_DETECTOR
private:
    std::unique_ptr<VisionService> vision_;
    bool PassFilter(const VisionServiceResult& result) const;
#endif
};

}  // namespace perceptive_grasp

#endif  // TARGET_DETECTOR_H
