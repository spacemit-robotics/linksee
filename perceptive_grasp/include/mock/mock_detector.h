/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file mock_detector.h
    * @brief X86 Standalone 模式下的目标检测 mock
    *
    * 使用 OpenCV DNN 直接加载 YOLOv8 ONNX 模型，
    * 不依赖 VisionService SDK，可在 x86 上独立运行。
    */

#ifndef MOCK_DETECTOR_H
#define MOCK_DETECTOR_H

#include "target_detector.h"

#include <opencv2/dnn.hpp>

namespace perceptive_grasp {

/**
    * @brief OpenCV DNN 版目标检测器 (x86 standalone)
    *
    * 直接用 cv::dnn::readNetFromONNX 加载 YOLOv8 模型。
    * 接口与 TargetDetector 完全一致。
    */
class MockDetector : public TargetDetector {
public:
    explicit MockDetector(const DetectorConfig& config);
    ~MockDetector() override = default;

    bool Init() override;
    bool Detect(const cv::Mat& image,
                std::vector<DetectionTarget>& targets) override;
    bool DetectBest(const cv::Mat& image, DetectionTarget& target) override;
    bool DetectByName(const cv::Mat& image, const std::string& target_name,
                        DetectionTarget& target) override;

private:
    cv::dnn::Net net_;
    float conf_threshold_ = 0.5f;
    float nms_threshold_ = 0.45f;
    int input_size_ = 640;

    bool LoadLabels(const std::string& label_file);
    void PostprocessYOLOv8(const cv::Mat& output, const cv::Mat& image,
                            std::vector<DetectionTarget>& targets);
};

}  // namespace perceptive_grasp

#endif  // MOCK_DETECTOR_H
