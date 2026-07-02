/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file mock_detector.cpp
    * @brief X86 Standalone 目标检测 - 使用 OpenCV DNN 加载 YOLOv8
    */

#include "mock/mock_detector.h"

#include <algorithm>
#include <fstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {

MockDetector::MockDetector(const DetectorConfig& config)
    : TargetDetector(config) {
    conf_threshold_ = config.min_confidence;
}

bool MockDetector::Init() {
    // 尝试从配置中获取模型路径
    // config_path 在 x86 模式下直接指向 ONNX 文件
    std::string model_path = config_.config_path;

    // 如果是 yaml 配置文件，尝试从中读取 model_path
    if (model_path.find(".yaml") != std::string::npos ||
        model_path.find(".yml") != std::string::npos) {
        // 简单处理: 在 x86 模式下，直接用默认路径
        model_path = "models/yolov8n.onnx";
        std::cout << "[MockDetector] YAML config detected, using default model: "
                    << model_path << std::endl;
    }

    // 展开 ~ 路径
    if (!model_path.empty() && model_path[0] == '~') {
        const char* home = getenv("HOME");
        if (home) model_path = std::string(home) + model_path.substr(1);
    }

    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
        if (net_.empty()) {
            std::cerr << "[MockDetector] Failed to load model: " << model_path
                        << std::endl;
            std::cerr << "[MockDetector] Falling back to dummy mode (no detection)"
                        << std::endl;
            return true;  // 允许无模型运行 (用于纯 pipeline 测试)
        }

        // 优先用 CUDA，没有就 CPU
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        std::cout << "[MockDetector] Model loaded: " << model_path << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "[MockDetector] OpenCV DNN error: " << e.what() << std::endl;
        std::cerr << "[MockDetector] Running in dummy mode (no detection)"
                    << std::endl;
    }

    // 加载 COCO 标签
    LoadLabels("assets/labels/coco.txt");
    if (label_names_.empty()) {
        // 内置 COCO 80 类的前几个常用类
        label_names_ = {
            "person",    "bicycle",   "car",       "motorcycle", "airplane",
            "bus",       "train",     "truck",     "boat",       "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird",
            "cat",       "dog",       "horse",     "sheep",      "cow",
            "elephant",  "bear",      "zebra",     "giraffe",    "backpack",
            "umbrella",  "handbag",   "tie",       "suitcase",   "frisbee",
            "skis",      "snowboard", "sports ball", "kite",     "baseball bat",
            "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
            "wine glass", "cup",      "fork",      "knife",      "spoon",
            "bowl",      "banana",    "apple",     "sandwich",   "orange",
            "broccoli",  "carrot",    "hot dog",   "pizza",      "donut",
            "cake",      "chair",     "couch",     "potted plant", "bed",
            "dining table", "toilet", "tv",        "laptop",     "mouse",
            "remote",    "keyboard",  "cell phone", "microwave", "oven",
            "toaster",   "sink",      "refrigerator", "book",    "clock",
            "vase",      "scissors",  "teddy bear", "hair drier", "toothbrush"
        };
    }

    std::cout << "[MockDetector] Initialized with " << label_names_.size()
                << " labels" << std::endl;
    return true;
}

bool MockDetector::Detect(const cv::Mat& image,
                            std::vector<DetectionTarget>& targets) {
    targets.clear();

    if (net_.empty() || image.empty()) {
        return true;  // 无模型时返回空结果
    }

    // YOLOv8 预处理
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0,
                            cv::Size(input_size_, input_size_),
                            cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);

    // 推理
    std::vector<cv::Mat> outputs;
    try {
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    } catch (const cv::Exception& e) {
        std::cerr << "[MockDetector] DNN forward failed: " << e.what()
                    << std::endl;
        std::cerr << "[MockDetector] Model may be incompatible with OpenCV DNN. "
                    << "Need a standard (non-quantized) YOLOv8 ONNX model."
                    << std::endl;
        // 禁用后续推理，避免重复报错
        net_ = cv::dnn::Net();
        return true;
    }

    if (outputs.empty()) return true;

    // YOLOv8 输出格式: [1, 84, 8400] (detection) 或 [1, 116, 8400] (seg)
    // 转置为 [8400, 84]
    PostprocessYOLOv8(outputs[0], image, targets);

    return true;
}

bool MockDetector::DetectBest(const cv::Mat& image, DetectionTarget& target) {
    std::vector<DetectionTarget> targets;
    if (!Detect(image, targets) || targets.empty()) return false;
    target = targets[0];
    return true;
}

bool MockDetector::DetectByName(const cv::Mat& image,
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

void MockDetector::PostprocessYOLOv8(const cv::Mat& output,
                                    const cv::Mat& image,
                                    std::vector<DetectionTarget>& targets) {
    // output shape: [1, 84, N] where N=8400 for 640x640 input
    // 84 = 4 (bbox) + 80 (classes)
    int rows = output.size[2];  // number of detections
    int cols = output.size[1];  // 4 + num_classes

    // Reshape to [N, 84]
    cv::Mat data = output.reshape(1, cols).t();  // [8400, 84]

    float x_factor = static_cast<float>(image.cols) / input_size_;
    float y_factor = static_cast<float>(image.rows) / input_size_;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; i++) {
        const float* row_ptr = data.ptr<float>(i);

        // 前 4 个是 cx, cy, w, h
        float cx = row_ptr[0];
        float cy = row_ptr[1];
        float w = row_ptr[2];
        float h = row_ptr[3];

        // 后面是各类别分数
        const float* scores_ptr = row_ptr + 4;
        int num_classes = cols - 4;

        // 找最大类别分数
        int max_class_id = 0;
        float max_score = scores_ptr[0];
        for (int c = 1; c < num_classes; c++) {
            if (scores_ptr[c] > max_score) {
                max_score = scores_ptr[c];
                max_class_id = c;
            }
        }

        if (max_score < conf_threshold_) continue;

        // 类别过滤
        if (!config_.target_labels.empty()) {
            bool found = false;
            for (int label : config_.target_labels) {
                if (max_class_id == label) { found = true; break; }
            }
            if (!found) continue;
        }

        // 转换坐标
        float x1 = (cx - w / 2.0f) * x_factor;
        float y1 = (cy - h / 2.0f) * y_factor;
        float bw = w * x_factor;
        float bh = h * y_factor;

        boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                            static_cast<int>(bw), static_cast<int>(bh));
        confidences.push_back(max_score);
        class_ids.push_back(max_class_id);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_,
                        indices);

    for (int idx : indices) {
        auto& box = boxes[idx];
        float area = static_cast<float>(box.width * box.height);
        if (area < config_.min_area) continue;

        DetectionTarget target;
        target.x1 = static_cast<float>(box.x);
        target.y1 = static_cast<float>(box.y);
        target.x2 = static_cast<float>(box.x + box.width);
        target.y2 = static_cast<float>(box.y + box.height);
        target.score = confidences[idx];
        target.label = class_ids[idx];
        target.center = cv::Point2f((target.x1 + target.x2) / 2.0f,
                                    (target.y1 + target.y2) / 2.0f);
        target.area = area;

        if (class_ids[idx] >= 0 &&
            class_ids[idx] < static_cast<int>(label_names_.size())) {
            target.label_name = label_names_[class_ids[idx]];
        }

        targets.push_back(std::move(target));
    }

    // 按面积降序
    std::sort(targets.begin(), targets.end(),
                [](const DetectionTarget& a, const DetectionTarget& b) {
                    return a.area > b.area;
                });
}

bool MockDetector::LoadLabels(const std::string& label_file) {
    std::ifstream ifs(label_file);
    if (!ifs.is_open()) return false;

    label_names_.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        auto start = line.find_first_not_of(" \t\r\n");
        auto end = line.find_last_not_of(" \t\r\n");
        if (start != std::string::npos) {
            label_names_.push_back(line.substr(start, end - start + 1));
        }
    }
    return true;
}

}  // namespace perceptive_grasp
