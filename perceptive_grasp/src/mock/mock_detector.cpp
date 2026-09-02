/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mock_detector.cpp
 * @brief X86 Standalone 目标检测 - 使用 OpenCV DNN 加载 YOLOv8
 */

#include "mock/mock_detector.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace perceptive_grasp {
namespace {

std::string ExpandUserPath(const std::string& path) {
    if (path.empty() || path.front() != '~') {
        return path;
    }
    const char* home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
}

bool HasYamlExtension(const std::string& path) {
    const std::string::size_type dot = path.find_last_of('.');
    if (dot == std::string::npos) {
        return false;
    }
    const std::string extension = path.substr(dot);
    return extension == ".yaml" || extension == ".yml";
}

std::string ParentPath(const std::string& path) {
    const std::string::size_type separator = path.find_last_of('/');
    if (separator == std::string::npos) {
        return ".";
    }
    if (separator == 0) {
        return "/";
    }
    return path.substr(0, separator);
}

std::string ResolveYamlPath(const std::string& yaml_path,
                            const std::string& configured_path) {
    const std::string path = ExpandUserPath(configured_path);
    if (path.empty() || path.front() == '/') {
        return path;
    }
    const std::string parent = ParentPath(yaml_path);
    return parent == "/" ? parent + path : parent + "/" + path;
}

struct YoloCandidate {
    cv::Rect box;
    int class_id = -1;
    float confidence = 0.0f;
    cv::Mat mask_coefficients;
};

}  // namespace

MockDetector::MockDetector(const DetectorConfig& config)
    : TargetDetector(config) {
    conf_threshold_ = config.min_confidence;
}

bool MockDetector::Init() {
    std::string model_path = config_.config_path;
    std::string label_path;

    const std::string detector_config_path = ExpandUserPath(model_path);
    if (HasYamlExtension(detector_config_path)) {
        try {
            const YAML::Node root = YAML::LoadFile(detector_config_path);
            const std::string configured_model =
                root["opencv_model_path"].as<std::string>(
                    root["model_path"].as<std::string>(""));
            model_path = ResolveYamlPath(
                detector_config_path, configured_model);
            label_path = ResolveYamlPath(
                detector_config_path,
                root["label_file_path"].as<std::string>(""));
            if (auto image_size = root["image_size"]; image_size &&
                image_size.IsSequence() && image_size.size() >= 1) {
                input_size_ = image_size[0].as<int>(input_size_);
            }
        } catch (const YAML::Exception& error) {
            std::cerr << "[MockDetector] Invalid detector config: "
                        << error.what() << std::endl;
            return false;
        }
    } else {
        model_path = ExpandUserPath(model_path);
    }

    bool model_loaded = false;
    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
        if (!net_.empty()) {
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            model_loaded = true;
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[MockDetector] OpenCV DNN error: " << e.what() << std::endl;
        net_ = cv::dnn::Net();
    }

    if (!label_path.empty()) {
        LoadLabels(label_path);
    }
    if (label_names_.empty()) {
        LoadLabels("assets/labels/coco.txt");
    }
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

    if (!model_loaded) {
        if (!config_.allow_color_only_fallback ||
            config_.simulation_color_targets.empty()) {
            std::cerr << "[MockDetector] Model is unavailable: " << model_path
                        << std::endl;
            return false;
        }
        std::cout << "[MockDetector] Model unavailable; explicit simulation "
                    << "color-only fallback enabled" << std::endl;
    } else {
        std::cout << "[MockDetector] Model loaded: " << model_path << std::endl;
    }
    std::cout << "[MockDetector] Initialized with " << label_names_.size()
                << " labels" << std::endl;
    return true;
}

bool MockDetector::Detect(const cv::Mat& image,
                            std::vector<DetectionTarget>& targets) {
    targets.clear();

    if (image.empty()) {
        return true;
    }
    if (net_.empty()) {
        AppendSimulationColorTargets(image, config_, &targets);
        return true;
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0 / 255.0,
                            cv::Size(input_size_, input_size_),
                            cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    try {
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    } catch (const cv::Exception& e) {
        std::cerr << "[MockDetector] DNN forward failed: " << e.what()
                    << std::endl;
        if (!config_.allow_color_only_fallback) {
            return false;
        }
        net_ = cv::dnn::Net();
        AppendSimulationColorTargets(image, config_, &targets);
        return !targets.empty();
    }

    if (outputs.empty()) return true;

    PostprocessYOLOv8(outputs, image, targets);
    AppendSimulationColorTargets(image, config_, &targets);

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

void MockDetector::PostprocessYOLOv8(
        const std::vector<cv::Mat>& outputs, const cv::Mat& image,
        std::vector<DetectionTarget>& targets) {
    if (outputs.empty()) return;

    const cv::Mat* prediction = nullptr;
    const cv::Mat* prototype = nullptr;
    for (const cv::Mat& output : outputs) {
        if (output.dims == 3 && prediction == nullptr) {
            prediction = &output;
        } else if (output.dims == 4 && prototype == nullptr) {
            prototype = &output;
        }
    }
    if (prediction == nullptr || prediction->size[0] != 1) return;

    const bool attributes_first =
        prediction->size[1] >= 5 && prediction->size[1] <= 512 &&
        (prediction->size[2] < 5 || prediction->size[2] > 512 ||
            prediction->size[1] <= prediction->size[2]);
    const int rows = attributes_first
        ? prediction->size[2] : prediction->size[1];
    const int columns = attributes_first
        ? prediction->size[1] : prediction->size[2];
    cv::Mat data = attributes_first
        ? prediction->reshape(1, columns).t()
        : prediction->reshape(1, rows);

    const int mask_dimensions = prototype == nullptr ? 0 : prototype->size[1];
    const int class_count = columns - 4 - mask_dimensions;
    if (class_count <= 0 || (!label_names_.empty() &&
        class_count > static_cast<int>(label_names_.size()))) {
        std::cerr << "[MockDetector] Unsupported YOLO output attributes: "
                    << columns << std::endl;
        return;
    }

    float x_factor = static_cast<float>(image.cols) / input_size_;
    float y_factor = static_cast<float>(image.rows) / input_size_;

    std::vector<YoloCandidate> candidates;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; i++) {
        const float* row_ptr = data.ptr<float>(i);

        // 前 4 个是 cx, cy, w, h
        float cx = row_ptr[0];
        float cy = row_ptr[1];
        float w = row_ptr[2];
        float h = row_ptr[3];

        const float* scores_ptr = row_ptr + 4;
        int max_class_id = 0;
        float max_score = scores_ptr[0];
        for (int c = 1; c < class_count; c++) {
            if (scores_ptr[c] > max_score) {
                max_score = scores_ptr[c];
                max_class_id = c;
            }
        }

        if (max_score < conf_threshold_) continue;

        if (!config_.target_labels.empty()) {
            bool found = false;
            for (int label : config_.target_labels) {
                if (max_class_id == label) { found = true; break; }
            }
            if (!found) continue;
        }

        float x1 = (cx - w / 2.0f) * x_factor;
        float y1 = (cy - h / 2.0f) * y_factor;
        float bw = w * x_factor;
        float bh = h * y_factor;

        cv::Rect box(static_cast<int>(x1), static_cast<int>(y1),
                    static_cast<int>(bw), static_cast<int>(bh));
        box &= cv::Rect(0, 0, image.cols, image.rows);
        if (box.empty()) continue;
        YoloCandidate candidate;
        candidate.box = box;
        candidate.class_id = max_class_id;
        candidate.confidence = max_score;
        if (mask_dimensions > 0) {
            candidate.mask_coefficients = cv::Mat(
                1, mask_dimensions, CV_32F,
                const_cast<float*>(row_ptr + 4 + class_count)).clone();
        }
        candidates.push_back(std::move(candidate));
        boxes.push_back(box);
        confidences.push_back(max_score);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_,
                        indices);

    for (int idx : indices) {
        const YoloCandidate& candidate = candidates[idx];
        const cv::Rect& box = candidate.box;
        float area = static_cast<float>(box.width * box.height);
        if (area < config_.min_area) continue;

        DetectionTarget target;
        target.x1 = static_cast<float>(box.x);
        target.y1 = static_cast<float>(box.y);
        target.x2 = static_cast<float>(box.x + box.width);
        target.y2 = static_cast<float>(box.y + box.height);
        target.score = candidate.confidence;
        target.label = candidate.class_id;
        target.center = cv::Point2f((target.x1 + target.x2) / 2.0f,
                                    (target.y1 + target.y2) / 2.0f);
        target.area = area;

        if (candidate.class_id >= 0 &&
            candidate.class_id < static_cast<int>(label_names_.size())) {
            target.label_name = label_names_[candidate.class_id];
        }
        const auto remap = config_.label_remap.find(target.label_name);
        if (remap != config_.label_remap.end()) {
            target.label_name = remap->second;
        }

        if (prototype != nullptr && !candidate.mask_coefficients.empty() &&
            prototype->size[0] == 1 && prototype->size[1] == mask_dimensions) {
            const int mask_height = prototype->size[2];
            const int mask_width = prototype->size[3];
            cv::Mat prototype_matrix(
                mask_dimensions, mask_height * mask_width, CV_32F,
                const_cast<float*>(prototype->ptr<float>()));
            cv::Mat logits = candidate.mask_coefficients * prototype_matrix;
            cv::exp(-logits, logits);
            logits = 1.0 / (1.0 + logits);
            cv::Mat low_resolution = logits.reshape(1, mask_height);
            cv::Mat probability;
            cv::resize(low_resolution, probability, image.size(), 0.0, 0.0,
                        cv::INTER_LINEAR);
            cv::Mat binary;
            cv::threshold(probability, binary, 0.5, 255.0, cv::THRESH_BINARY);
            binary.convertTo(target.mask, CV_8U);
            cv::Mat bounded = cv::Mat::zeros(image.size(), CV_8U);
            target.mask(box).copyTo(bounded(box));
            target.mask = std::move(bounded);
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
