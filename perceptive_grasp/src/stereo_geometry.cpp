/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file stereo_geometry.cpp
 * @brief Stereo calibration and output-intrinsics helpers.
 */

#include "stereo_geometry.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace perceptive_grasp {
namespace {

bool FlattenNumbers(const cv::FileNode& node, std::vector<double>& values) {
    if (node.isInt() || node.isReal()) {
        values.push_back(static_cast<double>(node));
        return true;
    }
    if (!node.isSeq()) {
        return false;
    }
    for (const auto& child : node) {
        if (!FlattenNumbers(child, values)) {
            return false;
        }
    }
    return true;
}

bool ReadMatrix(const cv::FileStorage& storage, const char* key, int rows,
                int cols, cv::Mat& matrix, std::string& error) {
    std::vector<double> values;
    const cv::FileNode node = storage[key];
    if (node.empty() || !FlattenNumbers(node, values) ||
        values.size() != static_cast<size_t>(rows * cols)) {
        std::ostringstream oss;
        oss << "invalid " << key << ": expected " << rows * cols
            << " numeric values";
        error = oss.str();
        return false;
    }

    matrix = cv::Mat(rows, cols, CV_64F);
    std::copy(values.begin(), values.end(), matrix.ptr<double>());
    return true;
}

bool ReadVector(const cv::FileStorage& storage, const char* key,
                cv::Mat& vector, std::string& error) {
    std::vector<double> values;
    const cv::FileNode node = storage[key];
    if (node.empty() || !FlattenNumbers(node, values) || values.empty()) {
        error = std::string("invalid ") + key + ": expected numeric values";
        return false;
    }

    vector = cv::Mat(1, static_cast<int>(values.size()), CV_64F);
    std::copy(values.begin(), values.end(), vector.ptr<double>());
    return true;
}

}  // namespace

bool LoadRectifiedLeftIntrinsics(const std::string& calibration_json,
                                 PinholeIntrinsics& intrinsics,
                                 std::string& error) {
    intrinsics = {};
    error.clear();
    try {
        cv::FileStorage storage(calibration_json,
                                cv::FileStorage::READ |
                                    cv::FileStorage::FORMAT_JSON);
        if (!storage.isOpened()) {
            error = "cannot open calibration file: " + calibration_json;
            return false;
        }

        std::vector<double> image_size;
        if (!FlattenNumbers(storage["image_size"], image_size) ||
            image_size.size() != 2) {
            error = "invalid image_size: expected [width, height]";
            return false;
        }
        const cv::Size size(static_cast<int>(std::lround(image_size[0])),
                            static_cast<int>(std::lround(image_size[1])));
        if (size.width <= 0 || size.height <= 0) {
            error = "invalid calibration image size";
            return false;
        }

        cv::Mat left_camera;
        cv::Mat right_camera;
        cv::Mat left_distortion;
        cv::Mat right_distortion;
        cv::Mat rotation;
        cv::Mat translation;
        if (!ReadMatrix(storage, "left_camera_matrix", 3, 3, left_camera,
                        error) ||
            !ReadMatrix(storage, "right_camera_matrix", 3, 3, right_camera,
                        error) ||
            !ReadVector(storage, "left_dist_coeffs", left_distortion, error) ||
            !ReadVector(storage, "right_dist_coeffs", right_distortion, error) ||
            !ReadMatrix(storage, "R", 3, 3, rotation, error) ||
            !ReadVector(storage, "T", translation, error) ||
            translation.total() != 3) {
            if (translation.total() != 3 && error.empty()) {
                error = "invalid T: expected 3 numeric values";
            }
            return false;
        }

        translation = translation.reshape(1, 3);
        cv::Mat left_rectification;
        cv::Mat right_rectification;
        cv::Mat left_projection;
        cv::Mat right_projection;
        cv::Mat disparity_to_depth;
        cv::stereoRectify(left_camera, left_distortion, right_camera,
                          right_distortion, size, rotation, translation,
                          left_rectification, right_rectification,
                          left_projection, right_projection, disparity_to_depth,
                          cv::CALIB_ZERO_DISPARITY, 0.0, size);

        intrinsics.width = size.width;
        intrinsics.height = size.height;
        intrinsics.fx = left_projection.at<double>(0, 0);
        intrinsics.fy = left_projection.at<double>(1, 1);
        intrinsics.cx = left_projection.at<double>(0, 2);
        intrinsics.cy = left_projection.at<double>(1, 2);
        if (!(intrinsics.fx > 0.0) || !(intrinsics.fy > 0.0) ||
            !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy)) {
            error = "stereoRectify produced invalid left-camera intrinsics";
            return false;
        }
        return true;
    } catch (const cv::Exception& exception) {
        error = std::string("failed to parse stereo calibration: ") +
                exception.what();
        return false;
    }
}

bool ScaleIntrinsicsWithLetterbox(const PinholeIntrinsics& source,
                                  int output_width,
                                  int output_height,
                                  PinholeIntrinsics& output,
                                  std::string& error) {
    output = {};
    error.clear();
    if (source.width <= 0 || source.height <= 0 || !(source.fx > 0.0) ||
        !(source.fy > 0.0) || output_width <= 0 || output_height <= 0) {
        error = "invalid source intrinsics or output size";
        return false;
    }

    const double scale = std::min(
        static_cast<double>(output_width) / source.width,
        static_cast<double>(output_height) / source.height);
    const double resized_width = source.width * scale;
    const double resized_height = source.height * scale;
    const double padding_x = (output_width - resized_width) * 0.5;
    const double padding_y = (output_height - resized_height) * 0.5;

    output.width = output_width;
    output.height = output_height;
    output.fx = source.fx * scale;
    output.fy = source.fy * scale;
    output.cx = source.cx * scale + padding_x;
    output.cy = source.cy * scale + padding_y;
    return true;
}

}  // namespace perceptive_grasp
