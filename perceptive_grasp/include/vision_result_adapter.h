/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file vision_result_adapter.h
    * @brief VisionService result adapter for detection-style grasping code
    */

#ifndef VISION_RESULT_ADAPTER_H
#define VISION_RESULT_ADAPTER_H

#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "vision_service.h"

struct VisionServiceResult {
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    float score = 0.0f;
    int label = -1;
    cv::Mat mask;
};

inline bool ConvertVisionResult(const vision::Result& result,
                                VisionServiceResult* out) {
    if (!out) return false;

    vision::BoundingBox bbox = vision::get_bbox(result);
    if (bbox.x2 <= bbox.x1 || bbox.y2 <= bbox.y1) return false;

    out->x1 = bbox.x1;
    out->y1 = bbox.y1;
    out->x2 = bbox.x2;
    out->y2 = bbox.y2;
    out->score = vision::get_score(result);
    out->label = vision::get_label(result);
    out->mask.release();

    if (const auto* seg = std::get_if<vision::Segmentation>(&result)) {
        if (seg->mask && !seg->mask->empty()) {
            out->mask = seg->mask->clone();
        }
    }
    return true;
}

inline VisionServiceStatus InferImageDetections(
        VisionService* vision,
        const cv::Mat& image,
        std::vector<VisionServiceResult>* results,
        VisionServiceResponse* response = nullptr) {
    if (!vision || !results) return VISION_SERVICE_INVALID_ARGUMENT;

    VisionServiceResponse local_response;
    auto status = vision->Infer(image, &local_response);
    if (status != VISION_SERVICE_OK || !local_response.ok) {
        return status == VISION_SERVICE_OK ? VISION_SERVICE_INFER_FAILED : status;
    }

    results->clear();
    for (const auto& result : local_response.results) {
        VisionServiceResult converted;
        if (ConvertVisionResult(result, &converted)) {
            results->push_back(std::move(converted));
        }
    }

    if (response) {
        *response = std::move(local_response);
    }
    return VISION_SERVICE_OK;
}

#endif  // VISION_RESULT_ADAPTER_H
