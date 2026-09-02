/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_debug_writer.h
 * @brief Pipeline camera, grasp-plan, and task-result debug output.
 */

#ifndef PIPELINE_DEBUG_WRITER_H
#define PIPELINE_DEBUG_WRITER_H

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "grasp_executor.h"
#include "grasp_planner.h"

namespace perceptive_grasp {

struct PipelinePlanDebugData {
    cv::Mat color;
    cv::Mat target_mask;
    std::string target_detected;
    float target_score = 0.0f;
    std::array<float, 4> target_bbox = {};
    cv::Point2f target_center{};
    std::string target_requested;
    std::string candidates;
    float grasp_px = 0.0f;
    float grasp_py = 0.0f;
    std::uint16_t depth_mm = 0;
    std::array<float, 3> camera_point_m = {};
    std::array<float, 3> base_point_m = {};
    std::string grasp_strategy;
    std::array<float, 3> object_dimensions_m = {};
    std::int64_t geometry_elapsed_ms = 0;
    float grasp_yaw_rad = NAN;
    Pose3D pre_grasp_pose{};
    Pose3D grasp_pose{};
    Pose3D retreat_pose{};
    Pose3D lift_pose{};
};

struct PipelineTaskResultDebugData {
    std::string terminal_state;
    std::string message;
    std::string target_requested;
    std::string target_detected;
    std::string candidates;
    ExecutorDiagnostics executor;
};

bool SavePipelineStepCameraDebug(
    const std::string& output_dir,
    const std::string& phase,
    const cv::Mat& color,
    const cv::Mat& depth,
    std::string& task_id);

bool SavePipelinePlanDebug(
    const std::string& output_dir,
    const PipelinePlanDebugData& data,
    std::string& task_id,
    std::string& image_path,
    std::string& json_path);

bool SavePipelineTaskResultDebug(
    const std::string& output_dir,
    const PipelineTaskResultDebugData& data,
    std::string& task_id,
    const std::string& image_path,
    const std::string& plan_json_path);

}  // namespace perceptive_grasp

#endif  // PIPELINE_DEBUG_WRITER_H
