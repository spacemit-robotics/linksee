/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_debug_writer_test.cpp
 * @brief Tests pipeline debug image and JSON output.
 */

#include "pipeline_debug_writer.h"

#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;
using perceptive_grasp::PipelinePlanDebugData;
using perceptive_grasp::PipelineTaskResultDebugData;
using perceptive_grasp::SavePipelinePlanDebug;
using perceptive_grasp::SavePipelineStepCameraDebug;
using perceptive_grasp::SavePipelineTaskResultDebug;

namespace {

std::string ReadFile(const fs::path& path) {
    std::ifstream input(path);
    std::ostringstream content;
    content << input.rdbuf();
    return content.str();
}

bool Contains(
    const std::string& content,
    const std::string& expected,
    const char* description) {
    if (content.find(expected) != std::string::npos) return true;
    std::cerr << "missing " << description << ": " << expected
        << std::endl;
    return false;
}

}  // namespace

int main() {
    const fs::path output_dir =
        fs::temp_directory_path() / "perceptive_pipeline_debug_test";
    std::error_code error;
    fs::remove_all(output_dir, error);

    std::string task_id = "unit_test";
    const cv::Mat color(12, 12, CV_8UC3, cv::Scalar(20, 30, 40));
    const cv::Mat depth(12, 12, CV_16UC1, cv::Scalar(350));
    if (!SavePipelineStepCameraDebug(
            output_dir.string(), "pre_grasp", color, depth, task_id)) {
        return 1;
    }

    PipelinePlanDebugData plan;
    plan.color = color;
    plan.target_detected = "cup\"test";
    plan.target_score = 0.9f;
    plan.target_bbox = {1.0f, 2.0f, 10.0f, 11.0f};
    plan.target_center = cv::Point2f(5.0f, 6.0f);
    plan.target_requested = "cup\nrequested";
    plan.candidates = "cup(0.9)";
    plan.grasp_px = 7.0f;
    plan.grasp_py = 8.0f;
    plan.base_point_m = {0.2f, 0.1f, 0.05f};
    plan.grasp_strategy = "side";
    plan.object_dimensions_m = {0.08f, 0.07f, 0.12f};
    plan.geometry_elapsed_ms = 42;
    std::string image_path;
    std::string json_path;
    if (!SavePipelinePlanDebug(
            output_dir.string(), plan, task_id,
            image_path, json_path)) {
        return 1;
    }

    const std::string plan_json = ReadFile(json_path);
    if (!Contains(
            plan_json, "\"target\": \"cup\\\"test\"",
            "escaped target") ||
        !Contains(
            plan_json, "\"target_requested\": \"cup\\nrequested\"",
            "escaped requested target") ||
        !Contains(
            plan_json, "\"camera_point_m\": null",
            "invalid camera point") ||
        !Contains(
            plan_json, "\"grasp_yaw_rad\": null",
            "invalid grasp yaw") ||
        !Contains(
            plan_json, "\"grasp_strategy\": \"side\"",
            "grasp strategy")) {
        return 1;
    }

    PipelineTaskResultDebugData result;
    result.terminal_state = "DONE";
    result.message = "Task completed!";
    result.target_requested = "cup";
    result.target_detected = "cup";
    result.candidates = "cup(0.9)";
    result.executor.last_action = "move_to_home";
    result.executor.gripper_check.decision = "HOLDING";
    if (!SavePipelineTaskResultDebug(
            output_dir.string(), result, task_id,
            image_path, json_path)) {
        return 1;
    }

    const std::string result_json =
        ReadFile(output_dir / "grasp_unit_test_result.json");
    if (!Contains(
            result_json, "\"terminal_state\": \"DONE\"",
            "terminal state") ||
        !Contains(
            result_json, "\"last_executor_result\": \"SUCCESS\"",
            "executor result") ||
        !Contains(
            result_json, "\"decision\": \"HOLDING\"",
            "gripper decision") ||
        !Contains(
            result_json, "\"target_yaw_rad\": null",
            "invalid wrist yaw")) {
        return 1;
    }

    if (!fs::exists(
            output_dir / "grasp_unit_test_pre_grasp.png") ||
        !fs::exists(
            output_dir / "grasp_unit_test_pre_grasp_depth.png") ||
        !fs::exists(image_path)) {
        std::cerr << "debug images were not written" << std::endl;
        return 1;
    }

    fs::remove_all(output_dir, error);
    return 0;
}
