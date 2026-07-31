/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file pipeline_debug_writer.cpp
 * @brief Pipeline camera, grasp-plan, and task-result debug output.
 */

#include "pipeline_debug_writer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace perceptive_grasp {

namespace {

namespace fs = std::filesystem;

std::string TimestampString() {
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm = {};
    localtime_r(&tt, &tm);

    std::ostringstream output;
    output << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count();
    return output.str();
}

std::string JsonEscape(const std::string& input) {
    std::ostringstream output;
    for (char character : input) {
        switch (character) {
            case '\\':
                output << "\\\\";
                break;
            case '"':
                output << "\\\"";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                output << character;
                break;
        }
    }
    return output.str();
}

void WritePoseJson(
    std::ofstream& output,
    const char* name,
    const Pose3D& pose,
    bool trailing_comma) {
    output << "  \"" << name << "\": {"
        << "\"x\": " << pose.x << ", "
        << "\"y\": " << pose.y << ", "
        << "\"z\": " << pose.z << ", "
        << "\"qw\": " << pose.qw << ", "
        << "\"qx\": " << pose.qx << ", "
        << "\"qy\": " << pose.qy << ", "
        << "\"qz\": " << pose.qz << "}";
    if (trailing_comma) output << ",";
    output << "\n";
}

void WriteOptionalFloat(std::ofstream& output, float value) {
    if (std::isfinite(value)) {
        output << value;
    } else {
        output << "null";
    }
}

const char* GraspResultName(GraspResult result) {
    switch (result) {
        case GraspResult::SUCCESS:
            return "SUCCESS";
        case GraspResult::EMPTY:
            return "EMPTY";
        case GraspResult::IK_FAILED:
            return "IK_FAILED";
        case GraspResult::OUT_OF_RANGE:
            return "OUT_OF_RANGE";
        case GraspResult::MOVE_FAILED:
            return "MOVE_FAILED";
        case GraspResult::TIMEOUT:
            return "TIMEOUT";
    }
    return "UNKNOWN";
}

void EnsureTaskId(std::string& task_id) {
    if (task_id.empty()) task_id = TimestampString();
}

}  // namespace

bool SavePipelineStepCameraDebug(
    const std::string& output_dir,
    const std::string& phase,
    const cv::Mat& color,
    const cv::Mat& depth,
    std::string& task_id) {
    try {
        fs::path directory(output_dir);
        fs::create_directories(directory);
        EnsureTaskId(task_id);
        const std::string stem = "grasp_" + task_id + "_" + phase;
        const fs::path color_path = directory / (stem + ".png");
        const fs::path depth_path = directory / (stem + "_depth.png");
        cv::imwrite(color_path.string(), color);

        cv::Mat depth_8u;
        cv::Mat depth_color;
        depth.convertTo(depth_8u, CV_8UC1, 255.0 / 1000.0);
        cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_TURBO);
        depth_color.setTo(cv::Scalar(0, 0, 0), depth == 0);
        cv::imwrite(depth_path.string(), depth_color);
        std::cout << "[Pipeline] Step camera debug saved: "
            << color_path << ", " << depth_path << std::endl;
        return true;
    } catch (const std::exception& error) {
        std::cerr << "[Pipeline] Failed to save step camera debug: "
            << error.what() << std::endl;
        return false;
    }
}

bool SavePipelinePlanDebug(
    const std::string& output_dir,
    const PipelinePlanDebugData& data,
    std::string& task_id,
    std::string& image_path,
    std::string& json_path) {
    try {
        fs::path directory(output_dir);
        fs::create_directories(directory);
        EnsureTaskId(task_id);
        const fs::path output_image =
            directory / ("grasp_" + task_id + ".png");
        const fs::path output_json =
            directory / ("grasp_" + task_id + ".json");
        image_path = output_image.string();
        json_path = output_json.string();

        if (!data.color.empty()) {
            cv::Mat annotated = data.color.clone();
            cv::rectangle(
                annotated,
                cv::Point(
                    static_cast<int>(data.target_bbox[0]),
                    static_cast<int>(data.target_bbox[1])),
                cv::Point(
                    static_cast<int>(data.target_bbox[2]),
                    static_cast<int>(data.target_bbox[3])),
                cv::Scalar(0, 255, 0), 2);
            cv::circle(
                annotated, data.target_center, 5,
                cv::Scalar(255, 0, 0), -1);
            cv::circle(
                annotated,
                cv::Point2f(data.grasp_px, data.grasp_py), 6,
                cv::Scalar(0, 0, 255), -1);
            cv::line(
                annotated, data.target_center,
                cv::Point2f(data.grasp_px, data.grasp_py),
                cv::Scalar(0, 255, 255), 2);
            cv::putText(
                annotated, data.target_detected,
                cv::Point(
                    static_cast<int>(data.target_bbox[0]),
                    std::max(
                        20, static_cast<int>(data.target_bbox[1]) - 8)),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 0), 2);
            cv::imwrite(output_image.string(), annotated);
        }

        std::ofstream output(output_json);
        output << "{\n";
        output << "  \"task_id\": \"" << JsonEscape(task_id) << "\",\n";
        output << "  \"target\": \""
            << JsonEscape(data.target_detected) << "\",\n";
        output << "  \"target_requested\": \""
            << JsonEscape(data.target_requested) << "\",\n";
        output << "  \"score\": " << data.target_score << ",\n";
        output << "  \"bbox\": [" << data.target_bbox[0] << ", "
            << data.target_bbox[1] << ", " << data.target_bbox[2] << ", "
            << data.target_bbox[3] << "],\n";
        output << "  \"pixel_center\": [" << data.target_center.x << ", "
            << data.target_center.y << "],\n";
        output << "  \"pixel_grasp\": [" << data.grasp_px << ", "
            << data.grasp_py << "],\n";
        output << "  \"depth_mm\": ";
        if (data.depth_mm != 0) {
            output << data.depth_mm;
        } else {
            output << "null";
        }
        output << ",\n";
        output << "  \"camera_point_m\": ";
        if (data.depth_mm != 0) {
            output << "[" << data.camera_point_m[0] << ", "
                << data.camera_point_m[1] << ", "
                << data.camera_point_m[2] << "]";
        } else {
            output << "null";
        }
        output << ",\n";
        output << "  \"base_point_m\": [" << data.base_point_m[0] << ", "
            << data.base_point_m[1] << ", " << data.base_point_m[2]
            << "],\n";
        output << "  \"grasp_strategy\": \""
            << JsonEscape(data.grasp_strategy) << "\",\n";
        output << "  \"object_dimensions_m\": ["
            << data.object_dimensions_m[0] << ", "
            << data.object_dimensions_m[1] << ", "
            << data.object_dimensions_m[2] << "],\n";
        output << "  \"geometry_elapsed_ms\": "
            << data.geometry_elapsed_ms << ",\n";
        output << "  \"grasp_yaw_rad\": ";
        WriteOptionalFloat(output, data.grasp_yaw_rad);
        output << ",\n";
        output << "  \"candidates\": \""
            << JsonEscape(data.candidates) << "\",\n";
        WritePoseJson(output, "pre_grasp_pose", data.pre_grasp_pose, true);
        WritePoseJson(output, "grasp_pose", data.grasp_pose, true);
        WritePoseJson(output, "retreat_pose", data.retreat_pose, true);
        WritePoseJson(output, "lift_pose", data.lift_pose, false);
        output << "}\n";

        std::cout << "[Pipeline] Debug saved: " << output_image
            << ", " << output_json << std::endl;
        return true;
    } catch (const std::exception& error) {
        std::cerr << "[Pipeline] Failed to save debug data: "
            << error.what() << std::endl;
        return false;
    }
}

bool SavePipelineTaskResultDebug(
    const std::string& output_dir,
    const PipelineTaskResultDebugData& data,
    std::string& task_id,
    const std::string& image_path,
    const std::string& plan_json_path) {
    try {
        fs::path directory(output_dir);
        fs::create_directories(directory);
        EnsureTaskId(task_id);
        const fs::path result_path =
            directory / ("grasp_" + task_id + "_result.json");
        const ExecutorDiagnostics& diagnostics = data.executor;
        const GripperCheckDiagnostics& gripper =
            diagnostics.gripper_check;
        const WristYawDiagnostics& wrist = diagnostics.wrist_yaw;

        std::ofstream output(result_path);
        output << "{\n";
        output << "  \"task_id\": \"" << JsonEscape(task_id) << "\",\n";
        output << "  \"terminal_state\": \""
            << JsonEscape(data.terminal_state) << "\",\n";
        output << "  \"message\": \"" << JsonEscape(data.message)
            << "\",\n";
        output << "  \"target_requested\": \""
            << JsonEscape(data.target_requested) << "\",\n";
        output << "  \"target_detected\": \""
            << JsonEscape(data.target_detected) << "\",\n";
        output << "  \"candidates\": \""
            << JsonEscape(data.candidates) << "\",\n";
        output << "  \"debug_image\": \"" << JsonEscape(image_path)
            << "\",\n";
        output << "  \"debug_plan_json\": \""
            << JsonEscape(plan_json_path) << "\",\n";
        output << "  \"last_executor_result\": \""
            << GraspResultName(diagnostics.last_result) << "\",\n";
        output << "  \"last_executor_action\": \""
            << JsonEscape(diagnostics.last_action) << "\",\n";
        output << "  \"last_executor_detail\": \""
            << JsonEscape(diagnostics.last_detail) << "\",\n";
        output << "  \"gripper_check\": {\n";
        output << "    \"phase\": \"" << JsonEscape(gripper.phase)
            << "\",\n";
        output << "    \"state\": \"" << JsonEscape(gripper.state)
            << "\",\n";
        output << "    \"decision\": \""
            << JsonEscape(gripper.decision) << "\",\n";
        output << "    \"holding_count\": " << gripper.holding_count
            << ",\n";
        output << "    \"load_holding_count\": "
            << gripper.load_holding_count << ",\n";
        output << "    \"opening_count\": " << gripper.opening_count
            << ",\n";
        output << "    \"contact_count\": " << gripper.contact_count
            << ",\n";
        output << "    \"empty_count\": " << gripper.empty_count << ",\n";
        output << "    \"check_count\": " << gripper.check_count << ",\n";
        output << "    \"baseline_sample_count\": "
            << gripper.baseline_sample_count << ",\n";
        output << "    \"load_threshold\": " << gripper.load_threshold
            << ",\n";
        output << "    \"empty_closed_position\": ";
        WriteOptionalFloat(output, gripper.empty_closed_position);
        output << ",\n";
        output << "    \"empty_closed_position_mad\": ";
        WriteOptionalFloat(output, gripper.empty_closed_position_mad);
        output << ",\n";
        output << "    \"empty_closed_load\": ";
        WriteOptionalFloat(output, gripper.empty_closed_load);
        output << ",\n";
        output << "    \"empty_closed_load_mad\": ";
        WriteOptionalFloat(output, gripper.empty_closed_load_mad);
        output << ",\n";
        output << "    \"min_object_position\": ";
        WriteOptionalFloat(output, gripper.min_object_position);
        output << ",\n";
        output << "    \"position\": ";
        WriteOptionalFloat(output, gripper.position);
        output << ",\n";
        output << "    \"load\": ";
        WriteOptionalFloat(output, gripper.load);
        output << "\n";
        output << "  },\n";
        output << "  \"wrist_yaw\": {\n";
        output << "    \"valid\": "
            << (wrist.valid ? "true" : "false") << ",\n";
        output << "    \"target_yaw_rad\": ";
        WriteOptionalFloat(output, wrist.target_yaw);
        output << ",\n";
        output << "    \"target_yaw_deg\": ";
        WriteOptionalFloat(
            output,
            std::isfinite(wrist.target_yaw)
                ? wrist.target_yaw * 180.0f /
                    static_cast<float>(M_PI)
                : NAN);
        output << ",\n";
        output << "    \"joint0\": ";
        WriteOptionalFloat(output, wrist.joint0);
        output << ",\n";
        output << "    \"scale\": ";
        WriteOptionalFloat(output, wrist.scale);
        output << ",\n";
        output << "    \"joint5_raw\": ";
        WriteOptionalFloat(output, wrist.joint5_raw);
        output << ",\n";
        output << "    \"joint5_limited\": ";
        WriteOptionalFloat(output, wrist.joint5_limited);
        output << ",\n";
        output << "    \"joint5_min\": ";
        WriteOptionalFloat(output, wrist.joint5_min);
        output << ",\n";
        output << "    \"joint5_max\": ";
        WriteOptionalFloat(output, wrist.joint5_max);
        output << "\n";
        output << "  }\n";
        output << "}\n";

        std::cout << "[Pipeline] Task result debug saved: "
            << result_path << std::endl;
        return true;
    } catch (const std::exception& error) {
        std::cerr << "[Pipeline] Failed to save task result debug: "
            << error.what() << std::endl;
        return false;
    }
}

}  // namespace perceptive_grasp
