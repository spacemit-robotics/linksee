/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_geometry.h
 * @brief Lightweight mask-guided 3D object geometry and grasp candidates.
 */

#ifndef GRASP_GEOMETRY_H
#define GRASP_GEOMETRY_H

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "grasp_planner.h"
#include "stereo_camera.h"
#include "target_detector.h"

namespace perceptive_grasp {

enum class GraspStrategy {
    TOP,
    SIDE,
};

const char* GraspStrategyName(GraspStrategy strategy);

struct GraspGeometryConfig {
    std::string strategy = "auto";
    int sample_stride = 2;
    int max_object_points = 4000;
    int min_object_points = 80;
    int plane_ransac_iterations = 48;
    float plane_distance_threshold_m = 0.008f;
    float max_table_tilt_deg = 20.0f;
    float table_clearance_m = 0.005f;
    float object_min_height_m = 0.008f;
    float object_max_height_m = 0.25f;
    float footprint_padding_m = 0.005f;
    float gripper_max_width_m = 0.075f;
    float side_min_height_m = 0.060f;
    float side_min_height_width_ratio = 1.0f;
    float side_approach_distance_m = 0.020f;
    float side_entry_clearance_m = 0.030f;
    float side_pregrasp_min_x_m = 0.270f;
    float side_gripper_offset_m = 0.015f;
    float side_grasp_forward_offset_m = 0.0f;
    float side_initial_lift_m = 0.050f;
    float side_lift_retreat_m = 0.025f;
    float side_grasp_height_ratio = 0.60f;
    float finger_half_height_m = 0.012f;
    int planning_timeout_ms = 350;
    int perception_budget_ms = 500;
};

struct TablePlane {
    cv::Point3f normal{0.0f, 0.0f, 1.0f};
    float d = 0.0f;
    int inlier_count = 0;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    bool bounds_valid = false;
};

struct ObjectGeometry3D {
    bool valid = false;
    cv::Point3f center{0.0f, 0.0f, 0.0f};
    cv::Point3f table_center{0.0f, 0.0f, 0.0f};
    cv::Point3f major_axis{1.0f, 0.0f, 0.0f};
    cv::Point3f minor_axis{0.0f, 1.0f, 0.0f};
    float length_m = 0.0f;
    float width_m = 0.0f;
    float height_m = 0.0f;
    int source_point_count = 0;
    int object_point_count = 0;
    TablePlane table;
};

struct GraspCandidate {
    GraspStrategy strategy = GraspStrategy::TOP;
    Pose3D grasp_pose{};
    Pose3D pre_grasp_pose{};
    Pose3D retreat_pose{};
    Pose3D lift_pose{};
    float entry_clearance_z_m = NAN;
    cv::Point3f approach_axis{0.0f, 0.0f, -1.0f};
    cv::Point3f opening_axis{0.0f, 1.0f, 0.0f};
    float grasp_yaw_rad = NAN;
    float required_width_m = 0.0f;
    float width_margin_m = 0.0f;
    float depth_quality = 0.0f;
    float path_clearance_m = 0.0f;
    float workspace_margin_m = 0.0f;
    float ik_margin_rad = NAN;
    float score = 0.0f;
    bool geometry_valid = false;
    std::string rejection_reason;
};

struct GraspGeometryResult {
    ObjectGeometry3D geometry;
    std::vector<cv::Point3f> object_points;
    std::vector<GraspCandidate> candidates;
    float foreground_depth_mm = NAN;
    float center_depth_mm = NAN;
    std::int64_t elapsed_ms = 0;
    std::string error;
};

/**
* @brief Check whether two independent 3D estimates describe the same object.
*
* This rejects transient RGB-D geometry after robot motion before the estimate
* can drive the mobile base or manipulator.
*/
bool AreObjectGeometriesConsistent(const ObjectGeometry3D& reference,
                                    const ObjectGeometry3D& current,
                                    std::string* reason = nullptr);

/** Check physical dimensions while allowing intentional object translation. */
bool AreObjectDimensionsConsistent(const ObjectGeometry3D& reference,
                                    const ObjectGeometry3D& current,
                                    std::string* reason = nullptr);

/**
* @brief Estimate table-relative object geometry and generate grasp candidates.
*
* The implementation samples aligned metric depth only inside the segmentation
* mask and its surrounding support region. It has no learned-model dependency.
*/
class GraspGeometryPlanner {
public:
    GraspGeometryPlanner(const GraspGeometryConfig& config,
                        const GraspPlannerConfig& planner_config);

    bool Plan(const cv::Mat& depth,
                const DetectionTarget& target,
                const StereoCamera& camera,
                const GraspPlanner& coordinate_planner,
                GraspGeometryResult& result,
                std::optional<GraspStrategy> locked_strategy =
                    std::nullopt) const;

    static bool EstimateObjectGeometry(
        const std::vector<cv::Point3f>& object_points,
        const std::vector<cv::Point3f>& support_points,
        const GraspGeometryConfig& config,
        ObjectGeometry3D& geometry,
        std::vector<cv::Point3f>* filtered_object_points,
        std::string& error);

    static std::vector<GraspCandidate> GenerateCandidates(
        const ObjectGeometry3D& geometry,
        const std::vector<cv::Point3f>& object_points,
        const GraspGeometryConfig& config,
        const GraspPlannerConfig& planner_config,
        std::optional<GraspStrategy> locked_strategy = std::nullopt);

private:
    GraspGeometryConfig config_;
    GraspPlannerConfig planner_config_;
};

}  // namespace perceptive_grasp

#endif  // GRASP_GEOMETRY_H
