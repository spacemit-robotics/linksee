/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_geometry_test.cpp
 * @brief Unit tests for lightweight table-relative grasp geometry.
 */

#include "grasp_geometry.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using perceptive_grasp::DetectionTarget;
using perceptive_grasp::AreObjectGeometriesConsistent;
using perceptive_grasp::AreObjectDimensionsConsistent;
using perceptive_grasp::GraspCandidate;
using perceptive_grasp::GraspGeometryConfig;
using perceptive_grasp::GraspGeometryPlanner;
using perceptive_grasp::GraspGeometryResult;
using perceptive_grasp::GraspPlanner;
using perceptive_grasp::GraspPlannerConfig;
using perceptive_grasp::GraspStrategy;
using perceptive_grasp::ObjectGeometry3D;
using perceptive_grasp::StereoCamera;

class SyntheticCamera final : public StereoCamera {
public:
    bool Init() override { return true; }
    bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) override {
        (void)color_frame;
        (void)depth_frame;
        return false;
    }
    std::int64_t LastFrameId() const override { return 0; }
    bool Deproject(int pixel_x,
                    int pixel_y,
                    uint16_t depth_mm,
                    float point_3d[3]) const override {
        point_3d[0] = 0.20f + static_cast<float>(pixel_x) * 0.001f;
        point_3d[1] = (static_cast<float>(pixel_y) - 60.0f) * 0.001f;
        point_3d[2] = (1000.0f - static_cast<float>(depth_mm)) * 0.001f;
        return true;
    }
};

class ObliqueSyntheticCamera final : public StereoCamera {
public:
    bool Init() override { return true; }
    bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) override {
        (void)color_frame;
        (void)depth_frame;
        return false;
    }
    std::int64_t LastFrameId() const override { return 0; }
    bool Deproject(int pixel_x,
                    int pixel_y,
                    uint16_t depth_mm,
                    float point_3d[3]) const override {
        const float distance_m = static_cast<float>(depth_mm) * 0.001f;
        point_3d[0] = distance_m;
        point_3d[1] =
            (static_cast<float>(pixel_x) - 80.0f) / 400.0f * distance_m;
        point_3d[2] =
            (-0.8f -
            (static_cast<float>(pixel_y) - 60.0f) / 400.0f) *
            distance_m;
        return true;
    }
};

std::vector<cv::Point3f> MakeTable() {
    std::vector<cv::Point3f> points;
    for (int x = 10; x <= 45; ++x) {
        for (int y = -20; y <= 20; ++y) {
            points.emplace_back(x * 0.01f, y * 0.01f, 0.0f);
        }
    }
    return points;
}

std::vector<cv::Point3f> MakeBox(float center_x,
                                float center_y,
                                float length,
                                float width,
                                float height) {
    std::vector<cv::Point3f> points;
    for (int ix = 0; ix <= 20; ++ix) {
        const float x = center_x - length * 0.5f +
            length * static_cast<float>(ix) / 20.0f;
        for (int iy = 0; iy <= 12; ++iy) {
            const float y = center_y - width * 0.5f +
                width * static_cast<float>(iy) / 12.0f;
            points.emplace_back(x, y, height);
        }
    }
    for (int iz = 1; iz <= 12; ++iz) {
        const float z = height * static_cast<float>(iz) / 12.0f;
        for (int ix = 0; ix <= 20; ++ix) {
            const float x = center_x - length * 0.5f +
                length * static_cast<float>(ix) / 20.0f;
            points.emplace_back(x, center_y - width * 0.5f, z);
            points.emplace_back(x, center_y + width * 0.5f, z);
        }
    }
    return points;
}

std::vector<cv::Point3f> MakeCurvedObject() {
    std::vector<cv::Point3f> points;
    constexpr float center_x = 0.28f;
    constexpr float center_y = 0.02f;
    constexpr float radius = 0.075f;
    constexpr float width = 0.035f;
    constexpr float height = 0.025f;
    for (int angle_index = 0; angle_index <= 48; ++angle_index) {
        const float angle = -1.9f + 3.8f *
            static_cast<float>(angle_index) / 48.0f;
        for (int width_index = 0; width_index <= 10; ++width_index) {
            const float offset = -width * 0.5f + width *
                static_cast<float>(width_index) / 10.0f;
            const float sample_radius = radius + offset;
            points.emplace_back(
                center_x + sample_radius * std::cos(angle),
                center_y + sample_radius * std::sin(angle),
                height);
        }
    }
    return points;
}

bool Near(float actual, float expected, float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

bool HasValidStrategy(const std::vector<GraspCandidate>& candidates,
                        GraspStrategy strategy) {
    for (const GraspCandidate& candidate : candidates) {
        if (candidate.strategy == strategy && candidate.geometry_valid) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    GraspGeometryConfig config;
    GraspPlannerConfig planner_config;
    planner_config.workspace.z_max = 0.30f;
    const std::vector<cv::Point3f> table = MakeTable();

    ObjectGeometry3D tall_geometry;
    std::vector<cv::Point3f> tall_filtered;
    std::string error;
    const std::vector<cv::Point3f> tall_box =
        MakeBox(0.28f, 0.02f, 0.065f, 0.050f, 0.120f);
    if (!GraspGeometryPlanner::EstimateObjectGeometry(
            tall_box, table, config, tall_geometry, &tall_filtered, error)) {
        std::cerr << "tall object estimation failed: " << error << std::endl;
        return 1;
    }
    if (!Near(tall_geometry.length_m, 0.065f, 0.008f) ||
        !Near(tall_geometry.width_m, 0.050f, 0.008f) ||
        !Near(tall_geometry.height_m, 0.120f, 0.008f)) {
        std::cerr << "unexpected tall object dimensions" << std::endl;
        return 1;
    }

    ObjectGeometry3D stable_geometry = tall_geometry;
    stable_geometry.center.x += 0.008f;
    stable_geometry.length_m += 0.006f;
    stable_geometry.width_m -= 0.004f;
    stable_geometry.height_m += 0.007f;
    if (!AreObjectGeometriesConsistent(tall_geometry, stable_geometry)) {
        std::cerr << "normal geometry noise was rejected" << std::endl;
        return 1;
    }
    ObjectGeometry3D shifted_geometry = tall_geometry;
    shifted_geometry.center.x += 0.012f;
    if (AreObjectGeometriesConsistent(tall_geometry, shifted_geometry)) {
        std::cerr << "unstable geometry center was accepted" << std::endl;
        return 1;
    }
    ObjectGeometry3D recovered_width_geometry = tall_geometry;
    recovered_width_geometry.width_m *= 1.38f;
    if (!AreObjectDimensionsConsistent(
            tall_geometry, recovered_width_geometry)) {
        std::cerr << "recoverable width variation was rejected" << std::endl;
        return 1;
    }
    ObjectGeometry3D collapsed_geometry = tall_geometry;
    collapsed_geometry.length_m *= 0.50f;
    collapsed_geometry.width_m *= 0.45f;
    if (AreObjectDimensionsConsistent(
            tall_geometry, collapsed_geometry)) {
        std::cerr << "collapsed object geometry was accepted" << std::endl;
        return 1;
    }
    ObjectGeometry3D transient_geometry = tall_geometry;
    transient_geometry.length_m = 0.154f;
    transient_geometry.height_m = 0.009f;
    if (AreObjectGeometriesConsistent(tall_geometry, transient_geometry)) {
        std::cerr << "post-motion transient geometry was accepted"
                    << std::endl;
        return 1;
    }
    ObjectGeometry3D translated_geometry = tall_geometry;
    translated_geometry.center.x += 0.10f;
    translated_geometry.table_center.x += 0.10f;
    if (!AreObjectDimensionsConsistent(
            tall_geometry, translated_geometry) ||
        AreObjectGeometriesConsistent(tall_geometry, translated_geometry)) {
        std::cerr << "translated object dimension check is incorrect"
                    << std::endl;
        return 1;
    }

    const std::vector<GraspCandidate> tall_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            tall_geometry, tall_filtered, config, planner_config);
    if (tall_candidates.empty() ||
        tall_candidates.front().strategy != GraspStrategy::SIDE ||
        !tall_candidates.front().geometry_valid ||
        HasValidStrategy(tall_candidates, GraspStrategy::TOP)) {
        std::cerr << "tall object did not prefer a valid side grasp"
                    << std::endl;
        return 1;
    }
    const GraspCandidate& side = tall_candidates.front();
    const float radial_norm = std::hypot(
        tall_geometry.center.x, tall_geometry.center.y);
    const float expected_approach_distance =
        config.side_approach_distance_m;
    const float expected_lift_retreat_distance = std::max(
        config.side_approach_distance_m,
        config.side_lift_retreat_m);
    const float expected_entry_clearance_z =
        tall_geometry.table_center.z +
        tall_geometry.height_m +
        config.finger_half_height_m +
        config.side_entry_clearance_m;
    const cv::Point3f side_center =
        tall_geometry.table_center +
        tall_geometry.table.normal *
            (tall_geometry.height_m * config.side_grasp_height_ratio);
    const cv::Point3f side_center_to_grasp(
        side.grasp_pose.x - side_center.x,
        side.grasp_pose.y - side_center.y,
        side.grasp_pose.z - side_center.z);
    const float fixed_jaw_projection =
        side_center_to_grasp.dot(side.opening_axis);
    const float expected_fixed_jaw_projection =
        0.5f * side.required_width_m -
        config.footprint_padding_m +
        config.side_gripper_offset_m;
    if (radial_norm < 1e-6f ||
        side.approach_axis.x * tall_geometry.center.x / radial_norm +
            side.approach_axis.y * tall_geometry.center.y / radial_norm <
            0.99f ||
        !Near(fixed_jaw_projection,
            expected_fixed_jaw_projection, 0.003f) ||
        !Near(side.grasp_pose.z,
            tall_geometry.height_m * config.side_grasp_height_ratio,
            0.008f) ||
        !Near(side.pre_grasp_pose.x,
            side.grasp_pose.x - side.approach_axis.x *
                expected_approach_distance,
            1e-5f) ||
        !Near(side.entry_clearance_z_m,
            expected_entry_clearance_z, 1e-5f) ||
        !Near(side.retreat_pose.x, side.grasp_pose.x, 1e-5f) ||
        !Near(side.retreat_pose.y, side.grasp_pose.y, 1e-5f) ||
        !Near(side.retreat_pose.z,
            side.grasp_pose.z + config.side_initial_lift_m, 1e-5f) ||
        !Near(side.lift_pose.x,
            side.grasp_pose.x - side.approach_axis.x *
                expected_lift_retreat_distance,
            1e-5f) ||
        !Near(side.lift_pose.y,
            side.grasp_pose.y - side.approach_axis.y *
                expected_lift_retreat_distance,
            1e-5f) ||
        !Near(side.lift_pose.z,
            side.pre_grasp_pose.z + config.side_initial_lift_m, 1e-5f)) {
        std::cerr << "side grasp geometry is not radial or lift-safe"
                    << std::endl;
        return 1;
    }
    if (side.width_margin_m < 0.0f ||
        side.depth_quality <= 0.0f ||
        side.path_clearance_m <= 0.0f ||
        !std::isfinite(side.workspace_margin_m)) {
        std::cerr << "side candidate quality metrics are incomplete"
                    << std::endl;
        return 1;
    }
    for (const GraspCandidate& candidate : tall_candidates) {
        if (candidate.strategy == GraspStrategy::TOP &&
            !Near(candidate.required_width_m, 0.060f, 0.008f)) {
            std::cerr << "tall object incorrectly used a local top section"
                        << std::endl;
            return 1;
        }
    }
    GraspPlannerConfig changed_top_offset = planner_config;
    changed_top_offset.gripper_offset = 0.030f;
    const std::vector<GraspCandidate> unchanged_side_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            tall_geometry, tall_filtered, config, changed_top_offset);
    if (unchanged_side_candidates.empty() ||
        unchanged_side_candidates.front().strategy != GraspStrategy::SIDE ||
        !Near(unchanged_side_candidates.front().grasp_pose.x,
            side.grasp_pose.x, 1e-5f) ||
        !Near(unchanged_side_candidates.front().grasp_pose.y,
            side.grasp_pose.y, 1e-5f)) {
        std::cerr << "top-grasp offset changed the side-grasp pose"
                    << std::endl;
        return 1;
    }
    GraspGeometryConfig changed_side_offset = config;
    changed_side_offset.side_gripper_offset_m += 0.010f;
    const std::vector<GraspCandidate> shifted_side_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            tall_geometry, tall_filtered, changed_side_offset,
            planner_config);
    if (shifted_side_candidates.empty() ||
        shifted_side_candidates.front().strategy != GraspStrategy::SIDE ||
        Near(shifted_side_candidates.front().grasp_pose.x,
            side.grasp_pose.x, 1e-5f) &&
        Near(shifted_side_candidates.front().grasp_pose.y,
            side.grasp_pose.y, 1e-5f)) {
        std::cerr << "side-grasp offset did not change the side pose"
                    << std::endl;
        return 1;
    }
    GraspGeometryConfig changed_side_forward = config;
    changed_side_forward.side_grasp_forward_offset_m = 0.015f;
    const std::vector<GraspCandidate> advanced_side_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            tall_geometry, tall_filtered, changed_side_forward,
            planner_config);
    if (advanced_side_candidates.empty() ||
        advanced_side_candidates.front().strategy != GraspStrategy::SIDE ||
        !Near(advanced_side_candidates.front().pre_grasp_pose.x,
            side.pre_grasp_pose.x + side.approach_axis.x * 0.015f, 1e-5f) ||
        !Near(advanced_side_candidates.front().pre_grasp_pose.y,
            side.pre_grasp_pose.y + side.approach_axis.y * 0.015f, 1e-5f) ||
        !Near(advanced_side_candidates.front().grasp_pose.x,
            side.grasp_pose.x + side.approach_axis.x * 0.015f, 1e-5f) ||
        !Near(advanced_side_candidates.front().grasp_pose.y,
            side.grasp_pose.y + side.approach_axis.y * 0.015f, 1e-5f)) {
        std::cerr << "side forward offset changed the approach distance"
                    << std::endl;
        return 1;
    }
    GraspGeometryConfig changed_entry_clearance = config;
    changed_entry_clearance.side_entry_clearance_m += 0.020f;
    const std::vector<GraspCandidate> elevated_side_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            tall_geometry, tall_filtered, changed_entry_clearance,
            planner_config);
    if (elevated_side_candidates.empty() ||
        elevated_side_candidates.front().strategy != GraspStrategy::SIDE ||
        !Near(
            elevated_side_candidates.front().entry_clearance_z_m,
            side.entry_clearance_z_m + 0.020f, 1e-5f) ||
        !Near(
            elevated_side_candidates.front().pre_grasp_pose.x,
            side.pre_grasp_pose.x, 1e-5f) ||
        !Near(
            elevated_side_candidates.front().pre_grasp_pose.y,
            side.pre_grasp_pose.y, 1e-5f) ||
        !Near(
            elevated_side_candidates.front().grasp_pose.z,
            side.grasp_pose.z, 1e-5f)) {
        std::cerr
            << "side entry clearance changed the grasp geometry contract"
            << std::endl;
        return 1;
    }

    ObjectGeometry3D flat_geometry;
    std::vector<cv::Point3f> flat_filtered;
    const std::vector<cv::Point3f> flat_box =
        MakeBox(0.28f, -0.03f, 0.110f, 0.040f, 0.025f);
    if (!GraspGeometryPlanner::EstimateObjectGeometry(
            flat_box, table, config, flat_geometry, &flat_filtered, error)) {
        std::cerr << "flat object estimation failed: " << error << std::endl;
        return 1;
    }
    const std::vector<GraspCandidate> flat_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            flat_geometry, flat_filtered, config, planner_config);
    if (flat_candidates.empty() ||
        flat_candidates.front().strategy != GraspStrategy::TOP ||
        !flat_candidates.front().geometry_valid ||
        HasValidStrategy(flat_candidates, GraspStrategy::SIDE)) {
        std::cerr << "flat object did not select top grasp only" << std::endl;
        return 1;
    }
    if (flat_candidates.front().width_margin_m < 0.0f ||
        flat_candidates.front().depth_quality <= 0.0f ||
        flat_candidates.front().path_clearance_m <= 0.0f ||
        !std::isfinite(flat_candidates.front().workspace_margin_m)) {
        std::cerr << "top candidate quality metrics are incomplete"
                    << std::endl;
        return 1;
    }
    ObjectGeometry3D cup_geometry;
    std::vector<cv::Point3f> cup_filtered;
    const std::vector<cv::Point3f> cup_box =
        MakeBox(0.28f, 0.0f, 0.055f, 0.050f, 0.100f);
    if (!GraspGeometryPlanner::EstimateObjectGeometry(
            cup_box, table, config, cup_geometry, &cup_filtered, error)) {
        std::cerr << "cup geometry estimation failed: " << error
                    << std::endl;
        return 1;
    }
    const std::vector<GraspCandidate> cup_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            cup_geometry, cup_filtered, config, planner_config);
    if (cup_candidates.empty() ||
        cup_candidates.front().strategy != GraspStrategy::SIDE ||
        !cup_candidates.front().geometry_valid ||
        HasValidStrategy(cup_candidates, GraspStrategy::TOP)) {
        std::cerr << "upright geometry did not require a valid side grasp"
                    << std::endl;
        return 1;
    }
    ObjectGeometry3D far_cup_geometry = cup_geometry;
    far_cup_geometry.center.x += 0.35f;
    far_cup_geometry.table_center.x += 0.35f;
    std::vector<cv::Point3f> far_cup_points = cup_filtered;
    for (cv::Point3f& point : far_cup_points) {
        point.x += 0.35f;
    }
    const std::vector<GraspCandidate> far_cup_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            far_cup_geometry, far_cup_points, config, planner_config);
    if (far_cup_candidates.empty() ||
        far_cup_candidates.front().strategy != GraspStrategy::SIDE ||
        !far_cup_candidates.front().geometry_valid ||
        far_cup_candidates.front().grasp_pose.x <=
            planner_config.workspace.x_max) {
        std::cerr << "far cup candidate was rejected before base alignment"
                    << std::endl;
        return 1;
    }

    GraspPlannerConfig clamped_planner_config = planner_config;
    clamped_planner_config.workspace.z_min = 0.04f;
    const std::vector<GraspCandidate> clamped_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            flat_geometry, flat_filtered, config, clamped_planner_config);
    if (clamped_candidates.empty() ||
        !clamped_candidates.front().geometry_valid ||
        !Near(clamped_candidates.front().grasp_pose.z, 0.04f, 1e-5f)) {
        std::cerr << "top grasp was not clamped to the workspace floor"
                    << std::endl;
        return 1;
    }

    ObjectGeometry3D near_limit_geometry;
    std::vector<cv::Point3f> near_limit_filtered;
    const std::vector<cv::Point3f> near_limit_box =
        MakeBox(0.28f, 0.0f, 0.060f, 0.073f, 0.120f);
    if (!GraspGeometryPlanner::EstimateObjectGeometry(
            near_limit_box, table, config, near_limit_geometry,
            &near_limit_filtered, error)) {
        std::cerr << "near-limit object estimation failed: " << error
                    << std::endl;
        return 1;
    }
    const std::vector<GraspCandidate> near_limit_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            near_limit_geometry, near_limit_filtered, config,
            planner_config);
    if (near_limit_candidates.empty() ||
        near_limit_candidates.front().strategy != GraspStrategy::SIDE ||
        !near_limit_candidates.front().geometry_valid ||
        !Near(near_limit_candidates.front().required_width_m,
            config.gripper_max_width_m, 1e-5f)) {
        std::cerr << "near-limit side grasp did not use full opening"
                    << std::endl;
        return 1;
    }

    ObjectGeometry3D wide_geometry;
    std::vector<cv::Point3f> wide_filtered;
    const std::vector<cv::Point3f> wide_box =
        MakeBox(0.28f, 0.0f, 0.120f, 0.090f, 0.025f);
    if (!GraspGeometryPlanner::EstimateObjectGeometry(
            wide_box, table, config, wide_geometry, &wide_filtered, error)) {
        std::cerr << "wide object estimation failed: " << error << std::endl;
        return 1;
    }
    const std::vector<GraspCandidate> wide_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            wide_geometry, wide_filtered, config, planner_config);
    if (HasValidStrategy(wide_candidates, GraspStrategy::TOP) ||
        HasValidStrategy(wide_candidates, GraspStrategy::SIDE)) {
        std::cerr << "unsafe wide object candidate was accepted" << std::endl;
        for (const GraspCandidate& candidate : wide_candidates) {
            std::cerr << "  strategy="
                        << perceptive_grasp::GraspStrategyName(
                            candidate.strategy)
                        << " valid=" << candidate.geometry_valid
                        << " width=" << candidate.required_width_m
                        << " reason=" << candidate.rejection_reason
                        << std::endl;
        }
        return 1;
    }

    ObjectGeometry3D curved_geometry;
    std::vector<cv::Point3f> curved_filtered;
    const std::vector<cv::Point3f> curved_object = MakeCurvedObject();
    if (!GraspGeometryPlanner::EstimateObjectGeometry(
            curved_object, table, config, curved_geometry, &curved_filtered,
            error)) {
        std::cerr << "curved object estimation failed: " << error << std::endl;
        return 1;
    }
    const std::vector<GraspCandidate> curved_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            curved_geometry, curved_filtered, config, planner_config);
    if (curved_candidates.empty() ||
        curved_candidates.front().strategy != GraspStrategy::TOP ||
        !curved_candidates.front().geometry_valid ||
        curved_candidates.front().required_width_m >
            config.gripper_max_width_m) {
        std::cerr << "curved object did not produce a local top grasp"
                    << std::endl;
        return 1;
    }

    cv::Mat synthetic_depth(120, 160, CV_16UC1, cv::Scalar(1000));
    cv::Mat synthetic_mask = cv::Mat::zeros(120, 160, CV_8UC1);
    const cv::Rect object_region(50, 40, 60, 45);
    synthetic_depth(object_region).setTo(900);
    synthetic_mask(object_region).setTo(255);
    perceptive_grasp::DetectionTarget synthetic_target{};
    synthetic_target.x1 = object_region.x;
    synthetic_target.y1 = object_region.y;
    synthetic_target.x2 = object_region.x + object_region.width;
    synthetic_target.y2 = object_region.y + object_region.height;
    synthetic_target.center = cv::Point2f(80.0f, 62.5f);
    synthetic_target.label_name = "cup";
    synthetic_target.mask = synthetic_mask;

    GraspPlanner coordinate_planner(planner_config);
    GraspGeometryPlanner image_planner(config, planner_config);
    SyntheticCamera synthetic_camera;
    GraspGeometryResult image_result;
    if (!image_planner.Plan(synthetic_depth, synthetic_target,
                            synthetic_camera, coordinate_planner,
                            image_result)) {
        std::cerr << "mask-guided geometry planning failed: "
                    << image_result.error << std::endl;
        return 1;
    }
    if (image_result.geometry.object_point_count < config.min_object_points ||
        image_result.elapsed_ms > 50) {
        std::cerr << "mask-guided geometry planning exceeded its budget"
                    << std::endl;
        return 1;
    }

    cv::Mat contaminated_depth = synthetic_depth.clone();
    contaminated_depth(cv::Rect(63, 50, 34, 25)).setTo(3000);
    GraspGeometryResult contaminated_result;
    if (!image_planner.Plan(
            contaminated_depth, synthetic_target, synthetic_camera,
            coordinate_planner, contaminated_result)) {
        std::cerr << "foreground depth filtering failed: "
                    << contaminated_result.error << std::endl;
        return 1;
    }
    if (!Near(
            contaminated_result.geometry.length_m,
            image_result.geometry.length_m, 0.005f) ||
        !Near(
            contaminated_result.geometry.width_m,
            image_result.geometry.width_m, 0.005f) ||
        contaminated_result.geometry.source_point_count >=
            image_result.geometry.source_point_count) {
        std::cerr << "far filled depth contaminated object geometry"
                    << std::endl;
        return 1;
    }

    cv::Mat hollow_depth = synthetic_depth.clone();
    hollow_depth(cv::Rect(57, 46, 46, 33)).setTo(1150);
    GraspGeometryResult hollow_result;
    if (!image_planner.Plan(
            hollow_depth, synthetic_target, synthetic_camera,
            coordinate_planner, hollow_result)) {
        std::cerr << "hollow-object foreground filtering failed: "
                    << hollow_result.error << std::endl;
        return 1;
    }
    if (!Near(hollow_result.geometry.length_m,
            image_result.geometry.length_m, 0.005f) ||
        !Near(hollow_result.geometry.width_m,
            image_result.geometry.width_m, 0.005f) ||
        !Near(hollow_result.geometry.height_m,
            image_result.geometry.height_m, 0.005f) ||
        !Near(hollow_result.foreground_depth_mm, 900.0f, 1.0f)) {
        std::cerr << "hollow-object background contaminated geometry"
                    << std::endl;
        return 1;
    }

    cv::Mat occluded_depth = synthetic_depth.clone();
    occluded_depth(cv::Rect(68, 52, 24, 18)).setTo(300);
    GraspGeometryResult occluded_result;
    if (!image_planner.Plan(
            occluded_depth, synthetic_target, synthetic_camera,
            coordinate_planner, occluded_result)) {
        std::cerr << "near-occluder depth filtering failed: "
                    << occluded_result.error << std::endl;
        return 1;
    }
    if (!Near(occluded_result.geometry.length_m,
            image_result.geometry.length_m, 0.005f) ||
        !Near(occluded_result.geometry.width_m,
            image_result.geometry.width_m, 0.005f) ||
        !Near(occluded_result.geometry.height_m,
            image_result.geometry.height_m, 0.005f) ||
        !Near(occluded_result.foreground_depth_mm, 900.0f, 1.0f)) {
        std::cerr << "near occluder was selected instead of the object"
                    << std::endl;
        return 1;
    }

    cv::Mat sparse_container_depth(120, 160, CV_16UC1);
    for (int y = 0; y < sparse_container_depth.rows; ++y) {
        const float ray_z =
            0.8f + (static_cast<float>(y) - 60.0f) / 400.0f;
        sparse_container_depth.row(y).setTo(
            static_cast<uint16_t>(std::lround(1000.0f / ray_z)));
    }
    cv::Mat sparse_container_mask =
        cv::Mat::zeros(sparse_container_depth.size(), CV_8UC1);
    sparse_container_mask(cv::Rect(65, 40, 31, 41)).setTo(255);
    sparse_container_depth.setTo(0, sparse_container_mask);

    DetectionTarget sparse_container_target;
    sparse_container_target.x1 = 65.0f;
    sparse_container_target.y1 = 40.0f;
    sparse_container_target.x2 = 96.0f;
    sparse_container_target.y2 = 81.0f;
    sparse_container_target.center = cv::Point2f(80.0f, 60.0f);
    sparse_container_target.label_name = "cup";
    sparse_container_target.mask = sparse_container_mask;

    GraspPlannerConfig oblique_planner_config;
    oblique_planner_config.t_base_camera = {0.0f, 0.0f, 1.0f};
    oblique_planner_config.workspace.x_max = 2.0f;
    oblique_planner_config.workspace.z_max = 0.30f;
    GraspGeometryConfig sparse_container_config = config;
    sparse_container_config.gripper_max_width_m = 0.11f;
    GraspPlanner oblique_coordinate_planner(oblique_planner_config);
    GraspGeometryPlanner sparse_container_planner(
        sparse_container_config, oblique_planner_config);
    ObliqueSyntheticCamera oblique_camera;
    GraspGeometryResult sparse_container_result;
    if (!sparse_container_planner.Plan(
            sparse_container_depth, sparse_container_target,
            oblique_camera, oblique_coordinate_planner,
            sparse_container_result)) {
        std::cerr << "sparse container silhouette fallback failed: "
                    << sparse_container_result.error << std::endl;
        return 1;
    }
    if (sparse_container_result.geometry.height_m < 0.08f ||
        sparse_container_result.geometry.height_m > 0.16f ||
        sparse_container_result.geometry.width_m < 0.06f ||
        sparse_container_result.geometry.width_m > 0.11f ||
        !HasValidStrategy(
            sparse_container_result.candidates, GraspStrategy::SIDE)) {
        std::cerr << "sparse container silhouette geometry is invalid"
                    << std::endl;
        return 1;
    }

    cv::Mat portrait_lie_depth(120, 160, CV_16UC1);
    for (int y = 0; y < portrait_lie_depth.rows; ++y) {
        const float ray_z =
            0.8f + (static_cast<float>(y) - 60.0f) / 400.0f;
        portrait_lie_depth.row(y).setTo(
            static_cast<uint16_t>(std::lround(1000.0f / ray_z)));
    }
    cv::Mat portrait_lie_mask =
        cv::Mat::zeros(portrait_lie_depth.size(), CV_8UC1);
    const cv::Rect portrait_lie_region(64, 30, 33, 61);
    portrait_lie_mask(portrait_lie_region).setTo(255);
    for (int y = portrait_lie_region.y;
        y < portrait_lie_region.y + portrait_lie_region.height; ++y) {
        const float ray_z =
            0.8f + (static_cast<float>(y) - 60.0f) / 400.0f;
        portrait_lie_depth.row(y).colRange(
            portrait_lie_region.x,
            portrait_lie_region.x + portrait_lie_region.width).setTo(
                static_cast<uint16_t>(std::lround(930.0f / ray_z)));
    }
    const cv::Rect portrait_lie_hollow(72, 48, 17, 25);
    for (int y = portrait_lie_hollow.y;
        y < portrait_lie_hollow.y + portrait_lie_hollow.height; ++y) {
        const float ray_z =
            0.8f + (static_cast<float>(y) - 60.0f) / 400.0f;
        portrait_lie_depth.row(y).colRange(
            portrait_lie_hollow.x,
            portrait_lie_hollow.x + portrait_lie_hollow.width).setTo(
                static_cast<uint16_t>(std::lround(1000.0f / ray_z)));
    }

    DetectionTarget portrait_lie_target;
    portrait_lie_target.x1 =
        static_cast<float>(portrait_lie_region.x);
    portrait_lie_target.y1 =
        static_cast<float>(portrait_lie_region.y);
    portrait_lie_target.x2 = static_cast<float>(
        portrait_lie_region.x + portrait_lie_region.width);
    portrait_lie_target.y2 = static_cast<float>(
        portrait_lie_region.y + portrait_lie_region.height);
    portrait_lie_target.center = cv::Point2f(80.0f, 60.0f);
    portrait_lie_target.label_name = "cup";
    portrait_lie_target.mask = portrait_lie_mask;

    GraspGeometryResult portrait_lie_result;
    if (!sparse_container_planner.Plan(
            portrait_lie_depth, portrait_lie_target,
            oblique_camera, oblique_coordinate_planner,
            portrait_lie_result)) {
        std::cerr << "portrait lying-object planning failed: "
                    << portrait_lie_result.error << std::endl;
        return 1;
    }
    if (portrait_lie_result.geometry.length_m <=
            portrait_lie_result.geometry.height_m ||
        !HasValidStrategy(
            portrait_lie_result.candidates, GraspStrategy::TOP) ||
        HasValidStrategy(
            portrait_lie_result.candidates, GraspStrategy::SIDE)) {
        std::cerr
            << "table-relative geometry did not override the upright "
            << "silhouette"
            << std::endl;
        return 1;
    }

    DetectionTarget fallen_container_target = sparse_container_target;
    fallen_container_target.x1 = 40.0f;
    fallen_container_target.x2 = 120.0f;
    fallen_container_target.y1 = 45.0f;
    fallen_container_target.y2 = 75.0f;
    GraspGeometryResult fallen_container_result;
    if (sparse_container_planner.Plan(
            sparse_container_depth, fallen_container_target,
            oblique_camera, oblique_coordinate_planner,
            fallen_container_result) ||
        fallen_container_result.error !=
            "too few valid depth samples in target mask") {
        std::cerr << "fallen container used the upright silhouette fallback"
                    << std::endl;
        return 1;
    }

    DetectionTarget missing_mask_target = synthetic_target;
    missing_mask_target.mask.release();
    GraspGeometryResult missing_mask_result;
    if (image_planner.Plan(synthetic_depth, missing_mask_target,
                            synthetic_camera, coordinate_planner,
                            missing_mask_result) ||
        missing_mask_result.error !=
            "target segmentation mask is unavailable") {
        std::cerr << "missing segmentation mask was not rejected"
                    << std::endl;
        return 1;
    }

    DetectionTarget clipped_box_target = synthetic_target;
    clipped_box_target.x1 = -100.0f;
    clipped_box_target.y1 = -100.0f;
    clipped_box_target.x2 = 1000.0f;
    clipped_box_target.y2 = 1000.0f;
    GraspGeometryResult clipped_box_result;
    if (!image_planner.Plan(synthetic_depth, clipped_box_target,
                            synthetic_camera, coordinate_planner,
                            clipped_box_result)) {
        std::cerr << "out-of-bounds detection box was not clipped: "
                    << clipped_box_result.error << std::endl;
        return 1;
    }

    std::cout << "mask_geometry_ms=" << image_result.elapsed_ms
                << " object_points="
                << image_result.geometry.object_point_count << std::endl;

    std::cout << "grasp_geometry_test passed" << std::endl;
    return 0;
}
