/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_strategy_regression_test.cpp
 * @brief Regression contract for established top- and side-grasp behavior.
 */

#include "grasp_geometry.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

using perceptive_grasp::GraspCandidate;
using perceptive_grasp::GraspGeometryConfig;
using perceptive_grasp::GraspGeometryPlanner;
using perceptive_grasp::GraspPlannerConfig;
using perceptive_grasp::GraspStrategy;
using perceptive_grasp::ObjectGeometry3D;

constexpr float kTolerance = 1e-5f;

bool Near(float actual, float expected, float tolerance = kTolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

float Distance(const perceptive_grasp::Pose3D& first,
                const perceptive_grasp::Pose3D& second) {
    return std::sqrt(
        std::pow(first.x - second.x, 2.0f) +
        std::pow(first.y - second.y, 2.0f) +
        std::pow(first.z - second.z, 2.0f));
}

bool HasValidStrategy(const std::vector<GraspCandidate>& candidates,
    GraspStrategy strategy) {
    return std::any_of(
        candidates.begin(), candidates.end(),
        [strategy](const GraspCandidate& candidate) {
            return candidate.strategy == strategy &&
                candidate.geometry_valid;
        });
}

ObjectGeometry3D MakeGeometry(float angle_rad,
    float length_m,
    float width_m,
    float height_m) {
    ObjectGeometry3D geometry;
    geometry.valid = true;
    geometry.table.normal = cv::Point3f(0.0f, 0.0f, 1.0f);
    geometry.table.d = 0.0f;
    geometry.table_center = cv::Point3f(0.29f, 0.015f, 0.0f);
    geometry.center = geometry.table_center +
        cv::Point3f(0.0f, 0.0f, height_m * 0.5f);
    geometry.major_axis =
        cv::Point3f(std::cos(angle_rad), std::sin(angle_rad), 0.0f);
    geometry.minor_axis =
        cv::Point3f(-std::sin(angle_rad), std::cos(angle_rad), 0.0f);
    geometry.length_m = length_m;
    geometry.width_m = width_m;
    geometry.height_m = height_m;
    geometry.object_point_count = 600;
    geometry.source_point_count = 600;
    return geometry;
}

std::vector<cv::Point3f> MakeObjectPoints(
    const ObjectGeometry3D& geometry) {
    std::vector<cv::Point3f> points;
    points.reserve(21 * 13 * 2);
    for (int length_index = 0; length_index <= 20; ++length_index) {
        const float length_offset = -geometry.length_m * 0.5f +
            geometry.length_m * static_cast<float>(length_index) / 20.0f;
        for (int width_index = 0; width_index <= 12; ++width_index) {
            const float width_offset = -geometry.width_m * 0.5f +
                geometry.width_m * static_cast<float>(width_index) / 12.0f;
            const cv::Point3f footprint =
                geometry.table_center +
                geometry.major_axis * length_offset +
                geometry.minor_axis * width_offset;
            points.push_back(
                footprint +
                geometry.table.normal * geometry.height_m);
            points.push_back(
                footprint +
                geometry.table.normal * (geometry.height_m * 0.5f));
        }
    }
    return points;
}

const GraspCandidate* FirstValid(
    const std::vector<GraspCandidate>& candidates) {
    const auto candidate = std::find_if(
        candidates.begin(), candidates.end(),
        [](const GraspCandidate& value) {
            return value.geometry_valid;
        });
    return candidate == candidates.end() ? nullptr : &*candidate;
}

bool CheckTopGrasp(float angle_rad,
    const GraspGeometryConfig& geometry_config,
    const GraspPlannerConfig& planner_config) {
    const ObjectGeometry3D geometry =
        MakeGeometry(angle_rad, 0.14f, 0.035f, 0.025f);
    const std::vector<cv::Point3f> points = MakeObjectPoints(geometry);
    const std::vector<GraspCandidate> candidates =
        GraspGeometryPlanner::GenerateCandidates(
            geometry, points, geometry_config, planner_config);
    const GraspCandidate* selected = FirstValid(candidates);
    if (selected == nullptr ||
        selected->strategy != GraspStrategy::TOP ||
        HasValidStrategy(candidates, GraspStrategy::SIDE)) {
        std::cerr << "banana did not preserve top grasp at angle "
            << angle_rad << std::endl;
        return false;
    }
    if (!Near(
            selected->pre_grasp_pose.x, selected->grasp_pose.x) ||
        !Near(
            selected->pre_grasp_pose.y, selected->grasp_pose.y) ||
        !Near(
            selected->pre_grasp_pose.z - selected->grasp_pose.z,
            planner_config.approach_height) ||
        selected->required_width_m > geometry_config.gripper_max_width_m) {
        std::cerr << "top grasp pose contract changed at angle "
            << angle_rad << std::endl;
        return false;
    }

    GraspGeometryConfig changed_side = geometry_config;
    changed_side.side_gripper_offset_m += 0.020f;
    changed_side.side_approach_distance_m += 0.040f;
    const std::vector<GraspCandidate> changed_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            geometry, points, changed_side, planner_config);
    const GraspCandidate* changed_selected = FirstValid(changed_candidates);
    if (changed_selected == nullptr ||
        changed_selected->strategy != GraspStrategy::TOP ||
        !Near(changed_selected->grasp_pose.x, selected->grasp_pose.x) ||
        !Near(changed_selected->grasp_pose.y, selected->grasp_pose.y) ||
        !Near(changed_selected->grasp_pose.z, selected->grasp_pose.z)) {
        std::cerr << "side-grasp parameters changed the banana top grasp"
            << std::endl;
        return false;
    }
    return true;
}

bool CheckSideGrasp(float angle_rad,
    const GraspGeometryConfig& geometry_config,
    const GraspPlannerConfig& planner_config) {
    const ObjectGeometry3D geometry =
        MakeGeometry(angle_rad, 0.070f, 0.055f, 0.100f);
    const std::vector<cv::Point3f> points = MakeObjectPoints(geometry);
    const std::vector<GraspCandidate> candidates =
        GraspGeometryPlanner::GenerateCandidates(
            geometry, points, geometry_config, planner_config);
    const GraspCandidate* selected = FirstValid(candidates);
    if (selected == nullptr ||
        selected->strategy != GraspStrategy::SIDE ||
        HasValidStrategy(candidates, GraspStrategy::TOP)) {
        std::cerr << "upright cup did not preserve side grasp at angle "
            << angle_rad << std::endl;
        return false;
    }

    const cv::Point3f approach_delta(
        selected->grasp_pose.x - selected->pre_grasp_pose.x,
        selected->grasp_pose.y - selected->pre_grasp_pose.y,
        selected->grasp_pose.z - selected->pre_grasp_pose.z);
    const float approach_projection =
        approach_delta.dot(selected->approach_axis);
    if (!Near(
            Distance(selected->pre_grasp_pose, selected->grasp_pose),
            geometry_config.side_approach_distance_m) ||
        !Near(
            approach_projection,
            geometry_config.side_approach_distance_m) ||
        !Near(selected->pre_grasp_pose.z, selected->grasp_pose.z) ||
        !Near(
            selected->retreat_pose.z - selected->grasp_pose.z,
            geometry_config.side_initial_lift_m)) {
        std::cerr << "side grasp approach or lift contract changed at angle "
            << angle_rad << std::endl;
        return false;
    }

    GraspPlannerConfig changed_top = planner_config;
    changed_top.gripper_offset += 0.030f;
    changed_top.approach_height += 0.050f;
    const std::vector<GraspCandidate> changed_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            geometry, points, geometry_config, changed_top);
    const GraspCandidate* changed_selected = FirstValid(changed_candidates);
    if (changed_selected == nullptr ||
        changed_selected->strategy != GraspStrategy::SIDE ||
        !Near(changed_selected->grasp_pose.x, selected->grasp_pose.x) ||
        !Near(changed_selected->grasp_pose.y, selected->grasp_pose.y) ||
        !Near(changed_selected->grasp_pose.z, selected->grasp_pose.z)) {
        std::cerr << "top-grasp parameters changed the cup side grasp"
            << std::endl;
        return false;
    }
    return true;
}

bool CheckLockedSideHysteresis(
    const GraspGeometryConfig& geometry_config,
    const GraspPlannerConfig& planner_config) {
    const ObjectGeometry3D borderline =
        MakeGeometry(0.0f, 0.060f, 0.055f, 0.058f);
    const std::vector<cv::Point3f> points = MakeObjectPoints(borderline);
    const std::vector<GraspCandidate> initial_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            borderline, points, geometry_config, planner_config);
    if (!HasValidStrategy(initial_candidates, GraspStrategy::TOP) ||
        HasValidStrategy(initial_candidates, GraspStrategy::SIDE)) {
        std::cerr << "side hysteresis changed initial strategy selection"
                    << std::endl;
        return false;
    }

    const std::vector<GraspCandidate> locked_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            borderline, points, geometry_config, planner_config,
            GraspStrategy::SIDE);
    if (!HasValidStrategy(locked_candidates, GraspStrategy::SIDE)) {
        std::cerr << "locked side strategy was lost to geometry jitter"
                    << std::endl;
        return false;
    }

    ObjectGeometry3D collapsed = borderline;
    collapsed.height_m = 0.040f;
    collapsed.center.z = collapsed.height_m * 0.5f;
    const std::vector<cv::Point3f> collapsed_points =
        MakeObjectPoints(collapsed);
    const std::vector<GraspCandidate> collapsed_candidates =
        GraspGeometryPlanner::GenerateCandidates(
            collapsed, collapsed_points, geometry_config, planner_config,
            GraspStrategy::SIDE);
    if (HasValidStrategy(collapsed_candidates, GraspStrategy::SIDE)) {
        std::cerr << "side hysteresis accepted collapsed geometry"
                    << std::endl;
        return false;
    }
    return true;
}

bool CheckLyingObjectUsesTop(
    const GraspGeometryConfig& geometry_config,
    const GraspPlannerConfig& planner_config) {
    const ObjectGeometry3D lying_object =
        MakeGeometry(0.0f, 0.099f, 0.066f, 0.073f);
    const std::vector<cv::Point3f> points =
        MakeObjectPoints(lying_object);

    for (const std::optional<GraspStrategy> locked_strategy : {
            std::optional<GraspStrategy>{},
            std::optional<GraspStrategy>{GraspStrategy::SIDE}}) {
        const std::vector<GraspCandidate> candidates =
            GraspGeometryPlanner::GenerateCandidates(
                lying_object, points, geometry_config, planner_config,
                locked_strategy);
        const GraspCandidate* selected = FirstValid(candidates);
        if (selected == nullptr ||
            selected->strategy != GraspStrategy::TOP ||
            HasValidStrategy(candidates, GraspStrategy::SIDE)) {
            std::cerr
                << "lying object did not select top grasp"
                << std::endl;
            return false;
        }
    }
    return true;
}

bool CheckRoundObjectUsesTop(
    const GraspGeometryConfig& geometry_config,
    const GraspPlannerConfig& planner_config) {
    const ObjectGeometry3D round_object =
        MakeGeometry(0.0f, 0.075f, 0.070f, 0.075f);
    const std::vector<cv::Point3f> points =
        MakeObjectPoints(round_object);
    const std::vector<GraspCandidate> candidates =
        GraspGeometryPlanner::GenerateCandidates(
            round_object, points, geometry_config, planner_config);
    const GraspCandidate* selected = FirstValid(candidates);
    if (selected == nullptr ||
        selected->strategy != GraspStrategy::TOP ||
        HasValidStrategy(candidates, GraspStrategy::SIDE)) {
        std::cerr
            << "round tabletop object did not select top grasp"
            << std::endl;
        return false;
    }
    return true;
}

bool CheckShortRoundObjectUsesTopWithReleaseThresholds(
    const GraspGeometryConfig& geometry_config,
    const GraspPlannerConfig& planner_config) {
    GraspGeometryConfig release_config = geometry_config;
    release_config.side_min_height_m = 0.080f;
    release_config.side_min_height_width_ratio = 1.2f;

    const ObjectGeometry3D short_round_object =
        MakeGeometry(0.0f, 0.047f, 0.047f, 0.065f);
    const std::vector<cv::Point3f> points =
        MakeObjectPoints(short_round_object);
    const std::vector<GraspCandidate> candidates =
        GraspGeometryPlanner::GenerateCandidates(
            short_round_object, points, release_config, planner_config);
    const GraspCandidate* selected = FirstValid(candidates);
    if (selected == nullptr ||
        selected->strategy != GraspStrategy::TOP ||
        HasValidStrategy(candidates, GraspStrategy::SIDE)) {
        std::cerr
            << "short round tabletop object did not select top grasp"
            << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main() {
    GraspGeometryConfig geometry_config;
    geometry_config.strategy = "auto";
    geometry_config.gripper_max_width_m = 0.10f;
    geometry_config.footprint_padding_m = 0.005f;
    geometry_config.side_min_height_m = 0.060f;
    geometry_config.side_min_height_width_ratio = 1.10f;
    geometry_config.side_approach_distance_m = 0.030f;
    geometry_config.side_entry_clearance_m = 0.040f;
    geometry_config.side_gripper_offset_m = 0.015f;
    geometry_config.side_grasp_forward_offset_m = 0.020f;
    geometry_config.side_initial_lift_m = 0.050f;

    GraspPlannerConfig planner_config;
    planner_config.approach_height = 0.10f;
    planner_config.grasp_depth = 0.010f;
    planner_config.gripper_offset = 0.005f;
    planner_config.workspace.z_max = 0.30f;

    const std::vector<float> angles = {
        0.0f,
        static_cast<float>(CV_PI) / 6.0f,
        static_cast<float>(CV_PI) / 3.0f,
        static_cast<float>(CV_PI) / 2.0f,
    };
    for (const float angle : angles) {
        if (!CheckTopGrasp(angle, geometry_config, planner_config) ||
            !CheckSideGrasp(angle, geometry_config, planner_config)) {
            return 1;
        }
    }
    if (!CheckLockedSideHysteresis(
            geometry_config, planner_config)) {
        return 1;
    }
    if (!CheckLyingObjectUsesTop(
            geometry_config, planner_config)) {
        return 1;
    }
    if (!CheckRoundObjectUsesTop(
            geometry_config, planner_config)) {
        return 1;
    }
    if (!CheckShortRoundObjectUsesTopWithReleaseThresholds(
            geometry_config, planner_config)) {
        return 1;
    }

    std::cout << "grasp_strategy_regression_test passed" << std::endl;
    return 0;
}
