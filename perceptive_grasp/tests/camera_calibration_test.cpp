/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file camera_calibration_test.cpp
 * @brief Unit tests for camera-specific hand-eye calibration loading.
 */

#include <cassert>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "camera_calibration.h"

namespace {

bool NearlyEqual(float left, float right) {
    return std::fabs(left - right) < 1e-6f;
}

}  // namespace

int main() {
    YAML::Node root;
    root["calibration"]["realsense"]["T_base_camera"]["translation"] =
        YAML::Load("[0.1, 0.2, 0.3]");
    root["calibration"]["realsense"]["T_base_camera"]["rotation"] =
        YAML::Load("[0.4, 0.5, 0.6]");
    root["calibration"]["spacemit_las2"]["T_base_camera"]["translation"] =
        YAML::Load("[1.1, 1.2, 1.3]");
    root["calibration"]["spacemit_las2"]["T_base_camera"]["rotation"] =
        YAML::Load("[1.4, 1.5, 1.6]");

    perceptive_grasp::GraspPlannerConfig realsense_config;
    perceptive_grasp::LoadCameraCalibration(
        root, "realsense", &realsense_config);
    assert(NearlyEqual(realsense_config.t_base_camera[0], 0.1f));
    assert(NearlyEqual(realsense_config.r_base_camera[2], 0.6f));

    perceptive_grasp::GraspPlannerConfig alias_config;
    perceptive_grasp::LoadCameraCalibration(
        root, "d435i", &alias_config);
    assert(NearlyEqual(alias_config.t_base_camera[2], 0.3f));

    const YAML::Node legacy_root = YAML::Load(
        "{calibration: {T_base_camera: {"
        "translation: [2.1, 2.2, 2.3], "
        "rotation: [2.4, 2.5, 2.6]}}}");
    perceptive_grasp::GraspPlannerConfig legacy_config;
    perceptive_grasp::LoadCameraCalibration(
        legacy_root, "realsense", &legacy_config);
    assert(NearlyEqual(legacy_config.t_base_camera[0], 2.1f));

    perceptive_grasp::GraspPlannerConfig las2_config;
    perceptive_grasp::LoadCameraCalibration(
        root, "spacemit_las2", &las2_config);
    assert(NearlyEqual(las2_config.t_base_camera[0], 1.1f));
    assert(NearlyEqual(las2_config.r_base_camera[2], 1.6f));

    bool missing_profile_rejected = false;
    try {
        perceptive_grasp::GraspPlannerConfig missing_config;
        perceptive_grasp::LoadCameraCalibration(
            root, "unknown", &missing_config);
    } catch (const std::runtime_error&) {
        missing_profile_rejected = true;
    }
    assert(missing_profile_rejected);

    return 0;
}
