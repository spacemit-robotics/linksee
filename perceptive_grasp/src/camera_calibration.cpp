/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file camera_calibration.cpp
 * @brief Load camera-backend-specific hand-eye calibration.
 */

#include "camera_calibration.h"

#include <stdexcept>
#include <string>

namespace perceptive_grasp {
namespace {

std::string NormalizeCameraType(const std::string& camera_type) {
    return camera_type == "d435i" ? "realsense" : camera_type;
}

void ValidateTransformVector(
    const YAML::Node& values,
    const std::string& field_name) {
    if (!values || !values.IsSequence() || values.size() != 3) {
        throw std::runtime_error(
            field_name + " must contain exactly three values");
    }
}

}  // namespace

void LoadCameraCalibration(
    const YAML::Node& root,
    const std::string& camera_type,
    GraspPlannerConfig* planner_config) {
    if (planner_config == nullptr) {
        throw std::invalid_argument("planner_config must not be null");
    }

    const std::string backend = NormalizeCameraType(camera_type);
    const YAML::Node calibration = root["calibration"];
    const YAML::Node backend_calibration =
        calibration ? calibration[backend] : YAML::Node();
    YAML::Node transform = backend_calibration
        ? backend_calibration["T_base_camera"]
        : YAML::Node();
    if ((!transform || !transform.IsMap()) &&
        backend == "realsense" && calibration) {
        transform = calibration["T_base_camera"];
    }
    const std::string field_prefix =
        "calibration." + backend + ".T_base_camera";

    if (!transform || !transform.IsMap()) {
        throw std::runtime_error(field_prefix + " configuration is required");
    }

    const YAML::Node translation = transform["translation"];
    const YAML::Node rotation = transform["rotation"];
    ValidateTransformVector(translation, field_prefix + ".translation");
    ValidateTransformVector(rotation, field_prefix + ".rotation");

    for (size_t index = 0; index < 3; ++index) {
        planner_config->t_base_camera[index] =
            translation[index].as<float>();
        planner_config->r_base_camera[index] = rotation[index].as<float>();
    }
}

}  // namespace perceptive_grasp
