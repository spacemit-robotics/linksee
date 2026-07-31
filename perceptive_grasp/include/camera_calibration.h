/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file camera_calibration.h
 * @brief Load camera-backend-specific hand-eye calibration.
 */

#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <string>

#include <yaml-cpp/yaml.h>

#include "grasp_planner.h"

namespace perceptive_grasp {

void LoadCameraCalibration(
    const YAML::Node& root,
    const std::string& camera_type,
    GraspPlannerConfig* planner_config);

}  // namespace perceptive_grasp

#endif  // CAMERA_CALIBRATION_H
