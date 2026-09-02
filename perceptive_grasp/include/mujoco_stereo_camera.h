/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_stereo_camera.h
 * @brief MuJoCo rendered camera backend.
 */

#ifndef MUJOCO_STEREO_CAMERA_H
#define MUJOCO_STEREO_CAMERA_H

#include <memory>

#include "stereo_camera.h"

namespace perceptive_grasp {

std::unique_ptr<StereoCamera> CreateMujocoStereoCamera(
    const StereoCameraConfig& config);

}  // namespace perceptive_grasp

#endif  // MUJOCO_STEREO_CAMERA_H
