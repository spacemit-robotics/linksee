/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file las2_stereo_camera.h
 * @brief spacemit_las2 stereo camera backend.
 */

#ifndef LAS2_STEREO_CAMERA_H
#define LAS2_STEREO_CAMERA_H

#include <memory>

#include "stereo_camera.h"

namespace perceptive_grasp {

/** Create a spacemit_las2 stereo camera backend. */
std::unique_ptr<StereoCamera> CreateLas2StereoCamera(
    const StereoCameraConfig& config);

}  // namespace perceptive_grasp

#endif  // LAS2_STEREO_CAMERA_H
