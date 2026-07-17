/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file las2_stereo_camera.h
 * @brief spacemit_las2 stereo camera backend.
 */

#ifndef PERCEPTIVE_GRASP_LAS2_STEREO_CAMERA_H_
#define PERCEPTIVE_GRASP_LAS2_STEREO_CAMERA_H_

#include <memory>

#include "stereo_camera.h"

namespace perceptive_grasp {

/** Create a spacemit_las2 stereo camera backend. */
std::unique_ptr<StereoCamera> CreateLas2StereoCamera(
    const StereoCameraConfig& config);

}  // namespace perceptive_grasp

#endif  // PERCEPTIVE_GRASP_LAS2_STEREO_CAMERA_H_
