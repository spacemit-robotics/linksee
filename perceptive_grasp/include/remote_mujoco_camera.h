/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_camera.h
 * @brief Remote MuJoCo stereo camera client.
 */

#ifndef REMOTE_MUJOCO_CAMERA_H
#define REMOTE_MUJOCO_CAMERA_H

#include <memory>

#include "stereo_camera.h"

namespace perceptive_grasp {

std::unique_ptr<StereoCamera> CreateRemoteMujocoCamera(
    const StereoCameraConfig& config);

}  // namespace perceptive_grasp

#endif  // REMOTE_MUJOCO_CAMERA_H
