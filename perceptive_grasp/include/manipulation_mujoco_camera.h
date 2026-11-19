/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file manipulation_mujoco_camera.h
 * @brief RGB-D camera backed by the control driver's shared simulation world.
 */
#ifndef MANIPULATION_MUJOCO_CAMERA_H
#define MANIPULATION_MUJOCO_CAMERA_H

#include <memory>

namespace spacemit::manipulation {
class MujocoRuntime;
}

namespace perceptive_grasp {
class StereoCamera;
struct StereoCameraConfig;

/** Construct a main-thread camera; frames sample the supplied live simulation. */
std::unique_ptr<StereoCamera> CreateManipulationMujocoCamera(
        std::shared_ptr<spacemit::manipulation::MujocoRuntime> runtime,
        const StereoCameraConfig& config);
}  // namespace perceptive_grasp
#endif
