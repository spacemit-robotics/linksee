/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file stereo_camera.cpp
 * @brief Stereo camera backend factory.
 */

#include "stereo_camera.h"

#include <iostream>

#ifdef HAVE_REALSENSE_CAMERA
#include "depth_camera.h"
#endif

#ifdef HAVE_LAS2_CAMERA
#include "las2_stereo_camera.h"
#endif

namespace perceptive_grasp {
namespace {

class UnsupportedRealSenseCamera final : public StereoCamera {
public:
    bool Init() override {
        std::cerr << "[StereoCamera] camera.type=realsense requested, but "
                  << "the realsense backend is not available in this build. "
                  << "Install the realsense dependency and rebuild."
                  << std::endl;
        return false;
    }

    bool GetFrames(cv::Mat&, cv::Mat&) override { return false; }

    std::int64_t LastFrameId() const override { return -1; }

    bool Deproject(int, int, uint16_t, float[3]) const override {
        return false;
    }
};

class UnsupportedSpacemitLas2Camera final : public StereoCamera {
public:
    bool Init() override {
        std::cerr << "[StereoCamera] camera.type=spacemit_las2 requested, but "
                  << "the spacemit_las2 backend is not available in this "
                  << "build. Install it under ~/las2_runtime or set "
                  << "LAS2_RUNTIME_DIR, then rebuild." << std::endl;
        return false;
    }

    bool GetFrames(cv::Mat&, cv::Mat&) override { return false; }

    std::int64_t LastFrameId() const override { return -1; }

    bool Deproject(int, int, uint16_t, float[3]) const override {
        return false;
    }
};

}  // namespace

std::unique_ptr<StereoCamera> CreateStereoCamera(
    const StereoCameraConfig& config) {
    if (config.type.empty() || config.type == "realsense" ||
        config.type == "d435i") {
#ifdef HAVE_REALSENSE_CAMERA
        return std::make_unique<DepthCamera>(config);
#else
        return std::make_unique<UnsupportedRealSenseCamera>();
#endif
    }

    if (config.type == "spacemit_las2") {
#ifdef HAVE_LAS2_CAMERA
        return CreateLas2StereoCamera(config);
#else
        return std::make_unique<UnsupportedSpacemitLas2Camera>();
#endif
    }

    std::cerr << "[StereoCamera] Unknown camera.type: " << config.type
              << std::endl;
    return nullptr;
}

}  // namespace perceptive_grasp
