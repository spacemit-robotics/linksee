/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file las2_stereo_camera.cpp
 * @brief spacemit_las2 stereo camera backend.
 */

#include "las2_stereo_camera.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "las2_usb_stereo.h"
#include "stereo_geometry.h"

namespace perceptive_grasp {
namespace {

bool IsSupportedCoreCount(int count) {
    return count >= 1 && count <= 16;
}

class Las2StereoCamera final : public StereoCamera {
public:
    explicit Las2StereoCamera(StereoCameraConfig config)
        : config_(std::move(config)), camera_(new las2::Camera()) {}

    ~Las2StereoCamera() override {
        if (!camera_) {
            return;
        }
        std::cout << "[Las2StereoCamera] Shutting down..." << std::endl;
        camera_->shutdown();
        std::cout << "[Las2StereoCamera] Stopped" << std::endl;

        // The LAS2 runtime calls shutdown again from Camera's
        // destructor. That second shutdown blocks in its OpenCL event thread,
        // so do not invoke the non-idempotent vendor destructor after the
        // explicit shutdown has released the device and worker resources.
        camera_ = nullptr;
    }

    bool Init() override {
        const auto& settings = config_.spacemit_las2;
        if (settings.video_device.empty() || settings.model_path.empty() ||
            settings.calib_path.empty()) {
            std::cerr << "[Las2StereoCamera] video_device, model_path and "
                         "calib_path are required"
                      << std::endl;
            return false;
        }
        if (!IsSupportedCoreCount(settings.core_count)) {
            std::cerr << "[Las2StereoCamera] core_count must be in [1, 16]"
                      << std::endl;
            return false;
        }
        if (settings.core_affinity.empty()) {
            std::cerr << "[Las2StereoCamera] core_affinity is required"
                      << std::endl;
            return false;
        }
        if (!(settings.min_depth_m >= 0.0f) ||
            !(settings.max_depth_m > settings.min_depth_m)) {
            std::cerr << "[Las2StereoCamera] invalid depth range" << std::endl;
            return false;
        }

        std::string error;
        if (!LoadRectifiedLeftIntrinsics(settings.calib_path,
                                         rectified_intrinsics_, error)) {
            std::cerr << "[Las2StereoCamera] " << error << std::endl;
            return false;
        }

        if (!camera_->initialize(settings.video_device.c_str(),
                                 settings.calib_path.c_str(),
                                 settings.core_count,
                                 settings.core_affinity.c_str(),
                                 settings.model_path.c_str())) {
            std::cerr << "[Las2StereoCamera] initialize failed: "
                      << camera_->last_error() << std::endl;
            return false;
        }

        initialized_ = true;
        std::cout << "[Las2StereoCamera] Initialized: dev="
                  << settings.video_device
                  << " cores=" << settings.core_count
                  << " affinity=" << settings.core_affinity
                  << " calibration=" << rectified_intrinsics_.width << "x"
                  << rectified_intrinsics_.height << std::endl;
        return true;
    }

    bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) override {
        if (!initialized_) {
            return false;
        }
        const auto& settings = config_.spacemit_las2;

        las2::Frame frame;
        if (!camera_->get_frame(frame, 5000)) {
            std::cerr << "[Las2StereoCamera] get_frame failed: "
                      << camera_->last_error() << std::endl;
            return false;
        }
        if (!ValidateFrame(frame)) {
            return false;
        }
        last_frame_id_ = frame.frame_id;

        if (frame.rgb.width != output_intrinsics_.width ||
            frame.rgb.height != output_intrinsics_.height) {
            std::string error;
            if (!ScaleIntrinsicsWithLetterbox(
                    rectified_intrinsics_, frame.rgb.width, frame.rgb.height,
                    output_intrinsics_, error)) {
                std::cerr << "[Las2StereoCamera] " << error << std::endl;
                return false;
            }
            std::cout << "[Las2StereoCamera] Output: " << frame.rgb.width
                      << "x" << frame.rgb.height << " fx="
                      << output_intrinsics_.fx << " fy="
                      << output_intrinsics_.fy << " cx="
                      << output_intrinsics_.cx << " cy="
                      << output_intrinsics_.cy << std::endl;
        }

        const cv::Mat rgb(frame.rgb.height, frame.rgb.width, CV_8UC3,
                          const_cast<std::uint8_t*>(frame.rgb.data),
                          static_cast<size_t>(frame.rgb.stride));
        cv::cvtColor(rgb, color_frame, cv::COLOR_RGB2BGR);

        depth_frame.create(frame.depth.height, frame.depth.width, CV_16UC1);
        for (int y = 0; y < frame.depth.height; ++y) {
            const auto* source = reinterpret_cast<const float*>(
                reinterpret_cast<const std::uint8_t*>(frame.depth.data) +
                static_cast<size_t>(y) * frame.depth.stride);
            auto* destination = depth_frame.ptr<std::uint16_t>(y);
            for (int x = 0; x < frame.depth.width; ++x) {
                const float depth_m = source[x];
                if (!std::isfinite(depth_m) ||
                    depth_m < settings.min_depth_m ||
                    depth_m > settings.max_depth_m || depth_m > 65.535f) {
                    destination[x] = 0;
                    continue;
                }
                destination[x] = static_cast<std::uint16_t>(
                    std::clamp(std::lround(depth_m * 1000.0f), 1L, 65535L));
            }
        }
        return true;
    }

    std::int64_t LastFrameId() const override { return last_frame_id_; }

    bool Deproject(int pixel_x, int pixel_y, std::uint16_t depth_mm,
                   float point_3d[3]) const override {
        if (!initialized_ || depth_mm == 0 ||
            output_intrinsics_.width <= 0 ||
            pixel_x < 0 || pixel_x >= output_intrinsics_.width ||
            pixel_y < 0 || pixel_y >= output_intrinsics_.height) {
            return false;
        }

        const float z = static_cast<float>(depth_mm) * 0.001f;
        point_3d[0] = static_cast<float>(
            (pixel_x - output_intrinsics_.cx) * z / output_intrinsics_.fx);
        point_3d[1] = static_cast<float>(
            (pixel_y - output_intrinsics_.cy) * z / output_intrinsics_.fy);
        point_3d[2] = z;
        return true;
    }

private:
    bool ValidateFrame(const las2::Frame& frame) const {
        if (!frame.rgb.data || !frame.depth.data || frame.rgb.width <= 0 ||
            frame.rgb.height <= 0 || frame.rgb.width != frame.depth.width ||
            frame.rgb.height != frame.depth.height ||
            frame.rgb.stride < frame.rgb.width * 3 ||
            frame.depth.stride <
                frame.depth.width * static_cast<int>(sizeof(float))) {
            std::cerr << "[Las2StereoCamera] unregistered color/depth frame"
                      << std::endl;
            return false;
        }
        return true;
    }

    StereoCameraConfig config_;
    las2::Camera* camera_ = nullptr;
    PinholeIntrinsics rectified_intrinsics_;
    PinholeIntrinsics output_intrinsics_;
    std::int64_t last_frame_id_ = -1;
    bool initialized_ = false;
};

}  // namespace

std::unique_ptr<StereoCamera> CreateLas2StereoCamera(
    const StereoCameraConfig& config) {
    return std::make_unique<Las2StereoCamera>(config);
}

}  // namespace perceptive_grasp
