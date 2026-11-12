/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file manipulation_mujoco_camera.cpp
 * @brief Convert shared-world RGB-D snapshots to the perception camera contract.
 */
#include "manipulation_mujoco_camera.h"

#include <array>
#include <cmath>
#include <iostream>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "manipulation/mujoco_runtime.h"
#include "manipulation/device_plan.h"
#include "manipulation/mujoco_driver.h"
#include "mujoco_render.h"
#include "mujoco_world.h"
#include "stereo_camera.h"

namespace perceptive_grasp {
namespace {
class ManipulationMujocoCamera final : public StereoCamera {
public:
    ManipulationMujocoCamera(std::shared_ptr<spacemit::manipulation::MujocoRuntime> runtime,
            const StereoCameraConfig& config)
        : runtime_(std::move(runtime)), device_plan_(config.mujoco.device_plan) {
        spec_ = {config.mujoco.camera_name, config.mujoco.width, config.mujoco.height,
                config.mujoco.min_depth_m, config.mujoco.max_depth_m};
    }

    bool Init() override {
        try {
            if (!runtime_) {
                if (device_plan_.empty()) throw std::runtime_error("simulation camera requires device_plan");
                const auto registry = spacemit::manipulation::MakeBuiltinDeviceRegistry();
                const auto plan = spacemit::manipulation::LoadDevicePlan(device_plan_, registry);
                if (!spacemit::manipulation::IsMujocoArmDriver(plan.composition.arm_driver) ||
                    !spacemit::manipulation::IsMujocoEffectorDriver(plan.composition.effector_driver) ||
                    plan.composition.arm_options != plan.composition.effector_options) {
                    throw std::runtime_error("simulation camera requires one shared MuJoCo device plan");
                }
                runtime_ = std::make_shared<spacemit::manipulation::MujocoRuntime>(
                    plan.definition, plan.composition.arm_options);
                runtime_->Start();
            }
            cv::Mat color;
            cv::Mat depth;
            return GetFrames(color, depth);
        } catch (const std::exception& error) {
            std::cerr << "[SimulationCamera] " << error.what() << std::endl;
            return false;
        }
    }

    bool GetFrames(cv::Mat& color, cv::Mat& depth) override {
        try {
            auto frame = renderer_.Capture(runtime_->Snapshot(), spec_);
            const cv::Mat rgb(frame.height, frame.width, CV_8UC3, frame.rgb.data());
            cv::cvtColor(rgb, color, cv::COLOR_RGB2BGR);
            depth.create(frame.height, frame.width, CV_16UC1);
            for (int y = 0; y < frame.height; ++y) {
                auto* row = depth.ptr<std::uint16_t>(y);
                for (int x = 0; x < frame.width; ++x) {
                    const double millimeters = frame.depth_m[y * frame.width + x] * 1000.0;
                    row[x] = std::isfinite(millimeters) && millimeters > 0 && millimeters <= 65535
                            ? static_cast<std::uint16_t>(std::lround(millimeters))
                            : 0;
                }
            }
            intrinsics_ = frame.intrinsics;
            frame_id_ = static_cast<std::int64_t>(frame.sample_id);
            return true;
        } catch (const std::exception& error) {
            std::cerr << "[SimulationCamera] " << error.what() << std::endl;
            return false;
        }
    }

    std::int64_t LastFrameId() const override { return frame_id_; }

    bool GetIntrinsics(float* fx, float* fy, float* cx, float* cy) const override {
        if (intrinsics_[0] <= 0 || intrinsics_[4] <= 0) return false;
        if (fx) *fx = static_cast<float>(intrinsics_[0]);
        if (fy) *fy = static_cast<float>(intrinsics_[4]);
        if (cx) *cx = static_cast<float>(intrinsics_[2]);
        if (cy) *cy = static_cast<float>(intrinsics_[5]);
        return true;
    }

    bool Deproject(int x, int y, std::uint16_t depth_mm, float point[3]) const override {
        if (!point || !depth_mm || x < 0 || y < 0 || x >= spec_.width || y >= spec_.height ||
                intrinsics_[0] <= 0 || intrinsics_[4] <= 0)
            return false;
        point[2] = static_cast<float>(depth_mm) * 0.001f;
        point[0] = static_cast<float>((x - intrinsics_[2]) * point[2] / intrinsics_[0]);
        point[1] = static_cast<float>((y - intrinsics_[5]) * point[2] / intrinsics_[4]);
        return true;
    }

private:
    std::shared_ptr<spacemit::manipulation::MujocoRuntime> runtime_;
    std::string device_plan_;
    mujoco_sim::Renderer renderer_;
    mujoco_sim::CameraSpec spec_;
    std::array<double, 9> intrinsics_{};
    std::int64_t frame_id_ = -1;
};
}  // namespace

std::unique_ptr<StereoCamera> CreateManipulationMujocoCamera(
        std::shared_ptr<spacemit::manipulation::MujocoRuntime> runtime,
        const StereoCameraConfig& config) {
    return std::make_unique<ManipulationMujocoCamera>(std::move(runtime), config);
}
}  // namespace perceptive_grasp
