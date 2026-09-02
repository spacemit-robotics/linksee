/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_camera.cpp
 * @brief Remote MuJoCo stereo camera client.
 */

#include "remote_mujoco_camera.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "remote_mujoco_protocol.h"

namespace perceptive_grasp {
namespace {

class RemoteMujocoCamera : public StereoCamera {
public:
    explicit RemoteMujocoCamera(const StereoCameraConfig& config)
        : config_(config) {}

    bool Init() override {
        cv::Mat color;
        cv::Mat depth;
        if (!GetFrames(color, depth)) {
            std::cerr << "[RemoteMujocoCamera] failed to get initial frame"
                    << std::endl;
            return false;
        }
        std::cout << "[RemoteMujocoCamera] Initialized: "
                << config_.remote_mujoco.host << ":"
                << config_.remote_mujoco.port
                << " size=" << color.cols << "x" << color.rows
                << std::endl;
        return true;
    }

    bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) override {
        remote_mujoco::Response response;
        std::string error;
        if (!remote_mujoco::SendRequest(
                config_.remote_mujoco.host,
                config_.remote_mujoco.port,
                config_.remote_mujoco.timeout_ms,
                remote_mujoco::Command::GET_FRAME,
                {}, &response, &error)) {
            std::cerr << "[RemoteMujocoCamera] get frame failed: "
                    << error << std::endl;
            return false;
        }
        if (!response.ok) {
            std::cerr << "[RemoteMujocoCamera] get frame rejected: "
                    << response.detail << std::endl;
            return false;
        }

        remote_mujoco::FramePacket frame;
        if (!remote_mujoco::DecodeFramePacket(
                response.payload, &frame, &error)) {
            std::cerr << "[RemoteMujocoCamera] decode frame failed: "
                    << error << std::endl;
            return false;
        }
        const size_t expected_color =
            static_cast<size_t>(frame.width) *
            static_cast<size_t>(frame.height) * 3U;
        const size_t expected_depth =
            static_cast<size_t>(frame.width) *
            static_cast<size_t>(frame.height) * sizeof(std::uint16_t);
        if (frame.width <= 0 || frame.height <= 0 ||
            frame.color_bgr.size() != expected_color ||
            frame.depth_u16.size() != expected_depth) {
            std::cerr << "[RemoteMujocoCamera] invalid frame dimensions"
                    << std::endl;
            return false;
        }

        color_frame.create(frame.height, frame.width, CV_8UC3);
        std::memcpy(color_frame.data, frame.color_bgr.data(),
                    frame.color_bgr.size());
        depth_frame.create(frame.height, frame.width, CV_16UC1);
        std::memcpy(depth_frame.data, frame.depth_u16.data(),
                    frame.depth_u16.size());
        last_frame_id_ = frame.frame_id;
        fx_ = frame.fx;
        fy_ = frame.fy;
        cx_ = frame.cx;
        cy_ = frame.cy;
        return true;
    }

    std::int64_t LastFrameId() const override { return last_frame_id_; }

    bool Deproject(int pixel_x, int pixel_y, uint16_t depth_mm,
                float point_3d[3]) const override {
        if (depth_mm == 0 || !std::isfinite(fx_) || !std::isfinite(fy_) ||
            fx_ <= 0.0f || fy_ <= 0.0f) {
            return false;
        }
        const float z = static_cast<float>(depth_mm) * 0.001f;
        point_3d[0] = (static_cast<float>(pixel_x) - cx_) * z / fx_;
        point_3d[1] = (static_cast<float>(pixel_y) - cy_) * z / fy_;
        point_3d[2] = z;
        return true;
    }

    bool GetIntrinsics(float* fx, float* fy,
                    float* cx, float* cy) const override {
        if (!std::isfinite(fx_) || !std::isfinite(fy_) ||
            fx_ <= 0.0f || fy_ <= 0.0f) {
            return false;
        }
        if (fx) *fx = fx_;
        if (fy) *fy = fy_;
        if (cx) *cx = cx_;
        if (cy) *cy = cy_;
        return true;
    }

private:
    StereoCameraConfig config_;
    std::int64_t last_frame_id_ = 0;
    float fx_ = NAN;
    float fy_ = NAN;
    float cx_ = NAN;
    float cy_ = NAN;
};

}  // namespace

std::unique_ptr<StereoCamera> CreateRemoteMujocoCamera(
    const StereoCameraConfig& config) {
    return std::make_unique<RemoteMujocoCamera>(config);
}

}  // namespace perceptive_grasp
