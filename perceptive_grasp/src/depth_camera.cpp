/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file depth_camera.cpp
    * @brief RealSense D435i 深度相机实现
    */

#include "depth_camera.h"

#include <cstring>
#include <iostream>

namespace perceptive_grasp {

DepthCamera::DepthCamera(const DepthCameraConfig& config) : config_(config) {}

DepthCamera::~DepthCamera() {
    if (initialized_) {
        try {
            pipeline_.stop();
        } catch (...) {
        }
    }
}

bool DepthCamera::Init() {
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, config_.width, config_.height,
                            RS2_FORMAT_BGR8, config_.fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, config_.width, config_.height,
                            RS2_FORMAT_Z16, config_.fps);

        profile_ = pipeline_.start(cfg);

        // 获取深度比例
        auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
        depth_scale_ = depth_sensor.get_depth_scale();

        // 获取彩色流内参 (用于反投影)
        auto color_stream = profile_.get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
        intrinsics_ = color_stream.get_intrinsics();

        // 丢弃前几帧让自动曝光稳定
        for (int i = 0; i < 30; i++) {
            pipeline_.wait_for_frames();
        }

        initialized_ = true;
        std::cout << "[DepthCamera] Initialized: " << config_.width << "x"
                    << config_.height << "@" << config_.fps << "fps"
                    << ", depth_scale=" << depth_scale_ << std::endl;
        return true;

    } catch (const rs2::error& e) {
        std::cerr << "[DepthCamera] RealSense error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[DepthCamera] Init error: " << e.what() << std::endl;
        return false;
    }
}

bool DepthCamera::GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) {
    if (!initialized_) return false;

    try {
        rs2::frameset frames = pipeline_.wait_for_frames(5000);

        // 对齐深度到彩色
        rs2::frameset aligned;
        if (config_.align_depth) {
            aligned = align_.process(frames);
        } else {
            aligned = frames;
        }

        auto color = aligned.get_color_frame();
        auto depth = aligned.get_depth_frame();

        if (!color || !depth) return false;

        // 深度滤波
        rs2::frame filtered_depth = depth;
        if (config_.spatial_filter) {
            filtered_depth = spatial_filter_.process(filtered_depth);
        }
        if (config_.temporal_filter) {
            filtered_depth = temporal_filter_.process(filtered_depth);
        }
        if (config_.hole_filling) {
            filtered_depth = hole_filter_.process(filtered_depth);
        }

        // 转换为 OpenCV Mat
        color_frame = cv::Mat(cv::Size(config_.width, config_.height), CV_8UC3,
                                const_cast<void*>(color.get_data()));
        color_frame = color_frame.clone();  // 深拷贝，避免帧被回收

        auto filtered_depth_frame = filtered_depth.as<rs2::depth_frame>();
        depth_frame = cv::Mat(cv::Size(config_.width, config_.height), CV_16UC1,
                                const_cast<void*>(filtered_depth_frame.get_data()));
        depth_frame = depth_frame.clone();

        return true;

    } catch (const rs2::error& e) {
        std::cerr << "[DepthCamera] Frame error: " << e.what() << std::endl;
        return false;
    }
}

bool DepthCamera::Deproject(int pixel_x, int pixel_y, uint16_t depth_mm,
                            float point_3d[3]) const {
    if (!initialized_ || depth_mm == 0) return false;

    float depth_m = static_cast<float>(depth_mm) * depth_scale_;

    // 使用 RealSense 内参反投影
    float pixel[2] = {static_cast<float>(pixel_x), static_cast<float>(pixel_y)};
    rs2_deproject_pixel_to_point(point_3d, &intrinsics_, pixel, depth_m);

    return true;
}

}  // namespace perceptive_grasp
