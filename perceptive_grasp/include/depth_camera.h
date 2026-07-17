/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file depth_camera.h
    * @brief RealSense D435i 深度相机封装
    */

#ifndef PERCEPTIVE_GRASP_DEPTH_CAMERA_H_
#define PERCEPTIVE_GRASP_DEPTH_CAMERA_H_

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>

#include "stereo_camera.h"

namespace perceptive_grasp {

/**
    * @brief RealSense D435i 深度相机封装
    *
    * 提供对齐的 RGB + Depth 帧获取，以及像素到 3D 点的反投影。
    */
class DepthCamera : public StereoCamera {
public:
    explicit DepthCamera(const StereoCameraConfig& config);
    ~DepthCamera();

    // Non-copyable
    DepthCamera(const DepthCamera&) = delete;
    DepthCamera& operator=(const DepthCamera&) = delete;

    /**
    * @brief 初始化相机 pipeline
    * @return true 成功
    */
    bool Init() override;

    /**
    * @brief 获取一帧对齐的 RGB + Depth
    * @param[out] color_frame BGR 彩色图 (CV_8UC3)
    * @param[out] depth_frame 深度图 (CV_16UC1, 单位: mm)
    * @return true 成功获取
    */
    bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) override;

    std::int64_t LastFrameId() const override { return last_frame_id_; }

    /**
    * @brief 像素坐标 + 深度 → 相机坐标系 3D 点
    * @param pixel_x 像素 x
    * @param pixel_y 像素 y
    * @param depth_mm 深度值 (mm)
    * @param[out] point_3d 输出 3D 点 [x, y, z] (米)
    * @return true 成功
    */
    bool Deproject(int pixel_x, int pixel_y, uint16_t depth_mm,
                    float point_3d[3]) const override;

    /**
    * @brief 获取相机内参 (用于外部计算)
    */
    rs2_intrinsics GetIntrinsics() const { return intrinsics_; }

    /**
    * @brief 获取深度比例因子 (depth_value * scale = 米)
    */
    float GetDepthScale() const { return depth_scale_; }

private:
    StereoCameraConfig config_;
    rs2::pipeline pipeline_;
    rs2::pipeline_profile profile_;
    rs2::align align_{RS2_STREAM_COLOR};

    // 滤波器
    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;
    rs2::hole_filling_filter hole_filter_;

    rs2_intrinsics intrinsics_{};
    float depth_scale_ = 0.001f;  // D435i 默认 1mm
    std::int64_t last_frame_id_ = -1;
    bool initialized_ = false;
};

}  // namespace perceptive_grasp

#endif  // PERCEPTIVE_GRASP_DEPTH_CAMERA_H_
