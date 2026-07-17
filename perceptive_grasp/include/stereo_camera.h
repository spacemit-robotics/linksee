/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file stereo_camera.h
* @brief Stereo camera abstraction for the grasp pipeline.
*/

#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include <cstdint>
#include <memory>
#include <string>

namespace cv {
class Mat;
}  // namespace cv

namespace perceptive_grasp {

/** Configuration for the RealSense stereo camera backend. */
struct RealSenseCameraConfig {
    int width = 640;
    int height = 480;
    int fps = 30;
    int motion_flush_frames = 16;
    bool align_depth = true;
    bool spatial_filter = true;
    bool temporal_filter = true;
    bool hole_filling = true;
};

/** Configuration for the spacemit_las2 stereo camera backend. */
struct SpacemitLas2CameraConfig {
    std::string video_device =
        "/dev/v4l/by-id/usb-DECXIN_DECXIN_Camera_01.00.00-video-index0";
    std::string model_path;
    std::string calib_path;
    int core_count = 1;
    std::string core_affinity = "8";
    float min_depth_m = 0.05f;
    float max_depth_m = 2.0f;
};

/** Selectable stereo camera backend configuration. */
struct StereoCameraConfig {
    std::string type = "realsense";
    RealSenseCameraConfig realsense;
    SpacemitLas2CameraConfig spacemit_las2;
};

/**
* @brief Interface for registered color and metric-depth frames.
*
* GetFrames() returns BGR color and CV_16UC1 depth images in the same pixel
* coordinate system. Depth values are expressed in millimeters.
*/
class StereoCamera {
public:
    virtual ~StereoCamera() = default;

    /** Initialize the selected camera backend. */
    virtual bool Init() = 0;

    /** Acquire registered color and depth frames. */
    virtual bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) = 0;

    /** Return the backend frame identifier from the last successful capture. */
    virtual std::int64_t LastFrameId() const = 0;

    /** Convert one depth pixel into camera-frame coordinates in meters. */
    virtual bool Deproject(int pixel_x, int pixel_y, uint16_t depth_mm,
                        float point_3d[3]) const = 0;
};

/** Create the camera backend selected by config.type. */
std::unique_ptr<StereoCamera> CreateStereoCamera(
    const StereoCameraConfig& config);

}  // namespace perceptive_grasp

#endif  // STEREO_CAMERA_H
