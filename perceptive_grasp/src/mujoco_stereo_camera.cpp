/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_stereo_camera.cpp
 * @brief MuJoCo rendered color and metric-depth camera backend.
 */

#include "mujoco_stereo_camera.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "mujoco_simulation.h"

namespace perceptive_grasp {
namespace {

class MujocoStereoCamera final : public StereoCamera {
public:
    explicit MujocoStereoCamera(const StereoCameraConfig& config)
        : config_(config) {}

    ~MujocoStereoCamera() override {
        if (context_ready_) {
            mjr_freeContext(&context_);
            mjv_freeScene(&scene_);
        }
        if (window_ != nullptr) {
            glfwDestroyWindow(window_);
        }
        if (glfw_initialized_) {
            glfwTerminate();
        }
    }

    bool Init() override {
        const auto& settings = config_.mujoco;
        std::string error;
        simulation_ = MujocoSimulation::Get(settings.xml_path, &error);
        if (!simulation_) {
            std::cerr << "[MujocoCamera] init failed: " << error << std::endl;
            return false;
        }

        camera_id_ = mj_name2id(
            simulation_->model(), mjOBJ_CAMERA, settings.camera_name.c_str());
        if (camera_id_ < 0) {
            std::cerr << "[MujocoCamera] camera not found: "
                    << settings.camera_name << std::endl;
            return false;
        }

        if (!glfwInit()) {
            std::cerr << "[MujocoCamera] glfwInit failed" << std::endl;
            return false;
        }
        glfw_initialized_ = true;
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(
            settings.width, settings.height, "perceptive_grasp_mujoco",
            nullptr, nullptr);
        if (window_ == nullptr) {
            std::cerr << "[MujocoCamera] hidden GLFW window creation failed"
                    << std::endl;
            return false;
        }
        glfwMakeContextCurrent(window_);

        mjv_defaultCamera(&camera_);
        camera_.type = mjCAMERA_FIXED;
        camera_.fixedcamid = camera_id_;
        mjv_defaultOption(&option_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);
        mjv_makeScene(simulation_->model(), &scene_, 2000);
        mjr_makeContext(simulation_->model(), &context_, mjFONTSCALE_100);
        mjr_setBuffer(mjFB_OFFSCREEN, &context_);
        mjr_resizeOffscreen(settings.width, settings.height, &context_);
        context_ready_ = true;

        ComputeIntrinsics();
        std::cout << "[MujocoCamera] Initialized: xml="
                << simulation_->xml_path()
                << " camera=" << settings.camera_name
                << " size=" << settings.width << "x" << settings.height
                << std::endl;
        return true;
    }

    bool GetFrames(cv::Mat& color_frame, cv::Mat& depth_frame) override {
        if (!simulation_ || !context_ready_) {
            return false;
        }

        const auto& settings = config_.mujoco;
        std::vector<unsigned char> rgb(
            static_cast<size_t>(settings.width) * settings.height * 3);
        std::vector<float> depth(
            static_cast<size_t>(settings.width) * settings.height);

        {
            std::lock_guard<std::mutex> lock(simulation_->mutex());
            glfwMakeContextCurrent(window_);
            mjrRect viewport = {0, 0, settings.width, settings.height};
            mjv_updateScene(
                simulation_->model(), simulation_->data(), &option_, nullptr,
                &camera_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);
            mjr_readPixels(rgb.data(), depth.data(), viewport, &context_);
            mjr_finish();
        }

        cv::Mat rgb_bottom_up(settings.height, settings.width, CV_8UC3,
                            rgb.data());
        cv::Mat rgb_top_down;
        cv::flip(rgb_bottom_up, rgb_top_down, 0);
        cv::cvtColor(rgb_top_down, color_frame, cv::COLOR_RGB2BGR);

        depth_frame = cv::Mat(settings.height, settings.width, CV_16UC1);
        const float extent =
            static_cast<float>(simulation_->model()->stat.extent);
        const float znear = simulation_->model()->vis.map.znear * extent;
        const float zfar = simulation_->model()->vis.map.zfar * extent;
        for (int y = 0; y < settings.height; ++y) {
            uint16_t* row = depth_frame.ptr<uint16_t>(y);
            const int source_y = settings.height - 1 - y;
            for (int x = 0; x < settings.width; ++x) {
                const float buffer_z =
                    depth[static_cast<size_t>(source_y) * settings.width + x];
                const float depth_m = DepthBufferToMeters(
                    buffer_z, znear, zfar);
                if (!std::isfinite(depth_m) ||
                    depth_m < settings.min_depth_m ||
                    depth_m > settings.max_depth_m) {
                    row[x] = 0;
                } else {
                    row[x] = static_cast<uint16_t>(
                        std::lround(depth_m * 1000.0f));
                }
            }
        }
        ++last_frame_id_;
        return true;
    }

    std::int64_t LastFrameId() const override { return last_frame_id_; }

    bool Deproject(int pixel_x, int pixel_y, uint16_t depth_mm,
            float point_3d[3]) const override {
        if (depth_mm == 0 || !point_3d ||
            pixel_x < 0 || pixel_x >= config_.mujoco.width ||
            pixel_y < 0 || pixel_y >= config_.mujoco.height) {
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
        if (fx) *fx = fx_;
        if (fy) *fy = fy_;
        if (cx) *cx = cx_;
        if (cy) *cy = cy_;
        return true;
    }

private:
    static float DepthBufferToMeters(float depth, float znear, float zfar) {
        depth = std::clamp(depth, 0.0f, 1.0f);
        return znear * zfar / (zfar - depth * (zfar - znear));
    }

    void ComputeIntrinsics() {
        const auto& settings = config_.mujoco;
        const float fovy_deg =
            simulation_->model()->cam_fovy[camera_id_] > 0.0
                ? static_cast<float>(simulation_->model()->cam_fovy[camera_id_])
                : 45.0f;
        const float fovy_rad = fovy_deg * static_cast<float>(M_PI) / 180.0f;
        fy_ = 0.5f * static_cast<float>(settings.height) /
            std::tan(0.5f * fovy_rad);
        fx_ = fy_;
        cx_ = 0.5f * (static_cast<float>(settings.width) - 1.0f);
        cy_ = 0.5f * (static_cast<float>(settings.height) - 1.0f);
    }

    StereoCameraConfig config_;
    std::shared_ptr<MujocoSimulation> simulation_;
    GLFWwindow* window_ = nullptr;
    bool glfw_initialized_ = false;
    bool context_ready_ = false;
    int camera_id_ = -1;
    mjvCamera camera_;
    mjvOption option_;
    mjvScene scene_;
    mjrContext context_;
    float fx_ = 1.0f;
    float fy_ = 1.0f;
    float cx_ = 0.0f;
    float cy_ = 0.0f;
    std::int64_t last_frame_id_ = 0;
};

}  // namespace

std::unique_ptr<StereoCamera> CreateMujocoStereoCamera(
    const StereoCameraConfig& config) {
    return std::make_unique<MujocoStereoCamera>(config);
}

}  // namespace perceptive_grasp
