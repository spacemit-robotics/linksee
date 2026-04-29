/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PIPELINE_H
#define PIPELINE_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "vision_service.h"

struct PipelineConfig {
    std::string device_node = "/dev/video0";
    uint32_t width = 1280;
    uint32_t height = 720;
    uint32_t fps = 30;
    std::string rtsp_url = "rtsp://0.0.0.0:8554/live";
    std::string config_path;   // VisionService YAML config
    std::string model_path;    // optional model override
};

struct PipelineStats {
    float capture_fps = 0.0f;
    float infer_fps = 0.0f;
    int detect_count = 0;
    uint32_t rtsp_clients = 0;
    uint64_t total_frames = 0;
};

class Pipeline {
public:
    explicit Pipeline(const PipelineConfig& config);
    ~Pipeline();

    // Non-copyable
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    bool Start();
    void Stop();

    void EnableInference();
    void DisableInference();
    bool IsInferenceEnabled() const { return ai_enabled_.load(); }

    PipelineStats GetStats() const;

private:
    void CaptureLoop();
    void InferLoop();

    PipelineConfig config_;

    // VisionService
    std::unique_ptr<VisionService> vision_;

    // Threads
    std::thread capture_thread_;
    std::thread infer_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> ai_enabled_{false};

    // Inference result cache (infer thread writes, capture thread reads)
    mutable std::mutex result_mutex_;
    std::vector<VisionServiceResult> cached_results_;

    // Latest frame for inference (capture thread writes, infer thread reads)
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat infer_frame_;        // BGR frame for inference
    bool frame_ready_ = false;

    // Stats
    std::atomic<float> capture_fps_{0.0f};
    std::atomic<float> infer_fps_{0.0f};
    std::atomic<int> detect_count_{0};
    std::atomic<uint64_t> total_frames_{0};
};

#endif  // PIPELINE_H
