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

enum class TrackingMode { OFF, TRACK_ALL, TRACK_SINGLE };

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
    int track_count = 0;
    uint32_t rtsp_clients = 0;
    uint64_t total_frames = 0;
    TrackingMode mode = TrackingMode::OFF;
    int selected_track_id = -1;
    bool track_lost = false;
};

struct SelectionBox {
    float x1, y1, x2, y2;  // normalized [0,1]
};

class Pipeline {
public:
    explicit Pipeline(const PipelineConfig& config);
    ~Pipeline();

    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    bool Start();
    void Stop();

    // Tracking control
    void StartTracking();       // track all
    void StopTracking();        // stop
    void SelectTarget(const SelectionBox& box);  // select single target

    TrackingMode GetTrackingMode() const;
    PipelineStats GetStats() const;

    // MJPEG frame access (for HTTP streaming)
    bool GetMjpegFrame(std::vector<uint8_t>& out_jpeg);

private:
    void CaptureLoop();
    void InferLoop();

    // IoU calculation for target matching
    static float ComputeIoU(float ax1, float ay1, float ax2, float ay2,
                            float bx1, float by1, float bx2, float by2);

    PipelineConfig config_;

    // VisionService
    std::unique_ptr<VisionService> vision_;

    // Threads
    std::thread capture_thread_;
    std::thread infer_thread_;
    std::atomic<bool> running_{false};

    // Tracking mode
    std::atomic<int> tracking_mode_{static_cast<int>(TrackingMode::OFF)};
    std::atomic<int> selected_track_id_{-1};
    std::atomic<bool> track_lost_{false};
    std::atomic<int> lost_counter_{0};
    static constexpr int kLostThreshold = 60;  // frames before declaring lost

    // Selection box (set by user, consumed by infer thread)
    std::mutex select_mutex_;
    SelectionBox pending_select_{0, 0, 0, 0};
    bool has_pending_select_ = false;

    // Inference result cache
    mutable std::mutex result_mutex_;
    std::vector<VisionServiceResult> cached_results_;

    // Latest frame for inference
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat infer_frame_;
    bool frame_ready_ = false;

    // MJPEG output buffer
    mutable std::mutex mjpeg_mutex_;
    std::vector<uint8_t> mjpeg_buf_;

    // Stats
    std::atomic<float> capture_fps_{0.0f};
    std::atomic<float> infer_fps_{0.0f};
    std::atomic<int> track_count_{0};
    std::atomic<uint64_t> total_frames_{0};
};

#endif  // PIPELINE_H
