/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "pipeline.h"
#include "nv12_draw.h"

#include <opencv2/imgproc.hpp>

extern "C" {
#include "sys_api.h"
#include "vb_api.h"
#include "uvc_api.h"
#include "vdec/vdec_api.h"
#include "venc/venc_api.h"
#include "mux/mux_api.h"
}

// Channel IDs used throughout
static constexpr int kUvcDev  = 0;
static constexpr int kUvcChn  = 0;
static constexpr int kVdecChn = 0;
static constexpr int kVencChn = 0;
static constexpr int kMuxChn  = 0;

// Color palette for detection boxes (up to 8 classes cycle)
static const nv12_draw::YuvColor kBoxColors[] = {
    nv12_draw::kRed,    nv12_draw::kGreen,  nv12_draw::kBlue,
    nv12_draw::kYellow, nv12_draw::kCyan,   nv12_draw::kWhite,
    {180, 80, 200},     {100, 200, 80},
};
static constexpr int kNumColors = sizeof(kBoxColors) / sizeof(kBoxColors[0]);

Pipeline::Pipeline(const PipelineConfig& config) : config_(config) {}

Pipeline::~Pipeline() {
    Stop();
}

bool Pipeline::Start() {
    // Create VisionService (lazy load - only init when AI enabled)
    if (!config_.config_path.empty()) {
        vision_ = VisionService::Create(config_.config_path, config_.model_path, true);
        if (!vision_) {
            fprintf(stderr, "[Pipeline] VisionService create failed: %s\n",
                    VisionService::LastCreateError().c_str());
            return false;
        }
    }

    // Init MPP modules
    if (SYS_Init() != 0) {
        fprintf(stderr, "[Pipeline] SYS_Init failed\n");
        return false;
    }
    if (VB_Init() != 0) {
        fprintf(stderr, "[Pipeline] VB_Init failed\n");
        return false;
    }
    if (UVC_Init() != 0) {
        fprintf(stderr, "[Pipeline] UVC_Init failed\n");
        return false;
    }
    if (VDEC_Init() != 0) {
        fprintf(stderr, "[Pipeline] VDEC_Init failed\n");
        return false;
    }
    if (VENC_Init() != 0) {
        fprintf(stderr, "[Pipeline] VENC_Init failed\n");
        return false;
    }
    if (MUX_Init() != 0) {
        fprintf(stderr, "[Pipeline] MUX_Init failed\n");
        return false;
    }

    // UVC device
    UvcDevAttr devAttr;
    memset(&devAttr, 0, sizeof(devAttr));
    strncpy(devAttr.acDevNode, config_.device_node.c_str(), sizeof(devAttr.acDevNode) - 1);

    if (UVC_CreateDev(kUvcDev, &devAttr) != 0) {
        fprintf(stderr, "[Pipeline] UVC_CreateDev failed\n");
        return false;
    }
    if (UVC_EnableDev(kUvcDev) != 0) {
        fprintf(stderr, "[Pipeline] UVC_EnableDev failed\n");
        return false;
    }

    // UVC channel (MJPEG capture)
    UvcChnAttr uvcChnAttr;
    memset(&uvcChnAttr, 0, sizeof(uvcChnAttr));
    uvcChnAttr.u32Width = config_.width;
    uvcChnAttr.u32Height = config_.height;
    uvcChnAttr.ePixelFormat = MPP_PIXEL_FORMAT_MJPEG;
    uvcChnAttr.u32Fps = config_.fps;
    uvcChnAttr.u32Depth = 1;

    if (UVC_SetChnAttr(kUvcDev, kUvcChn, &uvcChnAttr) != 0) {
        fprintf(stderr, "[Pipeline] UVC_SetChnAttr failed\n");
        return false;
    }
    if (UVC_EnableChn(kUvcDev, kUvcChn) != 0) {
        fprintf(stderr, "[Pipeline] UVC_EnableChn failed\n");
        return false;
    }

    // VDEC channel (MJPEG → NV12)
    VdecChnAttr vdecAttr;
    memset(&vdecAttr, 0, sizeof(vdecAttr));
    vdecAttr.eCodecType = MPP_STREAM_CODEC_MJPEG;
    vdecAttr.eOutputPixelFormat = MPP_PIXEL_FORMAT_NV12;
    vdecAttr.u32Width = config_.width;
    vdecAttr.u32Height = config_.height;

    if (VDEC_CreateChn(kVdecChn, &vdecAttr) != 0) {
        fprintf(stderr, "[Pipeline] VDEC_CreateChn failed\n");
        return false;
    }
    if (VDEC_EnableChn(kVdecChn) != 0) {
        fprintf(stderr, "[Pipeline] VDEC_EnableChn failed\n");
        return false;
    }

    // VENC channel (NV12 → H.264)
    VencChnAttr vencAttr;
    memset(&vencAttr, 0, sizeof(vencAttr));
    vencAttr.eCodecType = MPP_STREAM_CODEC_H264;
    vencAttr.eInputPixelFormat = MPP_PIXEL_FORMAT_NV12;
    vencAttr.u32Width = config_.width;
    vencAttr.u32Height = config_.height;
    vencAttr.u32Bitrate = 4000000;
    vencAttr.u32FrameRate = config_.fps;
    vencAttr.u32Gop = config_.fps;  // 1 second GOP
    vencAttr.eFrameBufMode = VENC_FRAME_BUF_DMABUF_EXTERNAL;
    vencAttr.eRcMode = VENC_RC_MODE_CBR;

    if (VENC_CreateChn(kVencChn, &vencAttr) != 0) {
        fprintf(stderr, "[Pipeline] VENC_CreateChn failed\n");
        return false;
    }
    if (VENC_EnableChn(kVencChn) != 0) {
        fprintf(stderr, "[Pipeline] VENC_EnableChn failed\n");
        return false;
    }

    // MUX channel (RTSP output)
    MuxChnAttr muxAttr;
    memset(&muxAttr, 0, sizeof(muxAttr));
    muxAttr.eOutputType = MUX_OUTPUT_RTSP;
    muxAttr.stStreamAttr.eCodecType = MUX_CODEC_H264;
    muxAttr.stStreamAttr.u32Width = config_.width;
    muxAttr.stStreamAttr.u32Height = config_.height;
    muxAttr.stStreamAttr.u32Fps = config_.fps;
    muxAttr.stStreamAttr.u32BitrateKbps = 4000;
    snprintf(muxAttr.szUrl, sizeof(muxAttr.szUrl), "%s", config_.rtsp_url.c_str());

    if (MUX_CreateChn(kMuxChn, &muxAttr) != 0) {
        fprintf(stderr, "[Pipeline] MUX_CreateChn failed\n");
        return false;
    }
    if (MUX_StartChn(kMuxChn) != 0) {
        fprintf(stderr, "[Pipeline] MUX_StartChn failed\n");
        return false;
    }

    // Warm up: discard initial frames
    printf("[Pipeline] Warming up (discarding initial frames)...\n");
    VideoFrameInfo warmFrame;
    for (int i = 0; i < 10; ++i) {
        memset(&warmFrame, 0, sizeof(warmFrame));
        if (UVC_GetFrame(kUvcDev, kUvcChn, &warmFrame, 3000) == 0) {
            UVC_ReleaseFrame(kUvcDev, kUvcChn, &warmFrame);
        }
    }

    // Start threads
    running_.store(true);
    capture_thread_ = std::thread(&Pipeline::CaptureLoop, this);
    infer_thread_ = std::thread(&Pipeline::InferLoop, this);

    printf("[Pipeline] Started. RTSP: %s\n", config_.rtsp_url.c_str());
    return true;
}

void Pipeline::Stop() {
    if (!running_.load()) return;

    running_.store(false);
    frame_cv_.notify_all();

    if (capture_thread_.joinable()) capture_thread_.join();
    if (infer_thread_.joinable()) infer_thread_.join();

    // Teardown MPP
    MUX_StopChn(kMuxChn);
    MUX_DestroyChn(kMuxChn);
    VENC_DisableChn(kVencChn);
    VENC_DestroyChn(kVencChn);
    VDEC_DisableChn(kVdecChn);
    VDEC_DestroyChn(kVdecChn);
    UVC_DisableChn(kUvcDev, kUvcChn);
    UVC_DisableDev(kUvcDev);
    UVC_DestroyDev(kUvcDev);

    MUX_Exit();
    VENC_Exit();
    VDEC_Exit();
    UVC_Exit();
    VB_Exit();
    SYS_Exit();

    printf("[Pipeline] Stopped.\n");
}

void Pipeline::EnableInference() {
    ai_enabled_.store(true);
    printf("[Pipeline] Inference enabled\n");
}

void Pipeline::DisableInference() {
    ai_enabled_.store(false);
    // Clear cached results
    std::lock_guard<std::mutex> lock(result_mutex_);
    cached_results_.clear();
    detect_count_.store(0);
    printf("[Pipeline] Inference disabled\n");
}

PipelineStats Pipeline::GetStats() const {
    PipelineStats stats;
    stats.capture_fps = capture_fps_.load();
    stats.infer_fps = infer_fps_.load();
    stats.detect_count = detect_count_.load();
    stats.total_frames = total_frames_.load();

    // Query MUX for RTSP client count
    MuxChnStat muxStat;
    if (MUX_GetChnStat(kMuxChn, &muxStat) == 0) {
        stats.rtsp_clients = muxStat.u32ActiveClients;
    }
    return stats;
}

void Pipeline::CaptureLoop() {
    using Clock = std::chrono::steady_clock;
    auto fps_start = Clock::now();
    uint32_t fps_count = 0;

    const uint32_t uvc_timeout = 3000;
    const uint32_t vdec_timeout = 1000;
    const uint32_t venc_timeout = 1000;

    while (running_.load()) {
        // 1. Get MJPEG frame from UVC
        VideoFrameInfo uvcFrame;
        memset(&uvcFrame, 0, sizeof(uvcFrame));
        int ret = UVC_GetFrame(kUvcDev, kUvcChn, &uvcFrame, uvc_timeout);
        if (ret != 0) {
            if (!running_.load()) break;
            continue;
        }

        if (uvcFrame.stVFrame.ulPlaneVirAddr[0] == 0 ||
            uvcFrame.stVFrame.u32PlaneSizeValid[0] == 0) {
            UVC_ReleaseFrame(kUvcDev, kUvcChn, &uvcFrame);
            continue;
        }

        // 2. Send MJPEG to VDEC
        StreamBufferInfo stream;
        memset(&stream, 0, sizeof(stream));
        stream.pu8Addr = reinterpret_cast<const U8*>(uvcFrame.stVFrame.ulPlaneVirAddr[0]);
        stream.u32Size = uvcFrame.stVFrame.u32PlaneSizeValid[0];
        stream.eCodecType = MPP_STREAM_CODEC_MJPEG;
        stream.bKeyFrame = MPP_TRUE;
        stream.bEndOfStream = MPP_FALSE;
        stream.u64PTS = uvcFrame.stVFrame.u64PTS;

        ret = VDEC_SendStream(kVdecChn, &stream, 0);
        UVC_ReleaseFrame(kUvcDev, kUvcChn, &uvcFrame);
        if (ret != 0) continue;

        // 3. Receive decoded NV12 frame
        VideoFrameInfo decFrame;
        memset(&decFrame, 0, sizeof(decFrame));

        ret = VDEC_GetFrame(kVdecChn, &decFrame, vdec_timeout);
        if (ret != 0) continue;
        UL ulVbBuff = decFrame.ulBufferId;

        // NV12 plane layout assumes stride == width and UV plane right after Y.
        // Hardware decoders can align stride (e.g. 16/32/64); enforce once per
        // stream so a silent stride mismatch surfaces as a clear error instead
        // of a smeared frame.
        uint32_t y_stride = decFrame.stVFrame.u32PlaneStride[0];
        if (!stride_checked_) {
            if (y_stride != 0 && y_stride != config_.width) {
                fprintf(stderr,
                        "[Pipeline] FATAL: VDEC Y plane stride=%u != width=%u. "
                        "Current nv12_draw / cv::cvtColor paths assume packed NV12. "
                        "Either pick an aligned resolution or rewrite buffer handling to use stride.\n",
                        y_stride, config_.width);
                VDEC_ReleaseFrame(kVdecChn, ulVbBuff);
                running_.store(false);
                break;
            }
            stride_checked_ = true;
        }

        // 4. If AI enabled: push frame to infer thread + draw cached results
        if (ai_enabled_.load()) {
            // Push NV12 frame to inference thread (non-blocking, latest-frame strategy)
            uint8_t* y_ptr = reinterpret_cast<uint8_t*>(decFrame.stVFrame.ulPlaneVirAddr[0]);
            uint32_t y_size = config_.width * config_.height;
            uint8_t* uv_ptr = y_ptr + y_size;

            // Convert NV12 to BGR for inference (only if infer thread is idle)
            {
                std::unique_lock<std::mutex> lock(frame_mutex_, std::try_to_lock);
                if (lock.owns_lock() && !frame_ready_) {
                    cv::Mat nv12(config_.height * 3 / 2, config_.width, CV_8UC1, y_ptr);
                    cv::cvtColor(nv12, infer_frame_, cv::COLOR_YUV2BGR_NV12);
                    frame_ready_ = true;
                    frame_cv_.notify_one();
                }
            }

            // Draw cached detection results on NV12
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                for (const auto& r : cached_results_) {
                    int bx1 = static_cast<int>(r.x1);
                    int by1 = static_cast<int>(r.y1);
                    int bx2 = static_cast<int>(r.x2);
                    int by2 = static_cast<int>(r.y2);
                    const auto& color = kBoxColors[r.label % kNumColors];

                    nv12_draw::draw_rect(
                        y_ptr, uv_ptr,
                        config_.width, config_.height,
                        bx1, by1, bx2, by2, color, 2);

                    // Draw label with score
                    char label_buf[32];
                    snprintf(label_buf, sizeof(label_buf), "%d:%.0f%%",
                        r.label, r.score * 100.0f);
                    nv12_draw::draw_text(
                        y_ptr, config_.width, config_.height,
                        bx1, by1 - 10, label_buf, 235, 1);
                }
            }
        }

        // 5. Send NV12 frame to VENC
        decFrame.eFrameType = FRAME_TYPE_VENC;
        ret = VENC_SendFrame(kVencChn, &decFrame, 0);
        VDEC_ReleaseFrame(kVdecChn, ulVbBuff);
        if (ret != 0) continue;

        // 6. Receive encoded stream and forward to MUX
        StreamBufferInfo encStream;
        memset(&encStream, 0, sizeof(encStream));
        ret = VENC_GetStream(kVencChn, &encStream, venc_timeout);
        if (ret == 0) {
            MuxPacket muxPkt;
            memset(&muxPkt, 0, sizeof(muxPkt));
            muxPkt.pu8Data = encStream.pu8Addr;
            muxPkt.u32Size = encStream.u32Size;
            muxPkt.bKeyFrame = encStream.bKeyFrame;
            muxPkt.eCodecType = MUX_CODEC_H264;
            muxPkt.u64PTS = encStream.u64PTS;

            MUX_SendPacket(kMuxChn, &muxPkt);
            VENC_ReleaseStream(kVencChn, &encStream);
        }

        // FPS calculation
        total_frames_.fetch_add(1);
        fps_count++;
        auto now = Clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_start).count();
        if (elapsed >= 1.0) {
            capture_fps_.store(static_cast<float>(fps_count / elapsed));
            fps_count = 0;
            fps_start = now;
        }
    }
}

void Pipeline::InferLoop() {
    if (!vision_) {
        fprintf(stderr, "[Pipeline] InferLoop: vision_ is null, exiting\n");
        return;
    }
    fprintf(stderr, "[Pipeline] InferLoop started\n");

    using Clock = std::chrono::steady_clock;
    auto fps_start = Clock::now();
    uint32_t fps_count = 0;

    while (running_.load()) {
        cv::Mat frame;

        // Wait for a new frame
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait_for(lock, std::chrono::milliseconds(100),
                            [this] { return frame_ready_ || !running_.load(); });
            if (!running_.load()) break;
            if (!frame_ready_) continue;
            frame = std::move(infer_frame_);
            frame_ready_ = false;
        }

        if (frame.empty()) {
            fprintf(stderr, "[Pipeline] InferLoop: frame is empty\n");
            continue;
        }
        if (!ai_enabled_.load()) continue;

        // Run inference
        std::vector<VisionServiceResult> results;
        auto status = vision_->InferImage(frame, &results);
        if (status != VISION_SERVICE_OK) {
            fprintf(stderr, "[Pipeline] InferImage failed: status=%d\n", status);
            continue;
        }

        // Update cached results
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            cached_results_ = std::move(results);
            detect_count_.store(static_cast<int>(cached_results_.size()));
        }

        // Infer FPS
        fps_count++;
        auto now = Clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_start).count();
        if (elapsed >= 1.0) {
            infer_fps_.store(static_cast<float>(fps_count / elapsed));
            fps_count = 0;
            fps_start = now;
        }
    }
}
