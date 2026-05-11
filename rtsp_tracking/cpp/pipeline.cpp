/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTSP Tracking Pipeline
 * USB Camera → MJPEG → VDEC(NV12) → [ByteTrack + Draw] → VENC(H.264) → RTSP
 *                                          ↓
 *                                   NV12→BGR→JPEG (MJPEG HTTP stream)
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "pipeline.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "nv12_draw.h"

extern "C" {
#include "sys_api.h"
#include "vb_api.h"
#include "uvc_api.h"
#include "vdec/vdec_api.h"
#include "venc/venc_api.h"
#include "mux/mux_api.h"
}

static constexpr int kUvcDev  = 0;
static constexpr int kUvcChn  = 0;
static constexpr int kVdecChn = 0;
static constexpr int kVencChn = 0;
static constexpr int kMuxChn  = 0;

// Color palette for tracking boxes
static const nv12_draw::YuvColor kBoxColors[] = {
    nv12_draw::kRed,    nv12_draw::kGreen,  nv12_draw::kBlue,
    nv12_draw::kYellow, nv12_draw::kCyan,   nv12_draw::kWhite,
    {180, 80, 200},     {100, 200, 80},
};
static constexpr int kNumColors = sizeof(kBoxColors) / sizeof(kBoxColors[0]);

// Selected target highlight color (bright green)
static const nv12_draw::YuvColor kSelectedColor = nv12_draw::kGreen;

Pipeline::Pipeline(const PipelineConfig& config) : config_(config) {}

Pipeline::~Pipeline() {
    Stop();
}

float Pipeline::ComputeIoU(float ax1, float ay1, float ax2, float ay2,
    float bx1, float by1, float bx2, float by2) {
    float ix1 = std::max(ax1, bx1);
    float iy1 = std::max(ay1, by1);
    float ix2 = std::min(ax2, bx2);
    float iy2 = std::min(ay2, by2);

    float iw = std::max(0.0f, ix2 - ix1);
    float ih = std::max(0.0f, iy2 - iy1);
    float inter = iw * ih;

    float area_a = (ax2 - ax1) * (ay2 - ay1);
    float area_b = (bx2 - bx1) * (by2 - by1);
    float union_area = area_a + area_b - inter;

    return (union_area > 0) ? (inter / union_area) : 0.0f;
}

bool Pipeline::Start() {
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
    vencAttr.u32Gop = config_.fps;
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

    printf("[Pipeline] RTSP streaming at: %s\n", config_.rtsp_url.c_str());

    // Start threads
    running_.store(true);
    capture_thread_ = std::thread(&Pipeline::CaptureLoop, this);
    infer_thread_ = std::thread(&Pipeline::InferLoop, this);

    return true;
}

void Pipeline::Stop() {
    running_.store(false);

    frame_cv_.notify_all();

    if (capture_thread_.joinable()) capture_thread_.join();
    if (infer_thread_.joinable()) infer_thread_.join();

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
}

void Pipeline::StartTracking() {
    tracking_mode_.store(static_cast<int>(TrackingMode::TRACK_ALL));
    selected_track_id_.store(-1);
    track_lost_.store(false);
    lost_counter_.store(0);
    printf("[Pipeline] Tracking mode: TRACK_ALL\n");
}

void Pipeline::StopTracking() {
    tracking_mode_.store(static_cast<int>(TrackingMode::OFF));
    selected_track_id_.store(-1);
    track_lost_.store(false);
    lost_counter_.store(0);
    printf("[Pipeline] Tracking mode: OFF\n");
}

void Pipeline::SelectTarget(const SelectionBox& box) {
    std::lock_guard<std::mutex> lock(select_mutex_);
    pending_select_ = box;
    has_pending_select_ = true;
    // Mode will switch to TRACK_SINGLE once a match is found in InferLoop
    printf("[Pipeline] Selection received: (%.3f,%.3f)-(%.3f,%.3f)\n",
        box.x1, box.y1, box.x2, box.y2);
}

TrackingMode Pipeline::GetTrackingMode() const {
    return static_cast<TrackingMode>(tracking_mode_.load());
}

PipelineStats Pipeline::GetStats() const {
    PipelineStats stats;
    stats.capture_fps = capture_fps_.load();
    stats.infer_fps = infer_fps_.load();
    stats.track_count = track_count_.load();
    stats.total_frames = total_frames_.load();
    stats.mode = static_cast<TrackingMode>(tracking_mode_.load());
    stats.selected_track_id = selected_track_id_.load();
    stats.track_lost = track_lost_.load();

    MuxChnStat muxStat;
    if (MUX_GetChnStat(kMuxChn, &muxStat) == 0) {
        stats.rtsp_clients = muxStat.u32ActiveClients;
    }
    return stats;
}

bool Pipeline::GetMjpegFrame(std::vector<uint8_t>& out_jpeg) {
    std::lock_guard<std::mutex> lock(mjpeg_mutex_);
    if (mjpeg_buf_.empty()) return false;
    out_jpeg = mjpeg_buf_;
    return true;
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
        stream.pu8Addr = (const U8*)uvcFrame.stVFrame.ulPlaneVirAddr[0];
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

        uint8_t* y_ptr = reinterpret_cast<uint8_t*>(decFrame.stVFrame.ulPlaneVirAddr[0]);
        uint32_t y_size = config_.width * config_.height;
        uint8_t* uv_ptr = y_ptr + y_size;

        // 4. Convert NV12→BGR for inference (pass to infer thread)
        auto mode = static_cast<TrackingMode>(tracking_mode_.load());
        if (mode != TrackingMode::OFF && vision_) {
            std::unique_lock<std::mutex> lock(frame_mutex_, std::try_to_lock);
            if (lock.owns_lock() && !frame_ready_) {
                cv::Mat nv12(config_.height * 3 / 2, config_.width, CV_8UC1, y_ptr);
                cv::cvtColor(nv12, infer_frame_, cv::COLOR_YUV2BGR_NV12);
                frame_ready_ = true;
                frame_cv_.notify_one();
            }
        }

        // 5. Draw tracking results on NV12
        if (mode != TrackingMode::OFF) {
            std::lock_guard<std::mutex> lock(result_mutex_);
            int sel_id = selected_track_id_.load();

            for (const auto& r : cached_results_) {
                if (mode == TrackingMode::TRACK_SINGLE && r.track_id != sel_id) {
                    continue;
                }

                int bx1 = static_cast<int>(r.x1);
                int by1 = static_cast<int>(r.y1);
                int bx2 = static_cast<int>(r.x2);
                int by2 = static_cast<int>(r.y2);

                nv12_draw::YuvColor color;
                if (mode == TrackingMode::TRACK_SINGLE) {
                    color = kSelectedColor;
                } else {
                    color = kBoxColors[r.track_id % kNumColors];
                }

                nv12_draw::draw_rect(y_ptr, uv_ptr,
                    config_.width, config_.height,
                    bx1, by1, bx2, by2, color, 2);

                char label[32];
                snprintf(label, sizeof(label), "ID:%d", r.track_id);
                nv12_draw::draw_text(y_ptr, config_.width, config_.height,
                    bx1, by1 - 10, label, 235, 1);
            }
        }

        // 6. Generate MJPEG frame for web streaming
        {
            cv::Mat nv12_mat(config_.height * 3 / 2, config_.width, CV_8UC1, y_ptr);
            cv::Mat bgr_display;
            cv::cvtColor(nv12_mat, bgr_display, cv::COLOR_YUV2BGR_NV12);

            std::vector<uint8_t> jpeg_buf;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
            cv::imencode(".jpg", bgr_display, jpeg_buf, params);

            std::lock_guard<std::mutex> lock(mjpeg_mutex_);
            mjpeg_buf_ = std::move(jpeg_buf);
        }

        // 7. Send NV12 to VENC
        decFrame.eFrameType = FRAME_TYPE_VENC;
        ret = VENC_SendFrame(kVencChn, &decFrame, 0);
        VDEC_ReleaseFrame(kVdecChn, ulVbBuff);
        if (ret != 0) continue;

        // 8. Receive encoded stream and forward to MUX
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
    if (!vision_) return;

    using Clock = std::chrono::steady_clock;
    auto fps_start = Clock::now();
    uint32_t fps_count = 0;

    while (running_.load()) {
        cv::Mat frame;

        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait_for(lock, std::chrono::milliseconds(100),
                [this] { return frame_ready_ || !running_.load(); });
            if (!running_.load()) break;
            if (!frame_ready_) continue;
            frame = std::move(infer_frame_);
            frame_ready_ = false;
        }

        auto mode = static_cast<TrackingMode>(tracking_mode_.load());
        if (frame.empty() || mode == TrackingMode::OFF) continue;

        // Run inference (ByteTrack returns results with track_id)
        std::vector<VisionServiceResult> results;
        auto status = vision_->InferImage(frame, &results);
        if (status != VISION_SERVICE_OK) continue;

        // Handle pending selection
        {
            std::lock_guard<std::mutex> lock(select_mutex_);
            if (has_pending_select_ && !results.empty()) {
                float fw = static_cast<float>(frame.cols);
                float fh = static_cast<float>(frame.rows);
                // Convert normalized coords to pixel coords
                float sx1 = pending_select_.x1 * fw;
                float sy1 = pending_select_.y1 * fh;
                float sx2 = pending_select_.x2 * fw;
                float sy2 = pending_select_.y2 * fh;

                float best_iou = 0.0f;
                int best_id = -1;
                for (const auto& r : results) {
                    float iou = ComputeIoU(sx1, sy1, sx2, sy2, r.x1, r.y1, r.x2, r.y2);
                    if (iou > best_iou) {
                        best_iou = iou;
                        best_id = r.track_id;
                    }
                }

                if (best_id >= 0 && best_iou > 0.1f) {
                    selected_track_id_.store(best_id);
                    tracking_mode_.store(static_cast<int>(TrackingMode::TRACK_SINGLE));
                    track_lost_.store(false);
                    lost_counter_.store(0);
                    printf("[Pipeline] Selected track ID: %d (IoU=%.2f)\n", best_id, best_iou);
                } else {
                    printf("[Pipeline] No matching target found (best IoU=%.2f)\n", best_iou);
                }
                has_pending_select_ = false;
            }
        }

        // Track loss detection for TRACK_SINGLE mode
        if (mode == TrackingMode::TRACK_SINGLE) {
            int sel_id = selected_track_id_.load();
            bool found = false;
            for (const auto& r : results) {
                if (r.track_id == sel_id) {
                    found = true;
                    break;
                }
            }
            if (found) {
                lost_counter_.store(0);
                track_lost_.store(false);
            } else {
                int cnt = lost_counter_.fetch_add(1) + 1;
                if (cnt >= kLostThreshold) {
                    track_lost_.store(true);
                }
            }
        }

        // Update cached results
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            cached_results_ = std::move(results);
            track_count_.store(static_cast<int>(cached_results_.size()));
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
