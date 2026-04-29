/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "http_server.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "pipeline.h"

#include "httplib.h"

struct HttpServer::Impl {
    httplib::Server svr;
    std::thread thread;
    Pipeline* pipeline;
    int port;
    std::string web_root;
};

HttpServer::HttpServer(Pipeline* pipeline, int port, const std::string& web_root)
    : impl_(std::make_unique<Impl>()) {
    impl_->pipeline = pipeline;
    impl_->port = port;
    impl_->web_root = web_root;

    auto& svr = impl_->svr;

    // CORS headers
    svr.set_default_headers({
        {"Access-Control-Allow-Origin", "*"},
        {"Access-Control-Allow-Methods", "GET, POST, OPTIONS"},
        {"Access-Control-Allow-Headers", "Content-Type"},
    });

    svr.Options(".*", [](const httplib::Request&, httplib::Response& res) {
        res.status = 204;
    });

    // POST /api/tracking/start - track all objects
    svr.Post("/api/tracking/start", [this](const httplib::Request&, httplib::Response& res) {
        impl_->pipeline->StartTracking();
        res.set_content(R"({"ok":true,"mode":"track_all"})", "application/json");
    });

    // POST /api/tracking/stop - stop tracking
    svr.Post("/api/tracking/stop", [this](const httplib::Request&, httplib::Response& res) {
        impl_->pipeline->StopTracking();
        res.set_content(R"({"ok":true,"mode":"off"})", "application/json");
    });

    // POST /api/tracking/select - select single target by ROI
    // Body: {"x1":0.1,"y1":0.2,"x2":0.5,"y2":0.6} (normalized coordinates)
    svr.Post("/api/tracking/select", [this](const httplib::Request& req, httplib::Response& res) {
        // Simple JSON parsing (avoid adding a JSON library dependency)
        auto parse_float = [](const std::string& body, const char* key) -> float {
            auto pos = body.find(key);
            if (pos == std::string::npos) return -1.0f;
            pos = body.find(':', pos);
            if (pos == std::string::npos) return -1.0f;
            return std::strtof(body.c_str() + pos + 1, nullptr);
        };

        SelectionBox box;
        box.x1 = parse_float(req.body, "\"x1\"");
        box.y1 = parse_float(req.body, "\"y1\"");
        box.x2 = parse_float(req.body, "\"x2\"");
        box.y2 = parse_float(req.body, "\"y2\"");

        if (box.x1 < 0 || box.y1 < 0 || box.x2 < 0 || box.y2 < 0) {
            res.status = 400;
            res.set_content(R"({"ok":false,"error":"missing x1/y1/x2/y2"})", "application/json");
            return;
        }

        impl_->pipeline->SelectTarget(box);
        res.set_content(R"({"ok":true,"mode":"selecting"})", "application/json");
    });

    // GET /api/status
    svr.Get("/api/status", [this](const httplib::Request&, httplib::Response& res) {
        auto stats = impl_->pipeline->GetStats();
        const char* mode_str = "off";
        if (stats.mode == TrackingMode::TRACK_ALL) mode_str = "track_all";
        else if (stats.mode == TrackingMode::TRACK_SINGLE) mode_str = "track_single";

        char buf[512];
        snprintf(buf, sizeof(buf),
            R"({"mode":"%s","capture_fps":%.1f,"inference_fps":%.1f,)"
            R"("track_count":%d,"rtsp_clients":%u,"total_frames":%lld,)"
            R"("selected_track_id":%d,"track_lost":%s})",
            mode_str,
            stats.capture_fps,
            stats.infer_fps,
            stats.track_count,
            stats.rtsp_clients,
            static_cast<int64_t>(stats.total_frames),
            stats.selected_track_id,
            stats.track_lost ? "true" : "false");
        res.set_content(buf, "application/json");
    });

    // GET /stream - MJPEG stream
    svr.Get("/stream", [this](const httplib::Request&, httplib::Response& res) {
        res.set_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [this](size_t /*offset*/, httplib::DataSink& sink) {
                std::vector<uint8_t> jpeg;
                while (true) {
                    if (!impl_->pipeline->GetMjpegFrame(jpeg) || jpeg.empty()) {
                        // No frame yet, wait a bit
                        std::this_thread::sleep_for(std::chrono::milliseconds(30));
                        continue;
                    }

                    std::string header = "--frame\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "Content-Length: " + std::to_string(jpeg.size()) + "\r\n"
                        "\r\n";
                    if (!sink.write(header.data(), header.size())) return false;
                    if (!sink.write(reinterpret_cast<const char*>(jpeg.data()), jpeg.size())) return false;
                    if (!sink.write("\r\n", 2)) return false;

                    std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30fps
                }
                return true;
            });
    });

    // Static file serving for web UI
    if (!web_root.empty()) {
        svr.set_mount_point("/", web_root);
    }
}

HttpServer::~HttpServer() {
    Stop();
}

void HttpServer::Start() {
    impl_->thread = std::thread([this] {
        printf("[HTTP] Listening on port %d\n", impl_->port);
        impl_->svr.listen("0.0.0.0", impl_->port);
    });
}

void HttpServer::Stop() {
    impl_->svr.stop();
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}
