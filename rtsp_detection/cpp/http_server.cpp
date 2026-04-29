/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "http_server.h"

#include <cstdint>
#include <cstdio>
#include <string>

#include "pipeline.h"

#include "httplib.h"

struct HttpServer::Impl {
    httplib::Server svr;
    std::thread thread;
    Pipeline* pipeline;
    int port;
    std::string web_root;
    std::string hls_dir;
};

HttpServer::HttpServer(Pipeline* pipeline, int port,
                    const std::string& web_root, const std::string& hls_dir)
    : impl_(std::make_unique<Impl>()) {
    impl_->pipeline = pipeline;
    impl_->port = port;
    impl_->web_root = web_root;
    impl_->hls_dir = hls_dir;

    auto& svr = impl_->svr;

    // CORS headers for browser access
    svr.set_default_headers({
        {"Access-Control-Allow-Origin", "*"},
        {"Access-Control-Allow-Methods", "GET, POST, OPTIONS"},
        {"Access-Control-Allow-Headers", "Content-Type"},
    });

    // OPTIONS preflight
    svr.Options(".*", [](const httplib::Request&, httplib::Response& res) {
        res.status = 204;
    });

    // POST /api/inference/enable
    svr.Post("/api/inference/enable", [this](const httplib::Request&, httplib::Response& res) {
        impl_->pipeline->EnableInference();
        res.set_content(R"({"ok":true,"inference_enabled":true})", "application/json");
    });

    // POST /api/inference/disable
    svr.Post("/api/inference/disable", [this](const httplib::Request&, httplib::Response& res) {
        impl_->pipeline->DisableInference();
        res.set_content(R"({"ok":true,"inference_enabled":false})", "application/json");
    });

    // GET /api/status
    svr.Get("/api/status", [this](const httplib::Request&, httplib::Response& res) {
        auto stats = impl_->pipeline->GetStats();
        char buf[512];
        snprintf(buf, sizeof(buf),
            R"({"inference_enabled":%s,"capture_fps":%.1f,"inference_fps":%.1f,)"
            R"("detection_count":%d,"rtsp_clients":%u,"total_frames":%llu})",
            impl_->pipeline->IsInferenceEnabled() ? "true" : "false",
            stats.capture_fps,
            stats.infer_fps,
            stats.detect_count,
            stats.rtsp_clients,
            static_cast<uint64_t>(stats.total_frames));
        res.set_content(buf, "application/json");
    });

    // Static file serving for web UI
    if (!web_root.empty()) {
        svr.set_mount_point("/", web_root);
    }

    // HLS file serving (m3u8 + ts segments)
    if (!hls_dir.empty()) {
        svr.set_mount_point("/stream", hls_dir);
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
