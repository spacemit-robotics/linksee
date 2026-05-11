/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTSP Tracking Application
 * USB Camera → MJPEG → VDEC(NV12) → [ByteTrack + Draw] → VENC(H.264) → RTSP
 * + HTTP API for tracking control + MJPEG browser stream + Web UI for ROI selection
 */

#include <csignal>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <string>
#include <thread>

#include "http_server.h"
#include "pipeline.h"

static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

static void print_usage(const char* prog) {
    fprintf(stderr,
            "Usage: %s [options]\n"
            "\n"
            "Options:\n"
            "  --device <path>       UVC device node (default: /dev/video0)\n"
            "  --width <n>           Capture width (default: 1280)\n"
            "  --height <n>          Capture height (default: 720)\n"
            "  --fps <n>             Frame rate (default: 30)\n"
            "  --rtsp-url <url>      RTSP output URL (default: rtsp://0.0.0.0:18554/live)\n"
            "  --http-port <n>       HTTP API port (default: 18080)\n"
            "  --config <path>       VisionService YAML config path\n"
            "  --model <path>        Model path override (optional)\n"
            "  --web-root <path>     Web static files directory (default: ./web)\n"
            "  --help                Show this help\n"
            "\n"
            "Example:\n"
            "  %s --device /dev/video0 --config config/rtsp_tracking.yaml\n"
            "\n"
            "Then:\n"
            "  - Web UI:  http://<board_ip>:18080/\n"
            "  - RTSP:    ffplay rtsp://<board_ip>:18554/live\n"
            "  - MJPEG:   http://<board_ip>:18080/stream\n",
            prog, prog);
}

int main(int argc, char* argv[]) {
    PipelineConfig config;
    int http_port = 18080;
    std::string web_root = "./web";

    for (int i = 1; i < argc; ++i) {
        auto arg = [&](const char* name) { return strcmp(argv[i], name) == 0; };
        auto next = [&]() -> const char* {
            if (i + 1 < argc) return argv[++i];
            fprintf(stderr, "Missing value for %s\n", argv[i]);
            exit(1);
        };

        if (arg("--device")) {
            config.device_node = next();
        } else if (arg("--width")) {
            config.width = atoi(next());
        } else if (arg("--height")) {
            config.height = atoi(next());
        } else if (arg("--fps")) {
            config.fps = atoi(next());
        } else if (arg("--rtsp-url")) {
            config.rtsp_url = next();
        } else if (arg("--http-port")) {
            http_port = atoi(next());
        } else if (arg("--config")) {
            config.config_path = next();
        } else if (arg("--model")) {
            config.model_path = next();
        } else if (arg("--web-root")) {
            web_root = next();
        } else if (arg("--help")) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    if (config.config_path.empty()) {
        fprintf(stderr, "Error: --config is required\n\n");
        print_usage(argv[0]);
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== RTSP Tracking Application ===\n");
    printf("  Device:     %s\n", config.device_node.c_str());
    printf("  Resolution: %ux%u @ %u fps\n", config.width, config.height, config.fps);
    printf("  RTSP URL:   %s\n", config.rtsp_url.c_str());
    printf("  HTTP port:  %d\n", http_port);
    printf("  Config:     %s\n", config.config_path.c_str());
    printf("  Web root:   %s\n", web_root.c_str());
    printf("=================================\n\n");

    // Create and start pipeline
    Pipeline pipeline(config);
    if (!pipeline.Start()) {
        fprintf(stderr, "Failed to start pipeline\n");
        return 1;
    }

    // Start HTTP server
    HttpServer http(&pipeline, http_port, web_root);
    http.Start();

    printf("\n");
    printf("  RTSP stream: %s\n", config.rtsp_url.c_str());
    printf("  Web UI:      http://0.0.0.0:%d/\n", http_port);
    printf("  MJPEG:       http://0.0.0.0:%d/stream\n", http_port);
    printf("\n");

    // Wait for signal
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("\nShutting down...\n");
    http.Stop();
    pipeline.Stop();

    printf("Done.\n");
    return 0;
}
