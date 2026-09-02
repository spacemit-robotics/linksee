/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file main.cpp
    * @brief perceptive_grasp application entry point.
    *
    * 用法:
    *   ./perceptive_grasp --config config/grasp_pipeline.yaml
    *   ./perceptive_grasp --config config/grasp_pipeline.yaml --target apple
    *   ./perceptive_grasp --config config/grasp_pipeline.yaml --loop
    */

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

#include <unistd.h>

namespace fs = std::filesystem;

#include "camera_calibration.h"
#include "grasp_pipeline.h"
#ifdef ENABLE_ROS2_VOICE
#include "voice_command_listener.h"
#endif

using perceptive_grasp::GraspPipeline;
using perceptive_grasp::JointConstraint;
using perceptive_grasp::PipelineConfig;
using perceptive_grasp::PipelineState;
#ifdef ENABLE_ROS2_VOICE
using perceptive_grasp::VoiceCommandListener;
#endif

static std::unique_ptr<GraspPipeline> g_pipeline;
#ifdef ENABLE_ROS2_VOICE
static std::unique_ptr<VoiceCommandListener> g_voice_listener;
#endif
static std::thread g_local_voice_thread;
static std::mutex g_pipeline_mutex;
static volatile std::sig_atomic_t g_shutdown_signal = 0;

static bool TriggerVoiceCommand(const std::string& command_text) {
    std::lock_guard<std::mutex> lock(g_pipeline_mutex);
    if (!g_pipeline) return false;
    return g_pipeline->TriggerVoiceCommand(command_text);
}

static void CleanupRuntime(bool destroy_pipeline = true) {
#ifdef ENABLE_ROS2_VOICE
    if (g_voice_listener) {
        g_voice_listener->Stop();
        g_voice_listener.reset();
    }
#endif
    if (destroy_pipeline) {
        std::lock_guard<std::mutex> lock(g_pipeline_mutex);
        g_pipeline.reset();
    }
    if (g_local_voice_thread.joinable()) {
        g_local_voice_thread.detach();
    }
}

static std::string EscapeStatusField(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char ch : text) {
        if (ch == '\\' || ch == ';' || ch == '=') {
            out.push_back('\\');
        }
        out.push_back(ch);
    }
    return out;
}

static std::string ExtractStatusTarget(const std::string& msg) {
    const std::string target_key = "target: ";
    auto pos = msg.find(target_key);
    if (pos != std::string::npos) {
        pos += target_key.size();
        auto end = msg.find_first_of(";,", pos);
        return msg.substr(pos, end == std::string::npos ? std::string::npos
                                                        : end - pos);
    }

    const std::string not_found = "Target not found: ";
    pos = msg.find(not_found);
    if (pos != std::string::npos) {
        pos += not_found.size();
        auto end = msg.find_first_of(";,", pos);
        return msg.substr(pos, end == std::string::npos ? std::string::npos
                                                        : end - pos);
    }
    return "";
}

static const char* StatusReasonFromMessage(PipelineState state,
                                            const std::string& msg) {
    if (state == PipelineState::DONE) return "success";
    if (msg.find("Target not found") != std::string::npos) return "target_not_found";
    if (msg.find("depth invalid") != std::string::npos) return "depth_invalid";
    if (msg.find("out of workspace") != std::string::npos) return "out_of_workspace";
    if (msg.find("IK failed") != std::string::npos) return "ik_failed";
    if (msg.find("timeout") != std::string::npos) return "timeout";
    if (msg.find("Grasp empty") != std::string::npos ||
        msg.find("grasp empty") != std::string::npos) {
        return "grasp_empty";
    }
    if (msg.find("Cancelled") != std::string::npos ||
        msg.find("Cancelling") != std::string::npos) {
        return "cancelled";
    }
    if (state == PipelineState::ERROR) return "error";
    return "";
}

static std::string MakeStatusEvent(PipelineState state,
                                    const std::string& msg) {
    std::ostringstream oss;
    oss << "state=" << PipelineStateName(state)
        << ";message=" << EscapeStatusField(msg);
    const std::string target = ExtractStatusTarget(msg);
    if (!target.empty()) {
        oss << ";target=" << EscapeStatusField(target);
    }
    const char* reason = StatusReasonFromMessage(state, msg);
    if (reason && reason[0] != '\0') {
        oss << ";reason=" << reason;
    }
    return oss.str();
}

static void WriteStatusEvent(PipelineState state, const std::string& msg) {
    const std::string line =
        "VOICE_STATUS\t" + MakeStatusEvent(state, msg) + "\n";
    ssize_t written;
    do {
        written = write(STDOUT_FILENO, line.data(), line.size());
    } while (written < 0 && errno == EINTR);
}

static void SignalHandler(int sig) {
    if (g_shutdown_signal == 0) {
        g_shutdown_signal = sig;
        const char msg[] =
            "\n[Main] Graceful shutdown requested; waiting for the current "
            "action and returning home. Press Ctrl+C again to force exit.\n";
        const ssize_t written = write(STDERR_FILENO, msg, sizeof(msg) - 1);
        (void)written;
        return;
    }

    const char msg[] =
        "\n[Main] Second signal received; forcing exit.\n";
    const ssize_t written = write(STDERR_FILENO, msg, sizeof(msg) - 1);
    (void)written;
    std::_Exit(128 + sig);
}

static void PrintUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
                << "Options:\n"
                << "  --config <yaml>   Pipeline config file (required)\n"
                << "  --target <name>   Target object name (e.g. apple, bottle)\n"
                << "  --voice-command <text>  Test one ASR text command (e.g. 抓香蕉)\n"
                << "  --voice-stdin     Read ASR text commands from stdin\n"
                << "  --status-stdout   Print status events to stdout for local TTS\n"
                << "  --loop            Auto-loop; Ctrl+C returns home and exits\n"
                << "  --step            Step mode: pause before each stage\n"
                << "  --plan-only       Validate perception and arm path without motion\n"
                << "  --validate-config Validate configuration without initializing hardware\n"
                << "  --remote-host <host>  Override remote_mujoco server host\n"
                << "  --remote-port <port>  Override remote_mujoco server port\n"
                << "  --help            Show this help\n";
#ifdef ENABLE_ROS2_VOICE
    std::cout << "  --voice           Listen for ASR text from ROS2 topic (optional)\n"
                << "  --tts             Publish pipeline status text to ROS2 topic (optional)\n"
                << "  --voice-topic <topic>   ROS2 std_msgs/String topic "
                << "(default: asr_text)\n"
                << "  --status-topic <topic>  ROS2 status topic "
                << "(default: grasp_status_text)\n";
#endif
    std::cout << "\nExamples:\n"
                << "  " << prog << " --config config/grasp_pipeline.yaml\n"
                << "  " << prog
                << " --config config/grasp_pipeline.yaml --target banana --step\n";
}

static void ResolveConfigPath(const fs::path& config_dir,
                            std::string* path) {
    if (path == nullptr || path->empty()) return;

    std::string expanded = *path;
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
        if (expanded == "~") {
            expanded = home;
        } else if (expanded.rfind("~/", 0) == 0) {
            expanded = (fs::path(home) / expanded.substr(2)).string();
        } else if (expanded.rfind("$HOME/", 0) == 0) {
            expanded = (fs::path(home) / expanded.substr(6)).string();
        }
    }

    fs::path resolved(expanded);
    if (!resolved.is_absolute()) {
        resolved = config_dir / resolved;
    }
    *path = fs::weakly_canonical(resolved).string();
}

static cv::Scalar ParseHsvTriplet(const YAML::Node& node,
                                const std::string& field_name) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error(field_name + " must contain [h, s, v]");
    }
    const int hue = node[0].as<int>();
    const int saturation = node[1].as<int>();
    const int value = node[2].as<int>();
    if (hue < 0 || hue > 179 || saturation < 0 || saturation > 255 ||
        value < 0 || value > 255) {
        throw std::runtime_error(field_name + " is outside the OpenCV HSV range");
    }
    return cv::Scalar(hue, saturation, value);
}

static PipelineConfig LoadConfig(const std::string& config_path) {
    PipelineConfig cfg;

    YAML::Node root = YAML::LoadFile(config_path);

    // A remote simulation uses one endpoint for both frame acquisition and
    // robot commands. Backend-local blocks are accepted when present.
    if (auto remote = root["remote_mujoco"]) {
        if (!remote.IsMap()) {
            throw std::runtime_error("remote_mujoco must be a mapping");
        }
        const std::string host = remote["host"].as<std::string>(
            cfg.camera.remote_mujoco.host);
        const int port = remote["port"].as<int>(
            cfg.camera.remote_mujoco.port);
        if (host.empty() || port < 1 || port > 65535) {
            throw std::runtime_error(
                "remote_mujoco host must be set and port must be in "
                "[1, 65535]");
        }
        cfg.camera.remote_mujoco.host = host;
        cfg.camera.remote_mujoco.port = port;
        cfg.executor.remote_mujoco.host = host;
        cfg.executor.remote_mujoco.port = port;
        cfg.camera.remote_mujoco.timeout_ms =
            remote["frame_timeout_ms"].as<int>(
                cfg.camera.remote_mujoco.timeout_ms);
        cfg.executor.remote_mujoco.timeout_ms =
            remote["action_timeout_ms"].as<int>(
                cfg.executor.remote_mujoco.timeout_ms);
    }

    // Camera
    if (auto cam = root["camera"]) {
        cfg.camera.type = cam["type"].as<std::string>(cfg.camera.type);
        if (cfg.camera.type == "realsense" ||
            cfg.camera.type == "d435i") {
            auto& settings = cfg.camera.realsense;
            if (auto realsense = cam["realsense"]) {
                settings.width = realsense["width"].as<int>(settings.width);
                settings.height = realsense["height"].as<int>(settings.height);
                settings.fps = realsense["fps"].as<int>(settings.fps);
                settings.motion_flush_frames = std::max(
                    0, realsense["motion_flush_frames"].as<int>(
                        settings.motion_flush_frames));
                settings.align_depth =
                    realsense["align_depth"].as<bool>(settings.align_depth);
                if (auto depth_filter = realsense["depth_filter"]) {
                    settings.spatial_filter =
                        depth_filter["spatial"].as<bool>(
                            settings.spatial_filter);
                    settings.temporal_filter =
                        depth_filter["temporal"].as<bool>(
                            settings.temporal_filter);
                    settings.hole_filling =
                        depth_filter["hole_filling"].as<bool>(
                            settings.hole_filling);
                }
            }
        } else if (cfg.camera.type == "spacemit_las2") {
            auto las2 = cam["spacemit_las2"];
            if (!las2 || !las2.IsMap()) {
                throw std::runtime_error(
                    "camera.spacemit_las2 configuration is required");
            }
            auto& settings = cfg.camera.spacemit_las2;
            settings.video_device = las2["video_device"].as<std::string>(
                settings.video_device);
            settings.model_path = las2["model_path"].as<std::string>(
                settings.model_path);
            settings.calib_path = las2["calib_path"].as<std::string>(
                settings.calib_path);
            settings.core_count =
                las2["core_count"].as<int>(settings.core_count);
            settings.core_affinity = las2["core_affinity"].as<std::string>(
                settings.core_affinity);
            if (auto depth = las2["depth"]) {
                settings.min_depth_m =
                    depth["min_m"].as<float>(settings.min_depth_m);
                settings.max_depth_m =
                    depth["max_m"].as<float>(settings.max_depth_m);
            }
        } else if (cfg.camera.type == "mujoco") {
            auto mujoco = cam["mujoco"];
            if (!mujoco || !mujoco.IsMap()) {
                throw std::runtime_error(
                    "camera.mujoco configuration is required");
            }
            auto& settings = cfg.camera.mujoco;
            settings.xml_path = mujoco["xml_path"].as<std::string>(
                settings.xml_path);
            settings.camera_name = mujoco["camera_name"].as<std::string>(
                settings.camera_name);
            settings.width = mujoco["width"].as<int>(settings.width);
            settings.height = mujoco["height"].as<int>(settings.height);
            if (auto depth = mujoco["depth"]) {
                settings.min_depth_m =
                    depth["min_m"].as<float>(settings.min_depth_m);
                settings.max_depth_m =
                    depth["max_m"].as<float>(settings.max_depth_m);
            }
        } else if (cfg.camera.type == "remote_mujoco") {
            auto remote = cam["remote_mujoco"];
            auto& settings = cfg.camera.remote_mujoco;
            if (remote) {
                if (!remote.IsMap()) {
                    throw std::runtime_error(
                        "camera.remote_mujoco must be a mapping");
                }
                settings.host = remote["host"].as<std::string>(settings.host);
                settings.port = remote["port"].as<int>(settings.port);
                settings.timeout_ms =
                    remote["timeout_ms"].as<int>(settings.timeout_ms);
            }
        } else {
            throw std::runtime_error("unsupported camera.type: " +
                                    cfg.camera.type);
        }
    }

    // Detection
    if (auto det = root["detection"]) {
        cfg.detector.config_path = det["config_path"].as<std::string>(
            cfg.detector.config_path);
        cfg.detector.min_confidence = det["min_confidence"].as<float>(
            cfg.detector.min_confidence);
        cfg.detector.min_area = det["min_area"].as<float>(
            cfg.detector.min_area);
        cfg.detect_stable_frames =
            std::max(1, det["stable_frames"].as<int>(
                            cfg.detect_stable_frames));
        cfg.detector.allow_color_only_fallback =
            det["allow_color_only_fallback"].as<bool>(false);
        cfg.detector.refine_with_simulation_colors =
            det["refine_with_simulation_colors"].as<bool>(false);
        if (cfg.detector.allow_color_only_fallback &&
            cfg.camera.type != "mujoco" &&
            cfg.camera.type != "remote_mujoco") {
            throw std::runtime_error(
                "detection.allow_color_only_fallback is only valid for "
                "mujoco camera backends");
        }
        if (cfg.detector.refine_with_simulation_colors &&
            cfg.camera.type != "mujoco" &&
            cfg.camera.type != "remote_mujoco") {
            throw std::runtime_error(
                "detection.refine_with_simulation_colors is only valid "
                "for mujoco camera backends");
        }
        if (auto labels = det["target_labels"]) {
            for (size_t i = 0; i < labels.size(); i++) {
                cfg.detector.target_labels.push_back(labels[i].as<int>());
            }
        }
        if (auto remap = det["label_remap"]) {
            if (!remap.IsMap()) {
                throw std::runtime_error(
                    "detection.label_remap must be a mapping");
            }
            for (const auto& item : remap) {
                const std::string source = item.first.as<std::string>();
                const std::string target = item.second.as<std::string>();
                if (source.empty() || target.empty()) {
                    throw std::runtime_error(
                        "detection.label_remap entries must not be empty");
                }
                cfg.detector.label_remap[source] = target;
            }
        }
        if (auto color_targets = det["simulation_color_targets"]) {
            if (cfg.camera.type != "mujoco" &&
                cfg.camera.type != "remote_mujoco") {
                throw std::runtime_error(
                    "detection.simulation_color_targets is only valid for "
                    "mujoco camera backends");
            }
            if (!color_targets.IsSequence()) {
                throw std::runtime_error(
                    "detection.simulation_color_targets must be a sequence");
            }
            for (size_t i = 0; i < color_targets.size(); ++i) {
                const YAML::Node item = color_targets[i];
                perceptive_grasp::SimulationColorTargetConfig target;
                target.label = item["label"].as<std::string>("");
                if (target.label.empty()) {
                    throw std::runtime_error(
                        "simulation color target label must not be empty");
                }
                const std::string prefix =
                    "detection.simulation_color_targets[" +
                    std::to_string(i) + "]";
                target.hsv_min = ParseHsvTriplet(
                    item["hsv_min"], prefix + ".hsv_min");
                target.hsv_max = ParseHsvTriplet(
                    item["hsv_max"], prefix + ".hsv_max");
                target.min_area = item["min_area"].as<float>(
                    target.min_area);
                target.max_area = item["max_area"].as<float>(
                    target.max_area);
                target.score = item["score"].as<float>(target.score);
                if (target.min_area <= 0.0f || target.max_area < 0.0f ||
                    (target.max_area > 0.0f &&
                        target.max_area < target.min_area) ||
                    target.score <= 0.0f || target.score > 1.0f) {
                    throw std::runtime_error(
                        prefix + " has an invalid area range or score");
                }
                for (int channel = 0; channel < 3; ++channel) {
                    if (target.hsv_min[channel] > target.hsv_max[channel]) {
                        throw std::runtime_error(
                            prefix + ".hsv_min must not exceed hsv_max");
                    }
                }
                cfg.detector.simulation_color_targets.push_back(target);
            }
        }
    }

    perceptive_grasp::LoadCameraCalibration(
        root, cfg.camera.type, &cfg.planner);

    // Grasp strategy
    if (auto g = root["grasp"]) {
        const YAML::Node top = g["top"];
        if (!top) {
            throw std::runtime_error("grasp.top configuration is required");
        }
        cfg.planner.approach_height = top["approach_height"].as<float>(
            cfg.planner.approach_height);
        cfg.planner.grasp_depth = top["grasp_depth"].as<float>(
            cfg.planner.grasp_depth);
        cfg.planner.gripper_offset = top["gripper_offset"].as<float>(
            cfg.planner.gripper_offset);
        cfg.top_grasp_point_x_ratio =
            top["grasp_point_x_ratio"].as<float>(
                cfg.top_grasp_point_x_ratio);
        if (cfg.top_grasp_point_x_ratio < 0.0f ||
            cfg.top_grasp_point_x_ratio > 1.0f) {
            throw std::runtime_error(
                "grasp.top.grasp_point_x_ratio must be in [0, 1]");
        }
        cfg.top_position_source = top["position_source"].as<std::string>(
            cfg.top_position_source);
        if (cfg.top_position_source != "mask_depth" &&
            cfg.top_position_source != "projected_geometry_center") {
            throw std::runtime_error(
                "grasp.top.position_source must be mask_depth or "
                "projected_geometry_center");
        }
        cfg.orientation.safe_mask_interior =
            top["safe_mask_interior"].as<bool>(
                cfg.orientation.safe_mask_interior);
        cfg.top_support_plane_occlusion_recovery =
            top["support_plane_occlusion_recovery"].as<bool>(
                cfg.top_support_plane_occlusion_recovery);
        cfg.top_support_plane_height_anchor =
            top["support_plane_height_anchor"].as<bool>(
                cfg.top_support_plane_height_anchor);
        cfg.top_projected_center_blend =
            top["projected_center_blend"].as<float>(
                cfg.top_projected_center_blend);
        cfg.top_sparse_projected_center_blend =
            top["sparse_projected_center_blend"].as<float>(
                cfg.top_projected_center_blend);
        if (cfg.top_projected_center_blend < 0.0f ||
            cfg.top_projected_center_blend > 1.0f) {
            throw std::runtime_error(
                "grasp.top.projected_center_blend must be in [0, 1]");
        }
        if (cfg.top_sparse_projected_center_blend < 0.0f ||
            cfg.top_sparse_projected_center_blend > 1.0f) {
            throw std::runtime_error(
                "grasp.top.sparse_projected_center_blend must be in "
                "[0, 1]");
        }
        cfg.top_minimum_grasp_height_m =
            top["minimum_grasp_height"].as<float>(
                cfg.top_minimum_grasp_height_m);
        if (cfg.top_minimum_grasp_height_m < 0.0f) {
            throw std::runtime_error(
                "grasp.top.minimum_grasp_height must be non-negative");
        }
        cfg.top_verification_lift_m =
            top["verification_lift_m"].as<float>(
                cfg.top_verification_lift_m);
        if (cfg.top_verification_lift_m < 0.0f ||
            cfg.top_verification_lift_m > cfg.planner.approach_height) {
            throw std::runtime_error(
                "grasp.top.verification_lift_m must be in [0, "
                "grasp.top.approach_height]");
        }
        cfg.executor.gripper_open = top["gripper_open"].as<float>(
            cfg.executor.gripper_open);
        cfg.executor.gripper_effort = g["gripper_effort"].as<float>(
            cfg.executor.gripper_effort);
        cfg.executor.gripper_hold_load_threshold =
            g["gripper_hold_load_threshold"].as<float>(
                cfg.executor.gripper_hold_load_threshold);
        cfg.executor.gripper_empty_position_margin =
            g["gripper_empty_position_margin"].as<float>(
                cfg.executor.gripper_empty_position_margin);
        cfg.executor.gripper_timeout_ms =
            g["gripper_timeout_ms"].as<int>(
                cfg.executor.gripper_timeout_ms);
        auto& settings = cfg.geometry;
        settings.strategy = g["strategy"].as<std::string>(
            settings.strategy);
        if (auto geometry = g["geometry"]) {
            settings.sample_stride = std::max(
                1, geometry["sample_stride"].as<int>(
                        settings.sample_stride));
            settings.max_object_points = std::max(
                200, geometry["max_object_points"].as<int>(
                        settings.max_object_points));
            settings.min_object_points = std::max(
                20, geometry["min_object_points"].as<int>(
                        settings.min_object_points));
            settings.plane_distance_threshold_m =
                geometry["plane_distance_threshold_m"].as<float>(
                    settings.plane_distance_threshold_m);
            settings.table_clearance_m =
                geometry["table_clearance_m"].as<float>(
                    settings.table_clearance_m);
            settings.footprint_padding_m =
                geometry["footprint_padding_m"].as<float>(
                    settings.footprint_padding_m);
            settings.gripper_max_width_m =
                geometry["gripper_max_width_m"].as<float>(
                    settings.gripper_max_width_m);
            settings.planning_timeout_ms = std::max(
                50, geometry["planning_timeout_ms"].as<int>(
                        settings.planning_timeout_ms));
            settings.perception_budget_ms = std::max(
                1,
                geometry["perception_budget_ms"].as<int>(
                    settings.perception_budget_ms));
        }
        if (auto side = g["side"]) {
            settings.side_min_height_m =
                side["min_height_m"].as<float>(
                    settings.side_min_height_m);
            settings.side_min_height_width_ratio =
                side["min_height_width_ratio"].as<float>(
                    settings.side_min_height_width_ratio);
            settings.side_approach_distance_m =
                side["approach_distance_m"].as<float>(
                    settings.side_approach_distance_m);
            settings.side_entry_clearance_m =
                side["entry_clearance_m"].as<float>(
                    settings.side_entry_clearance_m);
            settings.side_pregrasp_min_x_m =
                side["pregrasp_min_x_m"].as<float>(
                    settings.side_pregrasp_min_x_m);
            settings.side_single_sided_gripper =
                side["single_sided_gripper"].as<bool>(
                    settings.side_single_sided_gripper);
            settings.side_gripper_offset_m =
                side["gripper_offset_m"].as<float>(
                    settings.side_gripper_offset_m);
            settings.side_visible_surface_offset_m =
                side["visible_surface_offset_m"].as<float>(
                    settings.side_visible_surface_offset_m);
            settings.side_grasp_forward_offset_m =
                side["grasp_forward_offset_m"].as<float>(
                    settings.side_grasp_forward_offset_m);
            settings.side_grasp_height_ratio =
                side["grasp_height_ratio"].as<float>(
                    settings.side_grasp_height_ratio);
            settings.side_initial_lift_m =
                side["initial_lift_m"].as<float>(
                    settings.side_initial_lift_m);
            settings.side_lift_retreat_m =
                side["lift_retreat_m"].as<float>(
                    settings.side_lift_retreat_m);
        }
        if (settings.strategy != "auto" &&
            settings.strategy != "top" &&
            settings.strategy != "side") {
            throw std::runtime_error(
                "grasp.strategy must be auto, top, or side");
        }
        if (settings.min_object_points > settings.max_object_points) {
            throw std::runtime_error(
                "grasp.geometry.min_object_points must not exceed "
                "max_object_points");
        }
        if (settings.plane_distance_threshold_m <= 0.0f ||
            settings.table_clearance_m < 0.0f ||
            settings.footprint_padding_m < 0.0f ||
            settings.gripper_max_width_m <= 0.0f ||
            settings.side_min_height_m < 0.0f ||
            settings.side_min_height_width_ratio <= 0.0f ||
            settings.side_approach_distance_m <= 0.0f ||
            settings.side_entry_clearance_m < 0.0f ||
            settings.side_pregrasp_min_x_m < 0.0f ||
            settings.side_visible_surface_offset_m < 0.0f ||
            settings.side_grasp_forward_offset_m < 0.0f ||
            settings.side_grasp_height_ratio <= 0.0f ||
            settings.side_grasp_height_ratio >= 1.0f ||
            settings.side_initial_lift_m <= 0.0f ||
            settings.side_lift_retreat_m < 0.0f) {
            throw std::runtime_error(
                "grasp geometry, top, or side values are invalid");
        }
        if (auto ws = g["workspace"]) {
            auto& workspace = cfg.planner.workspace;
            workspace.x_min = ws["x_min"].as<float>(workspace.x_min);
            workspace.x_max = ws["x_max"].as<float>(workspace.x_max);
            workspace.y_min = ws["y_min"].as<float>(workspace.y_min);
            workspace.y_max = ws["y_max"].as<float>(workspace.y_max);
            workspace.z_min = ws["z_min"].as<float>(workspace.z_min);
            workspace.z_max = ws["z_max"].as<float>(workspace.z_max);
        }
    }

    // Orientation (自动夹爪方向对齐)
    if (auto ori = root["orientation"]) {
        cfg.auto_orient = ori["enabled"].as<bool>(cfg.auto_orient);
        cfg.orientation.aspect_ratio_threshold =
            ori["aspect_ratio_threshold"].as<float>(
                cfg.orientation.aspect_ratio_threshold);
        cfg.orientation.camera_yaw_offset =
            ori["camera_yaw_offset"].as<float>(
                cfg.orientation.camera_yaw_offset);
    }

    // Manipulator
    if (auto m = root["manipulator"]) {
        cfg.executor.manip_driver = m["driver"].as<std::string>(
            cfg.executor.manip_driver);
        cfg.executor.uart_device = m["uart_device"].as<std::string>(
            cfg.executor.uart_device);
        cfg.executor.baudrate = m["baudrate"].as<int>(
            cfg.executor.baudrate);
        cfg.executor.urdf_path = m["urdf_path"].as<std::string>(
            cfg.executor.urdf_path);
        cfg.executor.legacy_top_ik = m["legacy_top_ik"].as<bool>(
            cfg.executor.legacy_top_ik);

        // 解析 URDF 相对路径: 相对于配置文件所在目录
        if (!cfg.executor.urdf_path.empty() &&
            cfg.executor.urdf_path[0] != '/') {
            fs::path config_dir = fs::path(config_path).parent_path();
            fs::path resolved = config_dir / cfg.executor.urdf_path;
            if (fs::exists(resolved)) {
                cfg.executor.urdf_path = fs::canonical(resolved).string();
            }
        }

        cfg.executor.base_link = m["base_link"].as<std::string>(
            cfg.executor.base_link);
        cfg.executor.tip_link = m["tip_link"].as<std::string>(
            cfg.executor.tip_link);
        cfg.executor.move_speed = m["move_speed"].as<float>(
            cfg.executor.move_speed);
        cfg.executor.line_speed = m["line_speed"].as<float>(
            cfg.executor.line_speed);
        cfg.executor.pose_position_tolerance =
            m["pose_position_tolerance"].as<float>(
                cfg.executor.pose_position_tolerance);
        if (auto hj = m["home_joints"]) {
            cfg.executor.home_joints.clear();
            for (size_t i = 0; i < hj.size(); i++)
                cfg.executor.home_joints.push_back(hj[i].as<float>());
        }
        if (auto oj = m["observe_joints"]) {
            cfg.executor.observe_joints.clear();
            for (size_t i = 0; i < oj.size(); i++)
                cfg.executor.observe_joints.push_back(oj[i].as<float>());
        }
        if (auto srj = m["side_ready_joints"]) {
            cfg.executor.side_ready_joints.clear();
            for (size_t i = 0; i < srj.size(); ++i) {
                cfg.executor.side_ready_joints.push_back(
                    srj[i].as<float>());
            }
        }

        if (auto mj = m["mujoco"]) {
            auto& mujoco = cfg.executor.mujoco;
            mujoco.xml_path = mj["xml_path"].as<std::string>(
                mujoco.xml_path);
            mujoco.end_effector_site =
                mj["end_effector_site"].as<std::string>(
                    mujoco.end_effector_site);
            mujoco.gripper_actuator =
                mj["gripper_actuator"].as<std::string>(
                    mujoco.gripper_actuator);
            mujoco.robot_root_body =
                mj["robot_root_body"].as<std::string>(
                    mujoco.robot_root_body);
            mujoco.gripper_root_body =
                mj["gripper_root_body"].as<std::string>(
                    mujoco.gripper_root_body);
            mujoco.gripper_open_ctrl =
                mj["gripper_open_ctrl"].as<float>(
                    mujoco.gripper_open_ctrl);
            mujoco.gripper_close_ctrl =
                mj["gripper_close_ctrl"].as<float>(
                    mujoco.gripper_close_ctrl);
            mujoco.gravity_compensation =
                mj["gravity_compensation"].as<bool>(
                    mujoco.gravity_compensation);
            mujoco.arm_stiffness_scale =
                mj["arm_stiffness_scale"].as<float>(
                    mujoco.arm_stiffness_scale);
            if (!std::isfinite(mujoco.arm_stiffness_scale) ||
                mujoco.arm_stiffness_scale <= 0.0f) {
                throw std::runtime_error(
                    "manipulator.mujoco.arm_stiffness_scale must be positive");
            }
            mujoco.joint_tolerance_rad =
                mj["joint_tolerance_rad"].as<float>(
                    mujoco.joint_tolerance_rad);
            mujoco.ik_position_tolerance_m =
                mj["ik_position_tolerance_m"].as<float>(
                    mujoco.ik_position_tolerance_m);
            mujoco.cartesian_tracking_tolerance_m =
                mj["cartesian_tracking_tolerance_m"].as<float>(
                    mujoco.cartesian_tracking_tolerance_m);
            mujoco.ik_step_scale =
                mj["ik_step_scale"].as<float>(mujoco.ik_step_scale);
            mujoco.ik_damping =
                mj["ik_damping"].as<float>(mujoco.ik_damping);
            mujoco.ik_iterations =
                mj["ik_iterations"].as<int>(mujoco.ik_iterations);
            mujoco.settle_steps =
                mj["settle_steps"].as<int>(mujoco.settle_steps);
            mujoco.max_motion_steps =
                mj["max_motion_steps"].as<int>(
                    mujoco.max_motion_steps);
            if (auto names = mj["joint_names"]) {
                mujoco.joint_names.clear();
                for (size_t i = 0; i < names.size(); ++i) {
                    mujoco.joint_names.push_back(names[i].as<std::string>());
                }
            }
            if (auto names = mj["actuator_names"]) {
                mujoco.actuator_names.clear();
                for (size_t i = 0; i < names.size(); ++i) {
                    mujoco.actuator_names.push_back(
                        names[i].as<std::string>());
                }
            }
        }

        if (auto remote = m["remote_mujoco"]) {
            auto& settings = cfg.executor.remote_mujoco;
            settings.host = remote["host"].as<std::string>(settings.host);
            settings.port = remote["port"].as<int>(settings.port);
            settings.timeout_ms =
                remote["timeout_ms"].as<int>(settings.timeout_ms);
        }

        cfg.executor.ik_max_trials = m["ik_max_trials"].as<int>(cfg.executor.ik_max_trials);
        cfg.executor.wrist_yaw_scale = m["wrist_yaw_scale"].as<float>(cfg.executor.wrist_yaw_scale);

        if (auto jc = m["joint_constraints"]) {
            cfg.executor.joint_constraints.clear();
            for (size_t i = 0; i < jc.size(); ++i) {
                JointConstraint c;
                c.joint_index = jc[i]["joint"].as<int>(-1);
                c.min_rad = jc[i]["min"].as<float>(0.0f);
                c.max_rad = jc[i]["max"].as<float>(0.0f);
                if (c.joint_index >= 0) {
                    cfg.executor.joint_constraints.push_back(c);
                }
            }
        }
        if (auto jl = m["joint_limits"]) {
            if (!jl.IsSequence() || jl.size() == 0) {
                throw std::runtime_error(
                    "manipulator.joint_limits must be a non-empty sequence");
            }
            cfg.executor.joint_limits.clear();
            std::unordered_set<int> configured_joints;
            for (size_t i = 0; i < jl.size(); ++i) {
                if (!jl[i].IsMap() || !jl[i]["joint"] ||
                    !jl[i]["min"] || !jl[i]["max"]) {
                    throw std::runtime_error(
                        "manipulator.joint_limits[" + std::to_string(i) +
                        "] must contain joint, min and max");
                }
                JointConstraint limit;
                limit.joint_index = jl[i]["joint"].as<int>(-1);
                limit.min_rad = jl[i]["min"].as<float>(0.0f);
                limit.max_rad = jl[i]["max"].as<float>(0.0f);
                if (limit.joint_index < 0 ||
                    limit.min_rad > limit.max_rad) {
                    throw std::runtime_error(
                        "manipulator.joint_limits[" + std::to_string(i) +
                        "] has an invalid joint or range");
                }
                if (!configured_joints.insert(limit.joint_index).second) {
                    throw std::runtime_error(
                        "manipulator.joint_limits contains duplicate joint " +
                        std::to_string(limit.joint_index));
                }
                cfg.executor.joint_limits.push_back(limit);
            }
        }

        // 碰撞避免配置
        if (auto ca = m["collision_avoidance"]) {
            auto& cac = cfg.executor.collision_avoidance;
            cac.enabled = ca["enabled"].as<bool>(true);
            if (auto bz = ca["base_danger_zone"]) {
                if (bz.size() >= 2) {
                    cac.base_danger_min = bz[0].as<float>(-1.480f);
                    cac.base_danger_max = bz[1].as<float>(1.480f);
                }
            }
            cac.shoulder_threshold = ca["shoulder_threshold"].as<float>(-0.334f);
            cac.base_safe_margin = ca["base_safe_margin"].as<float>(0.1f);
        }
    }

    // Place
    if (auto p = root["place"]) {
        if (auto pj = p["place_joints"]) {
            cfg.executor.place_joints.clear();
            for (size_t i = 0; i < pj.size(); i++)
                cfg.executor.place_joints.push_back(pj[i].as<float>());
        }
        cfg.executor.place_release_open =
            p["release_open"].as<float>(cfg.executor.place_release_open);
    }

    const auto validate_joint_pose =
        [&cfg](const char* name, const std::vector<float>& joints) {
            for (const JointConstraint& limit :
                cfg.executor.joint_limits) {
                if (limit.joint_index < 0 ||
                    limit.joint_index >= static_cast<int>(joints.size()) ||
                    limit.min_rad > limit.max_rad) {
                    throw std::runtime_error(
                        "manipulator.joint_limits is invalid for " +
                        std::string(name));
                }
                const float value = joints[limit.joint_index];
                if (value < limit.min_rad || value > limit.max_rad) {
                    std::ostringstream error;
                    error << name << " joint " << limit.joint_index
                            << " target " << value
                            << " is outside configured range ["
                            << limit.min_rad << ", " << limit.max_rad << "]";
                    throw std::runtime_error(error.str());
                }
            }
        };
    validate_joint_pose(
        "manipulator.home_joints", cfg.executor.home_joints);
    validate_joint_pose(
        "manipulator.observe_joints", cfg.executor.observe_joints);
    validate_joint_pose(
        "manipulator.side_ready_joints", cfg.executor.side_ready_joints);
    validate_joint_pose(
        "place.place_joints", cfg.executor.place_joints);

    // Mobile base alignment
    if (auto base = root["mobile_base"]) {
        cfg.mobile_base.enabled =
            base["enabled"].as<bool>(cfg.mobile_base.enabled);
        cfg.mobile_base.driver =
            base["driver"].as<std::string>(cfg.mobile_base.driver);
        cfg.mobile_base.dev_path =
            base["dev_path"].as<std::string>(cfg.mobile_base.dev_path);
        cfg.mobile_base.baud =
            base["baud"].as<int>(cfg.mobile_base.baud);
        cfg.mobile_base.ctrl_dev =
            base["ctrl_dev"].as<std::string>(cfg.mobile_base.ctrl_dev);
        cfg.mobile_base.data_dev =
            base["data_dev"].as<std::string>(cfg.mobile_base.data_dev);
        cfg.mobile_base.service_name =
            base["service_name"].as<std::string>(cfg.mobile_base.service_name);
        cfg.mobile_base.wheel_diameter =
            base["wheel_diameter"].as<float>(cfg.mobile_base.wheel_diameter);
        cfg.mobile_base.wheel_base =
            base["wheel_base"].as<float>(cfg.mobile_base.wheel_base);
        cfg.mobile_base.wheel_track =
            base["wheel_track"].as<float>(cfg.mobile_base.wheel_track);
        cfg.mobile_base.left_wheel_gain =
            base["left_wheel_gain"].as<float>(
                cfg.mobile_base.left_wheel_gain);
        cfg.mobile_base.max_speed =
            base["max_speed"].as<float>(cfg.mobile_base.max_speed);
        cfg.mobile_base.max_angular =
            base["max_angular"].as<float>(cfg.mobile_base.max_angular);
        cfg.mobile_base.reduction_ratio =
            base["reduction_ratio"].as<float>(
                cfg.mobile_base.reduction_ratio);
        cfg.mobile_base.ff_factor =
            base["ff_factor"].as<float>(cfg.mobile_base.ff_factor);
        cfg.mobile_base.pid_kp =
            base["pid_kp"].as<float>(cfg.mobile_base.pid_kp);
        cfg.mobile_base.pid_ki =
            base["pid_ki"].as<float>(cfg.mobile_base.pid_ki);
        cfg.mobile_base.pid_kd =
            base["pid_kd"].as<float>(cfg.mobile_base.pid_kd);
        cfg.mobile_base.cfg_send_on_startup =
            base["cfg_send_on_startup"].as<bool>(
                cfg.mobile_base.cfg_send_on_startup);
        cfg.mobile_base.feedback_enable =
            base["feedback_enable"].as<bool>(
                cfg.mobile_base.feedback_enable);
        cfg.mobile_base.target_x =
            base["target_x"].as<float>(cfg.mobile_base.target_x);
        cfg.mobile_base.x_tolerance =
            base["x_tolerance"].as<float>(cfg.mobile_base.x_tolerance);
        cfg.mobile_base.x_hysteresis =
            base["x_hysteresis"].as<float>(
                cfg.mobile_base.x_hysteresis);
        cfg.mobile_base.y_tolerance =
            base["y_tolerance"].as<float>(cfg.mobile_base.y_tolerance);
        cfg.mobile_base.y_hysteresis =
            base["y_hysteresis"].as<float>(
                cfg.mobile_base.y_hysteresis);
        cfg.mobile_base.max_step_m =
            base["max_step_m"].as<float>(cfg.mobile_base.max_step_m);
        cfg.mobile_base.linear_speed =
            base["linear_speed"].as<float>(cfg.mobile_base.linear_speed);
        cfg.mobile_base.angular_speed =
            base["angular_speed"].as<float>(cfg.mobile_base.angular_speed);
        cfg.mobile_base.yaw_gain =
            base["yaw_gain"].as<float>(cfg.mobile_base.yaw_gain);
        cfg.mobile_base.min_cmd_duration_ms =
            base["min_cmd_duration_ms"].as<int>(
                cfg.mobile_base.min_cmd_duration_ms);
        cfg.mobile_base.min_rotation_duration_ms =
            base["min_rotation_duration_ms"].as<int>(
                cfg.mobile_base.min_rotation_duration_ms);
        cfg.mobile_base.max_cmd_duration_ms =
            base["max_cmd_duration_ms"].as<int>(
                cfg.mobile_base.max_cmd_duration_ms);
        cfg.mobile_base.settle_ms =
            base["settle_ms"].as<int>(cfg.mobile_base.settle_ms);
        cfg.mobile_base.max_align_attempts =
            base["max_align_attempts"].as<int>(
                cfg.mobile_base.max_align_attempts);
        cfg.mobile_base.min_progress_m =
            base["min_progress_m"].as<float>(
                cfg.mobile_base.min_progress_m);
        cfg.mobile_base.min_progress_ratio =
            base["min_progress_ratio"].as<float>(
                cfg.mobile_base.min_progress_ratio);
        cfg.mobile_base.min_progress_floor_m =
            base["min_progress_floor_m"].as<float>(
                cfg.mobile_base.min_progress_floor_m);
        cfg.mobile_base.max_visual_regression_m =
            base["max_visual_regression_m"].as<float>(
                cfg.mobile_base.max_visual_regression_m);
        cfg.mobile_base.max_total_travel_m =
            base["max_total_travel_m"].as<float>(
                cfg.mobile_base.max_total_travel_m);
        cfg.mobile_base.odom_min_translation_m =
            base["odom_min_translation_m"].as<float>(
                cfg.mobile_base.odom_min_translation_m);
        cfg.mobile_base.odom_min_rotation_rad =
            base["odom_min_rotation_rad"].as<float>(
                cfg.mobile_base.odom_min_rotation_rad);
        cfg.mobile_base.odom_min_command_ratio =
            base["odom_min_command_ratio"].as<float>(
                cfg.mobile_base.odom_min_command_ratio);
        cfg.mobile_base.max_direction_reversals =
            base["max_direction_reversals"].as<int>(
                cfg.mobile_base.max_direction_reversals);
    }

    // Timing between pipeline/executor stages
    if (auto timing = root["timing"]) {
        auto& t = cfg.executor.timing;
        t.observe_settle_ms =
            timing["observe_settle_ms"].as<int>(t.observe_settle_ms);
        t.observe_gripper_close_wait_ms =
            timing["observe_gripper_close_wait_ms"].as<int>(
                t.observe_gripper_close_wait_ms);
        t.pre_grasp_settle_ms =
            timing["pre_grasp_settle_ms"].as<int>(t.pre_grasp_settle_ms);
        t.gripper_open_wait_ms =
            timing["gripper_open_wait_ms"].as<int>(t.gripper_open_wait_ms);
        t.grasp_settle_ms =
            timing["grasp_settle_ms"].as<int>(t.grasp_settle_ms);
        t.gripper_close_wait_ms =
            timing["gripper_close_wait_ms"].as<int>(t.gripper_close_wait_ms);
        t.grasp_check_count = std::max(
            1, timing["grasp_check_count"].as<int>(t.grasp_check_count));
        t.grasp_check_interval_ms = std::max(
            1, timing["grasp_check_interval_ms"].as<int>(
                t.grasp_check_interval_ms));
        t.post_lift_settle_ms = std::max(
            0, timing["post_lift_settle_ms"].as<int>(
                t.post_lift_settle_ms));
        t.place_settle_ms =
            timing["place_settle_ms"].as<int>(t.place_settle_ms);
        t.release_wait_ms =
            timing["release_wait_ms"].as<int>(t.release_wait_ms);
        t.home_gripper_close_wait_ms =
            timing["home_gripper_close_wait_ms"].as<int>(
                t.home_gripper_close_wait_ms);
    }

    // Voice command interface (ASR text integration)
    if (auto voice = root["voice"]) {
        cfg.voice.input_topic =
            voice["input_topic"].as<std::string>(cfg.voice.input_topic);
        cfg.voice.status_topic =
            voice["status_topic"].as<std::string>(cfg.voice.status_topic);
        cfg.voice.node_name =
            voice["node_name"].as<std::string>(cfg.voice.node_name);
        cfg.voice.asr_model = voice["asr_model"].as<std::string>("");
        if (auto words = voice["trigger_words"]) {
            cfg.voice.trigger_words.clear();
            for (size_t i = 0; i < words.size(); i++)
                cfg.voice.trigger_words.push_back(words[i].as<std::string>());
        }
        cfg.voice.split_command_timeout_ms =
            voice["split_command_timeout_ms"].as<int>(
                cfg.voice.split_command_timeout_ms);
        if (auto asr = voice["asr"]) {
            cfg.voice.asr_device =
                asr["device"].as<int>(cfg.voice.asr_device);
            cfg.voice.asr_rate =
                asr["rate"].as<int>(cfg.voice.asr_rate);
            cfg.voice.asr_channels =
                asr["channels"].as<int>(cfg.voice.asr_channels);
            cfg.voice.vad_trigger_threshold =
                asr["vad_trigger_threshold"].as<float>(
                    cfg.voice.vad_trigger_threshold);
            cfg.voice.vad_stop_threshold =
                asr["vad_stop_threshold"].as<float>(
                    cfg.voice.vad_stop_threshold);
            cfg.voice.vad_min_speech_duration_ms =
                asr["vad_min_speech_duration_ms"].as<int>(
                    cfg.voice.vad_min_speech_duration_ms);
        }
        if (auto tts = voice["tts"]) {
            cfg.voice.tts_enabled =
                tts["enabled"].as<bool>(cfg.voice.tts_enabled);
            cfg.voice.tts_engine =
                tts["engine"].as<std::string>(cfg.voice.tts_engine);
            cfg.voice.tts_playback_device =
                tts["playback_device"].as<int>(
                    cfg.voice.tts_playback_device);
            cfg.voice.tts_playback_rate =
                tts["playback_rate"].as<int>(cfg.voice.tts_playback_rate);
            cfg.voice.tts_channels =
                tts["channels"].as<int>(cfg.voice.tts_channels);
            cfg.voice.tts_speed =
                tts["speed"].as<float>(cfg.voice.tts_speed);
            cfg.voice.tts_volume =
                tts["volume"].as<int>(cfg.voice.tts_volume);
            cfg.voice.tts_speak_all_states =
                tts["speak_all_states"].as<bool>(
                    cfg.voice.tts_speak_all_states);
        }
        if (auto words = voice["cancel_words"]) {
            cfg.voice.cancel_words.clear();
            for (size_t i = 0; i < words.size(); i++)
                cfg.voice.cancel_words.push_back(words[i].as<std::string>());
        }
        if (auto words = voice["home_words"]) {
            cfg.voice.home_words.clear();
            for (size_t i = 0; i < words.size(); i++)
                cfg.voice.home_words.push_back(words[i].as<std::string>());
        }
        if (auto aliases = voice["target_aliases"]) {
            cfg.voice.target_aliases.clear();
            for (auto it = aliases.begin(); it != aliases.end(); ++it) {
                cfg.voice.target_aliases[it->first.as<std::string>()] =
                    it->second.as<std::string>();
            }
        }
        cfg.target_missing_frames =
            voice["target_missing_frames"].as<int>(cfg.target_missing_frames);
    }

    // Debug artifacts
    if (auto debug = root["debug"]) {
        cfg.save_debug_data =
            debug["save_grasp_debug"].as<bool>(cfg.save_debug_data);
        cfg.debug_output_dir =
            debug["output_dir"].as<std::string>(cfg.debug_output_dir);
    }

    // Logging
    if (auto log = root["logging"]) {
        if (auto perf = log["performance"]) {
            cfg.performance_log_enabled = perf["enabled"].as<bool>(false);
        }
    }
    cfg.executor.performance_log_enabled = cfg.performance_log_enabled;

    const auto validate_normalized = [](const char* name, float value) {
        if (value < 0.0f || value > 1.0f) {
            throw std::runtime_error(std::string(name) +
                                    " must be in [0, 1]");
        }
    };
    validate_normalized("grasp.top.gripper_open",
                        cfg.executor.gripper_open);
    validate_normalized("grasp.gripper_effort", cfg.executor.gripper_effort);
    validate_normalized("place.release_open",
                        cfg.executor.place_release_open);
    validate_normalized(
        "detection.min_confidence", cfg.detector.min_confidence);
    if (cfg.executor.gripper_empty_position_margin < 0.0f) {
        throw std::runtime_error(
            "grasp.gripper_empty_position_margin must be non-negative");
    }
    if (cfg.camera.type == "remote_mujoco" &&
        cfg.executor.manip_driver == "remote_mujoco" &&
        (cfg.camera.remote_mujoco.host !=
            cfg.executor.remote_mujoco.host ||
            cfg.camera.remote_mujoco.port !=
            cfg.executor.remote_mujoco.port)) {
        throw std::runtime_error(
            "remote_mujoco camera and executor must use the same endpoint");
    }
    if (cfg.mobile_base.odom_min_translation_m < 0.0f ||
        cfg.mobile_base.odom_min_rotation_rad < 0.0f ||
        cfg.mobile_base.odom_min_command_ratio < 0.0f ||
        cfg.mobile_base.max_direction_reversals < 0) {
        throw std::runtime_error(
            "mobile_base odometry thresholds must be non-negative");
    }

    return cfg;
}

int main(int argc, char* argv[]) {
    std::string config_path;
    std::string target_name;
    std::string voice_command;
    std::string voice_topic;
    std::string status_topic;
    bool auto_loop = false;
    bool step_mode = false;
    bool plan_only = false;
    bool voice_mode = false;
    bool tts_mode = false;
    bool voice_stdin = false;
    bool status_stdout = false;
    bool validate_config_only = false;
    std::string remote_host;
    int remote_port = 0;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--target" && i + 1 < argc) {
            target_name = argv[++i];
        } else if (arg == "--voice-command" && i + 1 < argc) {
            voice_command = argv[++i];
        } else if (arg == "--voice") {
            voice_mode = true;
        } else if (arg == "--tts") {
            tts_mode = true;
        } else if (arg == "--voice-stdin") {
            voice_stdin = true;
        } else if (arg == "--status-stdout") {
            status_stdout = true;
        } else if (arg == "--voice-topic" && i + 1 < argc) {
            voice_topic = argv[++i];
        } else if (arg == "--status-topic" && i + 1 < argc) {
            status_topic = argv[++i];
        } else if (arg == "--loop") {
            auto_loop = true;
        } else if (arg == "--step") {
            step_mode = true;
        } else if (arg == "--plan-only") {
            plan_only = true;
        } else if (arg == "--validate-config") {
            validate_config_only = true;
        } else if (arg == "--remote-host" && i + 1 < argc) {
            remote_host = argv[++i];
        } else if (arg == "--remote-port" && i + 1 < argc) {
            remote_port = std::atoi(argv[++i]);
            if (remote_port < 1 || remote_port > 65535) {
                std::cerr << "Error: --remote-port must be in [1, 65535]"
                            << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Error: unknown or incomplete option: " << arg
                        << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    if (config_path.empty()) {
        std::cerr << "Error: --config is required\n" << std::endl;
        PrintUsage(argv[0]);
        return 1;
    }
    if (voice_stdin && step_mode) {
        std::cerr << "Error: --voice-stdin cannot be used with --step because "
                    "both read from stdin." << std::endl;
        return 1;
    }

    // 注册信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    std::cout << "=== perceptive_grasp ===" << std::endl;
    std::cout << "Config: " << config_path << std::endl;
    if (!target_name.empty())
        std::cout << "Target: " << target_name << std::endl;
    if (!voice_command.empty())
        std::cout << "Voice command: " << voice_command << std::endl;
    if (voice_mode) std::cout << "Mode: voice" << std::endl;
    if (tts_mode) std::cout << "Mode: tts-status" << std::endl;
    if (voice_stdin) std::cout << "Mode: voice-stdin" << std::endl;
    if (status_stdout) std::cout << "Mode: status-stdout" << std::endl;
    if (!voice_topic.empty()) std::cout << "Voice topic: " << voice_topic << std::endl;
    if (!status_topic.empty()) std::cout << "Status topic: " << status_topic << std::endl;
    if (auto_loop && voice_mode) {
        std::cout << "Mode: auto-loop requested (ignored in voice mode)"
                    << std::endl;
    } else if (auto_loop) {
        std::cout << "Mode: auto-loop" << std::endl;
    }
    if (step_mode) std::cout << "Mode: step (pause before each stage)" << std::endl;
    if (plan_only) std::cout << "Mode: plan-only (no motion commands)" << std::endl;
    std::cout << std::endl;

    // 加载配置
    PipelineConfig cfg;
    try {
        cfg = LoadConfig(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return 1;
    }
    cfg.auto_loop = auto_loop;
    cfg.step_mode = step_mode;
    cfg.plan_only = plan_only;
    if (plan_only) cfg.mobile_base.enabled = false;
    cfg.voice.enabled = voice_mode || voice_stdin;
    if (tts_mode || status_stdout) cfg.voice.tts_enabled = true;
    if (!voice_topic.empty()) cfg.voice.input_topic = voice_topic;
    if (!status_topic.empty()) cfg.voice.status_topic = status_topic;
    if (!remote_host.empty()) {
        cfg.camera.remote_mujoco.host = remote_host;
        cfg.executor.remote_mujoco.host = remote_host;
    }
    if (remote_port > 0) {
        cfg.camera.remote_mujoco.port = remote_port;
        cfg.executor.remote_mujoco.port = remote_port;
    }
    if (cfg.voice.enabled && cfg.auto_loop) {
        std::cout << "[Main] --loop ignored because voice mode waits for "
                    << "explicit commands" << std::endl;
        cfg.auto_loop = false;
    }

    // 解析相对路径: 以配置文件所在目录为基准
    fs::path config_dir = fs::path(config_path).parent_path();
    if (config_dir.empty()) config_dir = ".";

    ResolveConfigPath(config_dir, &cfg.executor.urdf_path);
    ResolveConfigPath(config_dir, &cfg.detector.config_path);
    ResolveConfigPath(config_dir, &cfg.debug_output_dir);
    ResolveConfigPath(config_dir, &cfg.camera.spacemit_las2.model_path);
    ResolveConfigPath(config_dir, &cfg.camera.spacemit_las2.calib_path);
    ResolveConfigPath(config_dir, &cfg.camera.mujoco.xml_path);
    ResolveConfigPath(config_dir, &cfg.executor.mujoco.xml_path);

    if (validate_config_only) {
        std::cout << "[Config] valid" << std::endl;
        return 0;
    }

    // 创建并初始化 Pipeline
    g_pipeline = std::make_unique<GraspPipeline>(cfg);

#ifdef ENABLE_ROS2_VOICE
    if (voice_mode || tts_mode) {
        std::string input_topic = voice_mode ? cfg.voice.input_topic : "";
        std::string output_topic =
            tts_mode ? cfg.voice.status_topic : "";
        VoiceCommandListener::CommandCallback command_callback;
        if (voice_mode) {
            command_callback = [](const std::string& command_text) {
                if (!TriggerVoiceCommand(command_text)) {
                    std::cerr << "[VoiceROS] Command ignored: "
                                << command_text << std::endl;
                }
            };
        }
        g_voice_listener = std::make_unique<VoiceCommandListener>(
            cfg.voice.node_name, input_topic, command_callback, output_topic);
        if (!g_voice_listener->Start()) {
            std::cerr << "Failed to start ROS2 voice/status bridge."
                        << std::endl;
            CleanupRuntime();
            return 1;
        }
    }
#else
    if (voice_mode || tts_mode) {
        std::cerr << "ROS2 voice/TTS requested, but this binary was built "
                    "without ROS2 support. Use --voice-stdin and "
                    "--status-stdout for local non-ROS voice I/O." << std::endl;
        return 1;
    }
#endif

    if (voice_stdin) {
        g_local_voice_thread = std::thread([]() {
            std::string command_text;
            while (std::getline(std::cin, command_text)) {
                if (command_text.empty()) continue;
                if (!TriggerVoiceCommand(command_text)) {
                    std::cerr << "[VoiceLocal] Command ignored: "
                                << command_text << std::endl;
                }
            }
            std::cout << "[VoiceLocal] stdin closed" << std::endl;
        });
    }

    // 状态回调可接 ROS2 TTS topic，也可输出到 stdout 供本地语音桥读取。
    g_pipeline->SetCallback([status_stdout](PipelineState state,
                                            const std::string& msg) {
        if (msg.empty()) return;
        if (status_stdout) {
            // A single write keeps third-party worker logs from splitting the
            // machine-readable status event between its prefix and payload.
            WriteStatusEvent(state, msg);
        }
#ifdef ENABLE_ROS2_VOICE
        if (g_voice_listener) {
            g_voice_listener->PublishStatus(MakeStatusEvent(state, msg));
        }
#else
        (void)state;
#endif
    });

    if (!g_pipeline->Init()) {
        std::cerr << "Pipeline initialization failed!" << std::endl;
        CleanupRuntime();
        return 1;
    }

    // 触发抓取
    if (!voice_command.empty()) {
        if (!TriggerVoiceCommand(voice_command)) {
            std::cerr << "Failed to trigger voice command." << std::endl;
            return 1;
        }
    } else if (!target_name.empty()) {
        g_pipeline->TriggerGrasp(target_name);
    } else if (cfg.voice.enabled) {
        if (voice_stdin) {
            std::cout << "[Voice] Waiting for command on stdin" << std::endl;
        } else {
            std::cout << "[Voice] Waiting for command on topic: "
                        << cfg.voice.input_topic << std::endl;
        }
    } else if (target_name.empty()) {
        g_pipeline->TriggerGrasp();
    }

    // 主循环
    g_pipeline->Run([]() {
        return g_shutdown_signal != 0;
    });
    const int exit_code = g_pipeline->GetState() == PipelineState::ERROR
        ? 1
        : 0;

    if (voice_stdin) {
        CleanupRuntime(false);
        std::cout << std::flush;
        std::cerr << std::flush;
        std::_Exit(exit_code);
    }

    CleanupRuntime();

    if (cfg.camera.type == "spacemit_las2") {
        // All application resources have already been released above. The
        // LAS2/OpenCL runtime can block in process-wide static
        // destructors, so bypass only that third-party teardown path.
        std::cout << std::flush;
        std::cerr << std::flush;
        std::_Exit(exit_code);
    }

    return exit_code;
}
