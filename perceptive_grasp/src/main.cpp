/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file main.cpp
    * @brief Perceptive Grasp - 视觉抓取 Demo 入口
    *
    * 用法:
    *   ./perceptive_grasp --config config/grasp_pipeline.yaml
    *   ./perceptive_grasp --config config/grasp_pipeline.yaml --target apple
    *   ./perceptive_grasp --config config/grasp_pipeline.yaml --loop
    */

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <unistd.h>

namespace fs = std::filesystem;

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

static const char* PipelineStateName(PipelineState state) {
    switch (state) {
        case PipelineState::IDLE: return "IDLE";
        case PipelineState::OBSERVING: return "OBSERVING";
        case PipelineState::DETECTING: return "DETECTING";
        case PipelineState::PLANNING: return "PLANNING";
        case PipelineState::APPROACHING: return "APPROACHING";
        case PipelineState::GRASPING: return "GRASPING";
        case PipelineState::LIFTING: return "LIFTING";
        case PipelineState::PLACING: return "PLACING";
        case PipelineState::HOMING: return "HOMING";
        case PipelineState::DONE: return "DONE";
        case PipelineState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
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

static void SignalHandler(int sig) {
    const char msg[] = "\n[Main] Signal received, exiting...\n";
    (void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
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
                << "  --loop            Auto-loop grasp mode\n"
                << "  --step            Step mode: pause before each stage\n"
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

static PipelineConfig LoadConfig(const std::string& config_path) {
    PipelineConfig cfg;

    YAML::Node root = YAML::LoadFile(config_path);

    // Camera
    if (auto cam = root["camera"]) {
        cfg.camera.width = cam["width"].as<int>(640);
        cfg.camera.height = cam["height"].as<int>(480);
        cfg.camera.fps = cam["fps"].as<int>(30);
        cfg.camera.align_depth = cam["align_depth"].as<bool>(true);
        if (auto df = cam["depth_filter"]) {
            cfg.camera.spatial_filter = df["spatial"].as<bool>(true);
            cfg.camera.temporal_filter = df["temporal"].as<bool>(true);
            cfg.camera.hole_filling = df["hole_filling"].as<bool>(true);
        }
    }

    // Detection
    if (auto det = root["detection"]) {
        cfg.detector.config_path = det["config_path"].as<std::string>("");
        cfg.detector.min_confidence = det["min_confidence"].as<float>(0.5f);
        cfg.detector.min_area = det["min_area"].as<float>(1000.0f);
        if (auto labels = det["target_labels"]) {
            for (size_t i = 0; i < labels.size(); i++) {
                cfg.detector.target_labels.push_back(labels[i].as<int>());
            }
        }
    }

    // Calibration
    if (auto cal = root["calibration"]) {
        if (auto tbc = cal["T_base_camera"]) {
            if (auto t = tbc["translation"]) {
                for (int i = 0; i < 3; i++)
                    cfg.planner.t_base_camera[i] = t[i].as<float>(0.0f);
            }
            if (auto r = tbc["rotation"]) {
                for (int i = 0; i < 3; i++)
                    cfg.planner.r_base_camera[i] = r[i].as<float>(0.0f);
            }
        }
    }

    // Grasp strategy
    if (auto g = root["grasp"]) {
        cfg.planner.approach_height = g["approach_height"].as<float>(0.10f);
        cfg.planner.grasp_depth = g["grasp_depth"].as<float>(0.02f);
        cfg.planner.gripper_offset = g["gripper_offset"].as<float>(0.015f);
        cfg.grasp_point_x_ratio = g["grasp_point_x_ratio"].as<float>(0.0f);
        cfg.executor.gripper_open = g["gripper_open"].as<float>(1.0f);
        cfg.executor.gripper_effort = g["gripper_effort"].as<float>(0.8f);
        cfg.executor.gripper_hold_load_threshold =
            g["gripper_hold_load_threshold"].as<float>(
                cfg.executor.gripper_hold_load_threshold);
        cfg.executor.gripper_timeout_ms =
            g["gripper_timeout_ms"].as<int>(
                cfg.executor.gripper_timeout_ms);
        if (auto ws = g["workspace"]) {
            cfg.planner.workspace.x_min = ws["x_min"].as<float>(-0.15f);
            cfg.planner.workspace.x_max = ws["x_max"].as<float>(0.25f);
            cfg.planner.workspace.y_min = ws["y_min"].as<float>(-0.15f);
            cfg.planner.workspace.y_max = ws["y_max"].as<float>(0.15f);
            cfg.planner.workspace.z_min = ws["z_min"].as<float>(-0.05f);
            cfg.planner.workspace.z_max = ws["z_max"].as<float>(0.20f);
        }
    }

    // Orientation (自动夹爪方向对齐)
    if (auto ori = root["orientation"]) {
        cfg.auto_orient = ori["enabled"].as<bool>(true);
        cfg.orientation.aspect_ratio_threshold =
            ori["aspect_ratio_threshold"].as<float>(1.5f);
        cfg.orientation.camera_yaw_offset =
            ori["camera_yaw_offset"].as<float>(0.0f);
    }

    // Manipulator
    if (auto m = root["manipulator"]) {
        cfg.executor.manip_driver = m["driver"].as<std::string>("so101");
        cfg.executor.uart_device = m["uart_device"].as<std::string>("/dev/ttyACM0");
        cfg.executor.baudrate = m["baudrate"].as<int>(1000000);
        cfg.executor.urdf_path = m["urdf_path"].as<std::string>("");

        // 解析 URDF 相对路径: 相对于配置文件所在目录
        if (!cfg.executor.urdf_path.empty() &&
            cfg.executor.urdf_path[0] != '/') {
            fs::path config_dir = fs::path(config_path).parent_path();
            fs::path resolved = config_dir / cfg.executor.urdf_path;
            if (fs::exists(resolved)) {
                cfg.executor.urdf_path = fs::canonical(resolved).string();
            }
        }

        cfg.executor.base_link = m["base_link"].as<std::string>("base_link");
        cfg.executor.tip_link = m["tip_link"].as<std::string>("Fixed_Jaw");
        cfg.executor.move_speed = m["move_speed"].as<float>(0.5f);
        cfg.executor.line_speed = m["line_speed"].as<float>(0.3f);
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
        t.grasp_check_count =
            timing["grasp_check_count"].as<int>(t.grasp_check_count);
        t.grasp_check_interval_ms =
            timing["grasp_check_interval_ms"].as<int>(
                t.grasp_check_interval_ms);
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
    bool voice_mode = false;
    bool tts_mode = false;
    bool voice_stdin = false;
    bool status_stdout = false;

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

    std::cout << "=== Perceptive Grasp Demo ===" << std::endl;
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
    cfg.voice.enabled = voice_mode || voice_stdin;
    if (tts_mode || status_stdout) cfg.voice.tts_enabled = true;
    if (!voice_topic.empty()) cfg.voice.input_topic = voice_topic;
    if (!status_topic.empty()) cfg.voice.status_topic = status_topic;
    if (cfg.voice.enabled && cfg.auto_loop) {
        std::cout << "[Main] --loop ignored because voice mode waits for "
                    << "explicit commands" << std::endl;
        cfg.auto_loop = false;
    }

    // 解析相对路径: 以配置文件所在目录为基准
    fs::path config_dir = fs::path(config_path).parent_path();
    if (config_dir.empty()) config_dir = ".";

    auto resolve_path = [&](std::string& p) {
        if (!p.empty() && !fs::path(p).is_absolute()) {
            fs::path resolved = fs::weakly_canonical(config_dir / p);
            p = resolved.string();
        }
    };
    resolve_path(cfg.executor.urdf_path);
    resolve_path(cfg.detector.config_path);
    resolve_path(cfg.debug_output_dir);

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

    // 状态回调：可接 ROS2 TTS topic，也可输出到本地 stdout 供非 ROS TTS 桥读取。
    g_pipeline->SetCallback([status_stdout](PipelineState state,
                                            const std::string& msg) {
        if (msg.empty()) return;
        if (status_stdout) {
            std::cout << "VOICE_STATUS\t" << MakeStatusEvent(state, msg)
                        << std::endl;
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
    g_pipeline->Run();

    if (voice_stdin) {
        CleanupRuntime(false);
        std::cout << std::flush;
        std::cerr << std::flush;
        std::_Exit(0);
    }

    CleanupRuntime();

    return 0;
}
