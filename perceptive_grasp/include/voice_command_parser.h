/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file voice_command_parser.h
    * @brief ASR text command parser for grasp targets.
    */

#ifndef VOICE_COMMAND_PARSER_H
#define VOICE_COMMAND_PARSER_H

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace perceptive_grasp {

struct VoiceCommandConfig {
    bool enabled = false;
    std::string input_topic = "asr_text";
    std::string status_topic = "grasp_status_text";
    std::string node_name = "perceptive_grasp_voice";
    std::vector<std::string> trigger_words = {"抓", "拿", "pick", "grab"};
    std::vector<std::string> cancel_words = {
        "停止", "停", "取消", "别抓", "不要抓", "stop", "cancel"
    };
    std::vector<std::string> home_words = {
        "结束", "待命", "休息", "回家", "回home", "回到home",
        "回初始", "回到初始", "end", "home"
    };
    std::unordered_map<std::string, std::string> target_aliases = {
        {"香蕉", "banana"}, {"苹果", "apple"},
        {"红苹果", "apple"}, {"青苹果", "apple"},
        {"橙子", "orange"}, {"桔子", "orange"},
        {"橘子", "orange"}, {"橙", "orange"},
        {"胡萝卜", "carrot"}, {"萝卜", "carrot"},
        {"红萝卜", "carrot"}, {"胡罗卜", "carrot"},
        {"中罗", "carrot"}, {"走罗", "carrot"},
        {"瓶子", "bottle"}, {"水瓶", "bottle"},
        {"矿泉水", "bottle"}, {"杯子", "cup"},
        {"水杯", "cup"}, {"碗", "bowl"},
        {"勺子", "spoon"}, {"叉子", "fork"},
        {"剪刀", "scissors"},
    };
    int split_command_timeout_ms = 5000;
    std::string asr_model;
    int asr_device = -1;
    int asr_rate = 16000;
    int asr_channels = 1;
    float vad_trigger_threshold = 0.4f;
    float vad_stop_threshold = 0.3f;
    int vad_min_speech_duration_ms = 100;
    bool tts_enabled = false;
    std::string tts_engine = "matcha:zh";
    int tts_playback_device = -1;
    int tts_playback_rate = 48000;
    int tts_channels = 1;
    float tts_speed = 1.0f;
    int tts_volume = 80;
    bool tts_speak_all_states = false;
};

class VoiceCommandParser {
public:
    explicit VoiceCommandParser(const VoiceCommandConfig& config);

    /**
    * @brief Parse ASR text such as "抓香蕉" or "grab banana".
    * @return target detector label, empty string for empty target, or nullopt
    *         when no trigger word matched.
    */
    std::optional<std::string> ParseTarget(const std::string& text) const;

    std::optional<std::string> ResolveTargetText(const std::string& text) const;
    std::optional<std::string> ResolveKnownTargetText(const std::string& text) const;

    bool IsCancelCommand(const std::string& text) const;
    bool IsHomeCommand(const std::string& text) const;

    static std::string NormalizeText(const std::string& text);

private:
    const VoiceCommandConfig& config_;
};

}  // namespace perceptive_grasp

#endif  // VOICE_COMMAND_PARSER_H
