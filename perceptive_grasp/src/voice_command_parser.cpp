/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file voice_command_parser.cpp
    * @brief ASR text command parser implementation.
    */

#include "voice_command_parser.h"

#include <cctype>

namespace perceptive_grasp {

namespace {

std::string TrimAscii(const std::string& text) {
    auto begin = text.begin();
    while (begin != text.end() &&
            std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }

    auto end = text.end();
    while (end != begin &&
            std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    return std::string(begin, end);
}

std::string ToLowerAscii(std::string text) {
    for (char& ch : text) {
        ch = static_cast<char>(
            std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

void EraseAll(std::string& text, const std::string& token) {
    if (token.empty()) return;
    size_t pos = 0;
    while ((pos = text.find(token, pos)) != std::string::npos) {
        text.erase(pos, token.size());
    }
}

std::optional<std::string> MatchKnownTarget(
    const std::string& normalized,
    const VoiceCommandConfig& config) {
    if (normalized.empty()) return "";

    for (const auto& [alias, label] : config.target_aliases) {
        if (normalized == VoiceCommandParser::NormalizeText(alias)) {
            return label;
        }
        if (normalized == VoiceCommandParser::NormalizeText(label)) {
            return label;
        }
    }

    for (const auto& [alias, label] : config.target_aliases) {
        const std::string alias_key =
            VoiceCommandParser::NormalizeText(alias);
        const std::string label_key =
            VoiceCommandParser::NormalizeText(label);
        if (!alias_key.empty() &&
            normalized.find(alias_key) != std::string::npos) {
            return label;
        }
        if (!label_key.empty() &&
            normalized.find(label_key) != std::string::npos) {
            return label;
        }
    }

    return std::nullopt;
}

}  // namespace

VoiceCommandParser::VoiceCommandParser(const VoiceCommandConfig& config)
    : config_(config) {}

std::string VoiceCommandParser::NormalizeText(const std::string& text) {
    std::string out = ToLowerAscii(TrimAscii(text));
    const std::vector<std::string> ignored = {
        " ", "\t", "\r", "\n",
        ".", ",", "!", "?", ";", ":", "\"", "'",
        "。", "，", "！", "？", "、", "；", "：",
        "“", "”", "‘", "’",
    };
    for (const auto& token : ignored) {
        EraseAll(out, token);
    }
    return out;
}

std::optional<std::string> VoiceCommandParser::ParseTarget(
    const std::string& text) const {
    std::string normalized = NormalizeText(text);
    if (normalized.empty()) return std::nullopt;

    size_t trigger_pos = std::string::npos;
    size_t trigger_len = 0;
    for (const auto& raw_word : config_.trigger_words) {
        std::string word = NormalizeText(raw_word);
        if (word.empty()) continue;
        size_t pos = normalized.find(word);
        if (pos == std::string::npos) continue;
        if (trigger_pos == std::string::npos || pos < trigger_pos) {
            trigger_pos = pos;
            trigger_len = word.size();
        }
    }

    if (trigger_pos == std::string::npos) return std::nullopt;

    std::string tail = normalized.substr(trigger_pos + trigger_len);
    if (tail.empty()) return "";

    return ResolveTargetText(tail);
}

std::optional<std::string> VoiceCommandParser::ResolveTargetText(
    const std::string& text) const {
    const std::string normalized = NormalizeText(text);
    if (normalized.empty()) return "";

    auto known = ResolveKnownTargetText(normalized);
    if (known.has_value()) return known;

    return normalized;
}

std::optional<std::string> VoiceCommandParser::ResolveKnownTargetText(
    const std::string& text) const {
    return MatchKnownTarget(NormalizeText(text), config_);
}

bool VoiceCommandParser::IsCancelCommand(const std::string& text) const {
    std::string normalized = NormalizeText(text);
    if (normalized.empty()) return false;

    for (const auto& raw_word : config_.cancel_words) {
        const std::string word = NormalizeText(raw_word);
        if (!word.empty() && normalized.find(word) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool VoiceCommandParser::IsHomeCommand(const std::string& text) const {
    std::string normalized = NormalizeText(text);
    if (normalized.empty()) return false;

    for (const auto& raw_word : config_.home_words) {
        const std::string word = NormalizeText(raw_word);
        if (!word.empty() && normalized.find(word) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace perceptive_grasp
