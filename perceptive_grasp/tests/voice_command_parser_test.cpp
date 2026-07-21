/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    */

#include <cassert>
#include <iostream>

#include "voice_command_parser.h"

using perceptive_grasp::VoiceCommandConfig;
using perceptive_grasp::VoiceCommandParser;

int main() {
    VoiceCommandConfig config;
    VoiceCommandParser parser(config);

    const auto banana = parser.ParseTarget("抓香蕉");
    assert(banana.has_value());
    assert(*banana == "banana");

    const auto english = parser.ParseTarget("grab banana");
    assert(english.has_value());
    assert(*english == "banana");

    const auto missing_target = parser.ParseTarget("抓");
    assert(missing_target.has_value());
    assert(missing_target->empty());

    assert(!parser.ParseTarget("抓紧").has_value());
    assert(!parser.ParseTarget("环境噪声").has_value());

    std::cout << "voice_command_parser_test passed" << std::endl;
    return 0;
}
