/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_protocol_test.cpp
 * @brief Tests remote MuJoCo rgb-d frame packet validation.
 */

#include "remote_mujoco_protocol.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace remote = perceptive_grasp::remote_mujoco;

namespace {

remote::FramePacket MakeFrame() {
    remote::FramePacket frame;
    frame.frame_id = 42;
    frame.width = 2;
    frame.height = 2;
    frame.fx = 100.0f;
    frame.fy = 101.0f;
    frame.cx = 0.5f;
    frame.cy = 0.5f;
    frame.color_bgr = {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
    };
    frame.depth_u16 = {
        0xE8, 0x03, 0xD0, 0x07,
        0xB8, 0x0B, 0xA0, 0x0F,
    };
    return frame;
}

bool CheckRoundTrip() {
    const remote::FramePacket expected = MakeFrame();
    const std::vector<std::uint8_t> encoded =
        remote::EncodeFramePacket(expected);
    remote::FramePacket decoded;
    std::string error;
    if (!remote::DecodeFramePacket(encoded, &decoded, &error)) {
        std::cerr << "valid frame rejected: " << error << std::endl;
        return false;
    }
    return decoded.frame_id == expected.frame_id &&
            decoded.width == expected.width &&
            decoded.height == expected.height &&
            decoded.color_bgr == expected.color_bgr &&
            decoded.depth_u16 == expected.depth_u16;
}

bool CheckTrailingDataRejected() {
    std::vector<std::uint8_t> encoded =
        remote::EncodeFramePacket(MakeFrame());
    encoded.push_back(0xFF);
    remote::FramePacket decoded;
    std::string error;
    return !remote::DecodeFramePacket(encoded, &decoded, &error) &&
            error.find("dimensions") != std::string::npos;
}

bool CheckDimensionMismatchRejected() {
    remote::FramePacket frame = MakeFrame();
    frame.width = 3;
    const std::vector<std::uint8_t> encoded =
        remote::EncodeFramePacket(frame);
    remote::FramePacket decoded;
    std::string error;
    return !remote::DecodeFramePacket(encoded, &decoded, &error) &&
            error.find("dimensions") != std::string::npos;
}

}  // namespace

int main() {
    if (!CheckRoundTrip()) {
        std::cerr << "frame round-trip test failed" << std::endl;
        return 1;
    }
    if (!CheckTrailingDataRejected()) {
        std::cerr << "trailing frame data was accepted" << std::endl;
        return 1;
    }
    if (!CheckDimensionMismatchRejected()) {
        std::cerr << "mismatched frame dimensions were accepted" << std::endl;
        return 1;
    }
    return 0;
}
