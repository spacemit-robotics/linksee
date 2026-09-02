/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_protocol.h
 * @brief TCP protocol helpers for the remote MuJoCo simulation backend.
 */

#ifndef REMOTE_MUJOCO_PROTOCOL_H
#define REMOTE_MUJOCO_PROTOCOL_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace perceptive_grasp {
namespace remote_mujoco {

constexpr std::uint32_t kMagic = 0x50474d4a;  // "PGMJ"
constexpr std::uint16_t kVersion = 1;

enum class Command : std::uint16_t {
    GET_FRAME = 1,
    MOVE_TO_OBSERVE = 10,
    MOVE_TO_SIDE_OBSERVE = 11,
    MOVE_TO_HOME = 12,
    MOVE_TO_PRE_GRASP = 13,
    OPEN_GRIPPER = 14,
    MOVE_TO_GRASP = 15,
    CLOSE_GRIPPER_AND_CHECK = 16,
    LIFT_FROM_GRASP = 17,
    VALIDATE_GRASP_POSES = 18,
    SET_SUPPORT_PLANE = 19,
    MOVE_TO_PLACE = 20,
    RELEASE_OBJECT = 21,
    CLOSE_GRIPPER = 22,
    GET_CURRENT_POSE = 23,
    TICK = 24,
    SET_TARGET_LABEL = 25,
    EMERGENCY_STOP = 26,
    RESET_SCENE = 27,
};

struct FramePacket {
    std::int64_t frame_id = 0;
    std::int32_t width = 0;
    std::int32_t height = 0;
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    std::vector<std::uint8_t> color_bgr;
    std::vector<std::uint8_t> depth_u16;
};

struct Response {
    bool ok = false;
    std::int32_t result = 0;
    std::string detail;
    std::vector<std::uint8_t> payload;
};

class BufferWriter {
public:
    void WriteU8(std::uint8_t value);
    void WriteI32(std::int32_t value);
    void WriteI64(std::int64_t value);
    void WriteF32(float value);
    void WriteString(const std::string& value);
    void WriteBytes(const void* data, size_t size);
    const std::vector<std::uint8_t>& Data() const { return data_; }

private:
    std::vector<std::uint8_t> data_;
};

class BufferReader {
public:
    explicit BufferReader(const std::vector<std::uint8_t>& data);
    bool ReadU8(std::uint8_t* value);
    bool ReadI32(std::int32_t* value);
    bool ReadI64(std::int64_t* value);
    bool ReadF32(float* value);
    bool ReadString(std::string* value);
    bool ReadBytes(size_t size, const std::uint8_t** data);
    bool AtEnd() const { return offset_ == data_.size(); }
    size_t Offset() const { return offset_; }

private:
    const std::vector<std::uint8_t>& data_;
    size_t offset_ = 0;
};

bool SendRequest(const std::string& host, int port, int timeout_ms,
                Command command,
                const std::vector<std::uint8_t>& payload,
                Response* response,
                std::string* error);

bool ReadRequest(int fd, Command* command,
                std::vector<std::uint8_t>* payload,
                std::string* error);

bool SendResponse(int fd, bool ok, std::int32_t result,
                const std::string& detail,
                const std::vector<std::uint8_t>& payload,
                std::string* error);

bool DecodeFramePacket(const std::vector<std::uint8_t>& payload,
                    FramePacket* frame,
                    std::string* error);

std::vector<std::uint8_t> EncodeFramePacket(const FramePacket& frame);

}  // namespace remote_mujoco
}  // namespace perceptive_grasp

#endif  // REMOTE_MUJOCO_PROTOCOL_H
