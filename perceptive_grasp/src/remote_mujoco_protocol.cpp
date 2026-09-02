/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file remote_mujoco_protocol.cpp
 * @brief TCP protocol helpers for the remote MuJoCo simulation backend.
 */

#include "remote_mujoco_protocol.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <limits>

namespace perceptive_grasp {
namespace remote_mujoco {
namespace {

constexpr size_t kMaxPayloadSize = 64U * 1024U * 1024U;
constexpr int kServerIoTimeoutMs = 5000;

struct Header {
    std::uint32_t magic = kMagic;
    std::uint16_t version = kVersion;
    std::uint16_t command = 0;
    std::uint32_t payload_size = 0;
};

static std::uint64_t HostToNetwork64(std::uint64_t value) {
    const std::uint32_t high = htonl(static_cast<std::uint32_t>(value >> 32));
    const std::uint32_t low = htonl(static_cast<std::uint32_t>(value));
    return (static_cast<std::uint64_t>(low) << 32) | high;
}

static std::uint64_t NetworkToHost64(std::uint64_t value) {
    const std::uint32_t high = ntohl(static_cast<std::uint32_t>(value >> 32));
    const std::uint32_t low = ntohl(static_cast<std::uint32_t>(value));
    return (static_cast<std::uint64_t>(low) << 32) | high;
}

static bool WaitFd(int fd, short events, int timeout_ms, std::string* error) {
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = events;
    int rc = 0;
    do {
        rc = poll(&pfd, 1, timeout_ms);
    } while (rc < 0 && errno == EINTR);
    if (rc > 0 && (pfd.revents & events) != 0) return true;
    if (error) {
        if (rc == 0) {
            *error = "tcp timeout";
        } else if (rc > 0) {
            *error = "tcp connection closed while waiting for I/O";
        } else {
            *error = "poll failed: " + std::string(std::strerror(errno));
        }
    }
    return false;
}

static bool ReadAll(int fd, void* data, size_t size,
                    int timeout_ms, std::string* error) {
    auto* out = static_cast<std::uint8_t*>(data);
    size_t offset = 0;
    while (offset < size) {
        if (!WaitFd(fd, POLLIN, timeout_ms, error)) return false;
        const ssize_t n = recv(fd, out + offset, size - offset, 0);
        if (n <= 0) {
            if (error) {
                *error = n == 0 ? "tcp peer closed" :
                    ("recv failed: " + std::string(std::strerror(errno)));
            }
            return false;
        }
        offset += static_cast<size_t>(n);
    }
    return true;
}

static bool WriteAll(int fd, const void* data, size_t size,
                    int timeout_ms, std::string* error) {
    const auto* input = static_cast<const std::uint8_t*>(data);
    size_t offset = 0;
    while (offset < size) {
        if (!WaitFd(fd, POLLOUT, timeout_ms, error)) return false;
        const ssize_t n = send(fd, input + offset, size - offset, MSG_NOSIGNAL);
        if (n <= 0) {
            if (error) {
                *error = "send failed: " + std::string(std::strerror(errno));
            }
            return false;
        }
        offset += static_cast<size_t>(n);
    }
    return true;
}

static bool ReadHeader(int fd, Header* header,
                    int timeout_ms, std::string* error) {
    Header wire{};
    if (!ReadAll(fd, &wire, sizeof(wire), timeout_ms, error)) return false;
    header->magic = ntohl(wire.magic);
    header->version = ntohs(wire.version);
    header->command = ntohs(wire.command);
    header->payload_size = ntohl(wire.payload_size);
    if (header->magic != kMagic || header->version != kVersion) {
        if (error) *error = "invalid remote mujoco protocol header";
        return false;
    }
    if (header->payload_size > kMaxPayloadSize) {
        if (error) *error = "remote mujoco payload too large";
        return false;
    }
    return true;
}

static bool WriteHeader(int fd, const Header& header,
                        int timeout_ms, std::string* error) {
    Header wire{};
    wire.magic = htonl(header.magic);
    wire.version = htons(header.version);
    wire.command = htons(header.command);
    wire.payload_size = htonl(header.payload_size);
    return WriteAll(fd, &wire, sizeof(wire), timeout_ms, error);
}

static int ConnectTcp(const std::string& host, int port, int timeout_ms,
        std::string* error) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const std::string service = std::to_string(port);
    const int gai = getaddrinfo(host.c_str(), service.c_str(), &hints, &result);
    if (gai != 0) {
        if (error) *error = "getaddrinfo failed: " + std::string(gai_strerror(gai));
        return -1;
    }

    int fd = -1;
    int last_error = ECONNREFUSED;
    for (addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;
        const int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            last_error = errno;
            close(fd);
            fd = -1;
            continue;
        }

        bool connected = connect(fd, ai->ai_addr, ai->ai_addrlen) == 0;
        if (!connected && errno == EINPROGRESS) {
            std::string wait_error;
            connected = WaitFd(fd, POLLOUT, timeout_ms, &wait_error);
            if (connected) {
                socklen_t length = sizeof(last_error);
                last_error = 0;
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR,
                        &last_error, &length) != 0) {
                    last_error = errno;
                }
                connected = last_error == 0;
            } else if (error) {
                *error = wait_error;
            }
        } else if (!connected) {
            last_error = errno;
        }
        if (connected && fcntl(fd, F_SETFL, flags) == 0) break;
        if (connected) last_error = errno;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(result);
    if (fd < 0 && error) {
        if (error->empty()) {
            *error = "connect failed: " +
                std::string(std::strerror(last_error));
        }
    }
    return fd;
}

}  // namespace

void BufferWriter::WriteU8(std::uint8_t value) {
    data_.push_back(value);
}

void BufferWriter::WriteI32(std::int32_t value) {
    const std::uint32_t encoded = htonl(static_cast<std::uint32_t>(value));
    WriteBytes(&encoded, sizeof(encoded));
}

void BufferWriter::WriteI64(std::int64_t value) {
    const std::uint64_t encoded =
        HostToNetwork64(static_cast<std::uint64_t>(value));
    WriteBytes(&encoded, sizeof(encoded));
}

void BufferWriter::WriteF32(float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "unexpected float");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    bits = htonl(bits);
    WriteBytes(&bits, sizeof(bits));
}

void BufferWriter::WriteString(const std::string& value) {
    WriteI32(static_cast<std::int32_t>(value.size()));
    WriteBytes(value.data(), value.size());
}

void BufferWriter::WriteBytes(const void* data, size_t size) {
    if (size == 0) return;
    const auto* input = static_cast<const std::uint8_t*>(data);
    data_.insert(data_.end(), input, input + size);
}

BufferReader::BufferReader(const std::vector<std::uint8_t>& data)
    : data_(data) {}

bool BufferReader::ReadU8(std::uint8_t* value) {
    if (offset_ + 1 > data_.size()) return false;
    *value = data_[offset_++];
    return true;
}

bool BufferReader::ReadI32(std::int32_t* value) {
    if (offset_ + sizeof(std::uint32_t) > data_.size()) return false;
    std::uint32_t encoded = 0;
    std::memcpy(&encoded, data_.data() + offset_, sizeof(encoded));
    offset_ += sizeof(encoded);
    *value = static_cast<std::int32_t>(ntohl(encoded));
    return true;
}

bool BufferReader::ReadI64(std::int64_t* value) {
    if (offset_ + sizeof(std::uint64_t) > data_.size()) return false;
    std::uint64_t encoded = 0;
    std::memcpy(&encoded, data_.data() + offset_, sizeof(encoded));
    offset_ += sizeof(encoded);
    *value = static_cast<std::int64_t>(NetworkToHost64(encoded));
    return true;
}

bool BufferReader::ReadF32(float* value) {
    if (offset_ + sizeof(std::uint32_t) > data_.size()) return false;
    std::uint32_t bits = 0;
    std::memcpy(&bits, data_.data() + offset_, sizeof(bits));
    offset_ += sizeof(bits);
    bits = ntohl(bits);
    std::memcpy(value, &bits, sizeof(bits));
    return true;
}

bool BufferReader::ReadString(std::string* value) {
    std::int32_t size = 0;
    if (!ReadI32(&size) || size < 0) return false;
    if (offset_ + static_cast<size_t>(size) > data_.size()) return false;
    value->assign(
        reinterpret_cast<const char*>(data_.data() + offset_),
        static_cast<size_t>(size));
    offset_ += static_cast<size_t>(size);
    return true;
}

bool BufferReader::ReadBytes(size_t size, const std::uint8_t** data) {
    if (offset_ + size > data_.size()) return false;
    *data = data_.data() + offset_;
    offset_ += size;
    return true;
}

bool SendRequest(const std::string& host, int port, int timeout_ms,
                Command command,
                const std::vector<std::uint8_t>& payload,
                Response* response,
                std::string* error) {
    if (payload.size() > std::numeric_limits<std::uint32_t>::max()) {
        if (error) *error = "request payload too large";
        return false;
    }
    const int fd = ConnectTcp(host, port, timeout_ms, error);
    if (fd < 0) return false;

    Header header{};
    header.command = static_cast<std::uint16_t>(command);
    header.payload_size = static_cast<std::uint32_t>(payload.size());
    bool ok = WriteHeader(fd, header, timeout_ms, error);
    if (ok && !payload.empty()) {
        ok = WriteAll(fd, payload.data(), payload.size(), timeout_ms, error);
    }

    Header reply_header{};
    std::vector<std::uint8_t> reply_payload;
    if (ok) {
        ok = ReadHeader(fd, &reply_header, timeout_ms, error);
    }
    if (ok && reply_header.command != 0) {
        if (error) *error = "invalid remote mujoco response command";
        ok = false;
    }
    if (ok) {
        reply_payload.resize(reply_header.payload_size);
        if (!reply_payload.empty()) {
            ok = ReadAll(fd, reply_payload.data(), reply_payload.size(),
                        timeout_ms, error);
        }
    }
    close(fd);
    if (!ok) return false;

    BufferReader reader(reply_payload);
    std::uint8_t status = 0;
    std::int32_t result = 0;
    std::string detail;
    if (!reader.ReadU8(&status) || !reader.ReadI32(&result) ||
        !reader.ReadString(&detail)) {
        if (error) *error = "malformed remote mujoco response";
        return false;
    }
    response->ok = status != 0;
    response->result = result;
    response->detail = detail;
    response->payload.clear();
    const size_t prefix_size = reader.Offset();
    if (prefix_size < reply_payload.size()) {
        response->payload.assign(
            reply_payload.begin() + static_cast<std::ptrdiff_t>(prefix_size),
            reply_payload.end());
    }
    return true;
}

bool ReadRequest(int fd, Command* command,
                std::vector<std::uint8_t>* payload,
                std::string* error) {
    Header header{};
    if (!ReadHeader(fd, &header, kServerIoTimeoutMs, error)) return false;
    *command = static_cast<Command>(header.command);
    payload->resize(header.payload_size);
    if (!payload->empty()) {
        return ReadAll(fd, payload->data(), payload->size(),
                kServerIoTimeoutMs, error);
    }
    return true;
}

bool SendResponse(int fd, bool ok, std::int32_t result,
                const std::string& detail,
                const std::vector<std::uint8_t>& payload,
                std::string* error) {
    BufferWriter writer;
    writer.WriteU8(ok ? 1 : 0);
    writer.WriteI32(result);
    writer.WriteString(detail);
    writer.WriteBytes(payload.data(), payload.size());
    const auto& data = writer.Data();
    if (data.size() > std::numeric_limits<std::uint32_t>::max()) {
        if (error) *error = "response payload too large";
        return false;
    }
    Header header{};
    header.command = 0;
    header.payload_size = static_cast<std::uint32_t>(data.size());
    if (!WriteHeader(fd, header, kServerIoTimeoutMs, error)) return false;
    if (!data.empty()) {
        return WriteAll(fd, data.data(), data.size(),
                        kServerIoTimeoutMs, error);
    }
    return true;
}

std::vector<std::uint8_t> EncodeFramePacket(const FramePacket& frame) {
    if (frame.color_bgr.size() >
            static_cast<size_t>(std::numeric_limits<std::int32_t>::max()) ||
        frame.depth_u16.size() >
            static_cast<size_t>(std::numeric_limits<std::int32_t>::max())) {
        return {};
    }
    BufferWriter writer;
    writer.WriteI64(frame.frame_id);
    writer.WriteI32(frame.width);
    writer.WriteI32(frame.height);
    writer.WriteF32(frame.fx);
    writer.WriteF32(frame.fy);
    writer.WriteF32(frame.cx);
    writer.WriteF32(frame.cy);
    writer.WriteI32(static_cast<std::int32_t>(frame.color_bgr.size()));
    writer.WriteI32(static_cast<std::int32_t>(frame.depth_u16.size()));
    writer.WriteBytes(frame.color_bgr.data(), frame.color_bgr.size());
    writer.WriteBytes(frame.depth_u16.data(), frame.depth_u16.size());
    return writer.Data();
}

bool DecodeFramePacket(const std::vector<std::uint8_t>& payload,
        FramePacket* frame,
        std::string* error) {
    BufferReader reader(payload);
    std::int32_t color_size = 0;
    std::int32_t depth_size = 0;
    if (!reader.ReadI64(&frame->frame_id) ||
        !reader.ReadI32(&frame->width) ||
        !reader.ReadI32(&frame->height) ||
        !reader.ReadF32(&frame->fx) ||
        !reader.ReadF32(&frame->fy) ||
        !reader.ReadF32(&frame->cx) ||
        !reader.ReadF32(&frame->cy) ||
        !reader.ReadI32(&color_size) ||
        !reader.ReadI32(&depth_size) ||
        frame->width <= 0 || frame->height <= 0 ||
        color_size < 0 || depth_size < 0) {
        if (error) *error = "malformed frame header";
        return false;
    }
    const std::uint8_t* color = nullptr;
    const std::uint8_t* depth = nullptr;
    if (!reader.ReadBytes(static_cast<size_t>(color_size), &color) ||
        !reader.ReadBytes(static_cast<size_t>(depth_size), &depth)) {
        if (error) *error = "malformed frame payload";
        return false;
    }
    const size_t pixel_count =
        static_cast<size_t>(frame->width) * static_cast<size_t>(frame->height);
    if (pixel_count > kMaxPayloadSize ||
        static_cast<size_t>(color_size) != pixel_count * 3U ||
        static_cast<size_t>(depth_size) != pixel_count * sizeof(std::uint16_t) ||
        !reader.AtEnd()) {
        if (error) *error = "remote mujoco frame dimensions do not match data";
        return false;
    }
    frame->color_bgr.assign(color, color + color_size);
    frame->depth_u16.assign(depth, depth + depth_size);
    return true;
}

}  // namespace remote_mujoco
}  // namespace perceptive_grasp
