/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <memory>
#include <string>

#include "linksee_app/runner.hpp"
#include "mlink.h"

namespace {

constexpr int kSuccessExitCode = 0;
constexpr int kFailureExitCode = 1;
constexpr char kDefaultTransport[] = "unix";
constexpr char kDefaultServerName[] = "linksee";
constexpr char kStartHostToolName[] = "start_host";
constexpr char kStartInferenceToolName[] = "start_inference";
constexpr char kStopHostToolName[] = "stop_host";
constexpr char kStopInferenceToolName[] = "stop_inference";

constexpr char kStartHostToolDescription[] = "Start Linksee host process";
constexpr char kStartInferenceToolDescription[] = "Start Linksee inference process";
constexpr char kStopHostToolDescription[] = "Stop Linksee host process";
constexpr char kStopInferenceToolDescription[] = "Stop Linksee inference process";

struct ServerDeleter {
    void operator()(mlink_server_t* server) const {
        if (server) {
            mlink_server_destroy(server);
        }
    }
};

struct ToolDeleter {
    void operator()(mlink_tool_t* tool) const {
        if (tool) {
            mlink_tool_destroy(tool);
        }
    }
};

struct PropertyListDeleter {
    void operator()(mlink_property_list_t* props) const {
        if (props) {
            mlink_property_list_destroy(props);
        }
    }
};

struct DeviceContext {
    linksee_app::Runner runner;
};

mlink_return_value StartHostCallback(const mlink_property_list_t* props, void* user_ctx) {
    (void)props;
    auto* ctx = static_cast<DeviceContext*>(user_ctx);
    if (!ctx) {
        return mlink_return_string("missing device context");
    }

    if (!ctx->runner.EnsureEnvironment()) {
        return mlink_return_string("failed to prepare lerobot environment");
    }

    const auto result = ctx->runner.RunStartHost();
    if (result.exit_code != 0) {
        return mlink_return_string("start_host command failed");
    }

    return mlink_return_string("start_host finished");
}

mlink_return_value StartInferenceCallback(const mlink_property_list_t* props, void* user_ctx) {
    (void)props;
    auto* ctx = static_cast<DeviceContext*>(user_ctx);
    if (!ctx) {
        return mlink_return_string("missing device context");
    }

    if (!ctx->runner.EnsureEnvironment()) {
        return mlink_return_string("failed to prepare lerobot environment");
    }

    const auto result = ctx->runner.RunStartInference();
    if (result.exit_code != 0) {
        return mlink_return_string("start_inference command failed");
    }

    return mlink_return_string("start_inference finished");
}

mlink_return_value StopHostCallback(const mlink_property_list_t* props, void* user_ctx) {
    (void)props;
    auto* ctx = static_cast<DeviceContext*>(user_ctx);
    if (!ctx) {
        return mlink_return_string("missing device context");
    }

    const auto result = ctx->runner.RunStopHost();
    if (result.exit_code != 0) {
        return mlink_return_string("stop_host command failed");
    }

    return mlink_return_string("stop_host finished");
}

mlink_return_value StopInferenceCallback(const mlink_property_list_t* props, void* user_ctx) {
    (void)props;
    auto* ctx = static_cast<DeviceContext*>(user_ctx);
    if (!ctx) {
        return mlink_return_string("missing device context");
    }

    const auto result = ctx->runner.RunStopInference();
    if (result.exit_code != 0) {
        return mlink_return_string("stop_inference command failed");
    }

    return mlink_return_string("stop_inference finished");
}

bool RegisterTool(
    mlink_server_t* server,
    const char* tool_name,
    const char* tool_description,
    mlink_tool_callback_t callback,
    DeviceContext* ctx) {
    std::unique_ptr<mlink_property_list_t, PropertyListDeleter> props(mlink_property_list_create());
    if (!props) {
        std::cerr << "Failed to create property list for " << tool_name << "\n";
        return false;
    }

    std::unique_ptr<mlink_tool_t, ToolDeleter> tool(
        mlink_tool_create(tool_name, tool_description, props.get(), callback, ctx, false));

    if (!tool) {
        std::cerr << "Failed to create tool " << tool_name << "\n";
        return false;
    }

    if (!mlink_server_add_tool(server, tool.get())) {
        std::cerr << "Failed to register tool " << tool_name << "\n";
        return false;
    }

    tool.release();
    return true;
}

bool ResolveTransport(const std::string& transport_name, transport_type* type) {
    if (transport_name == "tcp") {
        *type = TRANSPORT_TYPE_TCP;
        return true;
    }

    if (transport_name == "unix") {
        *type = TRANSPORT_TYPE_UNIX;
        return true;
    }

    return false;
}

void PrintUsage(const char* program) {
    std::cout << "Usage: " << program << " [tcp|unix] [server_name]\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    const char* transport_str = kDefaultTransport;
    const char* server_name = kDefaultServerName;
    enum transport_type type = TRANSPORT_TYPE_UNIX;

    if (argc > 1) {
        transport_str = argv[1];
    }
    if (argc > 2 && argv[2] && argv[2][0] != '\0') {
        server_name = argv[2];
    }

    if (!ResolveTransport(transport_str, &type)) {
        PrintUsage(argv[0]);
        return kFailureExitCode;
    }

    std::unique_ptr<mlink_server_t, ServerDeleter> server(mlink_server_init(type, server_name));
    if (!server) {
        std::cerr << "Failed to init mlink server\n";
        return kFailureExitCode;
    }

    DeviceContext ctx;
    const bool registered =
        RegisterTool(server.get(), kStartHostToolName, kStartHostToolDescription,
            StartHostCallback, &ctx) &&
        RegisterTool(server.get(), kStartInferenceToolName,
            kStartInferenceToolDescription, StartInferenceCallback, &ctx) &&
        RegisterTool(server.get(), kStopHostToolName, kStopHostToolDescription,
            StopHostCallback, &ctx) &&
        RegisterTool(server.get(), kStopInferenceToolName,
            kStopInferenceToolDescription, StopInferenceCallback, &ctx);
    if (!registered) {
        return kFailureExitCode;
    }

    std::cout << "linksee_device registered tools: start_host start_inference stop_host stop_inference\n";
    mlink_server_run(server.get());
    return kSuccessExitCode;
}
