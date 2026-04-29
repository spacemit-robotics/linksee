/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include <string>

#include "linksee_app/runner.hpp"

namespace {

constexpr int kFailureExitCode = 1;
constexpr char kStartHostCommand[] = "tool-start-host";
constexpr char kStartInferenceCommand[] = "tool-start-inference";
constexpr char kStopHostCommand[] = "tool-stop-host";
constexpr char kStopInferenceCommand[] = "tool-stop-inference";

void PrintUsage(const char* program) {
    std::cout << "Usage: " << program
    << " [tool-start-host|tool-start-inference|tool-stop-host|tool-stop-inference]\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        return kFailureExitCode;
    }

    linksee_app::Runner runner;
    const std::string command = argv[1];

    if (command == kStartHostCommand) {
        if (!runner.EnsureEnvironment()) {
            std::cerr << "[linksee_app] failed to prepare environment\n";
            return kFailureExitCode;
        }
        const auto result = runner.RunStartHost();
        std::cout << "[command] " << result.command << '\n';
        return result.exit_code;
    }

    if (command == kStartInferenceCommand) {
        if (!runner.EnsureEnvironment()) {
            std::cerr << "[linksee_app] failed to prepare environment\n";
            return kFailureExitCode;
        }
        const auto result = runner.RunStartInference();
        std::cout << "[command] " << result.command << '\n';
        return result.exit_code;
    }

    if (command == kStopHostCommand) {
        const auto result = runner.RunStopHost();
        std::cout << "[command] " << result.command << '\n';
        return result.exit_code;
    }

    if (command == kStopInferenceCommand) {
        const auto result = runner.RunStopInference();
        std::cout << "[command] " << result.command << '\n';
        return result.exit_code;
    }

    PrintUsage(argv[0]);
    return kFailureExitCode;
}
