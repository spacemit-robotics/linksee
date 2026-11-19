// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file read_joints.cpp
 * @brief Read joint positions through a configured device session, without motor writes.
 */
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "manipulation/device.h"
#include "manipulation/device_plan.h"

namespace {
volatile std::sig_atomic_t g_running = 1;
void SignalHandler(int) { g_running = 0; }
}  // namespace

int main(int argc, char* argv[]) {
    namespace manipulation = spacemit::manipulation;
    try {
        std::string plan_path;
        bool loop = false;
        int interval_ms = 500;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--help" || argument == "-h") {
                std::cout << "Usage: read_joints --plan device.yaml [--loop] [--interval 500]\n";
                return 0;
            }
            if (argument == "--plan" && index + 1 < argc) plan_path = argv[++index];
            else if (argument == "--loop") loop = true;
            else if (argument == "--interval" && index + 1 < argc)
                interval_ms = std::stoi(argv[++index]);
            else throw std::runtime_error("unknown or incomplete option: " + argument);
        }
        if (plan_path.empty() || interval_ms < 20 || interval_ms > 60000)
            throw std::runtime_error("--plan is required; interval must be 20..60000 ms");
        auto registry = manipulation::MakeBuiltinDeviceRegistry();
        const auto plan = manipulation::LoadDevicePlan(plan_path, registry);
        manipulation::DeviceSession device(
            plan.definition, manipulation::CreatePlanDevices(plan, registry));
        std::signal(SIGINT, SignalHandler);
        std::signal(SIGTERM, SignalHandler);
        device.Connect();
        do {
            const auto state = device.ReadArm();
            std::cout << std::fixed << std::setprecision(6) << "joints: [";
            for (std::size_t index = 0; index < state.joints_rad.size(); ++index) {
                if (index) std::cout << ", ";
                std::cout << state.joints_rad[index];
            }
            std::cout << "]" << std::endl;
            if (!loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        } while (g_running);
        // With no commands submitted, disconnect does not issue hold, disable or any targets.
        device.Disconnect();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "read_joints: " << error.what() << '\n';
        return 1;
    }
}
