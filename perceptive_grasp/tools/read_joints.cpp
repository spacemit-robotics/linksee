/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file read_joints.cpp
    * @brief 读取 SO101 机械臂当前关节角度的工具
    *
    * 用途:
    *   手动将机械臂掰到目标姿态后，运行此工具读取关节角度值，
    *   然后填入 grasp_pipeline.yaml 的 home_joints / observe_joints / place_joints。
    *
    * 用法:
    *   ./read_joints                          # 单次读取
    *   ./read_joints --loop                   # 持续读取 (实时显示)
    *   ./read_joints --device /dev/ttyACM1    # 指定串口
    *   ./read_joints --loop --interval 200    # 每 200ms 刷新一次
    *
    * 输出示例:
    *   [ReadJoints] Current joints (rad): [0.000, -1.023, 0.987, 0.012, -0.305]
    *   [ReadJoints] Current joints (deg): [0.0, -58.6, 56.6, 0.7, -17.5]
    *   [ReadJoints] YAML format:
    *     joints: [0.000, -1.023, 0.987, 0.012, -0.305]
    */

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

extern "C" {
#include "manipulator.h"
#include "so101_utils.h"
}

static volatile bool g_running = true;

static void SignalHandler(int sig) {
    (void)sig;
    g_running = false;
}

static void PrintUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
                << "Options:\n"
                << "  --device <path>     Serial device (default: /dev/ttyACM0)\n"
                << "  --baudrate <baud>   Baudrate (default: 1000000)\n"
                << "  --loop              Continuously read and display\n"
                << "  --interval <ms>     Loop interval in ms (default: 500)\n"
                << "  --help              Show this help\n"
                << "\nExamples:\n"
                << "  " << prog << "                    # Single read\n"
                << "  " << prog << " --loop             # Continuous display\n"
                << "  " << prog << " --device /dev/ttyACM1 --loop\n"
                << "\nTip: Manually move the arm to desired pose, then read the values.\n"
                << "     Copy the YAML output into grasp_pipeline.yaml.\n";
}

static void PrintJoints(struct manip_dev* arm) {
    manip_joint_t joints;
    int ret = manip_get_state(arm, &joints, nullptr);
    if (ret != MANIP_OK) {
        std::cerr << "[ReadJoints] Failed to read joints, error=" << ret
                    << std::endl;
        return;
    }

    // 打印弧度值
    printf("[ReadJoints] Current joints (rad): [");
    for (int i = 0; i < joints.count; i++) {
        if (i > 0) printf(", ");
        printf("%.3f", joints.joints[i]);
    }
    printf("]\n");

    // 打印角度值
    printf("[ReadJoints] Current joints (deg): [");
    for (int i = 0; i < joints.count; i++) {
        if (i > 0) printf(", ");
        printf("%.1f", joints.joints[i] * 180.0 / M_PI);
    }
    printf("]\n");

    // 打印 YAML 格式 (方便直接复制)
    printf("[ReadJoints] YAML format:\n");
    printf("  joints: [");
    for (int i = 0; i < joints.count; i++) {
        if (i > 0) printf(", ");
        printf("%.3f", joints.joints[i]);
    }
    printf("]\n");
}

int main(int argc, char* argv[]) {
    std::string device = "/dev/ttyACM1";
    int baudrate = 1000000;
    bool loop_mode = false;
    int interval_ms = 500;

    // 解析参数
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            return 0;
        } else if (arg == "--device" && i + 1 < argc) {
            device = argv[++i];
        } else if (arg == "--baudrate" && i + 1 < argc) {
            baudrate = std::atoi(argv[++i]);
        } else if (arg == "--loop") {
            loop_mode = true;
        } else if (arg == "--interval" && i + 1 < argc) {
            interval_ms = std::atoi(argv[++i]);
        }
    }

    // 注册信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // 初始化机械臂 (只读模式，不发送运动指令)
    struct so101_config cfg = {};
    cfg.uart_path = device.c_str();
    cfg.baud = static_cast<uint32_t>(baudrate);
    cfg.ids[0] = 1;
    cfg.ids[1] = 2;
    cfg.ids[2] = 3;
    cfg.ids[3] = 4;
    cfg.ids[4] = 5;
    cfg.urdf_path = nullptr;  // 不需要 URDF，只读关节角
    cfg.kin_solver_name = nullptr;

    std::cout << "[ReadJoints] Connecting to " << device
                << " @ " << baudrate << " baud..." << std::endl;

    struct manip_dev* arm = manip_alloc("so101", &cfg);
    if (!arm) {
        std::cerr << "[ReadJoints] Failed to connect to manipulator.\n"
                    << "  Check:\n"
                    << "  1. Device exists: ls " << device << "\n"
                    << "  2. Permission: sudo chmod 666 " << device << "\n"
                    << "  3. Arm is powered on\n";
        return 1;
    }

    std::cout << "[ReadJoints] Connected successfully.\n" << std::endl;

    if (loop_mode) {
        std::cout << "[ReadJoints] Loop mode (Ctrl+C to stop, interval="
                    << interval_ms << "ms)\n"
                    << "-------------------------------------------" << std::endl;
        while (g_running) {
            PrintJoints(arm);
            printf("\n");
            std::this_thread::sleep_for(
                std::chrono::milliseconds(interval_ms));
        }
    } else {
        PrintJoints(arm);
    }

    // 清理
    manip_stop(arm);
    manip_free(arm);

    std::cout << "\n[ReadJoints] Done." << std::endl;
    return 0;
}
