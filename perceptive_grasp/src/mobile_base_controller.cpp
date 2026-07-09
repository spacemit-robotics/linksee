/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mobile_base_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "grasp_executor.h"

#ifdef HAVE_CHASSIS
#include <chassis.h>
#endif

namespace perceptive_grasp {

namespace {

int ClampDurationMs(int duration_ms, const MobileBaseAlignmentConfig& config) {
    return std::clamp(
        duration_ms, config.min_cmd_duration_ms, config.max_cmd_duration_ms);
}

#ifdef HAVE_CHASSIS
void FillCommonChassisConfig(chassis_config& chassis_config,
    const MobileBaseAlignmentConfig& config) {
    chassis_config.type = CHASSIS_TYPE_DIFF_2WD;
    chassis_config.wheel_diameter = config.wheel_diameter;
    chassis_config.wheel_base = config.wheel_base;
    chassis_config.wheel_track = config.wheel_track;
    chassis_config.left_wheel_gain = config.left_wheel_gain;
    chassis_config.max_speed = config.max_speed;
    chassis_config.max_angular = config.max_angular;
}

bool ReadOdom(chassis_dev* chassis, chassis_velocity_t& velocity,
    chassis_pose_t& pose) {
    return chassis_get_odom(chassis, &velocity, &pose) == CHASSIS_OK;
}

void PrintOdom(const char* label, const chassis_velocity_t& velocity,
    const chassis_pose_t& pose) {
    std::cout << "[MobileBase] Odom " << label
            << ": pose=(" << pose.x << ", " << pose.y
            << ", yaw=" << pose.yaw << ")"
            << " vel=(" << velocity.vx << ", " << velocity.vy
            << ", wz=" << velocity.wz << ")" << std::endl;
}
#endif

}  // namespace

MobileBaseAlignmentCommand PlanMobileBaseAlignment(
    const MobileBaseAlignmentConfig& config, const float base_point[3],
    int align_attempts) {
    MobileBaseAlignmentCommand command;
    if (!config.enabled) {
        command.reason = "mobile base disabled";
        return command;
    }
    if (align_attempts >= config.max_align_attempts) {
        command.max_attempts_reached = true;
        command.reason = "max base alignment attempts reached";
        return command;
    }

    const float x_error = base_point[0] - config.target_x;
    const float y_error = base_point[1];

    if (std::fabs(y_error) > config.y_tolerance) {
        const float yaw_delta = std::fabs(y_error) * config.yaw_gain;
        command.type = MobileBaseAlignmentCommand::Type::ROTATE;
        command.linear_x = 0.0f;
        command.angular_z = std::copysign(config.angular_speed, y_error);
        command.duration_ms = ClampDurationMs(
            static_cast<int>((yaw_delta / config.angular_speed) * 1000.0f),
            config);
        command.reason = "target lateral offset";
        return command;
    }

    if (std::fabs(x_error) > config.x_tolerance) {
        const float step_m = std::min(std::fabs(x_error), config.max_step_m);
        command.type = MobileBaseAlignmentCommand::Type::DRIVE;
        command.linear_x = std::copysign(config.linear_speed, x_error);
        command.angular_z = 0.0f;
        command.duration_ms = ClampDurationMs(
            static_cast<int>((step_m / config.linear_speed) * 1000.0f),
            config);
        command.reason = x_error > 0.0f ? "target too far"
                                        : "target too close";
        return command;
    }

    command.reason = "target in comfortable range";
    return command;
}

MobileBaseController::MobileBaseController(
    const MobileBaseAlignmentConfig& config)
    : config_(config) {}

MobileBaseController::~MobileBaseController() {
    Brake();
#ifdef HAVE_CHASSIS
    if (chassis_) {
        chassis_free(static_cast<chassis_dev*>(chassis_));
        chassis_ = nullptr;
    }
#endif
}

bool MobileBaseController::Init() {
    std::cout << "[MobileBase] Init requested: enabled=" << config_.enabled
            << " driver=" << config_.driver << std::endl;
    if (!config_.enabled) {
        std::cout << "[MobileBase] Disabled" << std::endl;
        return true;
    }
    if (config_.driver == "none") {
        std::cerr << "[MobileBase] enabled but driver is none" << std::endl;
        return false;
    }
#ifndef HAVE_CHASSIS
    std::cerr << "[MobileBase] chassis SDK not linked; disable mobile_base "
            << "or build components/control/base first" << std::endl;
    return false;
#else
    if (config_.driver == "drv_uart_esp32") {
        std::cout << "[MobileBase] Creating UART chassis: dev="
                << config_.dev_path << " baud=" << config_.baud << std::endl;
        chassis_uart_config chassis_config = {};
        FillCommonChassisConfig(chassis_config.base, config_);
        chassis_config.dev_path = config_.dev_path.c_str();
        chassis_config.baud = static_cast<uint32_t>(config_.baud);

        chassis_ = chassis_alloc(config_.driver.c_str(), &chassis_config);
        if (!chassis_) {
            std::cerr << "[MobileBase] failed to create chassis driver"
                    << std::endl;
            return false;
        }
        std::cout << "[MobileBase] Initialized: driver=" << config_.driver
                << " dev=" << config_.dev_path
                << " baud=" << config_.baud << std::endl;
        return true;
    }

    if (config_.driver != "drv_rpmsg_esos") {
        std::cerr << "[MobileBase] unsupported driver: " << config_.driver
                << std::endl;
        return false;
    }

    chassis_rpmsg_config chassis_config = {};
    FillCommonChassisConfig(chassis_config.base, config_);
    chassis_config.ctrl_dev = config_.ctrl_dev.c_str();
    chassis_config.data_dev = config_.data_dev.c_str();
    chassis_config.service_name = config_.service_name.c_str();
    chassis_config.local_addr = 1003;
    chassis_config.remote_addr = 1002;
    chassis_config.reduction_ratio = config_.reduction_ratio;
    chassis_config.ff_factor = config_.ff_factor;
    chassis_config.pid_kp = config_.pid_kp;
    chassis_config.pid_ki = config_.pid_ki;
    chassis_config.pid_kd = config_.pid_kd;
    chassis_config.cfg_send_on_startup = config_.cfg_send_on_startup;
    chassis_config.feedback_enable = config_.feedback_enable;

    chassis_ = chassis_alloc(config_.driver.c_str(), &chassis_config);
    if (!chassis_) {
        std::cerr << "[MobileBase] failed to create chassis driver"
                << std::endl;
        return false;
    }
    std::cout << "[MobileBase] Initialized: driver=" << config_.driver
            << " ctrl=" << config_.ctrl_dev
            << " data=" << config_.data_dev << std::endl;
    return true;
#endif
}

GraspResult MobileBaseController::Execute(
    const MobileBaseAlignmentCommand& command) {
    if (command.type == MobileBaseAlignmentCommand::Type::NONE) {
        return GraspResult::SUCCESS;
    }
    if (!config_.enabled) {
        return GraspResult::SUCCESS;
    }
#ifndef HAVE_CHASSIS
    std::cerr << "[MobileBase] cannot execute without chassis SDK"
            << std::endl;
    return GraspResult::MOVE_FAILED;
#else
    if (!chassis_) {
        std::cerr << "[MobileBase] chassis not initialized" << std::endl;
        return GraspResult::MOVE_FAILED;
    }

    chassis_velocity_t velocity = {};
    velocity.vx = command.linear_x;
    velocity.vy = 0.0f;
    velocity.wz = command.angular_z;

    std::cout << "[MobileBase] Execute: reason=" << command.reason
            << " vx=" << velocity.vx
            << " wz=" << velocity.wz
            << " duration_ms=" << command.duration_ms << std::endl;

    auto* chassis = static_cast<chassis_dev*>(chassis_);
    chassis_velocity_t before_velocity = {};
    chassis_pose_t before_pose = {};
    const bool have_before_odom =
        ReadOdom(chassis, before_velocity, before_pose);
    if (have_before_odom) {
        PrintOdom("before", before_velocity, before_pose);
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(command.duration_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (chassis_set_velocity(chassis, &velocity) != CHASSIS_OK) {
            std::cerr << "[MobileBase] chassis_set_velocity failed"
                    << std::endl;
            Brake();
            return GraspResult::MOVE_FAILED;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now);
        if (remaining <= std::chrono::milliseconds::zero()) {
            break;
        }
        std::this_thread::sleep_for(
            std::min(std::chrono::milliseconds(100), remaining));
    }

    Brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.settle_ms));

    chassis_velocity_t after_velocity = {};
    chassis_pose_t after_pose = {};
    if (ReadOdom(chassis, after_velocity, after_pose)) {
        PrintOdom("after", after_velocity, after_pose);
        if (have_before_odom) {
            std::cout << "[MobileBase] Odom delta: dx="
                    << after_pose.x - before_pose.x
                    << " dy=" << after_pose.y - before_pose.y
                    << " dyaw=" << after_pose.yaw - before_pose.yaw
                    << std::endl;
        }
    }

    return GraspResult::SUCCESS;
#endif
}

void MobileBaseController::Brake() {
#ifdef HAVE_CHASSIS
    if (chassis_) {
        chassis_brake(static_cast<chassis_dev*>(chassis_));
    }
#endif
}

}  // namespace perceptive_grasp
