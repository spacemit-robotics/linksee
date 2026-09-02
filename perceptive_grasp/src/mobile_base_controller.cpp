/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file mobile_base_controller.cpp
* @brief Chassis-assisted target alignment implementation.
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

// The linksee UART chassis remains in its motor dead zone below this command.
// Keep short corrections bounded by duration while using an effective speed.
constexpr float kMinimumEffectiveLinearSpeedMps = 0.15f;
constexpr float kPi = 3.14159265358979323846f;

float NormalizeAngle(float angle) {
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

int ClampDurationMs(int duration_ms, const MobileBaseAlignmentConfig& config) {
    return std::clamp(
        duration_ms, config.min_cmd_duration_ms, config.max_cmd_duration_ms);
}

int ClampRotationDurationMs(
    int duration_ms, const MobileBaseAlignmentConfig& config) {
    return std::clamp(duration_ms,
                    std::max(config.min_cmd_duration_ms,
                            config.min_rotation_duration_ms),
                    config.max_cmd_duration_ms);
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
    const float x_error = base_point[0] - config.target_x;
    const float y_error = base_point[1];
    const float x_limit =
        config.x_tolerance + std::max(0.0f, config.x_hysteresis);
    const float y_limit =
        config.y_tolerance + std::max(0.0f, config.y_hysteresis);

    if (std::fabs(x_error) <= x_limit &&
        std::fabs(y_error) <= y_limit) {
        command.reason = "target in comfortable range";
        return command;
    }
    if (align_attempts >= config.max_align_attempts) {
        command.max_attempts_reached = true;
        command.reason = "max base alignment attempts reached";
        return command;
    }

    if (std::fabs(y_error) > y_limit) {
        const float target_range = std::hypot(base_point[0], y_error);
        const float target_angle = std::atan2(
            std::fabs(y_error), base_point[0]);
        const float allowed_angle = std::asin(std::clamp(
            y_limit / target_range, 0.0f, 1.0f));
        const float yaw_error = std::max(0.0f,
            target_angle - allowed_angle);
        const float yaw_gain = std::clamp(config.yaw_gain, 0.5f, 8.0f);
        const float yaw_delta = yaw_error * yaw_gain;
        command.type = MobileBaseAlignmentCommand::Type::ROTATE;
        command.linear_x = 0.0f;
        command.angular_z = std::copysign(config.angular_speed, y_error);
        command.duration_ms = ClampRotationDurationMs(
            static_cast<int>((yaw_delta / config.angular_speed) * 1000.0f),
            config);
        command.reason = "target lateral offset";
        return command;
    }

    if (std::fabs(x_error) > x_limit) {
        // Move only far enough to enter the comfortable range. Driving the
        // full error unnecessarily crosses the target when a minimum motor
        // pulse is longer than the remaining correction.
        const float correction_m = std::max(
            0.0f, std::fabs(x_error) - x_limit);
        const float step_m = std::min(correction_m, config.max_step_m);
        command.type = MobileBaseAlignmentCommand::Type::DRIVE;
        command.angular_z = 0.0f;
        command.duration_ms = ClampDurationMs(
            static_cast<int>((step_m / config.linear_speed) * 1000.0f),
            config);
        const float duration_s =
            static_cast<float>(command.duration_ms) / 1000.0f;
        const float distance_matched_speed = step_m / duration_s;
        const float minimum_speed = std::min(
            config.linear_speed, kMinimumEffectiveLinearSpeedMps);
        const float command_speed = std::clamp(
            distance_matched_speed, minimum_speed, config.linear_speed);
        command.linear_x = std::copysign(command_speed, x_error);
        command.reason = x_error > 0.0f ? "target too far"
                                        : "target too close";
        return command;
    }

    command.reason = "target in comfortable range";
    return command;
}

float MeasureMobileBaseAlignmentProgress(
    const float previous_base_point[3], const float current_base_point[3],
    const MobileBaseAlignmentCommand& previous_command) {
    if (previous_command.type == MobileBaseAlignmentCommand::Type::DRIVE) {
        const float direction = std::copysign(1.0f, previous_command.linear_x);
        return direction *
            (previous_base_point[0] - current_base_point[0]);
    }
    if (previous_command.type == MobileBaseAlignmentCommand::Type::ROTATE) {
        return std::fabs(previous_base_point[1]) -
            std::fabs(current_base_point[1]);
    }
    return 0.0f;
}

float RequiredMobileBaseAlignmentProgress(
    const MobileBaseAlignmentConfig& config,
    const float previous_base_point[3],
    const MobileBaseAlignmentCommand& previous_command) {
    float requested_correction_m = 0.0f;
    if (previous_command.type == MobileBaseAlignmentCommand::Type::DRIVE) {
        requested_correction_m =
            std::fabs(previous_command.linear_x) *
            static_cast<float>(previous_command.duration_ms) / 1000.0f;
    } else if (previous_command.type ==
            MobileBaseAlignmentCommand::Type::ROTATE) {
        const float y_limit =
            config.y_tolerance + std::max(0.0f, config.y_hysteresis);
        requested_correction_m = std::max(
            0.0f, std::fabs(previous_base_point[1]) - y_limit);
    }

    const float scaled =
        requested_correction_m * std::max(0.0f, config.min_progress_ratio);
    return std::clamp(scaled,
                    std::max(0.0f, config.min_progress_floor_m),
                    std::max(config.min_progress_floor_m,
                            config.min_progress_m));
}

MobileBaseMotionReport EvaluateMobileBaseMotion(
    const MobileBaseAlignmentConfig& config,
    const MobileBaseAlignmentCommand& command,
    const std::optional<MobileBaseOdometry>& before,
    const std::optional<MobileBaseOdometry>& after) {
    MobileBaseMotionReport report;
    if (command.type == MobileBaseAlignmentCommand::Type::NONE) {
        report.motion_confirmed = true;
        report.detail = "no chassis motion requested";
        return report;
    }

    const float duration_s =
        static_cast<float>(command.duration_ms) / 1000.0f;
    report.commanded_motion =
        command.type == MobileBaseAlignmentCommand::Type::DRIVE
        ? std::fabs(command.linear_x) * duration_s
        : std::fabs(command.angular_z) * duration_s;

    if (!before.has_value() || !after.has_value()) {
        report.detail =
            "odometry unavailable; waiting for visual confirmation";
        return report;
    }

    report.odometry_available = true;
    const float delta_x = after->x - before->x;
    const float delta_y = after->y - before->y;
    const float cos_yaw = std::cos(before->yaw);
    const float sin_yaw = std::sin(before->yaw);
    report.forward_m = cos_yaw * delta_x + sin_yaw * delta_y;
    report.lateral_m = -sin_yaw * delta_x + cos_yaw * delta_y;
    report.translation_m = std::hypot(delta_x, delta_y);
    report.yaw_rad = NormalizeAngle(after->yaw - before->yaw);

    const float command_ratio =
        std::max(0.0f, config.odom_min_command_ratio);
    if (command.type == MobileBaseAlignmentCommand::Type::DRIVE) {
        report.signed_progress =
            report.forward_m *
            std::copysign(1.0f, command.linear_x);
        report.required_progress = std::max(
            std::max(0.0f, config.odom_min_translation_m),
            report.commanded_motion * command_ratio);
    } else {
        report.signed_progress =
            report.yaw_rad *
            std::copysign(1.0f, command.angular_z);
        report.required_progress = std::max(
            std::max(0.0f, config.odom_min_rotation_rad),
            report.commanded_motion * command_ratio);
    }

    report.motion_confirmed =
        report.signed_progress >= report.required_progress;
    if (report.motion_confirmed) {
        report.detail = "odometry confirmed commanded motion";
    } else {
        report.detail = "odometry did not confirm commanded motion";
    }
    return report;
}

bool IsMobileBaseDirectionReversal(
    const MobileBaseAlignmentCommand& previous,
    const MobileBaseAlignmentCommand& current) {
    if (previous.type != current.type ||
        previous.type == MobileBaseAlignmentCommand::Type::NONE) {
        return false;
    }
    if (current.type == MobileBaseAlignmentCommand::Type::DRIVE) {
        return previous.linear_x * current.linear_x < 0.0f;
    }
    return previous.angular_z * current.angular_z < 0.0f;
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
    SetLastMotionReport(MobileBaseMotionReport{});
    if (command.type == MobileBaseAlignmentCommand::Type::NONE) {
        MobileBaseMotionReport report;
        report.motion_confirmed = true;
        report.detail = "no chassis motion requested";
        SetLastMotionReport(report);
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
    std::optional<MobileBaseOdometry> before_odom;
    if (have_before_odom) {
        PrintOdom("before", before_velocity, before_pose);
        before_odom = MobileBaseOdometry{
            before_pose.x, before_pose.y, before_pose.yaw};
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
            std::min(std::chrono::milliseconds(50), remaining));
    }

    Brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.settle_ms));

    chassis_velocity_t after_velocity = {};
    chassis_pose_t after_pose = {};
    std::optional<MobileBaseOdometry> after_odom;
    if (ReadOdom(chassis, after_velocity, after_pose)) {
        PrintOdom("after", after_velocity, after_pose);
        after_odom = MobileBaseOdometry{
            after_pose.x, after_pose.y, after_pose.yaw};
        if (have_before_odom) {
            std::cout << "[MobileBase] Odom delta: dx="
                    << after_pose.x - before_pose.x
                    << " dy=" << after_pose.y - before_pose.y
                    << " dyaw=" << after_pose.yaw - before_pose.yaw
                    << std::endl;
        }
    }

    const MobileBaseMotionReport report = EvaluateMobileBaseMotion(
        config_, command, before_odom, after_odom);
    SetLastMotionReport(report);
    std::cout << "[MobileBase] Motion report: odom="
            << (report.odometry_available ? "available" : "unavailable")
            << " commanded=" << report.commanded_motion
            << " measured=" << report.signed_progress
            << " required=" << report.required_progress
            << " result="
            << (report.motion_confirmed ? "CONFIRMED" : "UNCONFIRMED")
            << std::endl;
    if (report.odometry_available && !report.motion_confirmed) {
        std::cout
            << "[MobileBase] " << report.detail
            << "; continuing with visual confirmation after the chassis "
            << "has stopped"
            << std::endl;
    }

    return GraspResult::SUCCESS;
#endif
}

MobileBaseMotionReport MobileBaseController::LastMotionReport() const {
    std::lock_guard<std::mutex> lock(motion_report_mutex_);
    return last_motion_report_;
}

void MobileBaseController::SetLastMotionReport(
    const MobileBaseMotionReport& report) {
    std::lock_guard<std::mutex> lock(motion_report_mutex_);
    last_motion_report_ = report;
}

void MobileBaseController::Brake() {
#ifdef HAVE_CHASSIS
    if (chassis_) {
        chassis_brake(static_cast<chassis_dev*>(chassis_));
    }
#endif
}

}  // namespace perceptive_grasp
