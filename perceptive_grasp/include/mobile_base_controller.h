/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mobile_base_controller.h
 * @brief Chassis-assisted target alignment interfaces.
 */

#ifndef MOBILE_BASE_CONTROLLER_H
#define MOBILE_BASE_CONTROLLER_H

#include <string>

namespace perceptive_grasp {

enum class GraspResult;

/** Configuration for chassis-assisted target alignment. */
struct MobileBaseAlignmentConfig {
    bool enabled = false;
    std::string driver = "drv_uart_esp32";
    std::string dev_path = "/dev/ttyACM1";
    int baud = 115200;
    std::string ctrl_dev = "/dev/rpmsg_ctrl0";
    std::string data_dev = "/dev/rpmsg0";
    std::string service_name = "rpmsg:motor_ctrl";
    float wheel_diameter = 0.067f;
    float wheel_base = 0.183f;
    float wheel_track = 0.0f;
    float left_wheel_gain = 1.0f;
    float max_speed = 0.3f;
    float max_angular = 3.14f;
    float reduction_ratio = 56.0f;
    float ff_factor = 0.3f;
    float pid_kp = 0.05f;
    float pid_ki = 0.2f;
    float pid_kd = 0.01f;
    bool cfg_send_on_startup = true;
    bool feedback_enable = true;

    float target_x = 0.275f;
    float x_tolerance = 0.025f;
    float y_tolerance = 0.15f;
    float y_hysteresis = 0.025f;
    float max_step_m = 0.12f;
    float linear_speed = 0.20f;
    float angular_speed = 1.2f;
    // Compensates the measured yaw response of the Linksee UART chassis.
    float yaw_gain = 8.0f;
    int min_cmd_duration_ms = 200;
    int min_rotation_duration_ms = 1000;
    int max_cmd_duration_ms = 2000;
    int settle_ms = 500;
    int max_align_attempts = 6;
    float min_progress_m = 0.02f;
    float min_progress_ratio = 0.15f;
    float min_progress_floor_m = 0.003f;
    float max_total_travel_m = 0.24f;
};

/** One bounded chassis motion selected by the alignment planner. */
struct MobileBaseAlignmentCommand {
    enum class Type {
        NONE,
        DRIVE,
        ROTATE,
    };

    Type type = Type::NONE;
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    int duration_ms = 0;
    bool max_attempts_reached = false;
    std::string reason;
};

/**
 * @brief Plan one chassis correction from a target point in the arm base frame.
 *
 * @param config Alignment and chassis limits.
 * @param base_point Target position in the arm base frame, in meters.
 * @param align_attempts Number of corrections already executed for this task.
 * @return A bounded drive/rotate command, or NONE when no motion is required.
 */
MobileBaseAlignmentCommand PlanMobileBaseAlignment(
    const MobileBaseAlignmentConfig& config, const float base_point[3],
    int align_attempts);

/**
 * @brief Measure target improvement after one chassis motion.
 *
 * @return Signed progress in meters. Positive values indicate improvement.
 */
float MeasureMobileBaseAlignmentProgress(
    const float previous_base_point[3], const float current_base_point[3],
    const MobileBaseAlignmentCommand& previous_command);

/**
 * @brief Compute the minimum acceptable visual progress for one command.
 *
 * @return Required progress in meters.
 */
float RequiredMobileBaseAlignmentProgress(
    const MobileBaseAlignmentConfig& config,
    const float previous_base_point[3],
    const MobileBaseAlignmentCommand& previous_command);

/** Owns the selected chassis driver and executes bounded alignment commands. */
class MobileBaseController {
public:
    explicit MobileBaseController(const MobileBaseAlignmentConfig& config);
    ~MobileBaseController();

    /** Initialize the configured chassis backend. */
    bool Init();

    /** Execute one bounded chassis command and brake when it completes. */
    GraspResult Execute(const MobileBaseAlignmentCommand& command);

    /** Stop chassis motion immediately. */
    void Brake();

private:
    MobileBaseAlignmentConfig config_;
    void* chassis_ = nullptr;
};

}  // namespace perceptive_grasp

#endif  // MOBILE_BASE_CONTROLLER_H
