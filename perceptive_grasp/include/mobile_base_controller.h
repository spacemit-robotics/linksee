/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOBILE_BASE_CONTROLLER_H
#define MOBILE_BASE_CONTROLLER_H

#include <string>

namespace perceptive_grasp {

enum class GraspResult;

struct MobileBaseAlignmentConfig {
    bool enabled = false;
    std::string driver = "none";
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

    float target_x = 0.30f;
    float x_tolerance = 0.05f;
    float y_tolerance = 0.08f;
    float max_step_m = 0.12f;
    float linear_speed = 0.20f;
    float angular_speed = 1.2f;
    float yaw_gain = 8.0f;
    int min_cmd_duration_ms = 200;
    int max_cmd_duration_ms = 2000;
    int settle_ms = 500;
    int max_align_attempts = 6;
};

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

MobileBaseAlignmentCommand PlanMobileBaseAlignment(
    const MobileBaseAlignmentConfig& config, const float base_point[3],
    int align_attempts);

class MobileBaseController {
public:
    explicit MobileBaseController(const MobileBaseAlignmentConfig& config);
    ~MobileBaseController();

    bool Init();
    GraspResult Execute(const MobileBaseAlignmentCommand& command);
    void Brake();

private:
    MobileBaseAlignmentConfig config_;
    void* chassis_ = nullptr;
};

}  // namespace perceptive_grasp

#endif  // MOBILE_BASE_CONTROLLER_H
