/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mobile_base_controller.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

using perceptive_grasp::MobileBaseAlignmentCommand;
using perceptive_grasp::MobileBaseAlignmentConfig;
using perceptive_grasp::PlanMobileBaseAlignment;

bool Near(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1e-5f;
}

bool NearMs(int lhs, int rhs) {
    return std::abs(lhs - rhs) <= 1;
}

MobileBaseAlignmentConfig TestConfig() {
    MobileBaseAlignmentConfig config;
    config.enabled = true;
    config.target_x = 0.30f;
    config.x_tolerance = 0.05f;
    config.y_tolerance = 0.08f;
    config.max_step_m = 0.12f;
    config.linear_speed = 0.20f;
    config.angular_speed = 0.25f;
    config.yaw_gain = 2.0f;
    config.min_cmd_duration_ms = 200;
    config.max_cmd_duration_ms = 1500;
    config.max_align_attempts = 3;
    return config;
}

void TestNoMotionWhenTargetIsComfortable() {
    auto config = TestConfig();
    const float base_point[3] = {0.31f, 0.02f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
    assert(!command.max_attempts_reached);
}

void TestNoMotionForReachableSmallLateralOffset() {
    auto config = TestConfig();
    const float base_point[3] = {0.258f, 0.060f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
    assert(!command.max_attempts_reached);
}

void TestDriveForwardWhenTargetIsTooFar() {
    auto config = TestConfig();
    const float base_point[3] = {0.55f, 0.01f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::DRIVE);
    assert(Near(command.linear_x, config.linear_speed));
    assert(Near(command.angular_z, 0.0f));
    assert(NearMs(command.duration_ms, 600));
}

void TestDriveBackwardWhenTargetIsTooClose() {
    auto config = TestConfig();
    const float base_point[3] = {0.18f, 0.0f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::DRIVE);
    assert(Near(command.linear_x, -config.linear_speed));
    assert(Near(command.angular_z, 0.0f));
    assert(NearMs(command.duration_ms, 600));
}

void TestRotateCounterClockwiseWhenTargetIsLeft() {
    auto config = TestConfig();
    const float base_point[3] = {0.55f, 0.12f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::ROTATE);
    assert(Near(command.linear_x, 0.0f));
    assert(Near(command.angular_z, config.angular_speed));
    assert(command.duration_ms == 960);
}

void TestRotateClockwiseWhenTargetIsRight() {
    auto config = TestConfig();
    const float base_point[3] = {0.55f, -0.12f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::ROTATE);
    assert(Near(command.linear_x, 0.0f));
    assert(Near(command.angular_z, -config.angular_speed));
    assert(command.duration_ms == 960);
}

void TestStopsAligningAfterMaxAttempts() {
    auto config = TestConfig();
    const float base_point[3] = {0.55f, 0.12f, 0.05f};

    auto command = PlanMobileBaseAlignment(
        config, base_point, config.max_align_attempts);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
    assert(command.max_attempts_reached);
}

}  // namespace

int main() {
    TestNoMotionWhenTargetIsComfortable();
    TestNoMotionForReachableSmallLateralOffset();
    TestDriveForwardWhenTargetIsTooFar();
    TestDriveBackwardWhenTargetIsTooClose();
    TestRotateCounterClockwiseWhenTargetIsLeft();
    TestRotateClockwiseWhenTargetIsRight();
    TestStopsAligningAfterMaxAttempts();
    std::cout << "mobile_base_alignment_test passed" << std::endl;
    return 0;
}
