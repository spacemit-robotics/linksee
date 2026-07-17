/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mobile_base_alignment_test.cpp
 * @brief Unit tests for chassis-assisted target alignment.
 */

#include "mobile_base_controller.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

using perceptive_grasp::MobileBaseAlignmentCommand;
using perceptive_grasp::MobileBaseAlignmentConfig;
using perceptive_grasp::MeasureMobileBaseAlignmentProgress;
using perceptive_grasp::PlanMobileBaseAlignment;
using perceptive_grasp::RequiredMobileBaseAlignmentProgress;

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
    config.y_hysteresis = 0.0f;
    config.max_step_m = 0.12f;
    config.linear_speed = 0.20f;
    config.angular_speed = 0.25f;
    config.yaw_gain = 1.0f;
    config.min_cmd_duration_ms = 200;
    config.min_rotation_duration_ms = 200;
    config.max_cmd_duration_ms = 1500;
    config.max_align_attempts = 3;
    return config;
}

void TestDefaultComfortRangeIsTightForLinkseeArm() {
    const MobileBaseAlignmentConfig config;
    assert(Near(config.target_x, 0.275f));
    assert(Near(config.x_tolerance, 0.025f));
    assert(Near(config.yaw_gain, 8.0f));
    assert(Near(config.min_progress_ratio, 0.15f));
    assert(config.min_rotation_duration_ms == 1000);
    assert(Near(config.y_hysteresis, 0.025f));
    assert(Near(config.min_progress_floor_m, 0.003f));
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
    assert(NearMs(command.duration_ms, 288));
}

void TestRotateClockwiseWhenTargetIsRight() {
    auto config = TestConfig();
    const float base_point[3] = {0.55f, -0.12f, 0.05f};

    auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::ROTATE);
    assert(Near(command.linear_x, 0.0f));
    assert(Near(command.angular_z, -config.angular_speed));
    assert(NearMs(command.duration_ms, 288));
}

void TestRotationUsesMinimumAngleForCurrentTargetGeometry() {
    MobileBaseAlignmentConfig config;
    config.enabled = true;
    config.yaw_gain = 1.0f;
    config.min_rotation_duration_ms = config.min_cmd_duration_ms;
    const float base_point[3] = {0.299f, 0.180f, 0.0f};

    const auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::ROTATE);
    assert(command.duration_ms == config.min_cmd_duration_ms);
    const float yaw = command.angular_z *
        static_cast<float>(command.duration_ms) / 1000.0f;
    const float aligned_y = -base_point[0] * std::sin(yaw) +
        base_point[1] * std::cos(yaw);
    assert(std::fabs(aligned_y) <= config.y_tolerance);
}

void TestDefaultRotationCompensatesMeasuredChassisResponse() {
    MobileBaseAlignmentConfig config;
    config.enabled = true;
    const float base_point[3] = {0.253535f, 0.193781f, 0.0f};

    const auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::ROTATE);
    assert(Near(command.angular_z, config.angular_speed));
    assert(command.duration_ms == config.min_rotation_duration_ms);
}

void TestMeasuredRotationProgressPassesDefaultRequirement() {
    const MobileBaseAlignmentConfig config;
    MobileBaseAlignmentCommand command;
    command.type = MobileBaseAlignmentCommand::Type::ROTATE;
    const float previous[3] = {0.240984f, 0.198003f, 0.0f};

    const float required =
        RequiredMobileBaseAlignmentProgress(config, previous, command);

    assert(Near(required, 0.00345045f));
    assert(0.00894652f > required);
}

void TestSmallMeasuredProgressPassesNoiseFloor() {
    const MobileBaseAlignmentConfig config;
    MobileBaseAlignmentCommand command;
    command.type = MobileBaseAlignmentCommand::Type::ROTATE;
    const float previous[3] = {0.269916f, 0.184650f, 0.0f};

    const float required =
        RequiredMobileBaseAlignmentProgress(config, previous, command);

    assert(Near(required, 0.003f));
    assert(0.00405228f > required);
}

void TestBoundaryNoiseDoesNotTriggerLateralCorrection() {
    MobileBaseAlignmentConfig config;
    config.enabled = true;
    const float base_point[3] = {0.281835f, 0.160019f, 0.0f};

    const auto command = PlanMobileBaseAlignment(config, base_point, 1);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
    assert(command.reason == "target in comfortable range");
}

void TestMeasuredResidualIsInsideStableBoundary() {
    MobileBaseAlignmentConfig config;
    config.enabled = true;
    const float base_point[3] = {0.277748f, 0.170733f, 0.0f};

    const auto command = PlanMobileBaseAlignment(config, base_point, 1);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
}

void TestTargetOutsideStableBoundaryStillRotates() {
    MobileBaseAlignmentConfig config;
    config.enabled = true;
    const float base_point[3] = {0.281835f, 0.176774f, 0.0f};

    const auto command = PlanMobileBaseAlignment(config, base_point, 0);

    assert(command.type == MobileBaseAlignmentCommand::Type::ROTATE);
    assert(command.duration_ms == config.min_rotation_duration_ms);
}

void TestStopsAligningAfterMaxAttempts() {
    auto config = TestConfig();
    const float base_point[3] = {0.55f, 0.12f, 0.05f};

    auto command = PlanMobileBaseAlignment(
        config, base_point, config.max_align_attempts);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
    assert(command.max_attempts_reached);
}

void TestComfortableTargetWinsAtMaxAttempts() {
    auto config = TestConfig();
    const float base_point[3] = {0.32f, 0.01f, 0.05f};

    auto command = PlanMobileBaseAlignment(
        config, base_point, config.max_align_attempts);

    assert(command.type == MobileBaseAlignmentCommand::Type::NONE);
    assert(!command.max_attempts_reached);
}

void TestMeasuresForwardVisualProgress() {
    MobileBaseAlignmentCommand command;
    command.type = MobileBaseAlignmentCommand::Type::DRIVE;
    command.linear_x = 0.2f;
    const float previous[3] = {0.52f, 0.0f, 0.0f};
    const float current[3] = {0.44f, 0.0f, 0.0f};
    assert(Near(MeasureMobileBaseAlignmentProgress(
                    previous, current, command),
                0.08f));
}

void TestMeasuresBackwardVisualProgress() {
    MobileBaseAlignmentCommand command;
    command.type = MobileBaseAlignmentCommand::Type::DRIVE;
    command.linear_x = -0.2f;
    const float previous[3] = {0.18f, 0.0f, 0.0f};
    const float current[3] = {0.24f, 0.0f, 0.0f};
    assert(Near(MeasureMobileBaseAlignmentProgress(
                    previous, current, command),
                0.06f));
}

void TestShortCorrectionUsesScaledProgressRequirement() {
    auto config = TestConfig();
    config.min_progress_m = 0.02f;
    config.min_progress_ratio = 0.25f;
    config.min_progress_floor_m = 0.005f;
    MobileBaseAlignmentCommand command;
    command.type = MobileBaseAlignmentCommand::Type::DRIVE;
    command.linear_x = -0.2f;
    command.duration_ms = 210;
    const float previous[3] = {0.258f, 0.0f, 0.0f};
    assert(Near(RequiredMobileBaseAlignmentProgress(
                    config, previous, command),
                0.0105f));
}

void TestLargeCorrectionKeepsFullProgressRequirement() {
    auto config = TestConfig();
    config.min_progress_m = 0.02f;
    config.min_progress_ratio = 0.25f;
    config.min_progress_floor_m = 0.005f;
    MobileBaseAlignmentCommand command;
    command.type = MobileBaseAlignmentCommand::Type::DRIVE;
    command.linear_x = 0.2f;
    command.duration_ms = 600;
    const float previous[3] = {0.52f, 0.0f, 0.0f};
    assert(Near(RequiredMobileBaseAlignmentProgress(
                    config, previous, command),
                0.02f));
}

}  // namespace

int main() {
    TestDefaultComfortRangeIsTightForLinkseeArm();
    TestNoMotionWhenTargetIsComfortable();
    TestNoMotionForReachableSmallLateralOffset();
    TestDriveForwardWhenTargetIsTooFar();
    TestDriveBackwardWhenTargetIsTooClose();
    TestRotateCounterClockwiseWhenTargetIsLeft();
    TestRotateClockwiseWhenTargetIsRight();
    TestRotationUsesMinimumAngleForCurrentTargetGeometry();
    TestDefaultRotationCompensatesMeasuredChassisResponse();
    TestMeasuredRotationProgressPassesDefaultRequirement();
    TestSmallMeasuredProgressPassesNoiseFloor();
    TestBoundaryNoiseDoesNotTriggerLateralCorrection();
    TestMeasuredResidualIsInsideStableBoundary();
    TestTargetOutsideStableBoundaryStillRotates();
    TestStopsAligningAfterMaxAttempts();
    TestComfortableTargetWinsAtMaxAttempts();
    TestMeasuresForwardVisualProgress();
    TestMeasuresBackwardVisualProgress();
    TestShortCorrectionUsesScaledProgressRequirement();
    TestLargeCorrectionKeepsFullProgressRequirement();
    std::cout << "mobile_base_alignment_test passed" << std::endl;
    return 0;
}
