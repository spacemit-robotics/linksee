// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file executor_device.cpp
 * @brief Grasp-executor device ownership and fresh feedback gates.
 */
#include "executor_device.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "manipulation/device.h"
#include "manipulation/device_plan.h"

namespace perceptive_grasp {
namespace manipulation = spacemit::manipulation;

namespace {
// A latched device refuses every later call, so the first failure is the only useful one. The
// latch lives in this process: rerunning the command is a clean reconnect, no replug needed.
constexpr char kLatched[] =
        "grasp device latched by an earlier error; rerun the command to reconnect "
        "(see the first failure above for the cause)";
}  // namespace

std::pair<float, float> PlannerJointRange(double lower, double upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
        throw std::runtime_error("invalid device joint range");
    float minimum = static_cast<float>(lower);
    float maximum = static_cast<float>(upper);
    if (static_cast<double>(minimum) < lower)
        minimum = std::nextafter(minimum, std::numeric_limits<float>::infinity());
    if (static_cast<double>(maximum) > upper)
        maximum = std::nextafter(maximum, -std::numeric_limits<float>::infinity());
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum)
        throw std::runtime_error("device joint range is below planner precision");
    return {minimum, maximum};
}

ExecutorDevice::ExecutorDevice(std::unique_ptr<manipulation::DeviceSession> session)
    : session_(std::move(session)) {
    if (!session_ || !session_->HasEffector() ||
        session_->Definition().effector.channels.size() != 1 ||
        session_->Definition().effector.feedback != manipulation::EffectorFeedback::kPosition) {
        throw std::runtime_error("grasp executor requires one position-feedback gripper channel");
    }
    if (!session_->GetArmCapabilities().joint_point ||
        !session_->GetArmCapabilities().verified_hold ||
        !session_->GetEffectorCapabilities().verified_hold ||
        !session_->GetEffectorCapabilities().grasp_detection) {
        throw std::runtime_error(
            "grasp executor requires joint points, verified holds and grasp detection");
    }
    for (const auto& joint : session_->Definition().arm.joints) joints_.push_back(joint.name);
}

ExecutorDevice::~ExecutorDevice() = default;

std::unique_ptr<ExecutorDevice> ExecutorDevice::Create(const std::string& plan_path) {
    if (plan_path.empty()) {
        throw std::runtime_error("manipulator.device_plan is required; direct serial access removed");
    }
    auto registry = manipulation::MakeBuiltinDeviceRegistry();
    auto plan = manipulation::LoadDevicePlan(plan_path, registry);
    if (!plan.steps.empty()) {
        throw std::runtime_error("grasp device plan must have no steps; application owns actions");
    }
    return std::make_unique<ExecutorDevice>(std::make_unique<manipulation::DeviceSession>(
        plan.definition, manipulation::CreatePlanDevices(plan, registry)));
}

void ExecutorDevice::Connect() {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (connected_) throw std::runtime_error("grasp device is already connected");
    failed_ = false;
    active_commands_ = false;
    arm_sample_ = 0;
    gripper_sample_ = 0;
    // Connect is read-only even for a disabled device. Motion gates remain explicit below.
    try {
        session_->Connect();
        connected_ = true;
        const auto arm = session_->ReadArm();
        const auto gripper = session_->ReadEffector();
        if (arm.fault || arm.joint_reference_verified == false ||
            gripper.fault || gripper.loads.size() != 1 ||
            !std::isfinite(gripper.loads[0]) || !gripper.holding.has_value()) {
            throw std::runtime_error("grasp executor requires calibrated, fault-free position and load feedback");
        }
    } catch (...) {
        session_->Disconnect();
        connected_ = false;
        throw;
    }
}

void ExecutorDevice::Enable() {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (!connected_) throw std::runtime_error("grasp device is not connected");
    if (failed_) throw std::runtime_error(kLatched);
    // Arm first: the jaws are only allowed to take torque while the arm holds its pose.
    try {
        session_->EnableArm();
        session_->EnableEffector();
        const auto arm = ReadArm();
        const auto gripper = ReadGripper();
        if (!arm.enabled || !arm.position_mode || !gripper.enabled.value_or(false)) {
            throw std::runtime_error("grasp device did not confirm torque on the arm and jaws");
        }
    } catch (...) {
        Fail();
        throw;
    }
}

const std::vector<std::string>& ExecutorDevice::JointNames() const { return joints_; }

std::vector<double> ExecutorDevice::LowerLimits() const {
    std::vector<double> limits;
    for (const auto& joint : session_->Definition().arm.joints) limits.push_back(joint.minimum_rad);
    return limits;
}

std::vector<double> ExecutorDevice::UpperLimits() const {
    std::vector<double> limits;
    for (const auto& joint : session_->Definition().arm.joints) limits.push_back(joint.maximum_rad);
    return limits;
}

manipulation::ArmState ExecutorDevice::ReadArm() {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (failed_) throw std::runtime_error(kLatched);
    try {
        auto state = session_->ReadArm();
        if (state.fault || state.sample_id <= arm_sample_) {
            throw std::runtime_error("arm feedback is faulted or not fresh");
        }
        arm_sample_ = state.sample_id;
        return state;
    } catch (...) {
        Fail();
        throw;
    }
}

manipulation::EffectorState ExecutorDevice::ReadGripper() {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (failed_) throw std::runtime_error(kLatched);
    try {
        auto state = session_->ReadEffector();
        if (state.fault || state.positions.size() != 1 ||
            state.loads.size() != 1 || !std::isfinite(state.loads[0]) ||
            !state.holding.has_value() || state.sample_id <= gripper_sample_) {
            throw std::runtime_error("gripper feedback is faulted, incomplete or not fresh");
        }
        gripper_sample_ = state.sample_id;
        return state;
    } catch (...) {
        Fail();
        throw;
    }
}

void ExecutorDevice::FollowArm(const std::vector<float>& joints, double speed_scale) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (failed_) throw std::runtime_error(kLatched);
    manipulation::JointPoint point;
    point.target_rad.assign(joints.begin(), joints.end());
    point.speed_scale = speed_scale;
    try {
        const auto result = session_->FollowArm(point);
        if (result.status != manipulation::Result::kSucceeded) {
            throw std::runtime_error(result.detail);
        }
        active_commands_ = true;
    } catch (const manipulation::CommandRejected&) {
        // A refused continuation must not leave the previously accepted target running.
        if (active_commands_) Fail();
        throw;
    } catch (...) {
        Fail();
        throw;
    }
}

void ExecutorDevice::FollowGripper(double position, double duration_s) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (failed_) throw std::runtime_error(kLatched);
    try {
        const auto result = session_->FollowEffector({{position}, duration_s});
        if (result.status != manipulation::Result::kSucceeded) {
            throw std::runtime_error(result.detail);
        }
        active_commands_ = true;
    } catch (const manipulation::CommandRejected&) {
        if (active_commands_) Fail();
        throw;
    } catch (...) {
        Fail();
        throw;
    }
}

bool ExecutorDevice::Hold() noexcept {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    bool arm_stopped = false;
    bool gripper_stopped = false;
    try { arm_stopped = session_->HoldArm().status == manipulation::Result::kSucceeded; }
    catch (...) { failed_ = true; }
    try { gripper_stopped = session_->HoldEffector().status == manipulation::Result::kSucceeded; }
    catch (...) { failed_ = true; }
    if (!arm_stopped || !gripper_stopped) failed_ = true;
    if (arm_stopped && gripper_stopped) active_commands_ = false;
    return arm_stopped && gripper_stopped;
}

bool ExecutorDevice::Stop() noexcept {
    // Announce the latch before acquiring the device mutex: queued followers must refuse,
    // even if one was already waiting when the stop was requested.
    failed_ = true;
    return Hold();
}

void ExecutorDevice::Fail() noexcept {
    failed_ = true;
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    // Failed read-only inspection must remain read-only. A command failure after a partial
    // write is stopped by DeviceSession; accepted application targets additionally hold both roles.
    if (active_commands_) Hold();
}
bool ExecutorDevice::Failed() const {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    return failed_ || session_->Latched();
}

}  // namespace perceptive_grasp
