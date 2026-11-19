// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file executor_device_test.cpp
 * @brief Hardware-free regression for ownership, no-write connect, following and failure gates.
 */
#include "executor_device.h"

#include <cassert>
#include <chrono>
#include <condition_variable>
#include <filesystem>  // NOLINT(build/c++17)
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "manipulation/device.h"

namespace manipulation = spacemit::manipulation;
namespace {
struct State {
    manipulation::ArmState arm;
    manipulation::EffectorState gripper;
    int arm_writes = 0;
    int gripper_writes = 0;
    int torque_writes = 0;
    int holds = 0;
    bool frozen = false;
    bool stale = false;
    bool hold_succeeds = true;
    bool reject_arm = false;
    bool throw_arm = false;
    bool throw_gripper = false;
    double speed = 0;
    std::mutex pause_mutex;
    std::condition_variable pause;
    bool block_arm = false;
    bool arm_entered = false;
    bool release_arm = false;
};

class Arm final : public manipulation::Arm {
public:
    explicit Arm(std::shared_ptr<State> state) : state_(std::move(state)) {}
    manipulation::ArmCapabilities GetCapabilities() const override {
        manipulation::ArmCapabilities capabilities;
        capabilities.joint_point = capabilities.verified_hold = capabilities.enable = true;
        return capabilities;
    }
    void Connect() override {}
    void Disconnect() noexcept override {}
    manipulation::ArmState Read() override {
        if (!state_->frozen) ++state_->arm.sample_id;
        state_->arm.received_at = manipulation::Clock::now() -
            (state_->stale ? std::chrono::seconds(5) : std::chrono::seconds(0));
        return state_->arm;
    }
    // Confirmed, counted and idempotent, like the shipped drivers which return early.
    void Enable() override {
        if (state_->arm.enabled) return;
        ++state_->torque_writes;
        state_->arm.enabled = true;
    }
    void Submit(const manipulation::JointPoint& point) override {
        if (state_->reject_arm) throw manipulation::CommandRejected("test refusal");
        if (state_->block_arm) {
            std::unique_lock<std::mutex> guard(state_->pause_mutex);
            state_->arm_entered = true;
            state_->pause.notify_all();
            state_->pause.wait(guard, [&] { return state_->release_arm; });
        }
        ++state_->arm_writes;
        if (state_->throw_arm) throw std::runtime_error("partial arm write");
        state_->speed = point.speed_scale;
        state_->arm.joints_rad = point.target_rad;
    }
    bool Hold() override { ++state_->holds; return state_->hold_succeeds; }
private:
    std::shared_ptr<State> state_;
};

class Gripper final : public manipulation::EndEffector {
public:
    explicit Gripper(std::shared_ptr<State> state) : state_(std::move(state)) {}
    manipulation::EffectorCapabilities GetCapabilities() const override {
        return {true, 120, true, true, true};
    }
    void Connect() override {}
    void Disconnect() noexcept override {}
    manipulation::EffectorState Read() override {
        if (!state_->frozen) ++state_->gripper.sample_id;
        state_->gripper.received_at = manipulation::Clock::now();
        return state_->gripper;
    }
    void Enable() override {
        if (state_->gripper.enabled.value_or(false)) return;
        ++state_->torque_writes;
        state_->gripper.enabled = true;
    }
    void Submit(const manipulation::EffectorPoint& point) override {
        ++state_->gripper_writes;
        if (state_->throw_gripper) throw std::runtime_error("partial write");
        state_->gripper.positions = point.positions;
    }
    bool Hold() override { ++state_->holds; return state_->hold_succeeds; }
private:
    std::shared_ptr<State> state_;
};

struct Fixture {
    std::shared_ptr<State> state = std::make_shared<State>();
    manipulation::DeviceDefinition definition;
    std::string directory;
    Fixture() {
        definition.arm = {"test_arm", "test_resource", {{"joint", -2, 2}}, 0.01, 1, 0.2};
        definition.effector.device_id = "test_tool";
        definition.effector.channels = {"gripper"};
        state->arm.valid = true;
        state->arm.enabled = true;
        state->arm.position_mode = true;
        state->arm.joint_reference_verified = true;
        state->arm.joints_rad = {0};
        state->gripper.valid = true;
        state->gripper.enabled = true;
        state->gripper.positions = {0};
        state->gripper.loads = {0};
        state->gripper.holding = false;
        char pattern[] = "/tmp/grasp-device-test.XXXXXX";
        const char* path = mkdtemp(pattern);
        if (!path) throw std::runtime_error("mkdtemp failed");
        directory = path;
    }
    ~Fixture() { std::filesystem::remove_all(directory); }
    std::unique_ptr<perceptive_grasp::ExecutorDevice> Device() {
        manipulation::Devices devices;
        devices.arm = std::make_unique<Arm>(state);
        devices.effector = std::make_unique<Gripper>(state);
        return std::make_unique<perceptive_grasp::ExecutorDevice>(
            std::make_unique<manipulation::DeviceSession>(definition, std::move(devices), directory));
    }
};

template <typename Action> void Reject(Action action) {
    bool rejected = false;
    try { action(); } catch (const std::exception&) { rejected = true; }
    assert(rejected);
}

void ConnectionAndOwnership() {
    Fixture fixture;
    fixture.state->arm.enabled = false;
    fixture.state->arm.position_mode = false;
    fixture.state->gripper.enabled = false;
    auto device = fixture.Device();
    device->Connect();
    // Connection alone still takes no torque: the read-only path must stay read-only.
    assert(fixture.state->arm_writes == 0 && fixture.state->gripper_writes == 0);
    assert(fixture.state->torque_writes == 0 && fixture.state->holds == 0);
    assert(!device->ReadArm().enabled && !device->ReadGripper().enabled.value());
    assert(fixture.state->holds == 0 && fixture.state->arm_writes == 0);
    auto competing = fixture.Device();
    Reject([&] { competing->Connect(); });
    Reject([&] { device->FollowArm({0.1f}, 0.1); });
    assert(fixture.state->arm_writes == 0 && fixture.state->torque_writes == 0);
    assert(fixture.state->holds == 0);
}

void EnableTakesTorqueOnce() {
    Fixture fixture;
    // A freshly powered rig reports torque released on both roles.
    fixture.state->arm.enabled = false;
    fixture.state->gripper.enabled = false;
    auto device = fixture.Device();
    device->Connect();
    assert(fixture.state->torque_writes == 0);
    // One explicit enable writes torque to the arm and the jaws, and nothing else.
    device->Enable();
    assert(fixture.state->torque_writes == 2);
    assert(fixture.state->arm_writes == 0 && fixture.state->gripper_writes == 0);
    assert(fixture.state->holds == 0);
    assert(device->ReadArm().enabled && device->ReadGripper().enabled.value());
    // Already enabled: both drivers return before writing, so a repeat must not write again.
    device->Enable();
    assert(fixture.state->torque_writes == 2);
    assert(!device->Failed());
}

void EnableWithoutConnectionOrInPositionModeIsRefused() {
    Fixture fixture;
    auto device = fixture.Device();
    Reject([&] { device->Enable(); });
    assert(fixture.state->torque_writes == 0 && fixture.state->holds == 0);
    device->Connect();
    // A device that cannot confirm position mode must not be handed torque silently.
    fixture.state->arm.position_mode = false;
    Reject([&] { device->Enable(); });
    assert(device->Failed());
}

void FollowAndHold() {
    Fixture fixture;
    auto device = fixture.Device();
    device->Connect();
    device->FollowArm({0.2f}, 0.5);
    assert(fixture.state->arm_writes == 1 && fixture.state->speed == 0.5);
    assert(std::abs(device->ReadArm().joints_rad[0] - 0.2) < 1e-6);
    device->FollowGripper(0.6, 0.5);
    assert(device->ReadGripper().positions[0] == 0.6);
    fixture.state->gripper.positions = {0.12};
    fixture.state->gripper.loads = {-0.2};
    fixture.state->gripper.holding = true;
    assert(device->ReadGripper().holding == true);
    const auto arm_writes = fixture.state->arm_writes;
    const auto hand_writes = fixture.state->gripper_writes;
    assert(device->Hold());
    assert(fixture.state->arm_writes == arm_writes);
    assert(fixture.state->gripper_writes == hand_writes);
    assert(fixture.state->torque_writes == 0);
}

void ReadFailure(bool stale, bool active) {
    Fixture fixture;
    auto device = fixture.Device();
    device->Connect();
    if (active) device->FollowArm({0.1f}, 0.5);
    device->ReadArm();
    fixture.state->stale = stale;
    fixture.state->frozen = !stale;
    Reject([&] { device->ReadArm(); });
    assert(device->Failed());
    assert(active ? fixture.state->holds >= 2 : fixture.state->holds == 0);
    const int writes = fixture.state->arm_writes;
    Reject([&] { device->FollowArm({0.1f}, 0.5); });
    assert(fixture.state->arm_writes == writes);
}

void ReadOnlyFaultDoesNotCommandHold() {
    Fixture fixture;
    auto device = fixture.Device();
    device->Connect();
    fixture.state->arm.fault = true;
    Reject([&] { device->ReadArm(); });
    assert(device->Failed() && fixture.state->holds == 0);
    assert(fixture.state->arm_writes == 0 && fixture.state->gripper_writes == 0);
    assert(fixture.state->torque_writes == 0);
}

void MissingEvidence() {
    Fixture fixture;
    auto device = fixture.Device();
    device->Connect();
    fixture.state->gripper.loads.clear();
    Reject([&] { device->ReadGripper(); });
    assert(device->Failed());
    assert(fixture.state->holds == 0);
    Reject([&] { device->FollowGripper(1, 0.5); });
    assert(fixture.state->gripper_writes == 0);
}

void UncalibratedAndUnconfirmedHold() {
    Fixture fixture;
    fixture.state->arm.joint_reference_verified = false;
    auto device = fixture.Device();
    Reject([&] { device->Connect(); });
    assert(fixture.state->arm_writes == 0 && fixture.state->gripper_writes == 0);
    assert(fixture.state->torque_writes == 0 && fixture.state->holds == 0);
    // Failed validation releases ownership, without resetting the physical reference.
    fixture.state->arm.joint_reference_verified = true;
    auto ready = fixture.Device();
    ready->Connect();
    ready->FollowGripper(0.1, 0.5);
    fixture.state->hold_succeeds = false;
    assert(!ready->Hold() && ready->Failed());
    const int writes = fixture.state->gripper_writes;
    Reject([&] { ready->FollowGripper(1.0, 0.5); });
    assert(fixture.state->gripper_writes == writes);
    assert(fixture.state->torque_writes == 0);
}

void StopLatchesAfterInFlightWrite() {
    Fixture fixture;
    auto device = fixture.Device();
    device->Connect();
    fixture.state->block_arm = true;
    auto following = std::async(std::launch::async, [&] { device->FollowArm({0.1f}, 0.5); });
    {
        std::unique_lock<std::mutex> guard(fixture.state->pause_mutex);
        assert(fixture.state->pause.wait_for(guard, std::chrono::seconds(2),
            [&] { return fixture.state->arm_entered; }));
    }
    auto stopping = std::async(std::launch::async, [&] { return device->Stop(); });
    // Stop must await the existing submit before holding and reporting completion.
    assert(stopping.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout);
    {
        std::lock_guard<std::mutex> guard(fixture.state->pause_mutex);
        fixture.state->release_arm = true;
        fixture.state->pause.notify_all();
    }
    following.get();
    assert(stopping.get() && device->Failed());
    const int arm_writes = fixture.state->arm_writes;
    const int hand_writes = fixture.state->gripper_writes;
    const int holds = fixture.state->holds;
    Reject([&] { device->FollowArm({0.2f}, 0.5); });
    Reject([&] { device->FollowGripper(1, 0.5); });
    Reject([&] { device->ReadArm(); });
    Reject([&] { device->ReadGripper(); });
    Reject([&] { device->Connect(); });
    assert(fixture.state->arm_writes == arm_writes && arm_writes == 1);
    assert(fixture.state->gripper_writes == hand_writes);
    assert(fixture.state->torque_writes == 0);
    assert(fixture.state->holds == holds);
}

void PlannerLimitsDoNotExpandDeviceRange() {
    Fixture fixture;
    fixture.definition.arm.joints[0].minimum_rad = -1.84;
    fixture.definition.arm.joints[0].maximum_rad = 1.43;
    const auto bounds = perceptive_grasp::PlannerJointRange(-1.84, 1.43);
    assert(bounds.first >= -1.84 && bounds.second <= 1.43);
    assert(bounds.first <= bounds.second);
    auto device = fixture.Device();
    device->Connect();
    device->FollowArm({bounds.first}, 0.5);
    device->FollowArm({bounds.second}, 0.5);
    assert(fixture.state->arm_writes == 2 && !device->Failed());
    Reject([] { perceptive_grasp::PlannerJointRange(2, 1); });
}

void RefusedContinuationAndPartialWrite() {
    Fixture fixture;
    auto device = fixture.Device();
    device->Connect();
    device->FollowArm({0.1f}, 0.5);
    fixture.state->reject_arm = true;
    Reject([&] { device->FollowArm({0.2f}, 0.5); });
    assert(device->Failed() && fixture.state->holds >= 2);
    fixture.state->reject_arm = false;
    Reject([&] { device->FollowGripper(1, 0.5); });
    assert(fixture.state->gripper_writes == 0);

    Fixture partial;
    auto hand = partial.Device();
    hand->Connect();
    partial.state->throw_gripper = true;
    Reject([&] { hand->FollowGripper(0.2, 0.5); });
    assert(hand->Failed() && partial.state->gripper_writes == 1);
    assert(partial.state->holds == 1);  // DeviceSession stops the partially written effector.
    Reject([&] { hand->FollowArm({0.1f}, 0.5); });
    assert(partial.state->arm_writes == 0);

    Fixture partial_arm;
    auto arm = partial_arm.Device();
    arm->Connect();
    partial_arm.state->throw_arm = true;
    Reject([&] { arm->FollowArm({0.1f}, 0.5); });
    assert(arm->Failed() && partial_arm.state->arm_writes == 1);
    assert(partial_arm.state->holds == 1);  // No prior accepted target; the public layer stops it.
    Reject([&] { arm->FollowGripper(0.2, 0.5); });
    assert(partial_arm.state->gripper_writes == 0);
}
}  // namespace

int main() {
    ConnectionAndOwnership();
    EnableTakesTorqueOnce();
    EnableWithoutConnectionOrInPositionModeIsRefused();
    FollowAndHold();
    ReadFailure(false, false);
    ReadFailure(true, false);
    ReadFailure(false, true);
    ReadFailure(true, true);
    ReadOnlyFaultDoesNotCommandHold();
    MissingEvidence();
    UncalibratedAndUnconfirmedHold();
    StopLatchesAfterInFlightWrite();
    PlannerLimitsDoNotExpandDeviceRange();
    RefusedContinuationAndPartialWrite();
    std::cout << "executor device migration passed\n";
}
