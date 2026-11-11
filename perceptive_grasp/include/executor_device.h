// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file executor_device.h
 * @brief Public-device adapter used by the physical grasp executor.
 */
#ifndef EXECUTOR_DEVICE_H
#define EXECUTOR_DEVICE_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace spacemit::manipulation {
class DeviceSession;
struct ArmState;
struct EffectorState;
}

namespace perceptive_grasp {

/// Narrow a double device range to float planner bounds without enlarging it.
std::pair<float, float> PlannerJointRange(double lower, double upper);

/** Serialized device access. No connection or read implicitly enables, homes or clears faults. */
class ExecutorDevice {
public:
    explicit ExecutorDevice(std::unique_ptr<spacemit::manipulation::DeviceSession> session);
    ~ExecutorDevice();
    /// Validate a registered assembly offline, without connecting it.
    static std::unique_ptr<ExecutorDevice> Create(const std::string& plan_path);
    void Connect();
    const std::vector<std::string>& JointNames() const;
    std::vector<double> LowerLimits() const;
    std::vector<double> UpperLimits() const;
    /// Fresh read-only feedback; invalid samples latch and hold only previously accepted targets.
    spacemit::manipulation::ArmState ReadArm();
    spacemit::manipulation::EffectorState ReadGripper();
    /// Retarget through the public target-following capability, preserving application pacing.
    void FollowArm(const std::vector<float>& joints, double speed_scale);
    void FollowGripper(double position, double duration_s);
    /// Hold both roles, never release torque or open a potentially occupied gripper.
    bool Hold() noexcept;
    /// Latch before waiting for an in-flight write; later following targets are refused.
    bool Stop() noexcept;
    /// Latch, hold previously accepted targets, and reject writes until explicit reconnection.
    void Fail() noexcept;
    bool Failed() const;

private:
    std::unique_ptr<spacemit::manipulation::DeviceSession> session_;
    std::vector<std::string> joints_;
    mutable std::recursive_mutex mutex_;
    std::atomic<bool> failed_{false};
    bool connected_ = false;
    bool active_commands_ = false;
    unsigned long long arm_sample_ = 0;
    unsigned long long gripper_sample_ = 0;
};

}  // namespace perceptive_grasp
#endif  // EXECUTOR_DEVICE_H
