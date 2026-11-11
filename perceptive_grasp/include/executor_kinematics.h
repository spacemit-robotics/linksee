// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file executor_kinematics.h
 * @brief Fixed-size planning values bridged to the public kinematics capability.
 */
#ifndef EXECUTOR_KINEMATICS_H
#define EXECUTOR_KINEMATICS_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace spacemit::manipulation { class Kinematics; }
namespace perceptive_grasp {

constexpr int kKinematicsMaximumJoints = 16;
constexpr int kKinematicsOk = 0;
struct PlanningJoints {
    std::uint8_t count = 0;
    double q[kKinematicsMaximumJoints] = {};
};
struct PlanningPose { double x, y, z, qw, qx, qy, qz; };
struct PlanningInverseOptions {
    double timeout_s = 0;
    double epsilon = 0;
    double position_weight = 0;
};

/** Keeps the grasp planner's seed and deadline semantics independent of the chosen solver. */
class ExecutorKinematics {
public:
    ExecutorKinematics(const std::string& model_path, const std::string& solver,
                        const std::vector<std::string>& joint_names = {});
    ~ExecutorKinematics();
    int Count() const;
    int Limits(double* lower, double* upper) const;
    int Forward(const PlanningJoints& joints, PlanningPose* pose) const;
    int Inverse(const PlanningPose& target, const PlanningJoints* seed,
                const PlanningInverseOptions* options, PlanningJoints* result) const;
    const std::string& UrdfPath() const;
    const std::string& BaseLink() const;
    const std::string& TipLink() const;

private:
    std::unique_ptr<spacemit::manipulation::Kinematics> solver_;
    std::string urdf_path_;
    std::string base_link_;
    std::string tip_link_;
};

// These adapters deliberately retain the existing planner's explicit status checks.
int PlanningJointCount(const ExecutorKinematics* solver);
int PlanningJointLimits(const ExecutorKinematics* solver, double* lower, double* upper);
int PlanningForward(const ExecutorKinematics* solver, const PlanningJoints* joints,
                    PlanningPose* pose);
int PlanningInverse(const ExecutorKinematics* solver, const PlanningPose* target,
                    const PlanningJoints* seed, const PlanningInverseOptions* options,
                    PlanningJoints* result);

}  // namespace perceptive_grasp
#endif  // EXECUTOR_KINEMATICS_H
