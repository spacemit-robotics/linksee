// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file executor_kinematics.cpp
 * @brief Model-resolved public FK/IK with the existing grasp planner's explicit seeds.
 */
#include "executor_kinematics.h"

#include <algorithm>
#include <filesystem>  // NOLINT(build/c++17)
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include "manipulation/kinematics.h"

namespace perceptive_grasp {
namespace manipulation = spacemit::manipulation;

ExecutorKinematics::ExecutorKinematics(const std::string& model_path, const std::string& solver,
                                        const std::vector<std::string>& joint_names) {
    if (model_path.empty()) throw std::runtime_error("manipulator.model is required");
    const auto path = std::filesystem::canonical(model_path);
    const auto model = YAML::LoadFile(path.string());
    manipulation::KinematicsConfig config;
    urdf_path_ = std::filesystem::canonical(
        path.parent_path() / model["urdf"].as<std::string>()).string();
    base_link_ = model["base"].as<std::string>();
    tip_link_ = model["tip"].as<std::string>();
    config.urdf_path = urdf_path_;
    config.base_link = base_link_;
    config.tip_link = tip_link_;
    config.joint_names = model["joints"].as<std::vector<std::string>>();
    if ((!joint_names.empty() && config.joint_names != joint_names) ||
        config.joint_names.size() > kKinematicsMaximumJoints) {
        throw std::runtime_error("model joint order must match the device plan exactly");
    }
    solver_ = manipulation::MakeBuiltinKinematicsRegistry().Create(solver, config);
}

ExecutorKinematics::~ExecutorKinematics() = default;
int ExecutorKinematics::Count() const { return static_cast<int>(solver_->JointNames().size()); }

int ExecutorKinematics::Limits(double* lower, double* upper) const {
    if (!lower || !upper) return -1;
    std::copy(solver_->LowerLimits().begin(), solver_->LowerLimits().end(), lower);
    std::copy(solver_->UpperLimits().begin(), solver_->UpperLimits().end(), upper);
    return kKinematicsOk;
}

int ExecutorKinematics::Forward(const PlanningJoints& joints, PlanningPose* pose) const {
    if (!pose || joints.count != Count()) return -1;
    try {
        const auto result = solver_->Forward(std::vector<double>(joints.q, joints.q + joints.count));
        *pose = {result.x, result.y, result.z, result.qw, result.qx, result.qy, result.qz};
        return kKinematicsOk;
    } catch (...) { return -1; }
}

int ExecutorKinematics::Inverse(const PlanningPose& target, const PlanningJoints* seed,
                                const PlanningInverseOptions* options,
                                PlanningJoints* result) const {
    if (!result || (seed && seed->count != Count())) return -1;
    manipulation::InverseOptions parameters;
    parameters.max_iterations = 200;
    parameters.epsilon = 5e-4;
    if (options) {
        if (options->timeout_s > 0) parameters.timeout_s = options->timeout_s;
        if (options->epsilon > 0) parameters.epsilon = options->epsilon;
        if (options->position_weight > 0) parameters.position_weight = options->position_weight;
    }
    try {
        const auto solved = solver_->Inverse(
            {target.x, target.y, target.z, target.qw, target.qx, target.qy, target.qz},
            seed ? std::vector<double>(seed->q, seed->q + seed->count) :
                std::vector<double>(Count(), 0.0),
            parameters);
        if (!solved || solved->size() != static_cast<std::size_t>(Count())) return -1;
        result->count = static_cast<std::uint8_t>(solved->size());
        std::copy(solved->begin(), solved->end(), result->q);
        return kKinematicsOk;
    } catch (...) { return -1; }
}

const std::string& ExecutorKinematics::UrdfPath() const { return urdf_path_; }
const std::string& ExecutorKinematics::BaseLink() const { return base_link_; }
const std::string& ExecutorKinematics::TipLink() const { return tip_link_; }
int PlanningJointCount(const ExecutorKinematics* solver) { return solver ? solver->Count() : 0; }
int PlanningJointLimits(const ExecutorKinematics* solver, double* lower, double* upper) {
    return solver ? solver->Limits(lower, upper) : -1;
}
int PlanningForward(const ExecutorKinematics* solver, const PlanningJoints* joints,
                    PlanningPose* pose) {
    return solver && joints ? solver->Forward(*joints, pose) : -1;
}
int PlanningInverse(const ExecutorKinematics* solver, const PlanningPose* target,
                    const PlanningJoints* seed, const PlanningInverseOptions* options,
                    PlanningJoints* result) {
    return solver && target ? solver->Inverse(*target, seed, options, result) : -1;
}
}  // namespace perceptive_grasp
