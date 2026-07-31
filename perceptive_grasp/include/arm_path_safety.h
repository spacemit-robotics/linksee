/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file arm_path_safety.h
 * @brief URDF-based arm clearance checks for support surfaces.
 */

#ifndef ARM_PATH_SAFETY_H
#define ARM_PATH_SAFETY_H

#include <memory>
#include <string>
#include <vector>

namespace perceptive_grasp {

struct SupportPlane {
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_z = 1.0f;
    float d = 0.0f;
    bool valid = false;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    bool bounds_valid = false;
};

struct ArmPathSafetyResult {
    bool safe = false;
    float minimum_clearance_m = 0.0f;
    std::string link_name;
    int sample_index = -1;
    std::string detail;
};

class ArmPathSafety {
public:
    ArmPathSafety();
    ~ArmPathSafety();

    ArmPathSafety(const ArmPathSafety&) = delete;
    ArmPathSafety& operator=(const ArmPathSafety&) = delete;

    bool Init(const std::string& urdf_path,
        const std::string& base_link,
        const std::string& tip_link,
        std::string* error);

    ArmPathSafetyResult CheckConfiguration(
        const std::vector<float>& joints,
        const SupportPlane& support_plane,
        float required_clearance_m) const;

    ArmPathSafetyResult CheckPath(
        const std::vector<float>& start_joints,
        const std::vector<float>& target_joints,
        const SupportPlane& support_plane,
        float required_clearance_m,
        float maximum_joint_step_rad) const;

    ArmPathSafetyResult CheckSelfCollisionPath(
        const std::vector<float>& start_joints,
        const std::vector<float>& target_joints,
        float maximum_joint_step_rad) const;

    ArmPathSafetyResult CheckContactRetreatPath(
        const std::vector<float>& start_joints,
        const std::vector<float>& target_joints,
        const SupportPlane& support_plane,
        float required_clearance_m,
        float maximum_joint_step_rad,
        float maximum_start_penetration_m,
        float maximum_clearance_regression_m) const;

private:
    ArmPathSafetyResult CheckPathInternal(
        const std::vector<float>& start_joints,
        const std::vector<float>& target_joints,
        const SupportPlane& support_plane,
        float required_clearance_m,
        float maximum_joint_step_rad,
        bool check_support_surface) const;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace perceptive_grasp

#endif  // ARM_PATH_SAFETY_H
