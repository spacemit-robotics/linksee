/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file arm_path_safety_test.cpp
 * @brief Tests support-plane clearance along interpolated arm paths.
 */

#include "arm_path_safety.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

using perceptive_grasp::ArmPathSafety;
using perceptive_grasp::ArmPathSafetyResult;
using perceptive_grasp::SupportPlane;

namespace {

std::string WriteTestUrdf() {
    const std::string path =
        "/tmp/perceptive_grasp_path_safety_" +
        std::to_string(static_cast<long long>(getpid())) + ".urdf";
    std::ofstream output(path);
    output << R"(<robot name="path_safety_test">
    <link name="base_link"/>
    <joint name="arm_joint" type="revolute">
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="1" velocity="1" lower="-3.2" upper="3.2"/>
    </joint>
    <link name="arm_link">
    <collision>
        <origin xyz="0.10 0 0" rpy="0 0 0"/>
        <geometry><box size="0.20 0.02 0.02"/></geometry>
    </collision>
    </link>
</robot>)";
    return path;
}

std::string WriteCollisionTestUrdf() {
    const std::string path =
        "/tmp/perceptive_grasp_self_collision_" +
        std::to_string(static_cast<long long>(getpid())) + ".urdf";
    std::ofstream output(path);
    output << R"(<robot name="self_collision_test">
    <link name="base_link">
    <collision><geometry><box size="0.10 0.10 0.10"/></geometry></collision>
    </link>
    <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" velocity="1" lower="-3.2" upper="3.2"/>
    </joint>
    <link name="arm_link">
    <collision>
        <origin xyz="0.15 0 0"/>
        <geometry><box size="0.10 0.02 0.02"/></geometry>
    </collision>
    </link>
    <joint name="spacer_joint" type="fixed">
    <parent link="arm_link"/>
    <child link="spacer_link"/>
    </joint>
    <link name="spacer_link"/>
    <joint name="tip_joint" type="fixed">
    <parent link="spacer_link"/>
    <child link="tip_link"/>
    </joint>
    <link name="tip_link">
    <collision>
        <origin xyz="0.15 0 0"/>
        <geometry><box size="0.04 0.04 0.04"/></geometry>
    </collision>
    </link>
</robot>)";
    return path;
}

std::string WriteNearLinkCollisionTestUrdf(
    const std::string& suffix, float separation_m) {
    const std::string path =
        "/tmp/perceptive_grasp_near_link_" + suffix + "_" +
        std::to_string(static_cast<long long>(getpid())) + ".urdf";
    std::ofstream output(path);
    output << R"(<robot name="near_link_collision_test">
    <link name="base_link"/>
    <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" velocity="1" lower="-1" upper="1"/>
    </joint>
    <link name="shoulder_link">
    <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
    </link>
    <joint name="upper_joint" type="fixed">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    </joint>
    <link name="upper_arm_link"/>
    <joint name="lower_joint" type="fixed">
    <origin xyz=")"
        << separation_m << R"( 0 0"/>
    <parent link="upper_arm_link"/>
    <child link="lower_arm_link"/>
    </joint>
    <link name="lower_arm_link">
    <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
    </link>
</robot>)";
    return path;
}

}  // namespace

int main() {
    const std::string urdf_path = WriteTestUrdf();
    ArmPathSafety safety;
    std::string error;
    if (!safety.Init(urdf_path, "base_link", "arm_link", &error)) {
        std::cerr << "failed to initialize test URDF: " << error << std::endl;
        std::remove(urdf_path.c_str());
        return 1;
    }

    SupportPlane plane;
    plane.valid = true;
    const std::vector<float> start{0.0f};
    const std::vector<float> target{3.14159265f};
    const ArmPathSafetyResult start_result =
        safety.CheckConfiguration(start, plane, 0.005f);
    const ArmPathSafetyResult target_result =
        safety.CheckConfiguration(target, plane, 0.005f);
    const ArmPathSafetyResult path_result =
        safety.CheckPath(start, target, plane, 0.005f, 0.05f);

    SupportPlane distant_plane = plane;
    distant_plane.bounds_valid = true;
    distant_plane.min_x = 0.50f;
    distant_plane.max_x = 0.60f;
    distant_plane.min_y = 0.50f;
    distant_plane.max_y = 0.60f;
    const ArmPathSafetyResult distant_path_result =
        safety.CheckPath(start, target, distant_plane, 0.005f, 0.05f);

    const ArmPathSafetyResult missing_plane_result =
        safety.CheckPath(start, target, SupportPlane{}, 0.005f, 0.05f);
    const ArmPathSafetyResult self_collision_only_result =
        safety.CheckSelfCollisionPath(start, target, 0.05f);

    float contact_angle = 0.0f;
    ArmPathSafetyResult contact_configuration;
    for (int step = 1; step <= 100; ++step) {
        const float angle = 0.01f * static_cast<float>(step);
        const ArmPathSafetyResult candidate =
            safety.CheckConfiguration({angle}, plane, -0.005f);
        if (candidate.safe && candidate.minimum_clearance_m < 0.0f) {
            contact_angle = angle;
            contact_configuration = candidate;
            break;
        }
    }
    if (contact_angle <= 0.0f) {
        std::cerr << "failed to construct a contact retreat fixture"
            << std::endl;
        std::remove(urdf_path.c_str());
        return 1;
    }
    const std::vector<float> contact_start{contact_angle};
    const ArmPathSafetyResult strict_contact_result =
        safety.CheckPath(contact_start, start, plane, 0.005f, 0.01f);
    const ArmPathSafetyResult contact_retreat_result =
        safety.CheckContactRetreatPath(
            contact_start, start, plane, 0.005f, 0.01f, 0.005f, 0.001f);
    const ArmPathSafetyResult deeper_contact_result =
        safety.CheckContactRetreatPath(
            contact_start, {contact_angle + 0.10f}, plane,
            0.005f, 0.01f, 0.005f, 0.001f);

    const std::string collision_urdf_path = WriteCollisionTestUrdf();
    ArmPathSafety collision_safety;
    if (!collision_safety.Init(
            collision_urdf_path, "base_link", "tip_link", &error)) {
        std::cerr << "failed to initialize collision URDF: " << error
            << std::endl;
        std::remove(urdf_path.c_str());
        std::remove(collision_urdf_path.c_str());
        return 1;
    }
    const ArmPathSafetyResult collision_result =
        collision_safety.CheckConfiguration(start, distant_plane, 0.005f);
    const ArmPathSafetyResult self_collision_only_collision_result =
        collision_safety.CheckSelfCollisionPath(start, start, 0.05f);

    const std::string near_touch_urdf_path =
        WriteNearLinkCollisionTestUrdf("touch", 0.034f);
    ArmPathSafety near_touch_safety;
    if (!near_touch_safety.Init(
            near_touch_urdf_path, "base_link", "lower_arm_link", &error)) {
        std::cerr << "failed to initialize near-link touch URDF: " << error
            << std::endl;
        std::remove(urdf_path.c_str());
        std::remove(collision_urdf_path.c_str());
        std::remove(near_touch_urdf_path.c_str());
        return 1;
    }
    const ArmPathSafetyResult near_touch_result =
        near_touch_safety.CheckConfiguration(
            start, distant_plane, 0.005f);

    const std::string near_overlap_urdf_path =
        WriteNearLinkCollisionTestUrdf("overlap", 0.020f);
    ArmPathSafety near_overlap_safety;
    if (!near_overlap_safety.Init(
            near_overlap_urdf_path, "base_link", "lower_arm_link", &error)) {
        std::cerr << "failed to initialize near-link overlap URDF: " << error
            << std::endl;
        std::remove(urdf_path.c_str());
        std::remove(collision_urdf_path.c_str());
        std::remove(near_touch_urdf_path.c_str());
        std::remove(near_overlap_urdf_path.c_str());
        return 1;
    }
    const ArmPathSafetyResult near_overlap_result =
        near_overlap_safety.CheckConfiguration(
            start, distant_plane, 0.005f);

    ArmPathSafety production_safety;
    if (!production_safety.Init(
            PERCEPTIVE_GRASP_TEST_URDF,
            "base_link", "gripper_frame_link", &error)) {
        std::cerr << "failed to initialize production URDF: " << error
            << std::endl;
        std::remove(urdf_path.c_str());
        std::remove(collision_urdf_path.c_str());
        return 1;
    }
    SupportPlane production_plane;
    production_plane.valid = true;
    production_plane.bounds_valid = true;
    production_plane.min_x = 0.80f;
    production_plane.max_x = 0.90f;
    production_plane.min_y = 0.80f;
    production_plane.max_y = 0.90f;
    const std::vector<std::vector<float>> production_poses = {
        {1.550f, -1.620f, 1.420f, 1.147f, 0.189f},
        {1.550f, 0.050f, -0.217f, 1.606f, 0.015f},
        {1.550f, 0.021f, 1.400f, -1.700f, -0.036f},
        {-1.480f, 0.087f, -0.140f, 1.389f, 0.033f},
        {1.547f, -1.630f, 1.400f, 1.145f, 0.186f},
    };
    for (size_t index = 0; index < production_poses.size(); ++index) {
        const ArmPathSafetyResult production_result =
            production_safety.CheckConfiguration(
                production_poses[index], production_plane, 0.005f);
        if (!production_result.safe) {
            std::cerr << "production pose " << index
                << " was rejected: " << production_result.detail
                << std::endl;
            std::remove(urdf_path.c_str());
            std::remove(collision_urdf_path.c_str());
            return 1;
        }
    }
    const ArmPathSafetyResult folded_transition_result =
        production_safety.CheckPath(
            production_poses.back(), production_poses[1],
            production_plane, 0.005f, 0.040f);
    if (!folded_transition_result.safe) {
        std::cerr << "known-safe folded transition was rejected: "
            << folded_transition_result.detail << std::endl;
        std::remove(urdf_path.c_str());
        std::remove(collision_urdf_path.c_str());
        return 1;
    }
    std::vector<float> shoulder_lift_pose = production_poses.back();
    shoulder_lift_pose[1] = -0.334f;
    const ArmPathSafetyResult shoulder_lift_result =
        production_safety.CheckPath(
            production_poses.back(), shoulder_lift_pose,
            production_plane, 0.005f, 0.040f);
    if (!shoulder_lift_result.safe) {
        std::cerr << "calibrated body-avoidance lift was rejected: "
            << shoulder_lift_result.detail << std::endl;
        std::remove(urdf_path.c_str());
        std::remove(collision_urdf_path.c_str());
        return 1;
    }

    std::remove(urdf_path.c_str());
    std::remove(collision_urdf_path.c_str());
    std::remove(near_touch_urdf_path.c_str());
    std::remove(near_overlap_urdf_path.c_str());
    if (!start_result.safe || !target_result.safe) {
        std::cerr << "safe endpoint was rejected" << std::endl;
        return 1;
    }
    if (path_result.safe || path_result.sample_index <= 0 ||
        path_result.link_name != "arm_link") {
        std::cerr << "unsafe interpolated path was not rejected" << std::endl;
        return 1;
    }
    if (!distant_path_result.safe) {
        std::cerr << "finite support bounds rejected a distant path"
                    << std::endl;
        return 1;
    }
    if (missing_plane_result.safe ||
        missing_plane_result.detail.find("unavailable") == std::string::npos) {
        std::cerr << "missing support plane was not rejected" << std::endl;
        return 1;
    }
    if (!self_collision_only_result.safe) {
        std::cerr << "self-collision-only path rejected a safe path: "
            << self_collision_only_result.detail << std::endl;
        return 1;
    }
    if (strict_contact_result.safe ||
        contact_configuration.minimum_clearance_m >= 0.0f) {
        std::cerr << "strict path accepted a support contact start"
            << std::endl;
        return 1;
    }
    if (!contact_retreat_result.safe) {
        std::cerr << "safe contact retreat was rejected: "
            << contact_retreat_result.detail << std::endl;
        return 1;
    }
    if (deeper_contact_result.safe) {
        std::cerr << "deeper support motion was not rejected: "
            << deeper_contact_result.detail << std::endl;
        return 1;
    }
    if (collision_result.safe ||
        collision_result.detail.find("collision between") ==
            std::string::npos) {
        std::cerr << "self or body collision was not rejected: safe="
            << collision_result.safe
            << " detail=" << collision_result.detail << std::endl;
        return 1;
    }
    if (self_collision_only_collision_result.safe ||
        self_collision_only_collision_result.detail.find(
            "collision between") == std::string::npos) {
        std::cerr << "self-collision-only path accepted a collision: "
            << self_collision_only_collision_result.detail << std::endl;
        return 1;
    }
    if (!near_touch_result.safe) {
        std::cerr << "conservative near-link box contact was rejected: "
            << near_touch_result.detail << std::endl;
        return 1;
    }
    if (near_overlap_result.safe ||
        near_overlap_result.detail.find("collision between") ==
            std::string::npos) {
        std::cerr << "clear near-link overlap was not rejected: safe="
            << near_overlap_result.safe
            << " detail=" << near_overlap_result.detail << std::endl;
        return 1;
    }

    std::cout << "arm path safety test passed: "
            << path_result.detail << std::endl;
    return 0;
}
