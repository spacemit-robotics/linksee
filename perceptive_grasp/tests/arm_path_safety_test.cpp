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

    std::remove(urdf_path.c_str());
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

    std::cout << "arm path safety test passed: "
            << path_result.detail << std::endl;
    return 0;
}
