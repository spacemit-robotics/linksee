/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file debug_ik.cpp
    * @brief 仅做 IK 求解验证，不执行机械臂运动
    */

#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "executor_kinematics.h"
using perceptive_grasp::ExecutorKinematics;
using perceptive_grasp::PlanningPose;
using perceptive_grasp::PlanningJoints;
using perceptive_grasp::PlanningInverse;
using perceptive_grasp::kKinematicsOk;

namespace fs = std::filesystem;

struct Pose3D {
    float x, y, z;
    float qw, qx, qy, qz;
};

struct DebugIkAppConfig {
    std::string config_path;
    bool check_model = false;
    Pose3D pose{0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
};

struct JointConstraint {
    int joint_index = -1;
    float min_rad = 0.0f;
    float max_rad = 0.0f;
};

struct ExecutorConfig {
    std::string model;
    std::string solver = "pinocchio";
    std::string urdf_path;
    std::string base_link = "base_link";
    std::string tip_link = "gripper_frame_link";
    int ik_max_trials = 50;
    std::vector<JointConstraint> joint_constraints;
};

static DebugIkAppConfig ParseArgs(int argc, char* argv[]) {
    DebugIkAppConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--help") || (arg == "-h")) {
            std::cout << "Usage: " << argv[0]
                        << " --config <yaml> [--check-model | --pose x y z qw qx qy qz]\n";
            std::exit(0);
        } else if (arg == "--config" && i + 1 < argc) {
            cfg.config_path = argv[++i];
        } else if (arg == "--check-model") {
            cfg.check_model = true;
        } else if (arg == "--pose" && i + 7 < argc) {
            cfg.pose.x = std::stof(argv[++i]);
            cfg.pose.y = std::stof(argv[++i]);
            cfg.pose.z = std::stof(argv[++i]);
            cfg.pose.qw = std::stof(argv[++i]);
            cfg.pose.qx = std::stof(argv[++i]);
            cfg.pose.qy = std::stof(argv[++i]);
            cfg.pose.qz = std::stof(argv[++i]);
        }
    }
    return cfg;
}

static ExecutorConfig LoadExecutorConfig(const std::string& pipeline_config) {
    YAML::Node root = YAML::LoadFile(pipeline_config);
    ExecutorConfig cfg;

    auto m = root["manipulator"];
    if (m) {
        cfg.model = m["model"].as<std::string>("");
        if (!cfg.model.empty()) cfg.model =
            fs::absolute(fs::path(pipeline_config).parent_path() / cfg.model).string();
        cfg.solver = m["kinematics_solver"].as<std::string>(cfg.solver);
        cfg.urdf_path = m["urdf_path"].as<std::string>(cfg.urdf_path);
        cfg.base_link = m["base_link"].as<std::string>(cfg.base_link);
        cfg.tip_link = m["tip_link"].as<std::string>(cfg.tip_link);

        cfg.ik_max_trials = m["ik_max_trials"].as<int>(cfg.ik_max_trials);

        if (auto jc = m["joint_constraints"]) {
            cfg.joint_constraints.clear();
            for (size_t i = 0; i < jc.size(); ++i) {
                JointConstraint c;
                c.joint_index = jc[i]["joint"].as<int>(-1);
                c.min_rad = jc[i]["min"].as<float>(0.0f);
                c.max_rad = jc[i]["max"].as<float>(0.0f);
                if (c.joint_index >= 0) {
                    cfg.joint_constraints.push_back(c);
                }
            }
        }
    }

    if (!cfg.urdf_path.empty() && !fs::path(cfg.urdf_path).is_absolute()) {
        fs::path config_dir = fs::path(pipeline_config).parent_path();
        fs::path resolved = config_dir / cfg.urdf_path;
        if (fs::exists(resolved)) {
            cfg.urdf_path = fs::canonical(resolved).string();
        }
    }

    return cfg;
}

int main(int argc, char* argv[]) {
    DebugIkAppConfig app = ParseArgs(argc, argv);
    if (app.config_path.empty()) {
        std::cerr << "[debug_ik] Error: --config is required" << std::endl;
        return 1;
    }

    ExecutorConfig cfg = LoadExecutorConfig(app.config_path);
    if (cfg.model.empty()) {
        std::cerr << "[debug_ik] Error: manipulator.model is empty" << std::endl;
        return 1;
    }

    std::unique_ptr<ExecutorKinematics> kin;
    try {
        kin = std::make_unique<ExecutorKinematics>(cfg.model, cfg.solver);
    } catch (const std::exception& error) {
        std::cerr << "[debug_ik] " << error.what() << std::endl;
        return 1;
    }
    if (app.check_model) {
        std::cout << "model valid: " << kin->Count() << " joints\n";
        return 0;
    }

    std::cout << "[debug_ik] urdf: " << kin->UrdfPath() << std::endl;
    std::cout << "[debug_ik] pose: ["
                << app.pose.x << ", " << app.pose.y << ", " << app.pose.z << "]"
                << " quat=[" << app.pose.qw << ", " << app.pose.qx << ", "
                << app.pose.qy << ", " << app.pose.qz << "]" << std::endl;

    PlanningPose target{};
    target.x = app.pose.x;
    target.y = app.pose.y;
    target.z = app.pose.z;
    target.qw = app.pose.qw;
    target.qx = app.pose.qx;
    target.qy = app.pose.qy;
    target.qz = app.pose.qz;

    PlanningJoints out{};
    int ret = PlanningInverse(kin.get(), &target, nullptr, nullptr, &out);
    if (ret != kKinematicsOk) {
        std::cerr << "[debug_ik] kin_inverse failed: " << ret << std::endl;
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    const int arm_joint_count = out.count;
    std::cout << "[debug_ik] IK success, arm joints(rad): [";
    for (int i = 0; i < arm_joint_count; ++i) {
        if (i) std::cout << ", ";
        std::cout << out.q[i];
    }
    std::cout << "]" << std::endl;

    return 0;
}
