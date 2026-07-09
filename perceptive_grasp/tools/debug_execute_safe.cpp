/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file debug_execute_safe.cpp
    * @brief 安全执行调试工具：只执行 observe -> pre_grasp
    */

#include <cmath>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include "grasp_executor.h"

extern "C" {
#include "kinematics_interface.h"
}

namespace fs = std::filesystem;
using perceptive_grasp::ExecutorConfig;
using perceptive_grasp::GraspExecutor;
using perceptive_grasp::GraspResult;
using perceptive_grasp::JointConstraint;
using perceptive_grasp::Pose3D;

static constexpr const char* kDefaultConfigPath = "../config/grasp_pipeline.yaml";

// 顶抓基础四元数: 绕Y轴180° => 末端Z朝下 (与 grasp_planner.cpp 一致)
static constexpr float kTopDownQw = 0.0f;
static constexpr float kTopDownQx = 0.0f;
static constexpr float kTopDownQy = 1.0f;
static constexpr float kTopDownQz = 0.0f;

/**
    * @brief 计算顶抓+偏航的四元数
    *
    * 组合: q_total = q_top_down * q_yaw
    * q_top_down = (0, 0, 1, 0)  -- 绕Y轴180°, 末端Z朝下
    * q_yaw = (cos(yaw/2), 0, 0, sin(yaw/2))  -- 绕局部Z轴(即world -Z)旋转
    *
    * 用于根据物体旋转角调整夹爪朝向。
    */
static void ComputeTopDownQuat(float yaw_deg, float& qw, float& qx, float& qy, float& qz) {
    float yaw_rad = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    float half = yaw_rad * 0.5f;
    // q_yaw in local frame
    float yw = std::cos(half);
    float yz = std::sin(half);

    // Hamilton product: q_top_down * q_yaw
    // q_top_down = (w=0, x=0, y=1, z=0)
    // q_yaw      = (w=yw, x=0, y=0, z=yz)
    // result.w = 0*yw - 0*0 - 1*0 - 0*yz = 0*yw - 0*yz = -0*yz  => 0*yw + 0 + 0 - 0 = -1*0 - 0*yz
    // Let's just do full Hamilton:
    // (a1,b1,c1,d1)*(a2,b2,c2,d2):
    //   w = a1*a2 - b1*b2 - c1*c2 - d1*d2
    //   x = a1*b2 + b1*a2 + c1*d2 - d1*c2
    //   y = a1*c2 - b1*d2 + c1*a2 + d1*b2
    //   z = a1*d2 + b1*c2 - c1*b2 + d1*a2
    float a1 = kTopDownQw, b1 = kTopDownQx, c1 = kTopDownQy, d1 = kTopDownQz;
    float a2 = yw, b2 = 0.0f, c2 = 0.0f, d2 = yz;

    qw = a1*a2 - b1*b2 - c1*c2 - d1*d2;
    qx = a1*b2 + b1*a2 + c1*d2 - d1*c2;
    qy = a1*c2 - b1*d2 + c1*a2 + d1*b2;
    qz = a1*d2 + b1*c2 - c1*b2 + d1*a2;
}

struct AppConfig {
    std::string config_path;
    Pose3D pre_grasp_pose{};
    bool has_pose = false;
    float yaw_deg = 0.0f;   // 夹爪绕竖直轴旋转角度 (度)
    // 关节覆盖: 直接指定第4关节(wrist pitch)和第5关节(wrist yaw)
    float wrist_pitch = -1.0f;  // <0 表示不覆盖, 朝下范围 [1.102, 1.667]
    float wrist_yaw = -999.0f;  // -999 表示不覆盖
    bool has_joints = false;    // --joints 模式: 直接指定5个关节角
    std::vector<float> target_joints;
    bool auto_yes = false;
    int dwell_ms = 2000;
    int wait_done_ms = 15000;
    float safe_move_speed = 0.2f;
};

static void PrintUsage(const char* prog) {
    std::cout << "Usage:\n"
                << "  " << prog << " --pose x y z [--yaw DEG] [options]\n"
                << "  " << prog << " --joints j1 j2 j3 j4 j5 [options]\n"
                << "\n"
                << "Safe sequence:\n"
                << "  1. Init manipulator + gripper\n"
                << "  2. Move to observe_joints\n"
                << "  3. Move to given pre_grasp pose / joints\n"
                << "  4. Stop\n"
                << "\n"
                << "Modes:\n"
                << "  --pose x y z             Target position (auto top-down orientation)\n"
                << "  --joints j1 j2 j3 j4 j5 Direct joint angles (rad), skip IK\n"
                << "\n"
                << "Orientation control:\n"
                << "  --yaw DEG              Gripper yaw around vertical axis (degrees)\n"
                << "  --wrist-pitch RAD      Override joint 4 after IK (down range: 1.102~1.667)\n"
                << "  --wrist-yaw RAD        Override joint 5 after IK (gripper rotation)\n"
                << "\n"
                << "Options:\n"
                << "  --config <yaml>       Pipeline config (default: ../config/grasp_pipeline.yaml)\n"
                << "  --dwell-ms N          Hold after each pose in milliseconds (default: 2000)\n"
                << "  --wait-done-ms N      Max wait for motion completion in ms (default: 15000)\n"
                << "  --safe-move-speed S   Joint move speed for step 3 (default: 0.2)\n"
                << "  --yes                 Skip confirmation prompts\n"
                << "\n"
                << "This tool will NOT descend and will NOT close the gripper.\n";
}

static void DwellAfterPose(const char* stage_name, int dwell_ms) {
    if (dwell_ms <= 0) return;
    std::cout << "[debug_execute_safe] hold at " << stage_name
                << " for " << dwell_ms << " ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));
}

static bool ConfirmStep(const std::string& msg, bool auto_yes) {
    if (auto_yes) {
        std::cout << msg << " [auto-yes]" << std::endl;
        return true;
    }
    std::cout << msg << " [y/N]: ";
    std::string line;
    std::getline(std::cin, line);
    return line == "y" || line == "Y" || line == "yes" || line == "YES";
}

static AppConfig ParseArgs(int argc, char* argv[]) {
    AppConfig cfg;
    cfg.config_path = kDefaultConfigPath;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--config" && i + 1 < argc) {
            cfg.config_path = argv[++i];
        } else if (arg == "--pose" && i + 3 < argc) {
            cfg.pre_grasp_pose.x = std::stof(argv[++i]);
            cfg.pre_grasp_pose.y = std::stof(argv[++i]);
            cfg.pre_grasp_pose.z = std::stof(argv[++i]);
            cfg.has_pose = true;
        } else if (arg == "--joints" && i + 5 < argc) {
            cfg.has_joints = true;
            cfg.target_joints.clear();
            for (int j = 0; j < 5; ++j) {
                cfg.target_joints.push_back(std::stof(argv[++i]));
            }
        } else if (arg == "--yaw" && i + 1 < argc) {
            cfg.yaw_deg = std::stof(argv[++i]);
        } else if (arg == "--wrist-pitch" && i + 1 < argc) {
            cfg.wrist_pitch = std::stof(argv[++i]);
        } else if (arg == "--wrist-yaw" && i + 1 < argc) {
            cfg.wrist_yaw = std::stof(argv[++i]);
        } else if (arg == "--yes") {
            cfg.auto_yes = true;
        } else if (arg == "--dwell-ms" && i + 1 < argc) {
            cfg.dwell_ms = std::max(0, std::stoi(argv[++i]));
        } else if (arg == "--wait-done-ms" && i + 1 < argc) {
            cfg.wait_done_ms = std::max(1000, std::stoi(argv[++i]));
        } else if (arg == "--safe-move-speed" && i + 1 < argc) {
            cfg.safe_move_speed = std::stof(argv[++i]);
        }
    }

    // --pose 模式: 自动计算顶抓四元数
    if (cfg.has_pose) {
        ComputeTopDownQuat(cfg.yaw_deg,
                                cfg.pre_grasp_pose.qw,
                                cfg.pre_grasp_pose.qx,
                                cfg.pre_grasp_pose.qy,
                                cfg.pre_grasp_pose.qz);
    }

    return cfg;
}

static ExecutorConfig LoadExecutorConfig(const std::string& config_path) {
    YAML::Node root = YAML::LoadFile(config_path);
    ExecutorConfig cfg;

    if (auto g = root["grasp"]) {
        cfg.gripper_open = g["gripper_open"].as<float>(cfg.gripper_open);
        cfg.gripper_effort = g["gripper_effort"].as<float>(cfg.gripper_effort);
        cfg.gripper_hold_load_threshold =
            g["gripper_hold_load_threshold"].as<float>(
                cfg.gripper_hold_load_threshold);
        cfg.gripper_timeout_ms =
            g["gripper_timeout_ms"].as<int>(cfg.gripper_timeout_ms);
    }

    if (auto m = root["manipulator"]) {
        cfg.manip_driver = m["driver"].as<std::string>(cfg.manip_driver);
        cfg.uart_device = m["uart_device"].as<std::string>(cfg.uart_device);
        cfg.baudrate = m["baudrate"].as<int>(cfg.baudrate);
        cfg.urdf_path = m["urdf_path"].as<std::string>(cfg.urdf_path);
        cfg.base_link = m["base_link"].as<std::string>(cfg.base_link);
        cfg.tip_link = m["tip_link"].as<std::string>(cfg.tip_link);
        cfg.move_speed = m["move_speed"].as<float>(cfg.move_speed);
        cfg.line_speed = m["line_speed"].as<float>(cfg.line_speed);

        if (auto hj = m["home_joints"]) {
            cfg.home_joints.clear();
            for (size_t i = 0; i < hj.size(); ++i) cfg.home_joints.push_back(hj[i].as<float>());
        }
        if (auto oj = m["observe_joints"]) {
            cfg.observe_joints.clear();
            for (size_t i = 0; i < oj.size(); ++i) cfg.observe_joints.push_back(oj[i].as<float>());
        }

        cfg.ik_max_trials = m["ik_max_trials"].as<int>(cfg.ik_max_trials);
        cfg.wrist_yaw_scale = m["wrist_yaw_scale"].as<float>(cfg.wrist_yaw_scale);

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
        fs::path config_dir = fs::path(config_path).parent_path();
        fs::path resolved = config_dir / cfg.urdf_path;
        if (fs::exists(resolved)) cfg.urdf_path = fs::canonical(resolved).string();
    }

    return cfg;
}

int main(int argc, char* argv[]) {
    try {
        AppConfig app = ParseArgs(argc, argv);
        if (app.config_path.empty() || (!app.has_pose && !app.has_joints)) {
            PrintUsage(argv[0]);
            return 1;
        }

        ExecutorConfig exec_cfg = LoadExecutorConfig(app.config_path);

        std::cout << "[debug_execute_safe] config: " << app.config_path << std::endl;
        std::cout << "[debug_execute_safe] uart:   " << exec_cfg.uart_device << std::endl;
        std::cout << "[debug_execute_safe] urdf:   " << exec_cfg.urdf_path << std::endl;
        if (app.has_joints) {
            std::cout << "[debug_execute_safe] mode: direct joints [";
            for (size_t i = 0; i < app.target_joints.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << app.target_joints[i];
            }
            std::cout << "]" << std::endl;
        } else {
            std::cout << "[debug_execute_safe] mode: top-down (yaw=" << app.yaw_deg << " deg)" << std::endl;
            std::cout << "[debug_execute_safe] pre_grasp pose: ["
                        << app.pre_grasp_pose.x << ", "
                        << app.pre_grasp_pose.y << ", "
                        << app.pre_grasp_pose.z << "] quat=["
                        << app.pre_grasp_pose.qw << ", "
                        << app.pre_grasp_pose.qx << ", "
                        << app.pre_grasp_pose.qy << ", "
                        << app.pre_grasp_pose.qz << "]" << std::endl;
            if (app.wrist_pitch >= 0.0f) {
                std::cout << "[debug_execute_safe] wrist-pitch override: " << app.wrist_pitch << " rad" << std::endl;
            }
            if (app.wrist_yaw > -900.0f) {
                std::cout << "[debug_execute_safe] wrist-yaw override: " << app.wrist_yaw << " rad" << std::endl;
            }
        }
        std::cout << "[debug_execute_safe] dwell:  " << app.dwell_ms << " ms" << std::endl;
        std::cout << "[debug_execute_safe] wait done timeout: " << app.wait_done_ms << " ms" << std::endl;
        std::cout << "[debug_execute_safe] safe move speed: " << app.safe_move_speed << std::endl;

        if (!ConfirmStep("[step 1/3] Confirm hardware area is clear and power is on.", app.auto_yes)) {
            std::cout << "[debug_execute_safe] aborted." << std::endl;
            return 2;
        }

        GraspExecutor executor(exec_cfg);
        executor.SetWaitMotionTimeoutMs(app.wait_done_ms);
        if (!executor.Init()) {
            std::cerr << "[debug_execute_safe] executor init failed" << std::endl;
            return 1;
        }

        if (!ConfirmStep("[step 2/3] Move arm to observe pose?", app.auto_yes)) {
            std::cout << "[debug_execute_safe] aborted before observe move." << std::endl;
            return 2;
        }
        auto ret = executor.MoveToObserve();
        if (ret != GraspResult::SUCCESS) {
            std::cerr << "[debug_execute_safe] MoveToObserve failed" << std::endl;
            return 1;
        }
        DwellAfterPose("observe pose", app.dwell_ms);

        if (!ConfirmStep("[step 3/3] Move arm to pre_grasp pose only?", app.auto_yes)) {
            std::cout << "[debug_execute_safe] stopped at observe pose." << std::endl;
            return 0;
        }

        if (app.has_joints) {
            // 直接关节模式: 跳过 IK
            ret = executor.MoveToJointsSafe(app.target_joints, app.safe_move_speed);
        } else if (app.wrist_pitch < 0.0f && app.wrist_yaw <= -900.0f) {
            std::cout << "[debug_execute_safe] using executor pose path"
                        << " with FK arrival verification" << std::endl;
            ret = executor.MoveToPreGraspSafe(
                app.pre_grasp_pose, app.safe_move_speed);
        } else {
            // IK 模式: 多种子采样，筛选满足关节约束的解
            const auto& constraints = exec_cfg.joint_constraints;
            const int max_trials = exec_cfg.ik_max_trials;

            // 打印约束信息
            if (!constraints.empty()) {
                std::cout << "[debug_execute_safe] joint constraints:" << std::endl;
                for (const auto& c : constraints) {
                    std::cout << "  joint" << (c.joint_index + 1)
                                << " (index " << c.joint_index << "): ["
                                << c.min_rad << ", " << c.max_rad << "]" << std::endl;
                }
            } else {
                std::cout << "[debug_execute_safe] no joint constraints configured" << std::endl;
            }

            // 构造 IK 目标
            kin_pose_t ik_target;
            ik_target.x  = app.pre_grasp_pose.x;
            ik_target.y  = app.pre_grasp_pose.y;
            ik_target.z  = app.pre_grasp_pose.z;
            ik_target.qw = app.pre_grasp_pose.qw;
            ik_target.qx = app.pre_grasp_pose.qx;
            ik_target.qy = app.pre_grasp_pose.qy;
            ik_target.qz = app.pre_grasp_pose.qz;

            kin_ik_params_t ik_params = {};
            ik_params.epsilon = 1e-3;
            ik_params.position_weight = 1.0;
            ik_params.timeout_s = 0.1;

            // 获取关节数和限位
            int n_joints = kin_get_num_joints(executor.GetKinSolver());
            std::vector<double> lower(n_joints), upper(n_joints);
            kin_get_joint_limits(executor.GetKinSolver(), lower.data(), upper.data());

            std::mt19937 rng(42);
            std::vector<float> best_joints;
            bool found_valid = false;

            // 检查一组关节角是否满足所有约束
            auto check_constraints = [&](const kin_joints_t& q) -> bool {
                for (const auto& c : constraints) {
                    if (c.joint_index >= 0 && c.joint_index < q.count) {
                        float val = static_cast<float>(q.q[c.joint_index]);
                        if (val < c.min_rad || val > c.max_rad) return false;
                    }
                }
                return true;
            };

            for (int trial = 0; trial < max_trials; ++trial) {
                kin_joints_t q_seed;
                q_seed.count = static_cast<uint8_t>(n_joints);

                if (trial == 0) {
                    // 第一次用 observe_joints 作为种子
                    auto& obs = exec_cfg.observe_joints;
                    for (int j = 0; j < n_joints; ++j) {
                        q_seed.q[j] = (j < static_cast<int>(obs.size())) ? obs[j] : 0.0;
                    }
                } else {
                    // 随机种子
                    for (int j = 0; j < n_joints; ++j) {
                        std::uniform_real_distribution<double> dist(lower[j], upper[j]);
                        q_seed.q[j] = dist(rng);
                    }
                    // 对有约束的关节，强制种子在约束范围内
                    for (const auto& c : constraints) {
                        if (c.joint_index >= 0 && c.joint_index < n_joints) {
                            std::uniform_real_distribution<double> cdist(c.min_rad, c.max_rad);
                            q_seed.q[c.joint_index] = cdist(rng);
                        }
                    }
                }

                kin_joints_t q_result;
                int ik_ret = kin_inverse(executor.GetKinSolver(), &ik_target,
                                        &q_seed, &ik_params, &q_result);
                if (ik_ret != KIN_OK) continue;

                // 检查所有关节约束
                if (check_constraints(q_result)) {
                    best_joints.clear();
                    for (int j = 0; j < q_result.count; ++j) {
                        best_joints.push_back(static_cast<float>(q_result.q[j]));
                    }
                    found_valid = true;
                    std::cout << "[debug_execute_safe] IK found valid solution at trial "
                                << trial;
                    for (const auto& c : constraints) {
                        if (c.joint_index >= 0 && c.joint_index < q_result.count) {
                            std::cout << ", joint" << (c.joint_index + 1) << "="
                                        << q_result.q[c.joint_index];
                        }
                    }
                    std::cout << std::endl;
                    break;
                }

                // 记录第一个解作为 fallback
                if (trial == 0 && best_joints.empty()) {
                    for (int j = 0; j < q_result.count; ++j) {
                        best_joints.push_back(static_cast<float>(q_result.q[j]));
                    }
                }
            }

            if (best_joints.empty()) {
                std::cerr << "[debug_execute_safe] IK solve failed after "
                            << max_trials << " trials" << std::endl;
                return 1;
            }

            if (!found_valid) {
                std::cerr << "[debug_execute_safe] WARNING: no solution satisfying all constraints"
                            << ", using fallback" << std::endl;
                for (const auto& c : constraints) {
                    if (c.joint_index >= 0 && c.joint_index < static_cast<int>(best_joints.size())) {
                        std::cerr << "  joint" << (c.joint_index + 1) << "="
                                    << best_joints[c.joint_index]
                                    << " (required: [" << c.min_rad << ", " << c.max_rad << "])"
                                    << std::endl;
                    }
                }
            }

            std::cout << "[debug_execute_safe] IK joints(rad): [";
            for (size_t i = 0; i < best_joints.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << best_joints[i];
            }
            std::cout << "]" << std::endl;

            // 手动覆盖 wrist pitch (joint 4, index 3)
            if (app.wrist_pitch >= 0.0f && best_joints.size() >= 4) {
                std::cout << "[debug_execute_safe] override joint4 (wrist-pitch): "
                            << best_joints[3] << " -> " << app.wrist_pitch << std::endl;
                best_joints[3] = app.wrist_pitch;
            }

            // 手动覆盖 wrist yaw (joint 5, index 4)
            if (app.wrist_yaw > -900.0f && best_joints.size() >= 5) {
                std::cout << "[debug_execute_safe] override joint5 (wrist-yaw): "
                            << best_joints[4] << " -> " << app.wrist_yaw << std::endl;
                best_joints[4] = app.wrist_yaw;
            }

            std::cout << "[debug_execute_safe] final joints(rad): [";
            for (size_t i = 0; i < best_joints.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << best_joints[i];
            }
            std::cout << "]" << std::endl;

            // 只取前5个关节执行
            std::vector<float> arm_joints(best_joints.begin(),
                best_joints.begin() + std::min<size_t>(best_joints.size(), 5));
            ret = executor.MoveToJointsSafe(arm_joints, app.safe_move_speed);
        }

        if (ret == GraspResult::SUCCESS) {
            DwellAfterPose("pre_grasp pose", app.dwell_ms);
            std::cout << "[debug_execute_safe] reached pre_grasp pose safely."
                        << " No descend and no gripper action were executed." << std::endl;
            return 0;
        }

        std::cerr << "[debug_execute_safe] pre_grasp move failed, result=" << static_cast<int>(ret) << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "[debug_execute_safe] Exception: " << e.what() << std::endl;
        return 1;
    }
}
