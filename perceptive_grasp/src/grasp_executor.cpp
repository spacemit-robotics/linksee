/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_executor.cpp
    * @brief 抓取执行模块实现 - 机械臂 + 夹爪协调控制
    */

#include "grasp_executor.h"

extern "C" {
#include "grasp.h"
#include "kinematics_interface.h"
#include "manipulator.h"
#include "so101_utils.h"
#include "so101_gripper.h"
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <thread>

namespace perceptive_grasp {

namespace {

const char* GraspStateName(grasp_state_t state) {
    switch (state) {
        case GRASP_STATE_IDLE: return "IDLE";
        case GRASP_STATE_MOVING: return "MOVING";
        case GRASP_STATE_HOLDING: return "HOLDING";
        case GRASP_STATE_EMPTY: return "EMPTY";
        case GRASP_STATE_ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

}  // namespace

void GraspExecutor::RecordResult(GraspResult result, const std::string& action,
                                const std::string& detail) {
    diagnostics_.last_result = result;
    diagnostics_.last_action = action;
    diagnostics_.last_detail = detail;
}

GraspExecutor::~GraspExecutor() {
    if (gripper_) {
        grasp_stop(gripper_);
        grasp_free(gripper_);
    }
    if (arm_) {
        manip_free(arm_);
        // kin_ 由 manip_free 自动释放 (所有权已转移)
    }
}

bool GraspExecutor::Init() {
    // 构造 SO101 机械臂配置
    struct so101_config arm_cfg = {};
    arm_cfg.uart_path = config_.uart_device.c_str();
    arm_cfg.baud = static_cast<uint32_t>(config_.baudrate);
    arm_cfg.ids[0] = 1;
    arm_cfg.ids[1] = 2;
    arm_cfg.ids[2] = 3;
    arm_cfg.ids[3] = 4;
    arm_cfg.ids[4] = 5;
    arm_cfg.urdf_path = config_.urdf_path.c_str();
    arm_cfg.kin_solver_name = nullptr;  // 使用默认 pinocchio

    arm_ = manip_alloc(config_.manip_driver.c_str(), &arm_cfg);
    if (!arm_) {
        std::cerr << "[GraspExecutor] Failed to create manipulator" << std::endl;
        return false;
    }

    // 创建运动学求解器
    kin_ = kin_create(nullptr,  // 使用默认求解器 (tracik/pinocchio)
                        config_.urdf_path.c_str(), config_.base_link.c_str(),
                        config_.tip_link.c_str());
    if (!kin_) {
        std::cerr << "[GraspExecutor] Failed to create kinematics solver"
                    << std::endl;
        return false;
    }

    // 绑定运动学到机械臂 (所有权转移)
    int ret = manip_set_kinematics(arm_, kin_);
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] Failed to bind kinematics: " << ret
                    << std::endl;
        kin_destroy(kin_);
        kin_ = nullptr;
        return false;
    }

    // 构造 SO101 夹爪配置 (与机械臂共用同一串口)
    struct so101_gripper_config grip_cfg = {};
    grip_cfg.uart_path = config_.uart_device.c_str();
    grip_cfg.baud = static_cast<uint32_t>(config_.baudrate);
    grip_cfg.id = 6;  // SO101 夹爪默认 ID=6
    grip_cfg.grasp_cfg.max_effort = config_.gripper_effort;
    grip_cfg.grasp_cfg.hold_threshold = config_.gripper_hold_load_threshold;
    grip_cfg.grasp_cfg.timeout_ms =
        static_cast<uint32_t>(config_.gripper_timeout_ms);

    gripper_ = grasp_alloc("so101_gripper", &grip_cfg);
    if (!gripper_) {
        std::cerr << "[GraspExecutor] Failed to create gripper" << std::endl;
        return false;
    }

    std::cout << "[GraspExecutor] Initialized: arm=" << config_.manip_driver
                << ", urdf=" << config_.urdf_path << std::endl;
    return true;
}

GraspResult GraspExecutor::MoveToObserve() {
    // 先闭合夹爪并等待完成
    grasp_set_position(gripper_, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.observe_gripper_close_wait_ms));

    // 夹爪闭合后再移动到观察位 (碰撞安全)
    GraspResult result = MoveToJointsCollisionSafe(config_.observe_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_observe", "move_joints failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_observe",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_observe");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToHome() {
    GraspResult result = MoveToJointsCollisionSafe(config_.home_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_home", "move_joints failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_home", "motion timeout");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_home");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                            float grasp_yaw_rad) {
    bool has_yaw = !std::isnan(grasp_yaw_rad);
    if (has_yaw) {
        std::cout << "[GraspExecutor] Approach with yaw override: "
                    << grasp_yaw_rad << " rad ("
                    << grasp_yaw_rad * 180.0f / M_PI << "°)" << std::endl;
    }

    GraspResult result = has_yaw
        ? MoveToPoseWithYaw(pre_grasp_pose, config_.move_speed, grasp_yaw_rad)
        : MoveToPoseConstrained(pre_grasp_pose, config_.move_speed);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_pre_grasp",
                    result == GraspResult::IK_FAILED ? "ik failed"
                                                        : "move command failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_pre_grasp",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_pre_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::OpenGripperForGrasp() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.pre_grasp_settle_ms));
    grasp_set_position(gripper_, config_.gripper_open);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.gripper_open_wait_ms));
    RecordResult(GraspResult::SUCCESS, "open_gripper_for_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToGrasp(const Pose3D& grasp_pose,
                                        float grasp_yaw_rad) {
    bool has_yaw = !std::isnan(grasp_yaw_rad);
    GraspResult result = has_yaw
        ? MoveToPoseWithYaw(grasp_pose, config_.line_speed, grasp_yaw_rad)
        : MoveToPoseConstrained(grasp_pose, config_.line_speed);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_grasp",
                    result == GraspResult::IK_FAILED ? "ik failed"
                                                        : "move command failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_grasp",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CloseGripperAndCheck() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.grasp_settle_ms));
    grasp_execute(gripper_, GRASP_CMD_GRAB, config_.gripper_effort);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.gripper_close_wait_ms));

    grasp_state_t state = GRASP_STATE_ERROR;
    int holding_count = 0;
    int load_holding_count = 0;
    float position = NAN;
    float load = NAN;

    for (int i = 0; i < config_.timing.grasp_check_count; i++) {
        grasp_tick(gripper_, 0.05f);
        state = grasp_get_state(gripper_);
        if (state == GRASP_STATE_HOLDING) {
            holding_count++;
        }

        float cur_position = NAN;
        float cur_load = NAN;
        if (grasp_get_feedback(gripper_, &cur_position, &cur_load) == GRASP_OK) {
            position = cur_position;
            load = cur_load;
            if (cur_position > 0.05f &&
                cur_load >= config_.gripper_hold_load_threshold) {
                load_holding_count++;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(
            config_.timing.grasp_check_interval_ms));
    }

    const int required_holding = std::max(1, config_.timing.grasp_check_count / 2);
    diagnostics_.gripper_check.state = GraspStateName(state);
    diagnostics_.gripper_check.holding_count = holding_count;
    diagnostics_.gripper_check.load_holding_count = load_holding_count;
    diagnostics_.gripper_check.check_count = config_.timing.grasp_check_count;
    diagnostics_.gripper_check.load_threshold =
        config_.gripper_hold_load_threshold;
    diagnostics_.gripper_check.position = position;
    diagnostics_.gripper_check.load = load;

    std::cout << "[GraspExecutor] grasp check: state="
                << GraspStateName(state)
                << ", holding=" << holding_count << "/"
                << config_.timing.grasp_check_count
                << ", load_holding=" << load_holding_count << "/"
                << config_.timing.grasp_check_count
                << ", load_threshold="
                << config_.gripper_hold_load_threshold
                << ", position=" << position
                << ", load=" << load << std::endl;

    if (state == GRASP_STATE_HOLDING || holding_count >= required_holding) {
        RecordResult(GraspResult::SUCCESS, "close_gripper_and_check",
                    "holding state reached");
        return GraspResult::SUCCESS;
    }
    if (state == GRASP_STATE_EMPTY ||
        state == GRASP_STATE_IDLE ||
        (!std::isnan(position) && position <= 0.05f)) {
        std::cout << "[GraspExecutor] Grasp empty - nothing grabbed" << std::endl;
        RecordResult(GraspResult::EMPTY, "close_gripper_and_check",
                    "gripper closed without object");
        return GraspResult::EMPTY;
    }
    if (load_holding_count >= required_holding) {
        std::cout << "[GraspExecutor] Grasp inferred from sustained load"
                    << std::endl;
        RecordResult(GraspResult::SUCCESS, "close_gripper_and_check",
                    "sustained load above threshold");
        return GraspResult::SUCCESS;
    }
    if (state == GRASP_STATE_MOVING) {
        std::cerr << "[GraspExecutor] Gripper still moving after close check"
                    << std::endl;
        RecordResult(GraspResult::TIMEOUT, "close_gripper_and_check",
                    "gripper still moving after close check");
        return GraspResult::TIMEOUT;
    }
    std::cerr << "[GraspExecutor] Gripper error during close check" << std::endl;
    RecordResult(GraspResult::MOVE_FAILED, "close_gripper_and_check",
                "gripper error during close check");
    return GraspResult::MOVE_FAILED;
}

GraspResult GraspExecutor::LiftFromGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad) {
    bool has_yaw = !std::isnan(grasp_yaw_rad);
    GraspResult result = has_yaw
        ? MoveToPoseWithYaw(pre_grasp_pose, config_.line_speed, grasp_yaw_rad)
        : MoveToPoseConstrained(pre_grasp_pose, config_.line_speed);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "lift_from_grasp",
                    result == GraspResult::IK_FAILED ? "ik failed"
                                                        : "move command failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "lift_from_grasp",
                    "motion timeout");
        return GraspResult::TIMEOUT;
    }
    RecordResult(GraspResult::SUCCESS, "lift_from_grasp");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPlace() {
    GraspResult result = MoveToJointsCollisionSafe(config_.place_joints);
    if (result != GraspResult::SUCCESS) {
        RecordResult(result, "move_to_place", "move_joints failed");
        return result;
    }
    if (!WaitMotionDone()) {
        RecordResult(GraspResult::TIMEOUT, "move_to_place", "motion timeout");
        return GraspResult::TIMEOUT;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.place_settle_ms));
    RecordResult(GraspResult::SUCCESS, "move_to_place");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::ReleaseObject() {
    grasp_execute(gripper_, GRASP_CMD_RELEASE, config_.place_release_open);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.timing.release_wait_ms));
    RecordResult(GraspResult::SUCCESS, "release_object");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::CloseGripper() {
    grasp_set_position(gripper_, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config_.timing.home_gripper_close_wait_ms));
    RecordResult(GraspResult::SUCCESS, "close_gripper");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToJointsSafe(const std::vector<float>& joints,
                                            float speed_scale) {
    float old_speed = config_.move_speed;
    if (speed_scale > 0.0f) {
        config_.move_speed = speed_scale;
    }

    GraspResult result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed;
    if (result != GraspResult::SUCCESS) return result;
    if (!WaitMotionDone()) return GraspResult::TIMEOUT;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPreGraspSafe(const Pose3D& pre_grasp_pose,
                                                float speed_scale) {
    float speed = speed_scale > 0.0f ? speed_scale : config_.move_speed;
    GraspResult result = MoveToPoseWithIKJoints(pre_grasp_pose, speed);
    if (result != GraspResult::SUCCESS) return result;
    if (!WaitMotionDone()) return GraspResult::TIMEOUT;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::ExecuteGrasp(const Pose3D& grasp_pose,
                                        const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad) {
    GraspResult result = MoveToPreGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;

    result = OpenGripperForGrasp();
    if (result != GraspResult::SUCCESS) return result;

    result = MoveToGrasp(grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;

    result = CloseGripperAndCheck();
    if (result != GraspResult::SUCCESS) return result;

    result = LiftFromGrasp(pre_grasp_pose, grasp_yaw_rad);
    if (result != GraspResult::SUCCESS) return result;

    std::cout << "[GraspExecutor] Grasp successful!" << std::endl;
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::ExecutePlace() {
    GraspResult result = MoveToPlace();
    if (result != GraspResult::SUCCESS) return result;
    result = ReleaseObject();
    if (result != GraspResult::SUCCESS) return result;
    result = CloseGripper();
    if (result != GraspResult::SUCCESS) return result;
    return MoveToObserve();
}

void GraspExecutor::EmergencyStop() {
    if (arm_) manip_stop(arm_);
    if (gripper_) grasp_stop(gripper_);
    RecordResult(GraspResult::MOVE_FAILED, "emergency_stop",
                "emergency stop requested");
}

bool GraspExecutor::GetCurrentPose(Pose3D& pose) {
    if (!arm_) return false;

    manip_pose_t mp;
    int ret = manip_get_state(arm_, nullptr, &mp);
    if (ret != MANIP_OK) return false;

    pose.x = mp.x;
    pose.y = mp.y;
    pose.z = mp.z;
    pose.qw = mp.qw;
    pose.qx = mp.qx;
    pose.qy = mp.qy;
    pose.qz = mp.qz;
    return true;
}

void GraspExecutor::Tick(float dt_s) {
    if (arm_) manip_tick(arm_, dt_s);
    if (gripper_) grasp_tick(gripper_, dt_s);
}

// --- Private ---

GraspResult GraspExecutor::MoveToJoints(const std::vector<float>& joints) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    manip_joint_t target;
    target.count = static_cast<uint8_t>(joints.size());
    for (size_t i = 0; i < joints.size() && i < MANIP_MAX_JOINTS; i++) {
        target.joints[i] = joints[i];
    }

    int ret = manip_move_joints(arm_, &target, config_.move_speed);
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] move_joints failed: " << ret << std::endl;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

bool GraspExecutor::GetCurrentJoints(std::vector<float>& joints) {
    if (!arm_) return false;

    manip_joint_t cur;
    std::memset(&cur, 0, sizeof(cur));
    int ret = manip_get_state(arm_, &cur, nullptr);
    if (ret != MANIP_OK) return false;

    joints.clear();
    joints.reserve(cur.count);
    for (uint8_t i = 0; i < cur.count; ++i) {
        joints.push_back(cur.joints[i]);
    }
    return true;
}

bool GraspExecutor::NeedsCollisionAvoidance(
    const std::vector<float>& current_joints,
    const std::vector<float>& target_joints) {
    const auto& ca = config_.collision_avoidance;
    if (!ca.enabled) return false;
    if (target_joints.size() < 2 || current_joints.size() < 2) return false;

    float target_j0 = target_joints[0];
    float target_j1 = target_joints[1];
    float current_j0 = current_joints[0];
    float current_j1 = current_joints[1];

    bool target_shoulder_danger = (target_j1 < ca.shoulder_threshold);
    bool current_shoulder_danger = (current_j1 < ca.shoulder_threshold);

    // joint0 是否在危险区内
    auto in_danger_zone = [&](float j0) -> bool {
        return j0 > ca.base_danger_min && j0 < ca.base_danger_max;
    };

    // joint0 运动路径是否穿越危险区
    // (从 a 到 b 的过程中是否经过 [danger_min, danger_max])
    auto path_crosses_danger = [&](float a, float b) -> bool {
        if (in_danger_zone(a) || in_danger_zone(b)) return true;
        // a 和 b 都在安全区，看是否在危险区两侧
        bool a_below = (a <= ca.base_danger_min);
        bool b_below = (b <= ca.base_danger_min);
        // 如果一个在左侧一个在右侧，路径必穿越危险区
        return (a_below != b_below);
    };

    // 情况A: 目标 joint1 在危险区，且 joint0 运动路径穿越或在危险区内
    if (target_shoulder_danger && path_crosses_danger(current_j0, target_j0)) {
        return true;
    }

    // 情况B: 当前 joint1 在危险区，且 joint0 运动路径穿越或在危险区内
    // (从安全区外到安全区外，但路径穿越；或当前/目标在危险区内)
    if (current_shoulder_danger && path_crosses_danger(current_j0, target_j0)) {
        return true;
    }

    return false;
}

GraspResult GraspExecutor::MoveToJointsCollisionSafe(
    const std::vector<float>& target_joints) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    const auto& ca = config_.collision_avoidance;
    if (!ca.enabled || target_joints.size() < 2) {
        // 碰撞避免未启用，直接运动
        GraspResult result = MoveToJoints(target_joints);
        return result;
    }

    // 读取当前关节角
    std::vector<float> current_joints;
    if (!GetCurrentJoints(current_joints) || current_joints.size() < 2) {
        std::cerr << "[GraspExecutor] Cannot read current joints for collision check"
                    << std::endl;
        return MoveToJoints(target_joints);
    }

    if (!NeedsCollisionAvoidance(current_joints, target_joints)) {
        // 无碰撞风险，直接运动
        return MoveToJoints(target_joints);
    }

    // === 需要碰撞避免: 分步运动 ===
    std::cout << "[GraspExecutor] COLLISION AVOIDANCE: splitting motion"
                << std::endl;
    std::cout << "  current j0=" << current_joints[0]
                << " j1=" << current_joints[1] << std::endl;
    std::cout << "  target  j0=" << target_joints[0]
                << " j1=" << target_joints[1] << std::endl;

    float current_j0 = current_joints[0];
    float current_j1 = current_joints[1];
    float target_j0 = target_joints[0];
    float target_j1 = target_joints[1];

    bool current_shoulder_danger = (current_j1 < ca.shoulder_threshold);
    bool target_shoulder_danger = (target_j1 < ca.shoulder_threshold);

    auto in_danger_zone = [&](float j0) -> bool {
        return j0 > ca.base_danger_min && j0 < ca.base_danger_max;
    };

    GraspResult result;

    if (current_shoulder_danger && !target_shoulder_danger) {
        // 情况: 当前 j1 在危险区，目标 j1 不在危险区
        // 策略: 先把 j1 抬到安全值 (保持 j0 不动)，再执行完整运动
        std::cout << "  Strategy: lift shoulder first (j1 -> "
                    << ca.shoulder_threshold << ")" << std::endl;

        std::vector<float> step1_joints = current_joints;
        step1_joints[1] = ca.shoulder_threshold;  // 抬到阈值

        std::cout << "  Step 1: lift j1 to safe threshold" << std::endl;
        result = MoveToJoints(step1_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        std::cout << "  Step 2: move all joints to target" << std::endl;
        result = MoveToJoints(target_joints);
        return result;

    } else if (!current_shoulder_danger && target_shoulder_danger) {
        // 情况: 当前 j1 安全，目标 j1 在危险区
        // 策略: 先把 j0 转到安全位置，再执行完整运动
        float safe_j0;
        if (!in_danger_zone(target_j0)) {
            safe_j0 = target_j0;  // 目标 j0 本身安全，直接用
        } else {
            // 选择最近的安全边界
            float dist_to_min = std::fabs(current_j0 - (ca.base_danger_min - ca.base_safe_margin));
            float dist_to_max = std::fabs(current_j0 - (ca.base_danger_max + ca.base_safe_margin));
            safe_j0 = (dist_to_min <= dist_to_max)
                ? (ca.base_danger_min - ca.base_safe_margin)
                : (ca.base_danger_max + ca.base_safe_margin);
        }

        std::cout << "  Strategy: rotate base first (j0 -> " << safe_j0 << ")"
                    << std::endl;

        std::vector<float> step1_joints = current_joints;
        step1_joints[0] = safe_j0;

        std::cout << "  Step 1: rotate base to safe position" << std::endl;
        result = MoveToJoints(step1_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        std::cout << "  Step 2: move all joints to target" << std::endl;
        result = MoveToJoints(target_joints);
        return result;

    } else {
        // 情况: 当前和目标 j1 都在危险区 (或其他复杂情况)
        // 策略: 先抬 j1 → 再转 j0 → 最后完整运动
        float safe_j0;
        if (!in_danger_zone(target_j0)) {
            safe_j0 = target_j0;
        } else {
            float dist_to_min = std::fabs(current_j0 - (ca.base_danger_min - ca.base_safe_margin));
            float dist_to_max = std::fabs(current_j0 - (ca.base_danger_max + ca.base_safe_margin));
            safe_j0 = (dist_to_min <= dist_to_max)
                ? (ca.base_danger_min - ca.base_safe_margin)
                : (ca.base_danger_max + ca.base_safe_margin);
        }

        std::cout << "  Strategy: lift shoulder + rotate base (complex)"
                    << std::endl;

        // Step 1: 抬 j1 到安全值
        std::vector<float> step1_joints = current_joints;
        step1_joints[1] = ca.shoulder_threshold;

        std::cout << "  Step 1: lift j1 to safe threshold" << std::endl;
        result = MoveToJoints(step1_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        // Step 2: 转 j0 到安全位置
        std::vector<float> step2_joints = step1_joints;
        step2_joints[0] = safe_j0;

        std::cout << "  Step 2: rotate base to safe position j0=" << safe_j0
                    << std::endl;
        result = MoveToJoints(step2_joints);
        if (result != GraspResult::SUCCESS) return result;
        if (!WaitMotionDone()) return GraspResult::TIMEOUT;

        // Step 3: 完整运动到目标
        std::cout << "  Step 3: move all joints to target" << std::endl;
        result = MoveToJoints(target_joints);
        return result;
    }
}

GraspResult GraspExecutor::SolveIK(const Pose3D& pose, std::vector<float>& joints) {
    if (!arm_ || !kin_) return GraspResult::MOVE_FAILED;

    const auto ik_start = std::chrono::steady_clock::now();

    manip_pose_t target;
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    manip_joint_t solved;
    std::memset(&solved, 0, sizeof(solved));
    int ret = manip_solve_target_joints(arm_, &target, &solved);
    const auto ik_end = std::chrono::steady_clock::now();
    if (config_.performance_log_enabled) {
        const auto ik_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(ik_end - ik_start)
                .count();
        std::cout << "[Perf] ik_solve_ms=" << ik_ms
                    << " mode=direct ret=" << ret << std::endl;
    }
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] solve_target_joints failed: " << ret << std::endl;
        if (ret == MANIP_ERR_PARAM) return GraspResult::IK_FAILED;
        return GraspResult::MOVE_FAILED;
    }

    joints.clear();
    joints.reserve(solved.count);
    for (uint8_t i = 0; i < solved.count; ++i) {
        joints.push_back(solved.joints[i]);
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::SolveIKConstrained(const Pose3D& pose,
                                                std::vector<float>& joints) {
    if (!kin_) return GraspResult::MOVE_FAILED;

    const auto ik_start = std::chrono::steady_clock::now();

    const auto& constraints = config_.joint_constraints;

    // 如果没有约束，退回普通 IK
    if (constraints.empty()) {
        return SolveIK(pose, joints);
    }

    kin_pose_t ik_target;
    ik_target.x  = pose.x;
    ik_target.y  = pose.y;
    ik_target.z  = pose.z;
    ik_target.qw = pose.qw;
    ik_target.qx = pose.qx;
    ik_target.qy = pose.qy;
    ik_target.qz = pose.qz;

    kin_ik_params_t ik_params = {};
    ik_params.epsilon = 1e-3;
    ik_params.position_weight = 1.0;
    ik_params.timeout_s = 0.1;

    int n_joints = kin_get_num_joints(kin_);
    std::vector<double> lower(n_joints), upper(n_joints);
    kin_get_joint_limits(kin_, lower.data(), upper.data());

    std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::vector<float> fallback_joints;
    bool found_valid = false;

    auto check_constraints = [&](const kin_joints_t& q) -> bool {
        for (const auto& c : constraints) {
            if (c.joint_index >= 0 && c.joint_index < q.count) {
                float val = static_cast<float>(q.q[c.joint_index]);
                if (val < c.min_rad || val > c.max_rad) return false;
            }
        }
        return true;
    };

    const int max_trials = config_.ik_max_trials;

    for (int trial = 0; trial < max_trials; ++trial) {
        kin_joints_t q_seed;
        q_seed.count = static_cast<uint8_t>(n_joints);

        if (trial == 0) {
            // 第一次用 observe_joints 作为种子
            auto& obs = config_.observe_joints;
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
        int ik_ret = kin_inverse(kin_, &ik_target, &q_seed, &ik_params, &q_result);
        if (ik_ret != KIN_OK) continue;

        // 记录第一个收敛解作为 fallback
        if (fallback_joints.empty()) {
            for (int j = 0; j < q_result.count; ++j) {
                fallback_joints.push_back(static_cast<float>(q_result.q[j]));
            }
        }

        if (check_constraints(q_result)) {
            joints.clear();
            for (int j = 0; j < q_result.count; ++j) {
                joints.push_back(static_cast<float>(q_result.q[j]));
            }
            found_valid = true;
            if (config_.performance_log_enabled) {
                const auto ik_end = std::chrono::steady_clock::now();
                const auto ik_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        ik_end - ik_start)
                        .count();
                std::cout << "[Perf] ik_solve_ms=" << ik_ms
                            << " mode=constrained trials=" << (trial + 1)
                            << " fallback=0 result=success" << std::endl;
            }
            break;
        }
    }

    if (!found_valid) {
        if (!fallback_joints.empty()) {
            if (config_.performance_log_enabled) {
                const auto ik_end = std::chrono::steady_clock::now();
                const auto ik_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        ik_end - ik_start)
                        .count();
                std::cout << "[Perf] ik_solve_ms=" << ik_ms
                            << " mode=constrained trials=" << max_trials
                            << " fallback=1 result=success" << std::endl;
            }
            std::cerr << "[GraspExecutor] WARNING: no constrained IK solution found, "
                        << "using fallback" << std::endl;
            joints = fallback_joints;
            return GraspResult::SUCCESS;
        }
        if (config_.performance_log_enabled) {
            const auto ik_end = std::chrono::steady_clock::now();
            const auto ik_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    ik_end - ik_start)
                    .count();
            std::cout << "[Perf] ik_solve_ms=" << ik_ms
                        << " mode=constrained trials=" << max_trials
                        << " fallback=0 result=failed" << std::endl;
        }
        std::cerr << "[GraspExecutor] IK failed: no solution in "
                    << max_trials << " trials" << std::endl;
        return GraspResult::IK_FAILED;
    }

    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPoseWithIKJoints(const Pose3D& pose, float speed) {
    std::vector<float> joints;
    GraspResult result = SolveIK(pose, joints);
    if (result != GraspResult::SUCCESS) return result;

    std::cout << "[GraspExecutor] safe IK joints(rad): [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i];
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 通过碰撞安全路径执行
    float old_speed = config_.move_speed;
    config_.move_speed = speed;
    GraspResult move_result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed;
    if (move_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] safe move_joints failed" << std::endl;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPoseConstrained(const Pose3D& pose, float speed) {
    std::vector<float> joints;
    GraspResult result = SolveIKConstrained(pose, joints);
    if (result != GraspResult::SUCCESS) return result;

    std::cout << "[GraspExecutor] constrained IK joints(rad): [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i];
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 通过碰撞安全路径执行
    float old_speed2 = config_.move_speed;
    config_.move_speed = speed;
    GraspResult move_result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed2;
    if (move_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] constrained move_joints failed" << std::endl;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPoseWithYaw(const Pose3D& pose, float speed,
                                            float yaw_rad) {
    std::vector<float> joints;
    GraspResult result = SolveIKConstrained(pose, joints);
    if (result != GraspResult::SUCCESS) {
        diagnostics_.wrist_yaw = WristYawDiagnostics{};
        RecordResult(result, "move_to_pose_with_yaw", "ik failed before yaw");
        return result;
    }

    // 覆盖 joint5 (wrist yaw, index 4)
    // 补偿 joint0 (shoulder_pan) 对夹爪方向的影响:
    //   在顶抓姿态 (joint3 满足约束, 末端Z轴接近垂直) 下:
    //   gripper_angle ≈ joint0 + scale * joint5
    //   因此 joint5 = (gripper_angle - joint0) / scale
    if (joints.size() >= 5) {
        float joint0 = joints[0];
        const float scale = config_.wrist_yaw_scale;   // default: 1.0
        if (std::fabs(scale) < 1e-6f) {
            std::cerr << "[GraspExecutor] invalid wrist_yaw_scale=" << scale
                        << ", skip yaw override" << std::endl;
            diagnostics_.wrist_yaw = WristYawDiagnostics{};
            diagnostics_.wrist_yaw.target_yaw = yaw_rad;
            diagnostics_.wrist_yaw.joint0 = joint0;
            diagnostics_.wrist_yaw.scale = scale;
            RecordResult(GraspResult::IK_FAILED, "move_to_pose_with_yaw",
                        "invalid wrist_yaw_scale");
            return GraspResult::IK_FAILED;
        }

        float joint5_raw = (yaw_rad - joint0) / scale;

        float j5_min = -static_cast<float>(M_PI);
        float j5_max = static_cast<float>(M_PI);
        if (kin_) {
            int n_joints = kin_get_num_joints(kin_);
            if (n_joints > 4) {
                std::vector<double> lower(n_joints), upper(n_joints);
                kin_get_joint_limits(kin_, lower.data(), upper.data());
                j5_min = static_cast<float>(lower[4]);
                j5_max = static_cast<float>(upper[4]);
            }
        }

        float joint5 = joint5_raw;
        if (joint5 > j5_max) joint5 = j5_max;
        if (joint5 < j5_min) joint5 = j5_min;

        diagnostics_.wrist_yaw.valid = true;
        diagnostics_.wrist_yaw.target_yaw = yaw_rad;
        diagnostics_.wrist_yaw.joint0 = joint0;
        diagnostics_.wrist_yaw.scale = scale;
        diagnostics_.wrist_yaw.joint5_raw = joint5_raw;
        diagnostics_.wrist_yaw.joint5_limited = joint5;
        diagnostics_.wrist_yaw.joint5_min = j5_min;
        diagnostics_.wrist_yaw.joint5_max = j5_max;

        std::cout << "[GraspExecutor] wrist_yaw: target=" << yaw_rad
                    << " rad, joint0=" << joint0
                    << ", raw joint5=" << joint5_raw
                    << ", limited joint5=" << joint5
                    << " (limit=[" << j5_min << ", " << j5_max << "])"
                    << std::endl;
        joints[4] = joint5;
    }

    std::cout << "[GraspExecutor] IK+yaw joints(rad): [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << joints[i];
        if (i + 1 < joints.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // 通过碰撞安全路径执行
    float old_speed3 = config_.move_speed;
    config_.move_speed = speed;
    GraspResult move_result = MoveToJointsCollisionSafe(joints);
    config_.move_speed = old_speed3;
    if (move_result != GraspResult::SUCCESS) {
        std::cerr << "[GraspExecutor] move_joints (yaw) failed" << std::endl;
        RecordResult(move_result, "move_to_pose_with_yaw",
                    "move_joints failed after yaw override");
        return move_result;
    }
    RecordResult(GraspResult::SUCCESS, "move_to_pose_with_yaw");
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveToPose(const Pose3D& pose, float speed) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    manip_pose_t target;
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    int ret = manip_move_target(arm_, &target, speed);
    if (ret == MANIP_ERR_NOSYS) {
        // 如果 move_target 不支持，尝试 move_line
        ret = manip_move_line(arm_, &target, speed);
    }
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] move_target failed: " << ret << std::endl;
        if (ret == MANIP_ERR_PARAM) return GraspResult::IK_FAILED;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

GraspResult GraspExecutor::MoveLinear(const Pose3D& pose, float speed) {
    if (!arm_) return GraspResult::MOVE_FAILED;

    manip_pose_t target;
    target.x = pose.x;
    target.y = pose.y;
    target.z = pose.z;
    target.qw = pose.qw;
    target.qx = pose.qx;
    target.qy = pose.qy;
    target.qz = pose.qz;

    int ret = manip_move_line(arm_, &target, speed);
    if (ret != MANIP_OK) {
        std::cerr << "[GraspExecutor] move_line failed: " << ret << std::endl;
        if (ret == MANIP_ERR_PARAM) return GraspResult::IK_FAILED;
        return GraspResult::MOVE_FAILED;
    }
    return GraspResult::SUCCESS;
}

bool GraspExecutor::WaitMotionDone(int timeout_ms) {
    if (!arm_) return false;

    if (timeout_ms <= 0) {
        timeout_ms = wait_motion_timeout_ms_;
    }

    constexpr float kStableThreshold = 0.01f;  // rad, 关节角变化阈值
    constexpr int kStableCount = 10;            // 连续稳定次数 (10 * 50ms = 500ms)
    constexpr int kPollIntervalMs = 50;         // 轮询间隔

    manip_joint_t prev_joints;
    std::memset(&prev_joints, 0, sizeof(prev_joints));
    manip_get_state(arm_, &prev_joints, nullptr);

    int stable_counter = 0;
    auto start = std::chrono::steady_clock::now();

    // 先等一小段时间让运动开始
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        int elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                .count();

        if (elapsed_ms >= timeout_ms) {
            std::cerr << "[GraspExecutor] Motion timeout (" << timeout_ms << "ms)" << std::endl;
            return false;
        }

        Tick(static_cast<float>(kPollIntervalMs) / 1000.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));

        manip_joint_t cur_joints;
        std::memset(&cur_joints, 0, sizeof(cur_joints));
        manip_get_state(arm_, &cur_joints, nullptr);

        // 检查所有关节角变化是否小于阈值
        float max_diff = 0.0f;
        for (int i = 0; i < cur_joints.count && i < prev_joints.count; ++i) {
            float diff = std::fabs(cur_joints.joints[i] - prev_joints.joints[i]);
            if (diff > max_diff) max_diff = diff;
        }

        if (max_diff < kStableThreshold) {
            stable_counter++;
            if (stable_counter >= kStableCount) {
                return true;  // 关节角稳定，运动完成
            }
        } else {
            stable_counter = 0;  // 还在动，重置计数
        }

        prev_joints = cur_joints;
    }
}

}  // namespace perceptive_grasp
