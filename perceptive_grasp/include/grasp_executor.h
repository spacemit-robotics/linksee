/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_executor.h
    * @brief 抓取执行模块 - 机械臂 + 夹爪协调控制
    */

#ifndef GRASP_EXECUTOR_H
#define GRASP_EXECUTOR_H

#include <cmath>
#include <string>
#include <vector>

#include "grasp_planner.h"

// Forward declarations for C API (avoid pulling full headers into this header)
extern "C" {
struct manip_dev;
struct grasp_dev;
struct kin_solver;
typedef struct kin_solver kin_solver_t;
}

namespace perceptive_grasp {

/** 执行结果 */
enum class GraspResult {
    SUCCESS,          // 抓取成功 (HOLDING)
    EMPTY,            // 抓空了
    IK_FAILED,        // IK 求解失败
    OUT_OF_RANGE,     // 超出工作空间
    MOVE_FAILED,      // 运动执行失败
    TIMEOUT,          // 超时
};

struct GripperCheckDiagnostics {
    std::string phase;
    std::string state = "UNKNOWN";
    int holding_count = 0;
    int load_holding_count = 0;
    int check_count = 0;
    float load_threshold = 0.0f;
    float empty_closed_position = NAN;
    float min_object_position = NAN;
    float position = NAN;
    float load = NAN;
};

struct WristYawDiagnostics {
    bool valid = false;
    float target_yaw = NAN;
    float joint0 = NAN;
    float scale = NAN;
    float joint5_raw = NAN;
    float joint5_limited = NAN;
    float joint5_min = NAN;
    float joint5_max = NAN;
};

struct ExecutorDiagnostics {
    GraspResult last_result = GraspResult::SUCCESS;
    std::string last_action;
    std::string last_detail;
    GripperCheckDiagnostics gripper_check;
    WristYawDiagnostics wrist_yaw;
};

struct JointConstraint {
    int joint_index = -1;       // 关节索引 (0-based)
    float min_rad = 0.0f;       // 最小角度 (弧度)
    float max_rad = 0.0f;       // 最大角度 (弧度)
};

/** 碰撞避免配置 */
struct CollisionAvoidanceConfig {
    bool enabled = true;
    // joint0 (底座旋转) 危险区间
    float base_danger_min = -1.480f;
    float base_danger_max = 1.480f;
    // joint1 (肩关节) 阈值: 低于此值时臂伸向机身
    float shoulder_threshold = -0.334f;
    // 安全余量: 移到危险区边界外多少 rad
    float base_safe_margin = 0.1f;
};

struct TimingConfig {
    // 观察位到位后，等待相机曝光/画面稳定
    int observe_settle_ms = 500;
    // 观察前先闭合夹爪后的等待
    int observe_gripper_close_wait_ms = 100;
    // 到预抓取位后，张开夹爪前等待
    int pre_grasp_settle_ms = 150;
    // 张开夹爪后等待
    int gripper_open_wait_ms = 150;
    // 到抓取位后，闭合夹爪前等待
    int grasp_settle_ms = 100;
    // 闭合夹爪后等待
    int gripper_close_wait_ms = 500;
    // 夹爪状态检测次数与间隔
    int grasp_check_count = 10;
    int grasp_check_interval_ms = 50;
    // 抬起后等待夹爪负载稳定，再执行二次持物检查。
    int post_lift_settle_ms = 250;
    // 到放置位后，释放夹爪前等待
    int place_settle_ms = 100;
    // 释放夹爪后等待物体脱落
    int release_wait_ms = 800;
    // 放置后关闭夹爪等待
    int home_gripper_close_wait_ms = 100;
};

struct ExecutorConfig {
    // 机械臂
    std::string manip_driver = "so101";
    std::string uart_device = "/dev/ttyACM0";
    int baudrate = 1000000;

    // 运动学
    std::string urdf_path = "../urdf/so101.urdf";
    std::string base_link = "base_link";
    std::string tip_link = "gripper_frame_link";

    // 速度
    float move_speed = 1.0f;
    float line_speed = 0.5f;
    float pose_position_tolerance = 0.03f;

    // 夹爪
    float gripper_open = 0.5f;
    float gripper_effort = 0.8f;
    float place_release_open = 0.5f;
    float gripper_hold_load_threshold = 100.0f;  // Feetech load units
    float gripper_empty_position_margin = 0.03f;
    int gripper_timeout_ms = 3000;

    // 预定义姿态
    std::vector<float> home_joints = {
        1.816f, -1.850f, 1.639f, 1.147f, 0.189f};
    std::vector<float> observe_joints = {
        1.759f, 0.050f, -0.217f, 1.606f, 0.015f};
    std::vector<float> place_joints = {
        -1.636f, 0.087f, -0.140f, 1.389f, 0.033f};

    // 阶段间等待时间
    TimingConfig timing;

    // IK 关节约束 (用于多种子采样时筛选合格解)
    std::vector<JointConstraint> joint_constraints = {
        {3, 1.102f, 1.667f},
    };

    // IK 多种子采样参数
    int ik_max_trials = 50;

    // Wrist yaw 补偿参数 (从 FK 线性拟合: gripper_angle ≈ joint0 + scale*joint5)
    // 当 joint0≠0 时: joint5_new = (grasp_yaw - joint0) / scale
    float wrist_yaw_scale = 1.0f;    // joint5 对夹爪角度的比例系数；方向反时改符号

    // 碰撞避免
    CollisionAvoidanceConfig collision_avoidance;

    // 性能日志
    bool performance_log_enabled = false;
};

/**
    * @brief 抓取执行器
    *
    * 协调机械臂和夹爪完成抓取动作序列:
    * 1. 移到观察位
    * 2. 移到预抓取位 (目标上方)
    * 3. 直线下探到抓取点
    * 4. 闭合夹爪
    * 5. 检测是否抓住
    * 6. 抬起
    * 7. 移到放置位
    * 8. 释放
    */
class GraspExecutor {
public:
    explicit GraspExecutor(const ExecutorConfig& config) : config_(config) {}
#ifdef MOCK_EXECUTOR
    virtual ~GraspExecutor() = default;
#else
    virtual ~GraspExecutor();
#endif

    // Non-copyable
    GraspExecutor(const GraspExecutor&) = delete;
    GraspExecutor& operator=(const GraspExecutor&) = delete;

#ifdef MOCK_EXECUTOR
    virtual bool Init() = 0;
    virtual GraspResult MoveToObserve() = 0;
    virtual GraspResult MoveToHome() = 0;
    virtual GraspResult MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad = NAN) = 0;
    virtual GraspResult OpenGripperForGrasp() = 0;
    virtual GraspResult MoveToGrasp(const Pose3D& grasp_pose,
                                    float grasp_yaw_rad = NAN) = 0;
    virtual GraspResult CloseGripperAndCheck() = 0;
    virtual GraspResult LiftFromGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad = NAN) = 0;
    virtual GraspResult MoveToPlace() = 0;
    virtual GraspResult ReleaseObject() = 0;
    virtual GraspResult CloseGripper() = 0;
    virtual GraspResult ExecuteGrasp(const Pose3D& grasp_pose,
                                    const Pose3D& pre_grasp_pose,
                                    float grasp_yaw_rad = NAN) = 0;
    virtual GraspResult ExecutePlace() = 0;
    virtual void EmergencyStop() = 0;
    virtual bool GetCurrentPose(Pose3D& pose) = 0;
    virtual void Tick(float dt_s) = 0;
    virtual ExecutorDiagnostics GetDiagnostics() const = 0;
#else
    virtual bool Init();
    virtual GraspResult MoveToObserve();
    virtual GraspResult MoveToHome();
    virtual GraspResult MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad = NAN);
    virtual GraspResult OpenGripperForGrasp();
    virtual GraspResult MoveToGrasp(const Pose3D& grasp_pose,
                                    float grasp_yaw_rad = NAN);
    virtual GraspResult CloseGripperAndCheck();
    virtual GraspResult LiftFromGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad = NAN);
    virtual GraspResult MoveToPlace();
    virtual GraspResult ReleaseObject();
    virtual GraspResult CloseGripper();
    virtual GraspResult MoveToJointsSafe(const std::vector<float>& joints,
                                        float speed_scale = -1.0f);
    virtual GraspResult MoveToPreGraspSafe(const Pose3D& pre_grasp_pose,
                                            float speed_scale = -1.0f);
    virtual GraspResult ExecuteGrasp(const Pose3D& grasp_pose,
                                    const Pose3D& pre_grasp_pose,
                                    float grasp_yaw_rad = NAN);
    virtual GraspResult ExecutePlace();
    virtual void EmergencyStop();
    virtual bool GetCurrentPose(Pose3D& pose);
    virtual void Tick(float dt_s);
    virtual ExecutorDiagnostics GetDiagnostics() const { return diagnostics_; }
    void SetWaitMotionTimeoutMs(int timeout_ms) { wait_motion_timeout_ms_ = timeout_ms; }

    /**
    * @brief 对给定 pose 做 IK 求解，返回关节角
    * @param pose 目标位姿
    * @param[out] joints 求解结果
    * @return SUCCESS 或 IK_FAILED
    */
    GraspResult SolveIK(const Pose3D& pose, std::vector<float>& joints);

    /**
    * @brief 获取运动学求解器句柄 (供外部直接调用 kin_inverse 等)
    */
    kin_solver_t* GetKinSolver() const { return kin_; }
#endif

protected:
    ExecutorConfig config_;

#ifndef MOCK_EXECUTOR
private:
    struct manip_dev* arm_ = nullptr;
    struct grasp_dev* gripper_ = nullptr;
    kin_solver_t* kin_ = nullptr;
    int wait_motion_timeout_ms_ = 15000;
    float empty_closed_position_ = NAN;
    ExecutorDiagnostics diagnostics_;

    void RecordResult(GraspResult result, const std::string& action,
                        const std::string& detail = "");
    GraspResult CheckGripperHolding(const char* phase, bool after_lift);
    void CaptureEmptyClosedPosition();
    GraspResult MoveToJoints(const std::vector<float>& joints);
    GraspResult MoveToJointsCollisionSafe(const std::vector<float>& joints);
    GraspResult MoveToPoseWithIKJoints(const Pose3D& pose, float speed);
    GraspResult MoveToPoseConstrained(const Pose3D& pose, float speed);
    GraspResult MoveToPoseWithYaw(const Pose3D& pose, float speed,
                                    float yaw_rad);
    GraspResult SolveIKConstrained(const Pose3D& pose, std::vector<float>& joints);
    GraspResult MoveToPose(const Pose3D& pose, float speed);
    GraspResult MoveLinear(const Pose3D& pose, float speed);
    bool WaitMotionDone(int timeout_ms = -1);
    bool VerifyPoseReached(const char* action, const Pose3D& target_pose);
    bool GetCurrentJoints(std::vector<float>& joints);
    bool NeedsCollisionAvoidance(const std::vector<float>& current_joints,
                                const std::vector<float>& target_joints);
#endif
};

}  // namespace perceptive_grasp

#endif  // GRASP_EXECUTOR_H
