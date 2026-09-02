/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file grasp_executor.h
 * @brief 抓取执行模块 - 机械臂 + 夹爪协调控制
 */

#ifndef GRASP_EXECUTOR_H
#define GRASP_EXECUTOR_H

#include <cstddef>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "arm_path_safety.h"
#include "gripper_holding.h"
#include "grasp_planner.h"
#include "motion_completion.h"

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
    std::string decision = "INCONCLUSIVE";
    int holding_count = 0;
    int load_holding_count = 0;
    int opening_count = 0;
    int contact_count = 0;
    int empty_count = 0;
    int check_count = 0;
    int baseline_sample_count = 0;
    float load_threshold = 0.0f;
    float empty_closed_position = NAN;
    float empty_closed_position_mad = NAN;
    float empty_closed_load = NAN;
    float empty_closed_load_mad = NAN;
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
    float validation_min_joint_margin_rad = NAN;
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
    int gripper_close_wait_ms = 1000;
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
    float side_pose_position_tolerance = 0.005f;
    float side_lift_clearance_m = 0.010f;
    float support_surface_clearance_m = 0.005f;
    float path_joint_step_rad = 0.040f;
    float side_waypoint_joint_tolerance_rad = 0.040f;
    float place_joint_tolerance_rad = 0.080f;
    // True selects the SO101 fixed-jaw top-path solver. False selects the
    // fixed-yaw refinement used by simulation executors.
    bool legacy_top_ik = true;

    // 夹爪
    float gripper_open = 0.5f;
    float gripper_effort = 0.8f;
    float place_release_open = 0.5f;
    float gripper_hold_load_threshold = 100.0f;  // Feetech load units
    float gripper_empty_position_margin = 0.03f;
    int gripper_timeout_ms = 3000;

    // 预定义姿态
    std::vector<float> home_joints = {
        1.546f, -1.765f, 1.400f, 1.093f, 0.186f};
    std::vector<float> observe_joints = {
        1.544f, -0.067f, 0.101f, 1.382f, 0.188f};
    std::vector<float> side_ready_joints = {
        1.550f, 0.021f, 1.400f, -1.700f, -0.036f};
    std::vector<float> place_joints = {
        -1.500f, -0.122f, 0.101f, 1.383f, 0.330f};

    // 阶段间等待时间
    TimingConfig timing;

    // IK 关节约束 (用于多种子采样时筛选合格解)
    std::vector<JointConstraint> joint_constraints = {
        {3, 1.102f, 1.667f},
    };

    // Calibrated servo limits, kept inside the persistent register limits.
    std::vector<JointConstraint> joint_limits = {
        {0, -1.540f, 1.630f},
        {1, -1.770f, 1.680f},
        {2, -1.840f, 1.430f},
        {3, -1.790f, 1.670f},
        {4, -2.700f, 2.800f},
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

    struct MujocoConfig {
        std::string xml_path;
        std::string end_effector_site = "pinch";
        std::string gripper_actuator = "fingers_actuator";
        std::string robot_root_body = "ur5e_base";
        std::string gripper_root_body = "2f85_base";
        std::vector<std::string> joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        };
        std::vector<std::string> actuator_names = {
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        };
        float gripper_open_ctrl = 0.0f;
        float gripper_close_ctrl = 255.0f;
        bool gravity_compensation = true;
        float arm_stiffness_scale = 1.0f;
        float joint_tolerance_rad = 0.015f;
        float ik_position_tolerance_m = 0.006f;
        float cartesian_tracking_tolerance_m = 0.002f;
        float ik_step_scale = 0.65f;
        float ik_damping = 0.035f;
        int ik_iterations = 120;
        int settle_steps = 250;
        int max_motion_steps = 2500;
    } mujoco;

    struct RemoteMujocoConfig {
        std::string host = "127.0.0.1";
        int port = 9090;
        int timeout_ms = 15000;
    } remote_mujoco;
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

    virtual void SetTargetLabel(const std::string& target_label) {
        (void)target_label;
    }

#ifdef MOCK_EXECUTOR
    virtual bool Init() = 0;
    virtual GraspResult MoveToObserve() = 0;
    virtual GraspResult MoveToSideObserve() = 0;
    virtual GraspResult MoveToHome() = 0;
    virtual GraspResult MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad = NAN,
                                        bool use_top_constraints = true) = 0;
    virtual GraspResult OpenGripperForGrasp(
        float minimum_opening = NAN) = 0;
    virtual GraspResult MoveToGrasp(const Pose3D& grasp_pose,
                                    float grasp_yaw_rad = NAN,
                                    bool use_top_constraints = true) = 0;
    virtual GraspResult CloseGripperAndCheck() = 0;
    virtual GraspResult LiftFromGrasp(const Pose3D& retreat_pose,
                                        const Pose3D& lift_pose,
                                        float grasp_yaw_rad = NAN,
                                        bool use_top_constraints = true) = 0;
    virtual GraspResult ValidateGraspPoses(
        const Pose3D& pre_grasp_pose,
        const Pose3D& grasp_pose,
        const Pose3D& retreat_pose,
        const Pose3D& lift_pose,
        float entry_clearance_z_m,
        float grasp_yaw_rad,
        bool use_top_constraints,
        int timeout_ms,
        std::string* detail) = 0;
    virtual void SetSupportPlane(const SupportPlane& support_plane) = 0;
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
    virtual GraspResult MoveToSideObserve();
    virtual GraspResult MoveToHome();
    virtual GraspResult MoveToPreGrasp(const Pose3D& pre_grasp_pose,
                                        float grasp_yaw_rad = NAN,
                                        bool use_top_constraints = true);
    virtual GraspResult OpenGripperForGrasp(float minimum_opening = NAN);
    virtual GraspResult MoveToGrasp(const Pose3D& grasp_pose,
                                    float grasp_yaw_rad = NAN,
                                    bool use_top_constraints = true);
    virtual GraspResult CloseGripperAndCheck();
    virtual GraspResult LiftFromGrasp(const Pose3D& retreat_pose,
                                        const Pose3D& lift_pose,
                                        float grasp_yaw_rad = NAN,
                                        bool use_top_constraints = true);
    virtual GraspResult ValidateGraspPoses(
        const Pose3D& pre_grasp_pose,
        const Pose3D& grasp_pose,
        const Pose3D& retreat_pose,
        const Pose3D& lift_pose,
        float entry_clearance_z_m,
        float grasp_yaw_rad,
        bool use_top_constraints,
        int timeout_ms,
        std::string* detail);
    virtual void SetSupportPlane(const SupportPlane& support_plane);
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
    std::unique_ptr<ArmPathSafety> arm_path_safety_;
    SupportPlane support_plane_;
    std::vector<float> validated_side_staging_joints_;
    std::vector<float> validated_side_sweep_joints_;
    Pose3D validated_side_entry_pose_{};
    std::vector<float> validated_side_entry_joints_;
    std::vector<Pose3D> validated_side_poses_;
    std::vector<std::vector<float>> validated_side_joint_path_;
    size_t validated_side_path_index_ = 0;
    Pose3D validated_top_pre_grasp_pose_{};
    std::vector<float> validated_top_pre_grasp_joints_;
    std::vector<Pose3D> validated_top_poses_;
    std::vector<std::vector<float>> validated_top_joint_path_;
    size_t validated_top_path_index_ = 0;
    std::vector<float> active_target_joints_;
    int active_motion_timeout_ms_ = 3000;
    int wait_motion_timeout_ms_ = 15000;
    std::string last_motion_wait_detail_;
    GripperBaseline gripper_baseline_;
    ExecutorDiagnostics diagnostics_;

    void RecordResult(GraspResult result, const std::string& action,
                        const std::string& detail = "");
    GraspResult CheckGripperHolding(const char* phase, bool after_lift);
    void CaptureEmptyClosedPosition();
    bool WaitGripperOpening(float target_position);
    GraspResult MoveToJoints(const std::vector<float>& joints);
    GraspResult MoveToJointsCoordinated(
        const std::vector<float>& joints);
    GraspResult MoveToJointsCollisionSafe(
        const std::vector<float>& joints,
        bool allow_missing_support_surface = false);
    GraspResult BuildCollisionSafeJointPath(
        const std::vector<float>& start_joints,
        const std::vector<float>& target_joints,
        std::vector<std::vector<float>>& path,
        std::string* detail,
        bool allow_missing_support_surface = false) const;
    GraspResult ValidateJointPathSafety(
        const std::vector<float>& start_joints,
        const std::vector<float>& target_joints,
        std::string* detail,
        bool allow_contact_retreat = false,
        bool allow_missing_support_surface = false) const;
    GraspResult MoveToPoseWithIKJoints(const Pose3D& pose, float speed);
    GraspResult MoveToPoseSide(const Pose3D& pose, float speed);
    GraspResult MoveToSidePreGrasp(const Pose3D& pose, float speed);
    GraspResult MoveToSideLift(const Pose3D& retreat_pose,
                                const Pose3D& lift_pose,
                                float speed);
    GraspResult MoveToPoseConstrained(const Pose3D& pose, float speed);
    GraspResult MoveToPoseWithYaw(const Pose3D& pose, float speed,
                                    float yaw_rad);
    GraspResult SolveIKConstrained(
        const Pose3D& pose,
        std::vector<float>& joints,
        const std::vector<float>* seed_joints = nullptr,
        int timeout_ms = -1,
        std::function<bool(std::vector<float>&)> candidate_validator = {},
        float position_weight = 1.0f,
        float ik_epsilon = 1e-3f);
    GraspResult PlanTopJointPath(
        const std::vector<Pose3D>& poses,
        float grasp_yaw_rad,
        int timeout_ms,
        std::vector<std::vector<float>>& joint_path,
        std::string* detail,
        const std::vector<float>* start_joints = nullptr);
    GraspResult PlanTopJointPathLegacy(
        const std::vector<Pose3D>& poses,
        float grasp_yaw_rad,
        int timeout_ms,
        std::vector<std::vector<float>>& joint_path,
        std::string* detail,
        const std::vector<float>* start_joints);
    GraspResult MoveAlongValidatedTopPath(
        const Pose3D& target_pose,
        float speed,
        const char* action,
        float final_tolerance_rad,
        bool allow_contact_retreat = false);
    bool ApplyWristYaw(
        float grasp_yaw_rad,
        std::vector<float>& joints,
        std::string* detail,
        bool record_diagnostics);
    GraspResult SolveTopIKWithWristYaw(
        const Pose3D& pose,
        float grasp_yaw_rad,
        std::vector<float>& joints,
        const std::vector<float>* seed_joints,
        int timeout_ms,
        std::function<bool(std::vector<float>&)> candidate_validator,
        std::string* detail);
    GraspResult SolveIKSide(const Pose3D& pose,
                            int timeout_ms,
                            std::vector<float>& joints,
                            std::string* detail = nullptr,
                            const std::vector<float>* path_start = nullptr);
    GraspResult PlanSideJoint0Sweep(
        const Pose3D& pre_grasp_pose,
        float entry_clearance_z_m,
        int timeout_ms,
        std::vector<float>& staging_joints,
        std::vector<float>& sweep_joints,
        std::string* detail);
    std::vector<Pose3D> BuildSideLiftPath(const Pose3D& retreat_pose,
                                            const Pose3D& lift_pose) const;
    std::vector<Pose3D> BuildSideCartesianPath(
        const Pose3D& start_pose,
        const Pose3D& end_pose,
        float maximum_step_m) const;
    GraspResult PlanSideJointPath(
        const std::vector<Pose3D>& poses,
        int timeout_ms,
        std::vector<std::vector<float>>& joint_path,
        std::string* detail,
        const std::vector<float>* start_joints = nullptr);
    GraspResult ExecuteContinuousJointPath(
        const std::vector<std::vector<float>>& joint_path,
        float first_speed,
        float remaining_speed,
        float final_tolerance_rad,
        int completion_timeout_ms = -1,
        bool allow_contact_retreat = false,
        std::function<bool()> settled_acceptance = {});
    GraspResult CorrectSidePose(
        const Pose3D& pose, float speed, const char* action);
    bool TakeValidatedSidePath(
        const std::vector<Pose3D>& poses,
        std::vector<std::vector<float>>& joint_path);
    GraspResult SolveIKFast(const Pose3D& pose,
                            bool use_top_constraints,
                            int timeout_ms,
                            std::vector<float>& joints);
    GraspResult WaitMotionDone(
        int timeout_ms = -1,
        float target_tolerance_rad = 0.060f,
        std::function<bool()> settled_acceptance = {});
    bool VerifyPoseReached(const char* action, const Pose3D& target_pose);
    bool GetCurrentJoints(std::vector<float>& joints);
    bool NeedsCollisionAvoidance(const std::vector<float>& current_joints,
                                const std::vector<float>& target_joints) const;
#endif
};

}  // namespace perceptive_grasp

#endif  // GRASP_EXECUTOR_H
