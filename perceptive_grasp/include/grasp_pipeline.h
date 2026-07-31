/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    *
    * @file grasp_pipeline.h
    * @brief 视觉抓取主 Pipeline - 串联检测、定位、规划、执行
    */

#ifndef GRASP_PIPELINE_H
#define GRASP_PIPELINE_H

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "grasp_executor.h"
#include "grasp_geometry.h"
#include "grasp_planner.h"
#include "mobile_base_controller.h"
#include "orientation_estimator.h"
#include "pipeline_state.h"
#include "pipeline_timing.h"
#include "stereo_camera.h"
#include "target_association.h"
#include "target_detector.h"
#include "voice_command_parser.h"

namespace perceptive_grasp {

/** Pipeline 事件回调 (用于外部集成，如语音反馈) */
using PipelineCallback = std::function<void(PipelineState state,
                                            const std::string& message)>;

struct PipelineConfig {
    StereoCameraConfig camera;
    DetectorConfig detector;
    GraspPlannerConfig planner;
    GraspGeometryConfig geometry;
    ExecutorConfig executor;
    MobileBaseAlignmentConfig mobile_base;
    OrientationConfig orientation;  // 夹爪方向估计配置

    // Pipeline 行为
    int max_retries = 3;           // 抓空重试次数
    int detect_stable_frames = 3;  // 目标位置和轮廓连续稳定帧数
    bool auto_loop = false;        // 自动循环抓取
    bool auto_orient = true;       // 自动对齐夹爪方向 (根据物体形状)
    bool step_mode = false;        // 单步模式 (每阶段暂停确认)
    bool plan_only = false;        // 仅验证感知、IK 和路径，不执行运动
    bool performance_log_enabled = false;  // 是否打印检测/IK耗时日志
    int target_missing_frames = 20; // 指定目标连续未检出多少帧后报不存在
    // Top-grasp pixel offset: 0=center, 1=short-axis edge.
    float top_grasp_point_x_ratio = 0.5f;
    VoiceCommandConfig voice;       // 语音命令配置

    // 抓取调试数据保存
    bool save_debug_data = true;
    std::string debug_output_dir = "../debug_grasp_runs";

};

/**
    * @brief 视觉抓取主 Pipeline
    *
    * 状态机驱动，串联所有模块:
    *   IDLE → OBSERVING → DETECTING → PLANNING → APPROACHING
    *        → GRASPING → LIFTING → PLACING → DONE
    *
    * 外部触发方式:
    *   1. TriggerGrasp() - 抓取最近/最大的目标
    *   2. TriggerGrasp(label) - 抓取指定类别
    *   3. TriggerVoiceCommand(text) - 语音 ASR 文本入口
    */
class GraspPipeline {
public:
    explicit GraspPipeline(const PipelineConfig& config);
    ~GraspPipeline();

    /**
    * @brief 初始化所有模块
    * @return true 全部初始化成功
    */
    bool Init();

    /**
    * @brief 触发一次抓取 (抓最佳目标)
    * @return true 触发成功 (pipeline 开始执行)
    */
    bool TriggerGrasp();

    /**
    * @brief 触发抓取指定目标 (语音接口)
    * @param target_label 目标类别名称 (如 "apple", "bottle")
    * @return true 触发成功
    */
    bool TriggerGrasp(const std::string& target_label);

    /**
    * @brief 处理语音命令文本。
    *
    * 例如 ASR 输出 "抓香蕉" 或 "grab banana" 后调用此接口。
    * 负责解析触发词和目标名，实际 ASR/TTS 可通过外部模块接入。
    */
    bool TriggerVoiceCommand(const std::string& command_text);

    /**
    * @brief 停止当前任务
    */
    void Stop();

    /**
    * @brief 请求安全退出：等待当前动作结束，回到 home 位后退出主循环
    */
    void RequestGracefulShutdown();

    /**
    * @brief 获取当前状态
    */
    PipelineState GetState() const { return state_.load(); }

    /**
    * @brief 注册状态回调 (语音/UI 集成用)
    */
    void SetCallback(PipelineCallback callback);

    /**
    * @brief 主循环 (阻塞，通常在独立线程中运行)
    * @param external_shutdown_requested 外部安全退出请求探针
    */
    void Run(const std::function<bool()>& external_shutdown_requested = {});

    /**
    * @brief 单步执行 (非阻塞，外部循环调用)
    * @param dt_s 时间间隔
    */
    void SpinOnce(float dt_s);

private:
    struct PendingVoiceCommand {
        enum class Type {
            GRASP,
            CANCEL,
            HOME,
        };

        Type type = Type::GRASP;
        std::string target;
        std::string raw_text;
    };

    struct AsyncAction {
        bool active = false;
        bool cancelling = false;
        PipelineState owner = PipelineState::IDLE;
        std::string name;
        std::chrono::steady_clock::time_point started_at;
        std::int64_t started_cpu_ms = 0;
        std::future<GraspResult> future;
    };

    PipelineConfig config_;

    // 模块
    std::unique_ptr<StereoCamera> camera_;
    std::unique_ptr<TargetDetector> detector_;
    std::unique_ptr<GraspPlanner> planner_;
    std::unique_ptr<GraspGeometryPlanner> geometry_planner_;
    std::unique_ptr<GraspExecutor> executor_;
    std::unique_ptr<MobileBaseController> mobile_base_;

    // 状态
    std::atomic<PipelineState> state_{PipelineState::IDLE};
    std::string target_label_;  // 指定目标 (空=最佳)
    std::string auto_loop_target_label_;
    int auto_loop_iteration_ = 0;
    int retry_count_ = 0;
    int stable_count_ = 0;
    int missing_count_ = 0;
    int geometry_retry_count_ = 0;
    int base_align_attempts_ = 0;
    bool target_stationary_confirmed_ = false;
    bool motion_geometry_confirmation_pending_ = false;
    bool motion_geometry_reference_valid_ = false;
    int motion_geometry_sample_count_ = 0;
    int motion_geometry_consistent_count_ = 0;
    int motion_geometry_refresh_count_ = 0;
    ObjectGeometry3D motion_geometry_reference_;
    bool top_alignment_reference_valid_ = false;
    std::array<float, 3> top_alignment_reference_ = {};
    bool have_previous_base_alignment_point_ = false;
    std::array<float, 3> previous_base_alignment_point_ = {};
    MobileBaseAlignmentCommand previous_base_alignment_command_;
    bool last_top_support_plane_valid_ = false;
    SupportPlane last_top_support_plane_;
    float base_align_travel_m_ = 0.0f;
    int base_align_direction_reversals_ = 0;
    AsyncAction action_;
    bool failure_recovery_active_ = false;
    bool failure_recovery_succeeded_ = false;
    std::string pending_failure_message_;
    std::atomic<bool> object_may_be_held_{false};

    // 回调
    PipelineCallback callback_;
    std::mutex callback_mutex_;

    // 语音命令队列:
    // ROS2 订阅回调线程只入队，不直接修改抓取状态机。
    std::mutex voice_queue_mutex_;
    std::queue<PendingVoiceCommand> voice_queue_;
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> graceful_shutdown_requested_{false};
    std::atomic<bool> shutdown_requested_{false};
    bool return_to_observe_pending_ = false;
    bool return_to_home_pending_ = false;
    bool place_possible_object_pending_ = false;
    bool waiting_voice_target_ = false;
    std::chrono::steady_clock::time_point waiting_voice_target_since_;

    // 缓存的检测和规划结果
    DetectionTarget current_target_{};
    TargetTrack target_track_{};
    std::vector<DetectionTarget> last_candidates_;
    cv::Mat current_color_;
    cv::Mat current_depth_;
    Pose3D grasp_pose_{};
    Pose3D pre_grasp_pose_{};
    Pose3D retreat_pose_{};
    Pose3D lift_pose_{};
    GraspStrategy grasp_strategy_ = GraspStrategy::TOP;
    bool observation_strategy_selected_ = false;
    float grasp_opening_ = NAN;
    GraspGeometryResult grasp_geometry_result_;
    ObjectGeometry3D last_valid_geometry_;
    bool last_valid_geometry_available_ = false;
    GraspStrategy last_valid_strategy_ = GraspStrategy::TOP;
    bool last_valid_strategy_available_ = false;
    bool top_geometry_recovery_active_ = false;
    MobileBaseAlignmentCommand base_alignment_command_;
    float grasp_yaw_rad_ = NAN;  // 夹爪旋转角 (NAN=不覆盖)
    std::string task_id_;
    std::string last_debug_image_path_;
    std::string last_debug_json_path_;
    std::string last_status_message_;

    // One task may visit DETECTING/PLANNING/BASE_ALIGNING multiple times.
    PipelineTiming task_timing_;
    std::chrono::steady_clock::time_point perception_cycle_started_at_;
    bool perception_cycle_active_ = false;
    std::int64_t initialization_elapsed_ms_ = 0;

    void SetState(PipelineState new_state, const std::string& msg = "");
    void BeginTaskTiming();
    void PrintTaskSummary(PipelineState terminal_state,
                        const std::string& message);
    void ResetTaskState();
    void RestartAutoLoop(const char* previous_result);
    bool HasActiveAction() const { return action_.active; }
    bool StartAction(PipelineState owner, const std::string& name,
                    std::function<GraspResult()> fn);
    std::optional<GraspResult> PollAction(PipelineState owner,
                                            bool accept_any_owner = false);
    void ClearAction();
    bool ConsumeVoiceCommand();
    bool WaitForConfirm(const std::string& prompt);
    std::string FormatCandidates(size_t max_items = 5) const;
    std::string ResultMessage(const std::string& phase,
                                GraspResult result) const;
    bool RetryRecoverableMotion(const std::string& phase,
                                GraspResult result);
    GraspResult PlacePossibleObjectAndReturnHome();
    bool FlushCameraAfterMotion(const char* reason);
    bool SaveStepCameraDebug(const char* phase);
    bool BuildMaskTopGrasp(
        GraspCandidate& candidate,
        float& grasp_px,
        float& grasp_py,
        uint16_t& depth_mm,
        float cam_point[3],
        float base_point[3],
        std::string& error,
        float fallback_depth_mm = NAN,
        const SupportPlane* fallback_support_plane = nullptr,
        bool allow_global_mask_depth = true);
    bool EstimateSupportPlaneDepth(
        const SupportPlane& support_plane,
        int pixel_x,
        int pixel_y,
        float& depth_mm) const;
    bool ResolveTopSupportPlane(
        const TablePlane& table,
        SupportPlane& support_plane,
        std::string& source);
    bool BuildLowProfileTopFallback(
        const GraspGeometryResult& failed_geometry,
        GraspGeometryResult& recovered_geometry);
    bool ConfirmTopAlignmentPoint(const float alignment_point[3]);
    void HandleTopPlanning();
    bool RetryTransientTopPlanning(const std::string& reason);
    void SaveGraspDebug(float grasp_px, float grasp_py, uint16_t depth_mm,
                        const float cam_point[3], const float base_point[3]);
    void SaveTaskResultDebug(PipelineState terminal_state,
                            const std::string& message);
    void HandleIdle();
    void HandleObserving();
    void HandleDetecting();
    void HandlePlanning();
    void HandleBaseAligning();
    bool ValidateBaseAlignmentCommandTransition(
        const MobileBaseAlignmentCommand& command,
        std::string& error);
    void HandleApproaching();
    void HandleGrasping();
    void HandleLifting();
    void HandlePlacing();
    void HandleHoming();
    void HandleRecovering();
};

}  // namespace perceptive_grasp

#endif  // GRASP_PIPELINE_H
