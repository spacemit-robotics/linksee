# pipeline 状态机

pipeline 使用状态机组织感知、规划、底盘辅助和机械臂动作。正常抓取依次经过 `OBSERVING`、`DETECTING`、`PLANNING`、`APPROACHING`、`GRASPING`、`LIFTING`、`PLACING`、`HOMING` 和 `DONE`。

目标不在舒适抓取区且底盘辅助已启用时，状态机从 `PLANNING` 进入 `BASE_ALIGNING`，底盘动作完成后返回 `DETECTING`。任一阶段发生不可恢复错误时进入 `ERROR`。

## 1. 状态定义

| 状态 | 作用 |
|------|------|
| `IDLE` | 等待目标命令 |
| `OBSERVING` | 移动到观察姿态 |
| `DETECTING` | 获取立体相机帧并检测目标 |
| `PLANNING` | 计算抓取像素、深度、基座坐标、抓取姿态和 yaw |
| `BASE_ALIGNING` | 底盘短距离前进、后退或原地转向，动作完成后重新检测 |
| `APPROACHING` | 移动到预抓取位 |
| `GRASPING` | 张开夹爪、下探、闭合并检测是否夹住 |
| `LIFTING` | 抬回预抓取位 |
| `PLACING` | 移动到放置位并释放 |
| `HOMING` | 放置完成后回到收尾姿态 |
| `DONE` | 任务完成 |
| `ERROR` | 任务失败 |

## 2. 执行动作

主循环每 50 ms 调用一次 `SpinOnce()`。机械臂运动、夹爪动作和底盘对齐通过后台 `std::future` 执行。状态机使用 `StartAction()` 启动动作，使用 `PollAction()` 轮询结果，再根据 `GraspResult` 切换状态。

取消命令会停止当前任务，并让机械臂回到观察姿态等待下一条命令。若机械臂或夹爪动作正在执行，状态机会等待当前后台动作返回后清理任务。

抓取成功后，状态机在 `HOMING` 阶段执行收尾动作。普通指定目标模式回到 home 姿态并进入 `DONE`，随后退出程序；语音模式回到观察姿态并等待下一条抓取命令。语音模式收到 `结束`、`待命`、`休息`、`回家`、`回 home`、`回初始`、`end` 或 `home` 后回到 home 姿态并退出程序。

## 3. 底盘闭环

进入 `BASE_ALIGNING` 前，机械臂保持观察姿态。底盘执行一个短动作并刹车，经过程序内置的停车稳定时间后回到 `DETECTING`。pipeline 获取新帧、重新检测目标并重新规划，不复用移动前的目标坐标。

以下任一条件成立时，底盘对齐停止并进入 `ERROR`：

- 重新定位后的目标距离没有达到视觉进展门限。
- 对齐次数达到程序内置上限。
- 累计直行距离达到程序内置上限。
- 底盘驱动、串口或里程计反馈失败。

## 4. 状态输出

启用 `--status-stdout` 后，状态机会输出给语音桥使用的事件：

```text
VOICE_STATUS    state=OBSERVING;message=Moving to observe, target: banana;target=banana
VOICE_STATUS    state=DONE;message=Task completed!;reason=success
```

规划成功后保存 `grasp_*.json/png`，任务结束后保存 `grasp_*_result.json`。

结构化运行日志还会输出 `[Stage N]`、`[Action]` 和 `PIPELINE SUMMARY`，用于查看每个阶段的耗时和最终结果。性能字段说明见[故障诊断](debugging.md#9-分析-pipeline-性能)。

## 5. 失败终态

| 类型 | 示例 |
|------|------|
| 目标不存在 | `Target not found: carrot; candidates: banana(0.89)` |
| 深度无效 | `Target depth invalid at grasp pixel` |
| 工作空间越界 | `Target out of workspace: base_point=[...]` |
| 底盘对齐失败 | `Mobile base alignment failed: ...` |
| ik 失败 | `Pre-grasp move failed: IK failed` |
| 夹爪空抓 | `Grasp empty; max retries reached` |
| 夹爪超时 | `Gripper close timeout: timeout` |
| 执行器异常 | `motion or device error` |
