# 状态机

Pipeline 是抓取主流程状态机：

```text
IDLE -> OBSERVING -> DETECTING -> PLANNING -> APPROACHING
     -> GRASPING -> LIFTING -> PLACING -> HOMING -> DONE

mobile_base.enabled=true 且目标不在舒适抓取区时：

PLANNING -> BASE_ALIGNING -> DETECTING -> PLANNING
```

## 状态说明

| 状态 | 作用 |
|------|------|
| `IDLE` | 等待目标命令 |
| `OBSERVING` | 移动到观察姿态 |
| `DETECTING` | 获取 RGBD 并检测目标 |
| `PLANNING` | 计算抓取像素、深度、基座坐标、抓取姿态和 yaw |
| `BASE_ALIGNING` | 底盘短距离前进、后退或原地转向，动作完成后重新检测 |
| `APPROACHING` | 移动到预抓取位 |
| `GRASPING` | 张开夹爪、下探、闭合并检测是否夹住 |
| `LIFTING` | 抬回预抓取位 |
| `PLACING` | 移动到放置位并释放 |
| `HOMING` | 放置完成后回到收尾姿态 |
| `DONE` | 任务完成 |
| `ERROR` | 任务失败 |

## 异步执行

主循环每 50 ms 调用一次 `SpinOnce()`。机械臂运动和夹爪动作通过后台 `std::future` 执行，状态机只轮询动作结果：

```text
StartAction(...) -> PollAction(...) -> 根据 GraspResult 切换状态
```

取消命令会停止当前任务，并让机械臂回到观察姿态等待下一条命令。
若机械臂或夹爪动作正在执行，状态机会等待当前后台动作返回后清理任务。

底盘辅助对齐也通过后台动作执行。进入 `BASE_ALIGNING` 前，机械臂保持观察姿态；底盘只执行一次短动作，刹车并等待画面稳定后回到 `DETECTING`，重新检测目标和重新规划抓取。达到最大底盘对齐次数后，如果当前抓取点仍在机械臂工作空间内，状态机会继续执行机械臂抓取。

抓取成功后，状态机在 `HOMING` 阶段执行收尾动作。普通指定目标模式回到 Home 姿态并进入 `DONE`，随后退出程序；语音模式回到观察姿态并等待下一条抓取命令。语音模式收到 `结束`、`待命`、`休息`、`回家`、`回 home`、`回初始`、`end` 或 `home` 后回到 Home 姿态并退出程序。

## 状态输出

启用 `--status-stdout` 后，状态机会输出给语音桥使用的事件：

```text
VOICE_STATUS    state=OBSERVING;message=Moving to observe, target: banana;target=banana
VOICE_STATUS    state=DONE;message=Task completed!;reason=success
```

规划成功后保存 `grasp_*.json/png`，任务结束后保存 `grasp_*_result.json`。

## 失败消息

| 类型 | 示例 |
|------|------|
| 目标不存在 | `Target not found: carrot; candidates: banana(0.89)` |
| 深度无效 | `Target depth invalid at grasp pixel` |
| 工作空间越界 | `Target out of workspace: base_point=[...]` |
| 底盘对齐失败 | `Mobile base alignment failed: ...` |
| IK 失败 | `Pre-grasp move failed: IK failed` |
| 夹爪空抓 | `Grasp empty; max retries reached` |
| 夹爪超时 | `Gripper close timeout: timeout` |
| 执行器异常 | `motion or device error` |
