# pipeline 状态机

pipeline 使用状态机组织感知、规划、底盘辅助和机械臂动作。任务启动后先经过 `DETECTING` 和 `PLANNING`。普通目标按类别锁定顶抓，杯、瓶根据三维几何锁定顶抓或侧抓；随后进入 `OBSERVING` 并移动到对应观察位，再次检测和规划后才允许底盘或机械臂执行。

目标不在舒适抓取区且底盘辅助已启用时，状态机从 `PLANNING` 进入 `BASE_ALIGNING`，底盘动作完成后返回 `DETECTING`。运行阶段发生不可恢复错误时，状态机先进入 `RECOVERING`。单次任务回到 home 位；`--loop` 模式在夹爪未持物时回到当前策略对应的观察位，然后进入 `ERROR`。

## 1. 状态定义

| 状态 | 作用 |
|------|------|
| `IDLE` | 等待目标命令 |
| `OBSERVING` | 根据已锁定的抓取策略移动到顶抓或侧抓观察姿态 |
| `DETECTING` | 获取立体相机帧并检测目标 |
| `PLANNING` | 根据抓取策略执行二维定位或三维几何估计并检查工作空间；三维候选在本阶段完成 ik 筛选 |
| `BASE_ALIGNING` | 底盘短距离前进、后退或原地转向，动作完成后重新检测 |
| `APPROACHING` | 顶抓移动到预抓取位；侧抓移动到目标上方的安全预抓取位 |
| `GRASPING` | 张开夹爪，下降并进给到抓取位，闭合后检测是否夹住 |
| `LIFTING` | 顶抓向上抬升；侧抓先向上抬升再水平退出 |
| `PLACING` | 移动到放置位并释放 |
| `HOMING` | 放置完成后回到收尾姿态 |
| `RECOVERING` | 任务失败后回到当前运行模式对应的安全恢复姿态 |
| `DONE` | 任务完成 |
| `ERROR` | 任务失败 |

## 2. 执行动作

主循环每 50 ms 调用一次 `SpinOnce()`。机械臂运动、夹爪动作和底盘对齐通过后台 `std::future` 执行。状态机使用 `StartAction()` 启动动作，使用 `PollAction()` 轮询结果，再根据 `GraspResult` 切换状态。

首次规划只负责锁定抓取策略，不执行底盘对齐或抓取动作。普通目标按类别锁定顶抓，杯、瓶和显式侧抓请求使用三维几何。顶抓使用 `observe_joints`，侧抓使用 `side_ready_joints`。进入侧抓观察位时各关节协调运动，第四关节保持更快的前导进度，避免夹爪触地并推动底盘。观察动作完成后，pipeline 丢弃运动前数据并重新检测；后续规划只能使用已锁定策略的候选。

取消命令会停止当前任务。夹爪未持物时，机械臂回到观察姿态等待下一条命令；夹爪可能持物时，机械臂回到 home 位并退出程序。若机械臂或夹爪动作正在执行，状态机会等待当前后台动作返回后再执行恢复动作。

抓取成功后，普通指定目标模式在 `HOMING` 阶段回到 home 姿态并进入 `DONE`，随后退出程序；语音模式回到观察姿态并等待下一条抓取命令。`--loop` 模式在单轮放置完成后回到本轮抓取策略对应的观察位，再进入 `DONE` 并开始下一轮，不在轮次之间回到 home。下一轮未检测到目标时，pipeline 保持在 `DETECTING` 等待，不将其视为任务失败。退出循环时，程序完成当前动作并回到 home 后结束。

语音模式收到 `结束`、`待命`、`休息`、`回家`、`回 home`、`回初始`、`end` 或 `home` 后回到 home 姿态并退出程序。

抓取任务在观察、检测、规划、底盘对齐、接近、夹取、抬升、放置或收尾阶段失败时，状态机自动进入 `RECOVERING`，恢复动作不等待单步确认。

- 单次任务失败后，机械臂回到 home 位。
- `--loop` 模式在夹爪未持物时，单轮失败后回到当前抓取策略对应的观察位。下一轮保留已锁定的抓取策略并直接重新检测，不重复执行观察位运动。
- 夹爪闭合后发生抬升、放置或执行器错误时，系统按“可能持物”处理，强制回到 home 位并终止自动循环。确认并取下物体后再启动新任务。
- 恢复完成后，pipeline 记录原始失败原因并进入 `ERROR`；恢复动作失败时，最终结果同时记录原始错误和恢复错误。

初始化失败时执行器可能尚未可用，因此不会触发恢复动作。`plan-only` 模式不执行机械臂运动，也不会触发失败回 home。

## 3. 底盘闭环

进入 `BASE_ALIGNING` 前，机械臂已到达当前抓取策略对应的观察姿态。底盘执行一个短动作并刹车，经过程序内置的停车稳定时间后回到 `DETECTING`。pipeline 获取新帧、重新检测目标并重新规划，不复用移动前的目标坐标，也不会在底盘动作后切换顶抓和侧抓策略。

以下任一条件成立时，底盘对齐停止并进入 `ERROR`：

- 重新定位后的目标改善量没有超过视觉停滞门限。
- 对齐次数达到程序内置上限。
- 累计直行距离达到程序内置上限。
- 底盘驱动、串口或里程计反馈失败。

## 4. 状态输出

启用 `--status-stdout` 后，状态机会输出给语音桥使用的事件：

```text
VOICE_STATUS    state=OBSERVING;message=Moving to observe, target: banana;target=banana
VOICE_STATUS    state=DONE;message=Task completed!;reason=success
```

规划成功后保存 `grasp_*.json/png`。json 包含目标三维尺寸、抓取策略、候选位姿和几何耗时；任务结束后保存 `grasp_*_result.json`。

结构化运行日志还会输出 `[Stage N]`、`[Action]` 和 `PIPELINE SUMMARY`，用于查看每个阶段的耗时和最终结果。性能字段说明见[故障诊断](debugging.md#9-分析-pipeline-性能)。

## 5. 失败终态

| 类型 | 示例 |
|------|------|
| 目标不存在 | `Target not found: carrot; candidates: banana(0.89)` |
| 三维几何无效 | `3D grasp geometry failed: ...` |
| 候选不可执行 | `No safe 3D grasp candidate; ...` |
| 底盘对齐失败 | `Mobile base alignment failed: ...` |
| ik 失败 | `Pre-grasp move failed: IK failed` |
| 夹爪空抓 | `Grasp empty; max retries reached` |
| 夹爪超时 | `Gripper close timeout: timeout` |
| 执行器异常 | `motion or device error` |
