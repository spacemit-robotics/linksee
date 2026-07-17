# 故障诊断

按照“运行环境、立体相机、目标定位、抓取规划、执行机构”的顺序排查问题。前一阶段未通过时，不要继续测试后续真实动作。

## 1. 准备诊断环境

进入项目目录并加载运行环境：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
```

运行环境检查：

```bash
python3 scripts/check_runtime_env.py \
  --config config/grasp_pipeline.yaml
```

检查结果中的 `perceptive_grasp launcher` 对应结构化日志启动器，
`perceptive_grasp core` 对应实际运行抓取 pipeline 的 c++ 程序，二者都必须可执行。

硬件、权限、运行库和模型均可用时，最后一行显示：

```text
[SUMMARY] ready
```

出现 `[FAIL]` 时，先按相邻的 `[SUGGEST]` 修正配置或权限。设备连接和用户组设置见[硬件部署](hardware_setup.md)，软件和模型准备见[方案依赖](sdk_dependencies.md)。

排查第三方运行库、机械臂驱动、运动学或底盘驱动时，为主程序增加 `--debug`：

```bash
./build/perceptive_grasp \
  --debug \
  --config config/grasp_pipeline.yaml \
  --target banana
```

`--debug` 显示完整模块日志。默认模式只保留 pipeline 的结构化日志和模块错误。

## 2. 排查立体相机取图

使用 `debug_view` 验证当前 `camera.type` 选择的相机后端：

```bash
./build/debug_view \
  --config config/grasp_pipeline.yaml \
  --no-detect \
  --frames 3 \
  --output /tmp/debug_camera
```

命令成功后，`/tmp/debug_camera` 中生成：

- `frame_001_color.png`：彩色图像。
- `frame_001_depth.png`：深度伪彩色图。

依次确认：

1. 彩色图像清晰，目标没有被裁切或严重遮挡。
2. 目标表面的深度连续，黑色无效区域没有覆盖主要抓取位置。
3. 彩色图和深度图中的目标轮廓位于同一像素区域。

使用 realsense 后端时，取图失败通常由设备未连接、usb 带宽不足或设备权限引起。使用 spacemit_las2 后端时，重点检查环境检查输出中的视频格式、dma heap、模型和标定文件。两个后端的检查方法见[硬件部署](hardware_setup.md#5-验证立体相机)。

相机刚启动时画面不稳定，可使用 `--warmup <N>` 覆盖预热帧数。不要通过增加预热帧数掩盖持续存在的无效深度。

## 3. 排查目标检测和三维定位

使用 `debug_localize` 检查目标检测、深度反投影和手眼坐标变换：

```bash
./build/debug_localize \
  --config config/grasp_pipeline.yaml \
  --target banana \
  --frames 5 \
  --output /tmp/debug_localize
```

每帧生成一张标注图和一份定位结果，例如：

```text
/tmp/debug_localize/frame_001_localize.png
/tmp/debug_localize/frame_001_localize.txt
```

在定位结果中检查：

- `score`：检测置信度。
- `median_depth_mm_5x5`：目标中心附近的有效深度。
- `camera_point_m`：目标在相机坐标系中的三维位置。
- `base_point_m`：目标在机械臂基座坐标系中的三维位置。
- `in_workspace`：目标是否位于机械臂工作空间。

### 3.1 未检测到目标

先查看标注图，确认目标完整、光照稳定且类别名称与检测模型一致。然后检查：

- `detection.target_labels` 是否排除了目标类别。
- `detection.min_confidence` 是否过高。
- `detection.min_area` 是否大于目标实际面积。
- `detection.config_path` 指向的模型配置是否可读。

### 3.2 深度无效

定位结果出现 `invalid depth` 时，回到 `debug_view` 检查目标区域。调整相机距离、视角和光照，避免反光、透明表面和目标边缘落入抓取区域。

### 3.3 基座坐标明显错误

彩色图、深度图和 `camera_point_m` 正常，但 `base_point_m` 的方向或距离错误，通常表示手眼标定结果不适用于当前安装位置。确认相机和机械臂固定结构没有移动，再按[手眼标定](hand_eye_calibration.md)重新标定。

### 3.4 区分检测误差和手眼标定误差

先查看定位工具生成的标注图。如果检测框、分割区域和抓取像素都位于目标可夹取区域，而 `base_point_m` 与目标相对机械臂的实测位置不一致，问题来自深度或手眼标定，不应通过修改检测阈值补偿。

将同一目标放在工作区的左、中、右位置并分别定位：

- 图像中的抓取点随目标稳定移动，但基座坐标始终存在近似固定偏差时，重新标定或检查标定结果是否已写回配置。
- 偏差随目标位置明显变化时，检查立体相机标定、深度图与彩色图对齐以及相机固定结构。
- 基座坐标合理，但抓取点落在目标边缘或背景时，调整分割质量、`grasp_point_x_ratio` 或 `gripper_offset`。

## 4. 排查抓取规划和 ik

realsense 后端可使用 `debug_grasp` 执行不移动机械臂的规划检查：

```bash
./build/debug_grasp \
  --config config/grasp_pipeline.yaml \
  --target banana \
  --output /tmp/debug_grasp
```

重点检查：

- `pixel_grasp` 是否位于目标可夹取区域。
- `base_point_m` 是否与目标实际位置一致。
- `pre_grasp_m` 和 `grasp_m` 是否处于机械臂前方。
- `ik_pre_grasp` 和 `ik_grasp` 是否成功。
- `debug_grasp_annotated.png` 中的目标轴和夹爪方向是否合理。

`debug_grasp` 仅打开 realsense 后端。使用 spacemit_las2 时，先通过 `debug_localize` 验证定位，再使用主程序单步模式检查完整规划：

**单步模式仍会移动机械臂到观察姿态，并可能请求底盘对齐。只确认诊断所需的动作，出现预抓取提示时停止。**

```bash
./build/perceptive_grasp \
  --debug \
  --step \
  --config config/grasp_pipeline.yaml \
  --target banana
```

单步模式会在每次真实动作前等待确认。程序完成规划并提示“即将移动到预抓取位”时输入 `n`，可在不执行接近和抓取动作的情况下结束检查。

出现 `out of workspace` 时，先检查手眼标定和目标深度，再核对 `grasp.workspace`。出现 `IK failed` 时，检查 `manipulator.urdf_path`、机械臂当前姿态和目标位姿；不要通过扩大工作空间绕过机械结构限制。

## 5. 排查机械臂执行

### 5.1 读取当前关节角

使用配置文件中的稳定串口路径读取机械臂状态：

```bash
./build/read_joints \
  --device /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00
```

工具输出弧度、角度和可写入 yaml 的关节数组。读取失败时，检查串口是否被其他进程占用，并重新运行环境检查确认机械臂能够响应。

### 5.2 验证小范围运动

**该步骤会驱动真实机械臂。清理工作空间，并确保可以立即切断驱动电源。**

```bash
./build/debug_execute_safe \
  --config config/grasp_pipeline.yaml \
  --pose 0.28 0.00 0.10 \
  --yaw 90
```

首次测试保留交互确认，不要增加 `--yes`。工具先移动到观察姿态，再移动到指定位置；不会下探，也不会闭合夹爪。

如果 ik 成功但机械臂不动，使用主程序 `--debug` 日志检查串口、舵机响应和动作超时。机械臂运动方向或关节姿态异常时，停止运行并重新核对 urdf、关节零位和预定义姿态。

## 6. 排查底盘辅助对齐

### 6.1 底盘没有移动

主程序出现 `Mobile base alignment needed` 后，底盘应执行一次短距离移动或原地转向。没有动作时依次检查：

1. `mobile_base.enabled` 为 `true`。
2. `mobile_base.dev_path` 与环境检查识别出的底盘串口一致。
3. 完整日志中出现 `drv_uart_esp32` 初始化成功和底盘反馈信息。
4. 串口没有被其他进程占用，当前用户具有读写权限。

如果命令已经下发但轮速反馈接近 `0.000 rev/s`，底盘可能没有跨过低速启动区。当前发布配置使用 `angular_speed: 1.2`；降低该值可能导致底盘只有方向反馈而没有实际转动。

### 6.2 底盘越过目标或持续移动

底盘每次动作完成后都会停车，重新获取图像、检测目标并规划抓取。完整日志中应依次出现：

```text
[Pipeline] Mobile base alignment needed: ...
[Pipeline] Depth source=mask_foreground_q25 value=...mm samples=...
[Pipeline] Mobile base visual progress: ...m (required >= ...m)
```

重新定位后的目标框、深度和 `base_point_m` 应随底盘动作变化。目标进入 `target_x ± x_tolerance` 定义的前向舒适区，且横向偏移进入 `y_tolerance` 的稳定范围后，pipeline 才进入机械臂抓取阶段。

底盘越过目标时检查：

- 分割 mask 是否覆盖目标，而不是地面或背景。
- 目标表面的 spacemit_las2 深度是否有效。
- `calibration.T_base_camera` 是否与当前相机安装位置一致。
- `mobile_base.max_step_m` 是否过大。
- `mobile_base.linear_speed` 是否超过现场底盘可稳定控制的速度。

pipeline 内置视觉进展门限、最大对齐次数和累计直行距离限制。出现 `target distance did not improve` 或 `cumulative travel safety limit reached` 时，应修正深度和标定，不要绕过安全限制。

## 7. 排查语音交互

语音桥启动后应先收到完整的就绪事件，再进入监听状态：

```text
VOICE_STATUS	state=IDLE;message=Ready
[VoiceBridge] Listening: ...
```

出现 `perceptive_grasp not ready` 时，先处理同一终端中更早出现的相机、检测器、机械臂或底盘初始化错误。能够进入 `Listening` 但命令没有执行时，检查 asr 识别文本、`voice.trigger_words` 和 `voice.target_aliases`。

音频设备、文本命令验证和语音模型问题见[语音控制](voice_control.md)。

## 8. 检查任务调试产物

主程序默认将调试产物写入项目的 `debug_grasp_runs/` 目录。

规划成功后生成：

- `grasp_YYYYMMDD_HHMMSS_mmm.png`。
- `grasp_YYYYMMDD_HHMMSS_mmm.json`。

任务结束后生成：

- `grasp_YYYYMMDD_HHMMSS_mmm_result.json`。

失败时先查看结果文件中的以下字段：

- `terminal_state` 和 `message`：失败阶段和直接原因。
- `target_requested` 和 `target_detected`：请求目标与实际检测结果。
- `last_executor_action` 和 `last_executor_result`：最后一次执行动作及结果。
- `last_executor_detail`：驱动、ik 或动作超时详情。
- `gripper_check`：夹爪位置、负载、空夹爪全闭基线和持物判断结果。`phase=after_lift` 表示抬起后的二次确认结果。

结合对应的标注图确认目标框、分割 mask 和抓取像素是否落在同一目标上。

## 9. 分析 pipeline 性能

临时在 `config/grasp_pipeline.yaml` 中启用详细性能日志：

```yaml
logging:
  performance:
    enabled: true
```

运行一次目标抓取：

```bash
./build/perceptive_grasp \
  --config config/grasp_pipeline.yaml \
  --target banana
```

重点关注：

- `[Init] END module=camera_warmup`：立体相机首次取帧和深度推理耗时。
- `[Init] END module=detector_warmup`：yolo 首次推理耗时。
- `[Timing] stage=DETECTING`：单轮取帧和目标检测耗时。
- `[Timing] stage=PLANNING`：深度处理、坐标变换和抓取规划耗时。
- `[Timing] component=IK`：单次 ik 求解耗时。
- `[Action] START/END`：机械臂或底盘动作耗时和结果。
- `PIPELINE SUMMARY`：初始化、任务、端到端总耗时及各阶段结果。

`elapsed_ms` 表示墙钟耗时。`cpu_ms` 表示进程所有 linux 线程累计的 cpu 时间，多线程运行时可以大于 `elapsed_ms`。两者相除可估算该阶段平均占用的逻辑核数量，但不能直接表示 npu 或 x100 的硬件利用率。

k3 的 linux 逻辑核 `0-7` 为 a100 通用核，`8-15` 为 x100 ai 核。发布配置将 spacemit_las2 绑定到逻辑核 `8`，yolo 的 spacemit ep 使用其余 ai 核。性能结果以目标设备输出为准，完成分析后可关闭详细性能日志。
