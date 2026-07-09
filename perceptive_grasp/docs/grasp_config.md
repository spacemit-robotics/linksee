# 抓取配置

主配置文件是 [config/grasp_pipeline.yaml](../config/grasp_pipeline.yaml)。部署时优先修改本页列出的字段，未说明的字段保持默认值。

推荐配置顺序：

1. 运行环境检查，确认机械臂和底盘串口。
2. 写入 `manipulator.uart_device` 和 `mobile_base.dev_path`。
3. 完成手眼标定，写入 `calibration.T_base_camera`。
4. 使用默认抓取、底盘和时序参数做低速测试。
5. 根据现场抓取效果小幅调整抓取高度、偏移、速度和等待时间。

## 运行前串口配置

运行检查脚本：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

脚本会列出所有 `/dev/ttyACM*` 设备对应的 `/dev/serial/by-id/*` 稳定路径、当前配置角色和串口类型。例如：

```text
[FAIL] manipulator.uart_device role: /dev/ttyACM0 is a camera serial
[SUGGEST] manipulator.uart_device: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00"
[SUGGEST] mobile_base.dev_path: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5958002008-if00"
```

把 `[SUGGEST]` 后面的机械臂和底盘串口路径写入配置：

```yaml
manipulator:
  uart_device: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00"

mobile_base:
  enabled: true
  driver: "drv_uart_esp32"
  dev_path: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5958002008-if00"
  baud: 115200
```

配置原则：

- `manipulator.uart_device` 填脚本探测到 SO101 关节响应的串口。
- 当前 Linksee 真实底盘默认使用 `drv_uart_esp32`，`mobile_base.dev_path` 填底盘串口。
- 优先使用稳定识别号 `/dev/serial/by-id/...`，不要依赖临时设备号 `/dev/ttyACM0/1/2`。

## 相机与检测

```yaml
camera:
  width: 640
  height: 480
  fps: 30
  align_depth: true
  depth_filter:
    spatial: true
    temporal: true
    hole_filling: true

detection:
  config_path: yolov8_seg.yaml
  target_labels: []
  min_confidence: 0.4
  min_area: 1000
```

- `width`、`height`、`fps`：RealSense D435i 采集分辨率和帧率。
- `align_depth`：将深度对齐到彩色图，抓取定位应保持开启。
- `depth_filter`：深度滤波开关，默认开启以减少空洞和抖动。
- `config_path`：检测模型配置文件，相对 `config/` 解析。
- `target_labels`：COCO 类别过滤。空列表表示检测全部类别；常用类别包括 `39=bottle`、`41=cup`、`46=banana`、`47=apple`、`49=orange`、`51=carrot`。
- `min_confidence`：检测置信度阈值。误检较多时调高，漏检较多时调低。
- `min_area`：最小分割区域面积，用于过滤过小目标。

## 手眼标定

```yaml
calibration:
  T_base_camera:
    translation: [0.0557, 0.0641, 0.3010]
    rotation: [-2.3633, -0.0301, -1.5373]
```

- `translation`：相机坐标系到机械臂基座坐标系的平移，单位米。
- `rotation`：相机坐标系到机械臂基座坐标系的旋转，按 `[roll, pitch, yaw]` 填写，单位弧度。

更换相机、调整相机安装位置、移动机械臂基座，或出现稳定的位置偏差时，按 [手眼标定](hand_eye_calibration.md) 重新标定后写回该字段。

## 抓取策略

```yaml
grasp:
  approach_height: 0.10
  grasp_depth: 0.01
  gripper_open: 0.5
  gripper_effort: 0.8
  gripper_hold_load_threshold: 100.0
  gripper_timeout_ms: 3000
  gripper_offset: 0.0
  grasp_point_x_ratio: 0.5
  workspace:
    x_min: 0.0
    x_max: 0.5
    y_min: -0.3
    y_max: 0.3
    z_min: 0.0
    z_max: 0.20
```

- `approach_height`：预抓取点高于目标表面的高度。
- `grasp_depth`：最终抓取点相对目标表面的下探距离。
- `gripper_open`：抓取前夹爪张开程度，范围 `[0, 1]`。
- `gripper_effort`：夹爪闭合力度，范围 `[0, 1]`。
- `gripper_hold_load_threshold`：夹爪负载超过该值且未完全闭合时，认为可能夹住目标。
- `gripper_timeout_ms`：夹爪动作超时时间。
- `gripper_offset`：沿夹爪固定爪方向的 3D 抓取点补偿。
- `grasp_point_x_ratio`：在图像分割区域内沿物体短轴偏移取深度的位置。
- `workspace`：机械臂基座坐标系下的抓取工作空间限制。

抓取位置偏高或夹不住时，优先小幅调整 `grasp_depth`。抓取点偏向活动爪一侧时，再调整 `gripper_offset` 或 `grasp_point_x_ratio`。

## 夹爪方向

```yaml
orientation:
  enabled: true
  aspect_ratio_threshold: 1.2
  camera_yaw_offset: 1.57
```

- `enabled`：是否根据分割结果估计夹爪 yaw。
- `aspect_ratio_threshold`：物体长宽比低于该值时，不强制使用方向估计。
- `camera_yaw_offset`：相机安装 yaw 补偿，仅在无法使用黄色检测线方向、回退到长轴方向时使用。

执行器按下面关系将目标 yaw 映射到腕部关节：

```text
command_yaw ~= joint0 + wrist_yaw_scale * joint5
```

如果现场观察到夹爪旋转方向相反，优先检查 `manipulator.wrist_yaw_scale` 的符号。

## 机械臂执行

```yaml
manipulator:
  driver: "so101"
  uart_device: "/dev/serial/by-id/<check_runtime_env 推荐的机械臂串口>"
  baudrate: 1000000
  urdf_path: "../urdf/so101.urdf"
  base_link: "base_link"
  tip_link: "gripper_frame_link"
  move_speed: 1
  line_speed: 0.5
  pose_position_tolerance: 0.03
  home_joints: [1.816, -1.850, 1.639, 1.147, 0.189]
  observe_joints: [1.759, 0.050, -0.217, 1.606, 0.015]
  ik_max_trials: 50
  wrist_yaw_scale: 1.0
```

- `driver`：机械臂驱动，SO101 使用 `so101`。
- `uart_device`：SO101 舵机总线串口，按运行前检查输出的 `[SUGGEST] manipulator.uart_device` 填写。
- `baudrate`：舵机总线波特率。
- `urdf_path`、`base_link`、`tip_link`：IK/FK 使用的 URDF 和链路名称。
- `move_speed`：关节运动速度倍率。
- `line_speed`：直线运动速度倍率。
- `pose_position_tolerance`：末端到位检查容差；超过该误差时停止流程，不继续闭合夹爪。
- `home_joints`：退出或回家时的关节姿态。
- `observe_joints`：检测前和语音模式等待下一条命令时的观察姿态。
- `ik_max_trials`：IK 多种子采样次数。
- `wrist_yaw_scale`：腕部 yaw 映射比例。

`joint_constraints` 和 `collision_avoidance` 用于限制 IK 解和避免机身碰撞。发布部署时建议保持默认值，只有在安装结构变化或安全区重新评估后再调整。

## 放置动作

```yaml
place:
  place_joints: [-1.636, 0.087, -0.140, 1.389, 0.033]
  release_open: 0.5
```

- `place_joints`：抓取成功后的放置姿态。
- `release_open`：放置时夹爪张开程度。

## 底盘辅助对齐

```yaml
mobile_base:
  enabled: true
  driver: "drv_uart_esp32"
  dev_path: "/dev/serial/by-id/<check_runtime_env 推荐的底盘串口>"
  baud: 115200
  ctrl_dev: "/dev/rpmsg_ctrl0"
  data_dev: "/dev/rpmsg0"
  service_name: "rpmsg:motor_ctrl"
  wheel_diameter: 0.067
  wheel_base: 0.183
  wheel_track: 0.0
  left_wheel_gain: 1.0
  max_speed: 0.3
  max_angular: 3.14
  target_x: 0.30
  x_tolerance: 0.05
  y_tolerance: 0.08
  max_step_m: 0.12
  linear_speed: 0.20
  angular_speed: 1.2
  yaw_gain: 8.0
  min_cmd_duration_ms: 200
  max_cmd_duration_ms: 2000
  settle_ms: 500
  max_align_attempts: 6
```

- `enabled`：是否启用底盘辅助对齐。
- `driver`：底盘驱动，当前 Linksee 真实底盘默认使用 `drv_uart_esp32`。
- `dev_path`、`baud`：UART 底盘串口和波特率。
- `ctrl_dev`、`data_dev`、`service_name`：RPMsg 兼容字段，仅在 `driver` 改为 `drv_rpmsg_esos` 时使用。
- `wheel_diameter`、`wheel_base`、`wheel_track`、`left_wheel_gain`：底盘运动学参数。
- `max_speed`、`max_angular`：底盘控制库的速度上限。
- `target_x`：目标在机械臂基座坐标系下的舒适抓取距离。
- `x_tolerance`：前后距离允许误差，超出后底盘短距离前进或后退。
- `y_tolerance`：左右偏移允许误差，超出后底盘原地转向。
- `max_step_m`：单次底盘前后移动的最大距离。
- `linear_speed`、`angular_speed`：底盘短距离调整速度。`linear_speed` 默认与底盘 UART 测试程序保持一致；`angular_speed` 需要高于 UART 差速底盘的低速启动死区，默认值会让原地转向轮速约为 `0.52 rev/s`。
- `yaw_gain`：左右偏移转为转向量的比例。UART 底盘短时命令响应偏慢时，可适当增大该值以增加单次转向幅度。
- `min_cmd_duration_ms`、`max_cmd_duration_ms`：单次底盘动作持续时间限制。
- `settle_ms`：底盘动作结束后的稳定等待时间。
- `max_align_attempts`：单次抓取最多允许底盘调整次数。目标在画面侧边时需要多次“移动-检测-重规划”，默认值设为 6，避免过早让机械臂去够偏侧目标。

底盘每次只执行一个短动作，动作结束后重新检测目标并重新规划抓取。首次启用时确保机器人周围有安全空间。

## 时序参数

```yaml
timing:
  observe_settle_ms: 500
  observe_gripper_close_wait_ms: 100
  pre_grasp_settle_ms: 150
  gripper_open_wait_ms: 150
  grasp_settle_ms: 100
  gripper_close_wait_ms: 500
  grasp_check_count: 5
  grasp_check_interval_ms: 50
  place_settle_ms: 100
  release_wait_ms: 800
  home_gripper_close_wait_ms: 100
```

这些参数只控制阶段间固定等待时间；机械臂运动速度由 `manipulator.move_speed` 和 `manipulator.line_speed` 控制。相机画面不稳定时可适当增加 `observe_settle_ms`，夹爪闭合后状态判断不稳定时可增加 `gripper_close_wait_ms` 或 `grasp_check_count`。

## 语音配置

```yaml
voice:
  trigger_words: ["抓", "拿", "pick", "grab"]
  asr:
    device: 1
    rate: 16000
    channels: 1
  tts:
    playback_device: 1
    mixer_volume: 80
  cancel_words: ["停止", "停", "取消", "别抓", "不要抓", "stop", "cancel"]
  target_aliases:
    香蕉: "banana"
    苹果: "apple"
```

- `trigger_words`：触发抓取的关键词。
- `asr.device`：本地 ASR 输入设备编号。
- `asr.rate`、`asr.channels`：录音采样率和声道数。
- `tts.playback_device`：TTS 播放设备编号。
- `tts.mixer_volume`：ALSA PCM 播放音量，`-1` 表示不修改系统 mixer。
- `cancel_words`：停止或取消当前任务的关键词。
- `target_aliases`：中文目标名到检测类别名的映射，类别名需要与 YOLO/COCO label 一致。

是否进入语音模式由启动命令决定。`local_voice_bridge.py` 会自动以 `--voice-stdin --status-stdout` 启动抓取进程。

## 调试与日志

```yaml
debug:
  save_grasp_debug: true
  output_dir: "../debug_grasp_runs"

logging:
  performance:
    enabled: false
```

- `save_grasp_debug`：保存每次规划成功后的标注图和 JSON，便于复盘抓取失败原因。
- `output_dir`：调试输出目录，相对配置文件所在目录解析。
- `logging.performance.enabled`：是否打印检测推理、检测阶段和 IK 求解耗时。

发布部署建议保留 `save_grasp_debug: true`。现场稳定后，如需减少磁盘写入，可关闭该项。
