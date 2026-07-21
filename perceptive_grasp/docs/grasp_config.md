# 抓取配置参考

主配置文件为 [`config/grasp_pipeline.yaml`](../config/grasp_pipeline.yaml)。修改配置前，按[硬件部署](hardware_setup.md)确认设备路径和权限。

## 1. 配置立体相机与目标检测

`camera.type` 选择运行时使用的立体相机后端：

```yaml
camera:
  type: "spacemit_las2"  # realsense 或 spacemit_las2
```

cmake 会编译已发现的相机后端。程序只读取 `camera.type` 对应的配置块。

### 1.1 配置 realsense

```yaml
camera:
  type: "realsense"
  realsense:
    width: 640
    height: 480
    fps: 30
    motion_flush_frames: 16
    align_depth: true
    depth_filter:
      spatial: true
      temporal: true
      hole_filling: true
```

- `width`、`height`：彩色图和深度图的采集尺寸，单位像素。
- `fps`：相机采集帧率。
- `motion_flush_frames`：机械臂或底盘动作后丢弃的积压帧数。
- `align_depth`：将深度图对齐到彩色图，抓取定位时应保持开启。
- `depth_filter`：空间、时间和空洞填充滤波开关。

### 1.2 配置 spacemit_las2

```yaml
camera:
  spacemit_las2:
    video_device: "/dev/v4l/by-id/usb-DECXIN_DECXIN_Camera_01.00.00-video-index0"
    model_path: "~/las2_runtime/models/LAS2_M_256x320.fp16.iofp32.corr_func_nhwc.gelu.onnx"
    calib_path: "~/las2_runtime/config/matlab_stereo_opencv.json"
    core_count: 1
    core_affinity: "8"
    depth:
      min_m: 0.05
      max_m: 2.0
```

- `video_device`：双目相机视频设备，使用 `/dev/v4l/by-id/` 稳定路径。
- `model_path`：spacemit_las2 onnx 模型路径，支持以 `~/` 表示当前用户主目录。
- `calib_path`：双目标定 json 路径，支持以 `~/` 表示当前用户主目录。
- `core_count`：spacemit_las2 深度推理会话数。
- `core_affinity`：深度推理绑定的 x100 ai 逻辑核，条目数必须与 `core_count` 一致，有效范围为 `8–15`。
- `depth.min_m`、`depth.max_m`：参与抓取定位的有效深度范围，单位米。

程序根据 `calib_path` 中的双目标定结果完成图像校正和三维反投影。

### 1.3 配置目标检测

```yaml
detection:
  config_path: yolov8_seg.yaml
  target_labels: []
  min_confidence: 0.4
  min_area: 1000
  stable_frames: 1
```

- `config_path`：目标检测模型配置文件，相对 `config/` 目录解析。
- `target_labels`：允许检测的 coco 类别 id；空列表表示不限制类别。
- `min_confidence`：最低检测置信度。误检增多时调高，漏检增多时调低。
- `min_area`：最小检测区域面积，用于过滤过小目标，单位平方像素。
- `stable_frames`：连续检测到目标多少帧后进入抓取规划。

## 2. 配置手眼标定结果

```yaml
calibration:
  T_base_camera:
    translation: [-0.019765, 0.048938, 0.312118]
    rotation: [-2.126969, -0.007183, -1.598329]
```

- `translation`：相机坐标系到机械臂基座坐标系的平移，单位米。
- `rotation`：相机坐标系到机械臂基座坐标系的旋转，单位弧度。

通过[手眼标定](hand_eye_calibration.md)生成该结果。更换相机、机械臂安装位置或固定结构后必须重新标定。

## 3. 配置抓取策略

```yaml
grasp:
  approach_height: 0.10
  grasp_depth: 0.01
  gripper_offset: 0.005
  grasp_point_x_ratio: 1
  gripper_open: 0.5
  gripper_effort: 0.8
  gripper_hold_load_threshold: 100.0
  gripper_empty_position_margin: 0.03
  gripper_timeout_ms: 3000
  workspace:
    x_min: 0.0
    x_max: 0.5
    y_min: -0.3
    y_max: 0.3
    z_min: 0.0
    z_max: 0.20

orientation:
  enabled: true
  aspect_ratio_threshold: 1.2
```

- `approach_height`：预抓取点高于目标表面的距离，单位米。
- `grasp_depth`：最终抓取点相对目标表面的下探距离，单位米。
- `grasp_point_x_ratio`：二维抓取像素的无量纲偏移比例。程序根据分割结果计算目标短轴，并从目标中心向固定爪一侧移动抓取像素；`0` 表示目标中心，`0.5` 表示中心到短轴边缘的中点，`1` 表示短轴边缘。
- `gripper_offset`：三维工具补偿，单位米。程序完成深度反投影和手眼坐标变换后，根据抓取方向平移预抓取位和抓取位，用于补偿夹爪 tcp 与固定爪接触位置之间的机械偏差；该参数不会改变调试图中的抓取像素。
- `gripper_open`：下探前的夹爪张开度，范围为 `[0, 1]`；`0` 表示全闭，`1` 表示全开。
- `gripper_effort`：闭合抓取时请求的归一化力度，范围为 `[0, 1]`。
- `gripper_hold_load_threshold`：夹爪闭合后用于判断是否夹住目标的负载阈值。
- `gripper_empty_position_margin`：抓取后相对空夹爪全闭基线必须保留的最小开度。
- `gripper_timeout_ms`：单次夹爪动作的超时时间，单位毫秒。
- `workspace`：机械臂基座坐标系下允许抓取的三维范围，单位米。
- `orientation.enabled`：根据目标分割结果估计夹爪方向。
- `orientation.aspect_ratio_threshold`：目标长宽比达到该阈值时才估计夹爪方向。

两个偏移按以下顺序叠加：

```text
二维抓取像素偏移 = 目标短轴半长 × grasp_point_x_ratio
三维位姿偏移 dx = -gripper_offset × sin(grasp_yaw)
三维位姿偏移 dy =  gripper_offset × cos(grasp_yaw)
```

- `grasp_point_x_ratio` 作用于目标图像，用于决定从目标的哪个位置计算三维坐标。目标长宽比低于 `orientation.aspect_ratio_threshold` 或短轴长度不足 10 px 时，程序会改用目标中心，此时该参数不产生偏移。
- `gripper_offset` 作用于机械臂基座坐标系。值为 `0` 时不进行三维补偿；值大于 `0` 时向程序选定的固定爪一侧偏移。

底盘将目标移动到舒适区后，pipeline 仍会执行工作空间和 ik 可达性检查。

## 4. 配置机械臂与放置姿态

```yaml
manipulator:
  uart_device: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00"
  urdf_path: "../urdf/so101.urdf"
  move_speed: 1.0
  line_speed: 0.5
  home_joints: [1.816, -1.850, 1.639, 1.147, 0.189]
  observe_joints: [1.759, 0.050, -0.217, 1.606, 0.015]

place:
  place_joints: [-1.636, 0.087, -0.140, 1.389, 0.033]
  release_open: 0.5
```

- `uart_device`：机械臂舵机总线的稳定串口路径。
- `urdf_path`：机械臂 urdf 文件路径，相对主配置文件所在目录解析。
- `move_speed`：关节运动速度倍率，范围为 `[0, 1]`。
- `line_speed`：末端直线运动速度倍率，范围为 `[0, 1]`。
- `home_joints`：任务结束后的归位姿态，关节角单位弧度。
- `observe_joints`：检测目标前的观察姿态，关节角单位弧度。
- `place_joints`：抓取成功后的放置姿态，关节角单位弧度。
- `release_open`：放置目标时的夹爪张开度，范围为 `[0, 1]`。

夹爪闭合后会先检查持物状态，抬起到预抓取位后再检查一次。只有夹爪保持非零开度且负载持续超过阈值，pipeline 才进入放置阶段。

## 5. 配置底盘辅助对齐

```yaml
mobile_base:
  enabled: true
  dev_path: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5958002008-if00"
  target_x: 0.275
  x_tolerance: 0.045
  y_tolerance: 0.15
  max_step_m: 0.12
  linear_speed: 0.20
  angular_speed: 1.2
```

- `enabled`：启用底盘辅助对齐。
- `dev_path`：linksee 底盘的稳定串口路径。
- `target_x`：目标在机械臂基座坐标系下的期望前向距离，单位米。
- `x_tolerance`：前后距离允许误差，单位米。
- `y_tolerance`：目标相对机械臂基座中心线的左右偏移容差，单位米。程序在边界处保留 0.025 m 的稳定余量，避免定位波动触发底盘无法稳定执行的小幅转向。
- `max_step_m`：单次前进或后退的最大距离，单位米。
- `linear_speed`：前进或后退的命令速度，单位米每秒。
- `angular_speed`：原地转向的命令角速度，单位弧度每秒。

当前实现将横向目标固定为机械臂基座坐标系的 `y=0`：

```text
x_error = target.x - target_x
y_error = target.y
```

底盘每次动作结束后停车，重新获取图像、检测目标并计算三维坐标。前后距离和带稳定余量的左右偏移同时满足约束后，pipeline 才进入机械臂抓取阶段。

## 6. 配置语音交互

```yaml
voice:
  trigger_words: ["抓", "拿", "pick", "grab"]
  cancel_words: ["停止", "停", "取消", "别抓", "不要抓", "stop", "cancel"]
  home_words: ["结束", "待命", "休息", "回家", "回home", "回到home", "回初始", "回到初始", "end", "home"]
  split_command_timeout_ms: 5000

  echo_cancellation:
    mode: "webrtc_aec"
    delay_ms: 50
    noise_suppression: true
    high_pass_filter: true

  asr:
    device: 1
    rate: 16000
    channels: 1
    vad_trigger_threshold: 0.4
    vad_stop_threshold: 0.3
    vad_min_speech_duration_ms: 100

  tts:
    enabled: true
    engine: "matcha:zh"
    playback_device: 1
    playback_rate: 48000
    channels: 1
    speed: 1.0
    volume: 80
    mixer_volume: 80
    speak_all_states: false

  target_aliases:
    香蕉: "banana"
    苹果: "apple"
    胡萝卜: "carrot"
    瓶子: "bottle"
    杯子: "cup"
```

- `trigger_words`：触发抓取命令的关键词。
- `cancel_words`：停止当前任务并返回观察状态的关键词。
- `home_words`：机械臂归位并退出程序的关键词。
- `split_command_timeout_ms`：分段语音命令等待目标名称的超时时间，单位毫秒。
- `echo_cancellation.mode`：回声处理模式。硬件已输出消回声录音时使用 `hardware_aec`；普通麦克风和扬声器组合使用 `webrtc_aec`；无法使用回声消除时可使用 `half_duplex`，tts 播放期间会暂停识别。
- `echo_cancellation.delay_ms`：扬声器播放到回声进入麦克风的估计延迟，单位毫秒，仅用于 `webrtc_aec`。
- `echo_cancellation.noise_suppression`：是否对软件 aec 输出启用低等级噪声抑制。
- `echo_cancellation.high_pass_filter`：是否对软件 aec 输出启用高通滤波。

`webrtc_aec` 在 tts 播放及短暂的回声尾音期间处理麦克风输入，播放结束后自动恢复原始麦克风音频。vad 和 asr 在两种路径之间保持连续运行。

- `asr.device`：本地录音设备编号，按硬件检查输出的采集设备编号填写。
- `asr.rate`、`asr.channels`：录音采样率和声道数。
- `asr.vad_trigger_threshold`：开始收录语音的 vad 概率阈值。
- `asr.vad_stop_threshold`：结束收录语音的 vad 概率阈值。
- `asr.vad_min_speech_duration_ms`：允许识别的最短语音时长，单位毫秒。
- `tts.enabled`：是否启用状态语音播报。关闭后不启动 tts 引擎和播放设备。
- `tts.engine`：语音合成引擎及语言预设。
- `tts.playback_device`：本地播放设备编号，按硬件检查输出的播放设备编号填写。
- `tts.playback_rate`、`tts.channels`：播放采样率和声道数。
- `tts.speed`、`tts.volume`：合成语速和音量。
- `tts.mixer_volume`：启动语音桥时设置的 alsa pcm 音量；`-1` 表示不修改系统 mixer。
- `tts.speak_all_states`：是否播报全部 pipeline 状态；关闭时只播报关键状态。
- `target_aliases`：语音目标名称到 yolo 类别名的映射。

语音使用方法见[语音控制](voice_control.md)。
