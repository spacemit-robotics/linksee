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
    motion_flush_frames: 30
    align_depth: true
    depth_filter:
      spatial: true
      temporal: false
      hole_filling: false
```

- `width`、`height`：彩色图和深度图的采集尺寸，单位像素。
- `fps`：相机采集帧率。
- `motion_flush_frames`：机械臂或底盘动作后丢弃的积压帧数。
- `align_depth`：将深度图对齐到彩色图，抓取定位时应保持开启。
- `depth_filter`：深度滤波开关。时间滤波默认关闭，避免机器人动作后残留旧深度；空洞填充默认关闭，避免将周围地面深度扩散到弱纹理目标。杯、瓶缺少有效目标深度时，规划器使用周围地面点云和目标轮廓重建侧抓几何。

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
    translation: [0.005122, 0.056533, 0.310072]
    rotation: [-2.162887, -0.009940, -1.551754]
```

- `translation`：相机坐标系到机械臂基座坐标系的平移，单位米。
- `rotation`：相机坐标系到机械臂基座坐标系的旋转，单位弧度。

通过[手眼标定](hand_eye_calibration.md)生成该结果。更换相机、机械臂安装位置或固定结构后必须重新标定。

## 3. 配置抓取策略

```yaml
grasp:
  strategy: "auto"
  top:
    approach_height: 0.10
    grasp_depth: 0.015
    gripper_offset: 0.02
    grasp_point_x_ratio: 1.0
    gripper_open: 0.6
  side:
    min_height_m: 0.060
    approach_distance_m: 0.030
    pregrasp_min_x_m: 0.270
    gripper_offset_m: 0.005
    grasp_forward_offset_m: 0.020
    grasp_height_ratio: 0.60
    initial_lift_m: 0.050
    lift_retreat_m: 0.025
  gripper_effort: 0.8
  gripper_hold_load_threshold: 100.0
  gripper_empty_position_margin: 0.03
  gripper_timeout_ms: 3000
  geometry:
    sample_stride: 2
    max_object_points: 4000
    min_object_points: 80
    plane_distance_threshold_m: 0.008
    table_clearance_m: 0.005
    footprint_padding_m: 0.005
    gripper_max_width_m: 0.10
    planning_timeout_ms: 220
    perception_budget_ms: 500
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

**策略选择：**

- `strategy`：抓取方向选择。`auto` 保持普通目标的二维顶抓策略，并对杯、瓶估计三维几何后选择顶抓或侧抓；`top` 或 `side` 强制使用指定方向。

**顶抓参数：**

- `top.approach_height`：预抓取点高于目标表面的距离，单位米。
- `top.grasp_depth`：最终抓取点相对目标表面的下探距离，单位米。
- `top.gripper_offset`：沿夹爪开合方向施加的 tcp 机械补偿，单位米。
- `top.grasp_point_x_ratio`：二维抓取像素的无量纲偏移比例。程序根据分割结果计算目标短轴，并从目标中心向固定爪一侧移动抓取像素；`0` 表示目标中心，`0.5` 表示中心到短轴边缘的中点，`1` 表示短轴边缘。
- `top.gripper_open`：下探前的夹爪张开度，范围为 `[0, 1]`；`0` 表示全闭，`1` 表示全开。

**侧抓参数：**

- `side.min_height_m`：允许生成侧抓候选的最小目标高度，单位米。
- `side.approach_distance_m`：预抓取位到抓取位沿接近方向的水平进给距离，单位米。该值表示平面位移向量的长度，不表示单独的 `x` 或 `y` 分量；目标接近机械臂中心线时，进给主要发生在前向 `x` 轴，横向 `y` 基本不变。
- `side.pregrasp_min_x_m`：预抓取位相对机械臂基座的期望前向距离，单位米。底盘会调整目标位置，使闭合夹爪的侧向扫掠终点靠近该位置。
- `side.gripper_offset_m`：固定爪位于目标外侧后的额外 tcp 机械补偿，单位米。侧抓规划器首先沿夹爪开合方向偏移目标半宽，再叠加该参数，确保单动爪在预抓取和水平接近过程中不会扫过目标中心。
- `side.grasp_forward_offset_m`：预抓取位和抓取位沿接近方向共同施加的 tcp 机械补偿，单位米。该参数不会改变两者之间由 `approach_distance_m` 定义的进给距离。
- `side.grasp_height_ratio`：抓取中心相对目标高度的比例，范围为 `(0, 1)`。
- `side.initial_lift_m`：夹爪闭合后沿桌面法向抬升的距离，单位米。
- `side.lift_retreat_m`：抬升后沿接近方向反向退出的最小距离，单位米。

**公共夹爪参数：**

- `gripper_effort`：闭合抓取时请求的归一化力度，范围为 `[0, 1]`。
- `gripper_hold_load_threshold`：夹爪闭合后用于判断是否夹住目标的负载阈值。
- `gripper_empty_position_margin`：抓取后夹爪开度相对空夹爪全闭基线的最小差值。
- `gripper_timeout_ms`：单次夹爪动作的超时时间，单位毫秒。

**公共几何参数：**

- `workspace`：机械臂基座坐标系下允许抓取的三维范围，单位米。
- `geometry.sample_stride`：从目标掩码和周围桌面区域采样深度像素的间隔。
- `geometry.max_object_points`：单个目标参与几何估计的最大点数。
- `geometry.min_object_points`：允许生成候选所需的最少有效目标点数。
- `geometry.plane_distance_threshold_m`：桌面平面 ransac 的内点距离门限，单位米。
- `geometry.table_clearance_m`：目标点和夹爪相对桌面的最小安全间隙，单位米。
- `geometry.footprint_padding_m`：目标水平轮廓两侧增加的安全余量，单位米。
- `geometry.gripper_max_width_m`：允许候选使用的最大夹持宽度，单位米；应按实际夹爪开口标定。
- `geometry.planning_timeout_ms`：三维几何估计和候选 ik 筛选的总时限，单位毫秒。
- `geometry.perception_budget_ms`：单帧采集、目标检测、三维几何和候选 ik 筛选的总时限，单位毫秒；超时后不会执行机械臂动作。

**顶抓方向参数：**

- `orientation.enabled`：是否根据二维分割掩码估计顶抓夹爪方向。
- `orientation.aspect_ratio_threshold`：二维目标长宽比低于该值时不覆盖顶抓腕部方向。

普通目标的顶抓使用二维分割掩码和对齐深度定位，不读取 `geometry` 参数。杯、瓶以及显式配置为 `side` 的目标使用点云、桌面平面和 `geometry` 参数生成候选。选中顶抓后读取 `top` 和顶抓方向参数；选中侧抓后读取 `side` 参数。公共夹爪和工作空间参数对两种策略生效。

底盘将目标移动到舒适区后，pipeline 会重新检测并重新构建点云。新的抓取候选仍需通过工作空间和 ik 可达性检查。

## 4. 配置机械臂与放置姿态

```yaml
manipulator:
  uart_device: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00"
  urdf_path: "../urdf/so101.urdf"
  move_speed: 1.0
  line_speed: 0.5
  home_joints: [1.550, -1.620, 1.420, 1.147, 0.189]
  observe_joints: [1.550, 0.050, -0.217, 1.606, 0.015]
  side_ready_joints: [1.550, 0.021, 1.490, -1.700, -0.036]
  joint_limits:
    - {joint: 0, min: -1.540, max: 1.630}
    - {joint: 1, min: -1.690, max: 1.680}
    - {joint: 2, min: -1.840, max: 1.500}
    - {joint: 3, min: -1.790, max: 1.670}
    - {joint: 4, min: -2.700, max: 2.800}

place:
  place_joints: [-1.480, 0.087, -0.140, 1.389, 0.033]
  release_open: 0.5
```

- `uart_device`：机械臂舵机总线的稳定串口路径。
- `urdf_path`：机械臂 urdf 文件路径，相对主配置文件所在目录解析。
- `move_speed`：关节运动速度倍率，范围为 `[0, 1]`。
- `line_speed`：末端直线运动速度倍率，范围为 `[0, 1]`。
- `home_joints`：任务结束后的归位姿态，关节角单位弧度。
- `observe_joints`：检测目标前的通用观察姿态，关节角单位弧度。顶抓从该姿态进入预抓取位，不能与侧抓准备姿态混用。
- `side_ready_joints`：侧抓前的水平夹爪准备姿态，关节角单位弧度。侧抓路径从该姿态进入目标前方的预抓取位。
- `joint_limits`：当前机械臂舵机持久化限位对应的应用侧安全范围，关节编号从 `0` 开始。预定义姿态、直接关节动作和 ik 解均不得超出该范围。
- `place_joints`：抓取成功后的放置姿态，关节角单位弧度。
- `release_open`：放置目标时的夹爪张开度，范围为 `[0, 1]`。

夹爪闭合后会先检查持物状态，抬起到预抓取位后再检查一次。只有夹爪保持非零开度且负载持续超过阈值，pipeline 才进入放置阶段。

侧抓时，机械臂先协调移动到 `side_ready_joints`，其中第四关节保持更快的前导进度。随后保持夹爪闭合并抬升到目标顶部以上，转动第一关节并移动到高位安全预抓取位。该阶段的末端路径保持在根据目标点云估计的物体顶部安全高度以上。

到达高位安全预抓取位后，夹爪先张开，再下降到目标前方并沿水平接近方向移动 `side.approach_distance_m`，最后闭合夹爪。高位入口、下降和水平进给路径会在机械臂动作前统一完成 ik 与路径安全检查；任一路径无解时不会开始靠近目标。

## 5. 配置底盘辅助对齐

```yaml
mobile_base:
  enabled: true
  dev_path: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5958002008-if00"
  target_x: 0.275
  x_tolerance: 0.035
  y_tolerance: 0.15
  max_step_m: 0.12
  linear_speed: 0.20
  angular_speed: 1.2
  min_cmd_duration_ms: 200
```

- `enabled`：启用底盘辅助对齐。
- `dev_path`：linksee 底盘的稳定串口路径。
- `target_x`：目标在机械臂基座坐标系下的期望前向距离，单位米。
- `x_tolerance`：前后距离允许误差，单位米。linksee 底盘采用较宽容差，避免短距离动作越过目标值后反复前后修正。
- `y_tolerance`：目标相对机械臂基座中心线的左右偏移容差，单位米。程序在边界处使用 0.025 m 的稳定余量，避免定位波动触发底盘无法稳定执行的小幅转向。
- `max_step_m`：单次前进或后退的最大距离，单位米。
- `linear_speed`：前进或后退的命令速度，单位米每秒。
- `angular_speed`：原地转向的命令角速度，单位弧度每秒。
- `min_cmd_duration_ms`：单次底盘速度命令的最短持续时间，单位毫秒。

横向对齐目标为机械臂基座坐标系的 `y=0`：

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
