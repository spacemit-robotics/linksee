# 抓取配置参考

本文说明三份发布配置的字段、默认值和约束：

- [`grasp_pipeline.yaml`](../config/grasp_pipeline.yaml)：Linksee 真机。
- [`grasp_pipeline_mujoco_ur5e.yaml`](../config/grasp_pipeline_mujoco_ur5e.yaml)：PC 本地 MuJoCo。
- [`grasp_pipeline_remote_mujoco_ur5e.yaml`](../config/grasp_pipeline_remote_mujoco_ur5e.yaml)：PC 或 K3 连接远程 MuJoCo 服务。

修改真机配置前，按[硬件部署](hardware_setup.md)确认设备路径和权限。

## 1. 配置 RGB-D 输入与目标检测

`camera.type` 选择运行时使用的 RGB-D 输入后端：

```yaml
camera:
  type: "spacemit_las2"  # realsense 或 spacemit_las2
```

CMake 会编译已发现的相机后端。程序只读取 `camera.type` 对应的配置块。

### 1.1 配置 `realsense`

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
- `motion_flush_frames`：机械臂或底盘动作后非阻塞清理的最大积压帧数。程序清空当前队列后只等待一帧新数据，不会固定执行该数量的完整图像处理。
- `align_depth`：将深度图对齐到彩色图，抓取定位时应保持开启。
- `depth_filter.spatial`：是否启用空间深度滤波。
- `depth_filter.temporal`：是否启用时间深度滤波。真机配置为 `false`，避免机器人动作后残留旧深度。
- `depth_filter.hole_filling`：是否填充深度空洞。真机配置为 `false`，避免将周围桌面深度扩散到弱纹理目标。直立目标内部缺少有效深度时，规划器使用周围桌面点云和目标轮廓重建几何。

### 1.2 配置 `spacemit_las2`

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
- `core_affinity`：深度推理绑定的 X100 AI 逻辑核，条目数必须与 `core_count` 一致，有效范围为 `8–15`。
- `depth.min_m`、`depth.max_m`：参与抓取定位的有效深度范围，单位米。

程序根据 `calib_path` 中的双目标定结果完成图像校正和三维反投影。

### 1.3 配置 MuJoCo 相机

```yaml
camera:
  type: "mujoco"
  mujoco:
    xml_path: "../simulation/mujoco/ur5e_scene.xml"
    camera_name: "realsense"
    width: 640
    height: 480
    depth:
      min_m: 0.05
      max_m: 3.0
```

- `mujoco.xml_path`：MuJoCo 场景 XML 路径，相对主配置文件所在目录解析。
- `mujoco.camera_name`：场景 XML 中用于 RGB-D 渲染的相机名称。
- `mujoco.width`、`mujoco.height`：渲染图像尺寸，单位像素。
- `mujoco.depth.min_m`、`mujoco.depth.max_m`：输出深度图的有效距离范围，单位米。

`camera.type: "remote_mujoco"` 从远程仿真服务获取相同格式的彩色图、深度图和相机内参。远程服务地址由 [`remote_mujoco`](#7-配置-mujoco-服务与远程连接) 配置块统一设置。

### 1.4 配置目标检测

```yaml
detection:
  config_path: yolov8_seg.yaml
  target_labels: []
  min_confidence: 0.10
  min_area: 1000
  stable_frames: 3
```

- `detection.config_path`：目标检测模型配置文件，相对 `config/` 目录解析。
- `detection.target_labels`：允许检测的 COCO 类别 ID；空列表表示不限制类别。
- `detection.min_confidence`：最低检测置信度。误检增多时调高，漏检增多时调低。
- `detection.min_area`：最小检测区域面积，用于过滤过小目标，单位平方像素。
- `detection.stable_frames`：目标检测框位置和面积连续稳定多少帧后进入抓取规划。目标仍在移动时，pipeline 保持在检测阶段；稳定后只执行一次三维几何估计和策略选择。

MuJoCo 配置还支持以下合成场景字段：

- `detection.label_remap`：检测模型类别名到仿真场景目标名的映射。键和值均不能为空。
- `detection.allow_color_only_fallback`：模型未检出目标时，是否允许从 `detection.simulation_color_targets` 生成候选掩码。该字段只能用于 `mujoco` 和 `remote_mujoco` 相机后端。
- `detection.refine_with_simulation_colors`：是否使用场景颜色范围细化模型输出的同名目标掩码。该字段只能用于 MuJoCo 相机后端。
- `detection.simulation_color_targets`：合成场景颜色目标列表。每项包含 `label`、`hsv_min`、`hsv_max`、`min_area`、`max_area` 和可选的 `score`。HSV 范围使用 OpenCV 取值：H 为 `0–179`，S 和 V 为 `0–255`；`max_area: 0` 表示不设面积上限；`score` 取值范围为 `(0, 1]`，默认值为 `0.99`。

## 2. 配置手眼标定结果

```yaml
calibration:
  realsense:
    T_base_camera:
      translation: [-0.020684, 0.043910, 0.300747]
      rotation: [-2.088074, 0.015059, -1.529363]
  spacemit_las2:
    T_base_camera:
      translation: [-0.019765, 0.048938, 0.312118]
      rotation: [-2.126969, -0.007183, -1.598329]
  mujoco:
    T_base_camera:
      translation: [-0.020684, 0.043910, 0.300747]
      rotation: [-2.088074, 0.015059, -1.529363]
  remote_mujoco:
    T_base_camera:
      translation: [-0.020684, 0.043910, 0.300747]
      rotation: [-2.088074, 0.015059, -1.529363]
```

- `realsense`：RealSense 相机安装位置对应的手眼标定结果。
- `spacemit_las2`：`spacemit_las2` 相机安装位置对应的手眼标定结果。
- `mujoco`：本地 MuJoCo 场景中虚拟相机到 UR5e 基座的变换。
- `remote_mujoco`：远程 MuJoCo 服务中虚拟相机到 UR5e 基座的变换，必须与服务端场景一致。
- `translation`：相机坐标系到机械臂基座坐标系的平移，单位米。
- `rotation`：相机坐标系到机械臂基座坐标系的旋转，单位弧度。

程序只加载 `camera.type` 对应的标定结果。真机标定通过[手眼标定](hand_eye_calibration.md)生成或更新；MuJoCo 标定必须与场景 XML 中的虚拟相机位姿一致。更换真机相机、机械臂安装位置或固定结构后，必须重新标定对应相机后端。

## 3. 配置抓取策略

```yaml
grasp:
  strategy: "auto"
  top:
    approach_height: 0.10
    grasp_depth: 0.015
    gripper_offset: 0.0
    grasp_point_x_ratio: 1.0
    position_source: "mask_depth"
    safe_mask_interior: false
    support_plane_occlusion_recovery: false
    support_plane_height_anchor: false
    minimum_grasp_height: 0.0
    verification_lift_m: 0.0
    gripper_open: 0.6
  side:
    min_height_m: 0.060
    min_height_width_ratio: 1.0
    approach_distance_m: 0.020
    entry_clearance_m: 0.030
    pregrasp_min_x_m: 0.270
    gripper_offset_m: 0.010
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
    planning_timeout_ms: 750
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

- `strategy`：抓取方向选择。`auto` 根据目标三维高度、宽度和候选可达性选择顶抓或侧抓，不读取目标类别；`top` 或 `side` 强制使用指定方向。

**顶抓参数：**

- `top.approach_height`：预抓取点高于目标表面的距离，单位米。
- `top.grasp_depth`：最终抓取点相对目标表面的下探距离，单位米。
- `top.gripper_offset`：沿夹爪开合方向施加的 tcp 机械补偿，单位米。
- `top.grasp_point_x_ratio`：二维抓取像素的无量纲偏移比例。程序根据分割结果计算目标短轴，并从目标中心向固定爪一侧移动抓取像素；`0` 表示目标中心，`0.5` 表示中心到短轴边缘的中点，`1` 表示短轴边缘。
- `top.position_source`：顶抓位置来源。`mask_depth` 根据分割掩码和对齐深度生成抓取位置；`projected_geometry_center` 使用三维几何中心的投影位置。
- `top.safe_mask_interior`：是否针对凹形掩码寻找局部安全截面。Linksee 真机配置为 `false`，MuJoCo 配置为 `true`。
- `top.support_plane_occlusion_recovery`：是否在掩码深度疑似被机械臂遮挡时改用支撑平面深度。Linksee 真机配置为 `false`，MuJoCo 配置为 `true`。
- `top.support_plane_height_anchor`：是否将最终 TCP 高度锚定到支撑平面。Linksee 真机配置为 `false`，MuJoCo 配置为 `true`。
- `grasp.top.projected_center_blend`：使用 `projected_geometry_center` 时，从候选抓取点向投影几何中心移动的比例，范围为 `[0, 1]`。细长目标会根据水平长宽比降低实际混合比例。
- `grasp.top.sparse_projected_center_blend`：点云不足并使用支撑平面恢复顶抓时，从掩码抓取点向支撑面投影中心移动的比例，范围为 `[0, 1]`。
- `top.minimum_grasp_height`：最终 TCP 相对支撑平面的最小高度，单位米，必须为非负值。
- `top.verification_lift_m`：闭合夹爪后用于持物确认的第一段抬升距离，单位米，范围为 `[0, top.approach_height]`；`0` 表示直接使用预抓取高度作为退出位。
- `top.gripper_open`：下探前的夹爪张开度，范围为 `[0, 1]`；`0` 表示全闭，`1` 表示全开。

**侧抓参数：**

- `side.min_height_m`：允许生成侧抓候选的最小目标高度，单位米。
- `side.min_height_width_ratio`：目标高度与桌面投影最大尺寸的最小比值。`auto` 仅在目标高度和该比值同时达到门限时首次选择侧抓，避免将横放的细长物体误判为直立物体；同一轮已经锁定侧抓后允许小幅测量波动，但仍执行夹爪宽度、间隙、工作空间和 IK 检查。
- `side.approach_distance_m`：预抓取位到抓取位沿接近方向的水平进给距离，单位米。该值表示平面位移向量的长度，不表示单独的 `x` 或 `y` 分量；目标接近机械臂中心线时，进给主要发生在前向 `x` 轴，横向 `y` 基本不变。
- `side.entry_clearance_m`：从观察位进入侧抓区域时，夹爪下缘高于目标最高点的安全距离，单位米。该参数只控制入场高度，不改变预抓取位到抓取位的水平进给距离。
- `side.pregrasp_min_x_m`：预抓取位相对机械臂基座的期望前向距离，单位米。底盘会调整目标位置，使闭合夹爪的侧向扫掠终点靠近该位置。
- `grasp.side.single_sided_gripper`：夹爪是否只有一侧手指主动移动。设为 `true` 时，规划器将目标半宽计入固定爪偏移；对称夹爪配置为 `false`。
- `side.gripper_offset_m`：固定爪位于目标外侧后的额外 tcp 机械补偿，单位米。侧抓规划器首先沿夹爪开合方向偏移目标半宽，再叠加该参数，确保单动爪在预抓取和水平接近过程中不会扫过目标中心。
- `grasp.side.visible_surface_offset_m`：从相机可见表面向估计体积中心修正侧抓点的距离，单位米，必须为非负值。
- `side.grasp_forward_offset_m`：预抓取位和抓取位沿接近方向共同施加的 tcp 机械补偿，单位米。该参数不会改变两者之间由 `approach_distance_m` 定义的进给距离。
- `side.grasp_height_ratio`：抓取中心相对目标高度的比例，范围为 `(0, 1)`。
- `side.initial_lift_m`：夹爪闭合后沿桌面法向抬升的距离，单位米。
- `side.lift_retreat_m`：抬升后沿接近方向反向退出的最小距离，单位米。

**公共夹爪参数：**

- `gripper_effort`：闭合抓取时请求的归一化控制量，范围为 `[0, 1]`；
  SO101 UART 夹爪将它作为闭合速度比例，并保留 `0.2` 的最低速度以避免失步。
- `gripper_hold_load_threshold`：持物判断使用的绝对负载下限。程序还会根据空夹爪负载分布计算动态阈值，并采用两者中的较大值。
- `gripper_empty_position_margin`：抓取后夹爪开度相对空夹爪位置分布中位数的最小差值。程序会同时考虑基线波动，避免测量噪声造成误判。
- `gripper_timeout_ms`：单次夹爪动作的超时时间，单位毫秒。

**公共几何参数：**

- `workspace.x_min`、`workspace.x_max`：机械臂基座坐标系下允许抓取的前向范围，单位米。
- `workspace.y_min`、`workspace.y_max`：机械臂基座坐标系下允许抓取的横向范围，单位米。
- `workspace.z_min`、`workspace.z_max`：机械臂基座坐标系下允许抓取的垂直范围，单位米。
- `geometry.sample_stride`：从目标掩码和周围桌面区域采样深度像素的间隔。
- `geometry.max_object_points`：单个目标参与几何估计的最大点数。
- `geometry.min_object_points`：允许生成候选所需的最少有效目标点数。
- `geometry.plane_distance_threshold_m`：桌面平面 ransac 的内点距离门限，单位米。
- `geometry.table_clearance_m`：目标点和夹爪相对桌面的最小安全间隙，单位米。
- `geometry.footprint_padding_m`：目标水平轮廓两侧增加的安全余量，单位米。
- `geometry.gripper_max_width_m`：允许候选使用的最大夹持宽度，单位米；应按实际夹爪开口标定。
- `geometry.planning_timeout_ms`：单个抓取候选执行 IK 和路径安全校验的最长时间，单位毫秒。
- `geometry.perception_budget_ms`：单帧采集、目标检测、三维几何和候选 IK 筛选的性能预算，单位毫秒。安全方案超过该预算时记录 `OVERRUN` 告警，但不会仅因耗时超限而终止；工作空间、IK 和路径安全检查仍须全部通过。

**顶抓方向参数：**

- `orientation.enabled`：是否根据二维分割掩码估计顶抓夹爪方向。Linksee 真机默认开启；顶抓路径在同一 IK 分支内局部修正 TCP，避免五自由度机械臂为追踪偏置 TCP 而突变到不可达的腕部分支。
- `orientation.aspect_ratio_threshold`：二维目标长宽比低于该值时不覆盖顶抓腕部方向。

`auto` 使用点云、桌面平面和 `geometry` 参数选择抓取策略。选中顶抓后，pipeline 使用二维分割掩码和对齐深度生成抓取位姿，并读取 `top` 和顶抓方向参数；选中侧抓后读取 `side` 参数。公共夹爪和工作空间参数对两种策略生效。

底盘将目标移动到舒适区后，pipeline 会重新检测并重新构建点云。新的抓取候选仍需通过工作空间和 IK 可达性检查。

## 4. 配置机械臂与放置姿态

```yaml
manipulator:
  uart_device: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00"
  urdf_path: "../urdf/so101.urdf"
  tip_link: "gripper_frame_link"
  legacy_top_ik: true
  move_speed: 1.0
  line_speed: 0.5
  home_joints: [1.546, -1.765, 1.400, 1.093, 0.186]
  observe_joints: [1.544, -0.067, 0.101, 1.382, 0.188]
  side_ready_joints: [1.550, 0.021, 1.400, -1.700, -0.036]
  joint_limits:
    - {joint: 0, min: -1.540, max: 1.630}
    - {joint: 1, min: -1.770, max: 1.680}
    - {joint: 2, min: -1.840, max: 1.430}
    - {joint: 3, min: -1.790, max: 1.670}
    - {joint: 4, min: -2.700, max: 2.800}

place:
  place_joints: [-1.500, -0.122, 0.101, 1.383, 0.330]
  release_open: 0.5
```

- `driver`：机械臂执行后端。Linksee 真机使用 `so101`，本地仿真使用 `mujoco_ur5e`，远程仿真使用 `remote_mujoco`。
- `uart_device`：机械臂舵机总线的稳定串口路径。
- `urdf_path`：机械臂 urdf 文件路径，相对主配置文件所在目录解析。
- `tip_link`：逆运动学、路径安全检查和到位验证使用的 TCP 坐标系。Linksee 真机配置使用 `gripper_frame_link`；仿真执行器使用各自模型中的末端站点。
- `legacy_top_ik`：选择 SO101 固定夹爪朝向的顶抓 IK 路径。Linksee 真机配置为 `true`；仿真配置为 `false`，由对应仿真执行器规划路径。
- `move_speed`：关节运动速度倍率，范围为 `[0, 1]`。
- `line_speed`：末端直线运动速度倍率，范围为 `[0, 1]`。
- `home_joints`：任务结束后的归位姿态，关节角单位弧度。
- `observe_joints`：检测目标前的通用观察姿态，关节角单位弧度。顶抓从该姿态进入预抓取位，不能与侧抓准备姿态混用。
- `side_ready_joints`：侧抓前的水平夹爪准备姿态，关节角单位弧度。侧抓路径从该姿态进入目标前方的预抓取位。
- `joint_limits`：当前机械臂舵机持久化限位对应的应用侧安全范围，关节编号从 `0` 开始。预定义姿态、直接关节动作和 IK 解均不得超出该范围。
- `place_joints`：抓取成功后的放置姿态，关节角单位弧度。Linksee 在底座转向放置侧时受腕部线束耦合影响，第 5 轴使用真机稳定角 `0.330 rad`，不复用观察位的 `0.188 rad`。
- `release_open`：放置目标时的夹爪张开度，范围为 `[0, 1]`。

### 4.1 配置 MuJoCo 执行器

`manipulator.driver: "mujoco_ur5e"` 使用以下配置块：

```yaml
manipulator:
  driver: "mujoco_ur5e"
  mujoco:
    xml_path: "../simulation/mujoco/ur5e_scene.xml"
    end_effector_site: "robot_gripper_pinch"
    gripper_actuator: "robot_gripper_fingers_actuator"
    robot_root_body: "robot_base"
    gripper_root_body: "robot_gripper_base_mount"
    joint_names:
      - "robot_shoulder_pan_joint"
      - "robot_shoulder_lift_joint"
      - "robot_elbow_joint"
      - "robot_wrist_1_joint"
      - "robot_wrist_2_joint"
      - "robot_wrist_3_joint"
    actuator_names:
      - "robot_shoulder_pan"
      - "robot_shoulder_lift"
      - "robot_elbow"
      - "robot_wrist_1"
      - "robot_wrist_2"
      - "robot_wrist_3"
    gripper_open_ctrl: 0.0
    gripper_close_ctrl: 255.0
    gravity_compensation: true
    arm_stiffness_scale: 12.0
    joint_tolerance_rad: 0.04
    ik_position_tolerance_m: 0.006
    cartesian_tracking_tolerance_m: 0.003
    ik_step_scale: 0.80
    ik_damping: 0.010
    ik_iterations: 300
    settle_steps: 500
    max_motion_steps: 4000
```

- `manipulator.mujoco.xml_path`：执行器加载的 MuJoCo 场景 XML 路径。
- `manipulator.mujoco.end_effector_site`：用于末端位姿和逆运动学的 site 名称。
- `manipulator.mujoco.gripper_actuator`：夹爪控制器名称。
- `manipulator.mujoco.robot_root_body`、`manipulator.mujoco.gripper_root_body`：机器人与夹爪碰撞树的根 body 名称。
- `manipulator.mujoco.joint_names`、`manipulator.mujoco.actuator_names`：机械臂关节与控制器名称列表；两个列表的顺序与 `home_joints`、`observe_joints` 和 `place_joints` 一致。
- `manipulator.mujoco.gripper_open_ctrl`、`manipulator.mujoco.gripper_close_ctrl`：夹爪完全张开和闭合时发送给 MuJoCo 控制器的控制值。
- `manipulator.mujoco.gravity_compensation`：是否向机械臂控制器加入重力补偿。
- `manipulator.mujoco.arm_stiffness_scale`：机械臂位置控制刚度倍率，必须为正数。
- `manipulator.mujoco.joint_tolerance_rad`：关节动作完成判定容差，单位弧度。
- `manipulator.mujoco.ik_position_tolerance_m`：逆运动学位置收敛容差，单位米。
- `manipulator.mujoco.cartesian_tracking_tolerance_m`：笛卡尔路径跟踪允许误差，单位米。
- `manipulator.mujoco.ik_step_scale`：每次逆运动学迭代应用的步长比例。
- `manipulator.mujoco.ik_damping`：阻尼最小二乘逆运动学的阻尼系数。
- `manipulator.mujoco.ik_iterations`：单个目标位姿允许的最大逆运动学迭代次数。
- `manipulator.mujoco.settle_steps`：动作或夹爪命令后用于稳定物理状态的仿真步数。
- `manipulator.mujoco.max_motion_steps`：单次机械臂动作允许的最大仿真步数。

机械臂进入观察位时会连续采集空夹爪位置和负载，建立本次任务的基线分布。夹爪闭合后会检查持物状态，抬起后再检查一次。pipeline 综合夹爪开度、负载和驱动状态判断是否持物；持续负载与明显高于空夹噪声的部分开度可以共同确认持物，但单独出现空夹硬限位负载时不会判为成功。抬升后的证据仍不确定时，pipeline 保留可能持物状态并执行安全放置恢复，不会按抓空再次接近目标。

侧抓时，机械臂先协调移动到 `side_ready_joints`，其中第四关节保持更快的前导进度。随后保持夹爪闭合并抬升到目标顶部以上，转动第一关节并移动到高位安全预抓取位。该阶段的末端路径保持在根据目标点云估计的物体顶部安全高度以上。

到达高位安全预抓取位后，夹爪先张开，再下降到目标前方并沿水平接近方向移动 `side.approach_distance_m`，最后闭合夹爪。高位入口、下降和水平进给路径会在机械臂动作前统一完成 IK 与路径安全检查；任一路径无解时不会开始靠近目标。

## 5. 配置底盘辅助对齐

```yaml
mobile_base:
  enabled: true
  dev_path: "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A99050376-if00"
  target_x: 0.275
  x_tolerance: 0.035
  y_tolerance: 0.15
  max_step_m: 0.12
  linear_speed: 0.20
  angular_speed: 1.2
  min_cmd_duration_ms: 350
  max_align_attempts: 6
  max_total_travel_m: 0.24
```

- `enabled`：启用底盘辅助对齐。
- `dev_path`：linksee 底盘的稳定串口路径。
- `target_x`：目标在机械臂基座坐标系下的期望前向距离，单位米；Linksee 真机配置为 `0.275 m`。
- `x_tolerance`：前后距离允许误差，单位米。Linksee 真机配置为 `0.035 m`。
- `y_tolerance`：目标相对机械臂基座中心线的左右偏移容差，单位米。程序在边界处使用 0.025 m 的稳定余量，避免定位波动触发底盘无法稳定执行的小幅转向。
- `max_step_m`：单次前进或后退的最大距离，单位米。
- `linear_speed`：前进或后退的命令速度，单位米每秒。
- `angular_speed`：原地转向的命令角速度，单位弧度每秒。
- `min_cmd_duration_ms`：单次底盘速度命令的最短持续时间，单位毫秒。该值应高于底盘电机启动死区；linksee 底盘推荐保持 `350`。
- `max_align_attempts`：单轮任务允许执行的最大底盘对齐次数。
- `max_total_travel_m`：单轮任务允许累计执行的最大直线移动距离，单位米。达到上限后，pipeline 停止底盘动作并进入机械臂安全校验或失败恢复。

前后对齐命令只修正到舒适区边界，不会直接按目标中心误差移动。每次底盘动作后必须由新相机帧确认目标距离产生足够变化；若目标仍需对齐但视觉位移不足，任务会立即安全失败，不再重复移动底盘或尝试过远的机械臂姿态。

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
    backend: "sensevoice"
    device: 1
    rate: 16000
    channels: 1
    channel_index: -1
    mixer_volume: 40
    sensevoice:
      model_dir: "~/.cache/models/asr/sensevoice"
      provider: "cpu"
      num_threads: 4
    qwen3_asr:
      endpoint: "http://127.0.0.1:8063/v1/chat/completions"
      model: "qwen3-asr"
      timeout_sec: 10
      context_max_terms: 0
      max_transcript_chars: 32
      auto_start: true
      model_dir: "~/.cache/models/asr/qwen3asr/qwen3-asr-0.6B-dynq-q40"
      server_threads: 4
      startup_timeout_sec: 90
    vad_trigger_threshold: 0.4
    vad_stop_threshold: 0.3
    vad_min_speech_duration_ms: 100
    vad_endpoint_hold_ms: 600
    vad_max_speech_duration_ms: 4000

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
- `echo_cancellation.mode`：回声处理模式。硬件已输出消回声录音时使用 `hardware_aec`；普通麦克风和扬声器组合使用 `webrtc_aec`；无法使用回声消除时可使用 `half_duplex`，TTS 播放期间会暂停识别。
- `echo_cancellation.delay_ms`：扬声器播放到回声进入麦克风的估计延迟，单位毫秒，仅用于 `webrtc_aec`。
- `echo_cancellation.noise_suppression`：是否对软件 AEC 输出启用低等级噪声抑制。
- `echo_cancellation.high_pass_filter`：是否对软件 AEC 输出启用高通滤波。

`webrtc_aec` 在 TTS 播放及短暂的回声尾音期间处理麦克风输入，播放结束后自动恢复原始麦克风音频。VAD 和 ASR 在两种路径之间保持连续运行。

- `asr.backend`：语音识别后端，可选 `sensevoice` 或 `qwen3_asr`。
- `voice.asr.device_name`：录音设备名称。配置后优先按名称解析当前设备编号；未配置时使用 `asr.device`。
- `asr.device`：本地录音设备编号，按硬件检查输出的采集设备编号填写。
- `asr.rate`、`asr.channels`：录音采样率和声道数。
- `asr.channel_index`：送入语音识别的物理声道编号。`-1` 表示对全部声道求平均；单声道输入只接受 `-1` 或 `0`。
- `asr.mixer_volume`：启动语音桥时设置的 ALSA 录音增益；`-1` 表示不修改系统 mixer。
- `asr.sensevoice.model_dir`：SenseVoice 模型目录。空值使用 SDK 默认模型路径。
- `asr.sensevoice.provider`：SenseVoice 推理 provider，真机配置为 `cpu`。
- `asr.sensevoice.num_threads`：SenseVoice 推理线程数。
- `asr.qwen3_asr.endpoint`：Qwen3-ASR 的 OpenAI 兼容 HTTP 接口地址，必须使用 `http` 或 `https`。
- `asr.qwen3_asr.model`：请求中的模型名称，不能为空。
- `asr.qwen3_asr.timeout_sec`：单次识别请求超时，单位秒，必须为正数。
- `asr.qwen3_asr.context_max_terms`：从命令词和目标别名生成识别上下文时保留的最大词条数；`0` 表示不自动生成上下文。
- `asr.qwen3_asr.max_transcript_chars`：接受的最大识别文本长度，必须为正整数；超长结果会被丢弃。
- `asr.qwen3_asr.auto_start`：本机端点不可用时，是否自动启动 `llama-server`。远程端点不会自动启动。
- `asr.qwen3_asr.model_dir`：自动启动服务时加载的 Qwen3-ASR 模型目录。
- `asr.qwen3_asr.server_threads`：自动启动服务时使用的推理线程数，必须为正整数。
- `asr.qwen3_asr.startup_timeout_sec`：等待自动启动服务就绪的最长时间，单位秒，必须为正数。
- `asr.vad_trigger_threshold`：开始收录语音的 vad 概率阈值。
- `asr.vad_stop_threshold`：结束收录语音的 vad 概率阈值。
- `asr.vad_min_speech_duration_ms`：允许识别的最短语音时长，单位毫秒。
- `asr.vad_endpoint_hold_ms`：VAD 低于停止阈值后继续等待同一语音段恢复的时间，单位毫秒。
- `asr.vad_max_speech_duration_ms`：单个语音段允许的最长时间，单位毫秒。达到上限后提交当前音频并重置 VAD。
- `tts.enabled`：是否启用状态语音播报。关闭后不启动 tts 引擎和播放设备。
- `tts.engine`：语音合成引擎及语言预设。
- `voice.tts.playback_device_name`：播放设备名称。配置后优先按名称解析当前设备编号；未配置时使用 `tts.playback_device`。
- `tts.playback_device`：本地播放设备编号，按硬件检查输出的播放设备编号填写。
- `tts.playback_rate`、`tts.channels`：播放采样率和声道数。
- `tts.speed`、`tts.volume`：合成语速和音量。
- `tts.mixer_volume`：启动语音桥时设置的 alsa pcm 音量；`-1` 表示不修改系统 mixer。
- `tts.speak_all_states`：是否播报全部 pipeline 状态；关闭时只播报关键状态。
- `target_aliases`：语音目标名称到 yolo 类别名的映射。

语音使用方法见[语音控制](voice_control.md)。

## 7. 配置 MuJoCo 服务与远程连接

PC 本地仿真服务读取 `mujoco_server`：

```yaml
mujoco_server:
  listen: "127.0.0.1"
  port: 9090
```

- `mujoco_server.listen`：仿真服务监听地址。仅本机使用时配置为 `127.0.0.1`；跨设备连接时配置为 `0.0.0.0` 或 PC 的可用监听地址。
- `mujoco_server.port`：仿真服务监听端口，范围为 `1–65535`。启动参数 `--listen` 和 `--port` 会覆盖配置值。

远程 pipeline 使用一个端点获取 RGB-D 帧并发送机械臂命令：

```yaml
remote_mujoco:
  host: "10.0.91.182"
  port: 9090
  frame_timeout_ms: 5000
  action_timeout_ms: 20000
```

- `remote_mujoco.host`：MuJoCo 服务所在 PC 的主机名或 IP 地址，不能为空。
- `remote_mujoco.port`：MuJoCo 服务端口，范围为 `1–65535`。
- `remote_mujoco.frame_timeout_ms`：等待远程 RGB-D 帧的超时时间，单位毫秒。
- `remote_mujoco.action_timeout_ms`：等待远程机械臂动作完成的超时时间，单位毫秒。

启动参数 `--remote-host` 和 `--remote-port` 会同时覆盖相机与机械臂使用的共享端点。部署步骤见[MuJoCo 仿真运行](simulation.md)。
