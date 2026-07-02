# 抓取配置

主配置文件是 `config/grasp_pipeline.yaml`。部署时只修改本页列出的字段。

## 1. 抓取参数

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
```

- `approach_height`：预抓取点高于目标表面的高度。
- `grasp_depth`：最终抓取点相对目标表面的下探距离。
- `gripper_open`：抓取前夹爪张开程度。
- `gripper_effort`：夹爪闭合力度。
- `gripper_hold_load_threshold`：判断夹住物体的负载阈值。
- `gripper_timeout_ms`：夹爪动作超时时间。
- `gripper_offset`：3D 抓取点补偿。
- `grasp_point_x_ratio`：取深度时沿物体短轴偏移的比例。

## 2. 方向参数

```yaml
orientation:
  enabled: true
  aspect_ratio_threshold: 1.2
  camera_yaw_offset: 1.57
```

- `enabled`：根据物体分割结果估计夹爪 yaw。
- `aspect_ratio_threshold`：物体长宽比低于该值时不使用方向估计。
- `camera_yaw_offset`：相机安装 yaw 补偿。

执行器按下面关系计算腕部 yaw：

```text
command_yaw ~= joint0 + wrist_yaw_scale * joint5
```

## 3. 机械臂参数

```yaml
manipulator:
  uart_device: "/dev/ttyACM0"
  baudrate: 1000000
  home_joints: [...]
  observe_joints: [...]
  wrist_yaw_scale: 1.0
```

- `uart_device`：SO101 串口。
- `baudrate`：舵机总线波特率。
- `home_joints`：收到结束、待命、回家或 home 指令后的退出前姿态。
- `observe_joints`：检测前的观察姿态；抓取完成后也会停留在该姿态等待下一条命令。
- `wrist_yaw_scale`：腕部 yaw 映射比例。

## 4. 放置参数

```yaml
place:
  place_joints: [...]
  release_open: 0.5
```

- `place_joints`：放置姿态。
- `release_open`：释放物体时夹爪张开程度。

## 5. 等待参数

```yaml
timing:
  observe_settle_ms: 500
  gripper_open_wait_ms: 150
  gripper_close_wait_ms: 500
  release_wait_ms: 800
```

这些参数只控制固定等待时间；机械臂速度由 `manipulator.move_speed` 和
`manipulator.line_speed` 控制。
