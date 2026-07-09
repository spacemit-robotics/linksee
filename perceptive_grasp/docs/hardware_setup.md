# 硬件环境

本文说明 Perceptive Grasp 在 Linksee 机器人上的硬件连接和运行前检查。

## 1. 硬件组成

| 模块 | 作用 | 检查重点 |
|------|------|----------|
| K3 控制板 | 运行抓取程序、视觉推理和语音桥 | 已加载 SDK 环境，当前用户有设备访问权限 |
| RealSense D435i | 提供 RGB-D 图像 | `/dev/video*` 可访问，能识别到 RealSense 设备 |
| SO101 机械臂 | 执行抓取、抬起、放置动作 | 舵机总线串口可访问，运行检查能探测到 SO101 响应 |
| SO101 夹爪 | 张开、闭合并读取负载 | 与机械臂同一舵机总线，运行检查和抓取程序初始化通过 |
| Linksee 底盘 | 目标超出舒适抓取区时做短距离辅助对齐 | 底盘 UART 串口可访问，`drv_uart_esp32` 已注册 |
| 麦克风和扬声器 | 语音命令输入和状态播报 | ALSA 采集、播放设备可访问 |

## 2. 连接要求

连接硬件前先确认机器人处于稳定支撑状态，机械臂和底盘周围留出安全空间。

- D435i 连接到 K3 USB 口，安装位置保持固定；调整相机位置后需要重新做手眼标定。
- SO101 舵机总线连接到 K3 USB 口，机械臂供电正常后再运行抓取程序。
- Linksee 底盘通过 UART 串口连接到 K3，底盘供电和急停状态应正常。
- 麦克风和扬声器连接到 K3，并能被 ALSA 识别。

## 3. 运行前诊断

每次更换硬件、重新插拔 USB 或首次部署时，先运行诊断脚本：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

诊断脚本会检查：

- 机械臂串口是否能访问，并确认 SO101 是否响应。
- 底盘串口是否能访问，并确认底盘驱动是否已注册。
- D435i 和 `/dev/video*` 权限是否正常。
- Pinocchio、`libmanipulator.so`、`libchassis.so` 等运行时依赖是否可用。
- 语音 Python 包、录音设备和播放设备是否可用。

检查通过时结尾应显示：

```text
[SUMMARY] ready
```

## 4. 设备识别

诊断脚本会列出串口角色和稳定设备路径，例如：

```text
[INFO] serial devices:
  /dev/ttyACM1: role=manipulator.uart_device; by-id=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A057974-if00
  /dev/ttyACM2: role=mobile_base.dev_path; by-id=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5958002008-if00
[OK] manipulator.uart_device role: SO101 responds on /dev/ttyACM1
[OK] mobile_base.dev_path role: chassis candidate /dev/ttyACM2
```

按诊断结果把 `by-id` 稳定路径写入 `config/grasp_pipeline.yaml`。具体字段说明见 [抓取配置](grasp_config.md)。

如果脚本输出 `[SUGGEST]`，说明当前配置与实际设备不一致，需要按建议更新配置后重新检查。

## 5. 权限检查

当前用户需要能访问串口、相机和音频设备。诊断脚本会检查 `dialout`、`video`、`audio` 用户组状态。

如果提示权限不足，按脚本建议加入用户组后，重新登录或执行：

```bash
exec su - "$USER"
```

然后重新加载环境并再次运行诊断脚本：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

## 6. 相机验证

诊断通过后，可以用调试工具采集几帧 RGB 和 Depth 图像：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp/build
source ~/spacemit_robot/build/envsetup.sh
./debug_view --no-detect --frames 3 --output /tmp/debug_camera
```

命令成功后，`/tmp/debug_camera` 下应生成 RGB 和 Depth 图片。若图像为空、深度缺失或目标区域没有深度，先检查 D435i 连接、视野遮挡、曝光和目标距离。

## 7. 底盘与机械臂安全检查

真实运行前确认：

- 机器人前后左右没有人员和障碍物。
- 机械臂运动范围内没有遮挡物。
- 底盘急停、供电和地面摩擦条件正常。
- 配置文件中的机械臂和底盘串口均使用诊断脚本确认过的 `/dev/serial/by-id/...` 路径。

抓取程序只有在目标超出机械臂舒适抓取区时才会执行底盘辅助对齐。底盘每次只做短距离动作，停车后重新检测目标并重新规划。

## 8. 音频设备检查

语音模式需要可用的录音和播放设备。诊断脚本会输出设备列表：

```text
[INFO] capture devices:
  [0] ...
[INFO] playback devices:
  [1] ...
```

根据实际使用的麦克风和扬声器，在 [抓取配置](grasp_config.md) 中设置语音设备编号。只运行 `./perceptive_grasp --target ...` 时不需要启用语音桥；使用 `scripts/local_voice_bridge.py` 时才需要完整音频链路。

## 9. 常见问题

- 找不到 D435i：检查 USB 连接和 `v4l2-ctl --list-devices`，确认系统能识别 RealSense 相关 video 节点。
- 串口角色错误：重新运行诊断脚本，按 `[SUGGEST]` 更新 `manipulator.uart_device` 和 `mobile_base.dev_path`。
- 权限不足：确认当前用户属于 `dialout`、`video`、`audio` 组，并重新登录或执行 `exec su - "$USER"`。
- 底盘不动作：先确认 `check_runtime_env.py` 显示底盘串口角色正确且 `drv_uart_esp32` 已注册，再查看抓取日志中的 `[MobileBase] Execute` 和 `Odom delta`。
- 相机有图但无法定位：确认 `camera.align_depth: true`，并检查目标区域是否存在有效深度。
