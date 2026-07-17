# 硬件部署

本文说明 perceptive grasp 在 Linksee 机器人上的硬件连接和设备权限设置。

## 1. 硬件清单

| 硬件 | 作用 | 接口 |
|---|---|---|
| spacemit k3 | 运行感知、规划、控制和语音程序 | 主控 |
| 立体相机 | 输出彩色图像和深度信息 | usb uart |
| 机械臂与夹爪 | 执行抓取、放置和归位 | usb uart |
| 底盘 | 执行短距离辅助对齐 | usb uart |
| 麦克风和扬声器 | 接收命令和播报状态 | usb audio / 板载音频 |

立体相机可使用 realsense d435i 深度相机或 spacemit_las2 双目相机。两种后端使用同一套抓取流程，但设备节点、运行库和标定数据不同。

## 2. 连接硬件

连接设备前关闭机械臂和底盘动力，并确认机器人处于稳定支撑状态。

1. 将立体相机固定在机身上，使工作区位于彩色图像和深度视野内。
2. 将机械臂舵机总线连接到 k3 usb 口。
3. 将底盘连接到 k3 usb 口。
4. 将立体相机连接到 k3 usb 口。连接到 usb 3.0 及以上接口。
5. 连接麦克风和扬声器。

首次使用或调整相机位置、相机角度或机械臂安装位置后，先执行[手眼标定](hand_eye_calibration.md)。

## 3. 配置设备权限

运行用户需要访问串口、视频和音频设备：

```bash
sudo usermod -aG dialout,video,audio "$USER"
exec su - "$USER"
```

spacemit_las2 运行时还需要访问 `/dev/dma_heap/system`。使用 spacemit_las2 后端时创建 udev 规则：

```bash
echo 'SUBSYSTEM=="dma_heap", KERNEL=="system", GROUP="video", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-las2-dma-heap.rules
sudo udevadm control --reload-rules
sudo chgrp video /dev/dma_heap/system
sudo chmod 0660 /dev/dma_heap/system
```

## 4. 识别机械臂和底盘串口

运行环境检查：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

脚本会探测机械臂关节响应，并列出机械臂和底盘的稳定设备路径：

```text
[OK] manipulator.uart_device role: SO101 responds on /dev/ttyACM0
[SUGGEST] manipulator.uart_device: "/dev/serial/by-id/..."
[OK] mobile_base.dev_path role: chassis candidate /dev/ttyACM1
[SUGGEST] mobile_base.dev_path: "/dev/serial/by-id/..."
```

将建议路径写入 `config/grasp_pipeline.yaml`。优先使用 `/dev/serial/by-id/...`，不要使用可能随插拔顺序变化的 `/dev/ttyACM*`。字段说明见[抓取配置参考](grasp_config.md)。

## 5. 验证立体相机

### 5.1 验证 realsense d435i

确认系统能够发现相机：

```bash
source /opt/ros/humble/setup.sh
rs-enumerate-devices --version
```

构建应用后采集彩色图像和深度图：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp/build
source ~/spacemit_robot/build/envsetup.sh
./debug_view \
  --config ../config/grasp_pipeline.yaml \
  --no-detect \
  --frames 3 \
  --output /tmp/debug_camera
```

`/tmp/debug_camera` 中应生成非空的彩色图像和深度图。

### 5.2 验证 spacemit_las2 双目相机

确认相机设备和采集格式：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/v4l/by-id/<LAS2_VIDEO_DEVICE> --list-formats-ext
```

spacemit_las2 运行库需要 `YUYV 4000x1200@30`、可访问的 `/dev/dma_heap/system`、匹配的 onnx 模型和双目标定 json。环境检查应显示：

```text
[OK] LAS2 video device: ...
[OK] LAS2 YUYV capture format: YUYV 4000x1200@30 available
[OK] LAS2 DMA heap: /dev/dma_heap/system
[OK] LAS2 calibration fields: complete
```

构建应用后，可使用同一个 `debug_view` 命令采集 spacemit_las2 的彩色图像和深度图。工具根据 `camera.type` 自动选择后端。

## 6. 验证音频设备

列出 alsa 采集和播放设备：

```bash
arecord -l
aplay -l
```

环境检查会输出可供 `voice.asr.device` 和 `voice.tts.playback_device` 使用的设备编号。只运行命令行抓取时不需要音频设备。

## 7. 完成运行前检查

硬件和配置正确时，环境检查最后一行显示：

```text
[SUMMARY] ready
```

如果检查失败，先按输出中的 `[FAIL]` 和 `[SUGGEST]` 修正设备或配置，再运行抓取程序。常见故障见[故障诊断](debugging.md)。

真实运动前确认：

- 机械臂工作空间内没有人员和障碍物。
- 底盘前后左右留有移动和转向空间。
- 底盘急停、供电和地面摩擦条件正常。
- 立体相机、机械臂和底盘安装没有松动。
