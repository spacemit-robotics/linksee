# 硬件与运行环境

本项目在 K3 上使用 SO101 串口、RealSense D435i、麦克风和扬声器。运行前统一使用 `scripts/check_runtime_env.py` 做检查和权限修复。

## 1. 连接硬件

- SO101/Feetech 总线连接到 K3 USB 口，配置为 `/dev/ttyACM0`。
- RealSense D435i 连接到 K3 USB 口。
- 麦克风和扬声器连接到 K3，并能被系统识别为 audio 设备。

配置文件中的串口保持为：

```yaml
manipulator:
  uart_device: "/dev/ttyACM0"
```

## 2. 安装系统工具

```bash
sudo apt install -y ros-humble-librealsense2 v4l-utils
```

## 3. 检查设备节点

- 检查 SO101 串口：

```bash
ls -l /dev/ttyACM*
```

结果中应包含 `/dev/ttyACM*`。

- 检查 USB 设备：

```bash
lsusb
```

结果中应包含 Intel RealSense D435i。

- 检查 video 节点：

```bash
ls -l /dev/video*
```

结果中应包含 `/dev/video0` 及其他 RealSense/USB 摄像头节点。

- 检查摄像头设备名称：

```bash
v4l2-ctl --list-devices
```

结果中应包含 RealSense 或 USB Camera 设备名称。

- 检查音频节点：

```bash
ls -l /dev/snd
```

结果中应包含：

- `/dev/snd/controlC*`
- `/dev/snd/pcm*`

## 4. 运行环境诊断

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

脚本会检查串口、相机、机械臂 IK 后端、语音 Python 包和音频设备。需要补权限或 Python 依赖时，脚本会弹出 `y/N` 确认并直接执行修复命令。

脚本加入 `dialout`、`video`、`audio` 用户组后，执行下面命令让当前终端重新加载用户组：

```bash
exec su - "$USER"
```

重新进入项目后再运行同一条诊断命令，最终结果应为：

```text
[OK] audio capture devices
[OK] audio playback devices
[SUMMARY] ready
```

## 5. 验证相机

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp/build
source ~/spacemit_robot/build/envsetup.sh
./debug_view --no-detect --frames 3 --output /tmp/debug_camera
```

命令成功后，`/tmp/debug_camera` 下应生成 RGB 和 Depth 图片。

## 6. 选择音频设备

运行环境诊断脚本会列出 SpaceMIT audio 可用设备编号：

```text
[INFO] capture devices:
  [0] ...
[INFO] playback devices:
  [0] ...
```

把选定编号写入 `config/grasp_pipeline.yaml`：

```yaml
voice:
  asr:
    device: 1
    rate: 16000
    channels: 1
  tts:
    playback_device: 1
    mixer_volume: 80
```

K3 上默认使用 USB Camera 的单声道麦克风输入。TTS 播报使用 USB Audio 输出，并在启动时把该设备的 ALSA `PCM` 音量设为 80%。
