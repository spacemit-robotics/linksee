# MuJoCo 仿真运行

本方案将仿真环境和抓取 pipeline 分为独立进程：

- PC 运行 MuJoCo 物理仿真、RGB-D 渲染和第三视角窗口。
- 抓取 pipeline 可以运行在同一台 PC，也可以运行在 K3。
- 两个进程通过 `remote_mujoco` 协议传输彩色图、深度图、相机内参和机械臂命令。

仿真和真机均使用 `perceptive_grasp` 启动器。状态机、目标检测、三维估计、抓取规划和语音状态协议保持一致，配置文件只切换相机与机械臂后端。

## 1. 功能范围

仿真场景提供以下能力：

- 使用 MuJoCo Menagerie 的 UR5e 和 Robotiq 2F-85 MJCF、外观网格及简化碰撞体。
- 苹果和香蕉使用 stl 外观网格与表面纹理，简化碰撞体只参与物理接触，不参与渲染。
- 使用固定虚拟 RGB-D 相机输出像素对齐的彩色图像、深度信息和相机内参。相机位姿取自 Linksee 真机手眼标定结果，以斜向下视角覆盖工作区。
- 复用真机的 YOLOv8-Seg 模型、检测结果格式、目标点云、桌面平面、顶抓和侧抓规划。K3 使用 `vision_service`，PC 使用 OpenCV DNN 加载同一类 ONNX 模型。
- 低矮目标的有效点云不足时，使用分割掩码深度和估计得到的桌面平面生成顶抓位姿，并继续执行 TCP 桌面净空、IK 和完整路径检查。
- 使用 MuJoCo 执行关节驱动、接触求解、重力、摩擦和物体运动。
- 检查机械臂与桌面、机械臂自碰撞以及完整抓取和放置路径。
- 仅在两个指垫同时接触目标后建立夹持约束；未接触目标时不允许物体随夹爪移动。
- 抓取后由机械臂将物体搬运到桌面红色区域，释放后检查目标是否位于区域内。
- 在互不重叠的绿色取物区和红色放置区之间完成搬运。

仿真配置关闭底盘对齐。加宽的绿色取物区位于机械臂前方，红色放置区位于机械臂右侧，两者互不重叠。`apple`、`banana`、`cube` 和 `cup` 根据旋转后的占用宽度动态排成一行，避免目标在相机视角中相互遮挡。每次重置场景时，程序都会随机调整排列顺序、位置和水平摆放角度。前三个目标执行顶抓，`cup` 执行侧抓。

语音交互运行在抓取 pipeline 所在设备。PC/K3 跨机部署时，麦克风、扬声器、VAD、ASR、AEC 和 TTS 均运行在 K3，PC 仿真服务不处理音频。

检测始终先执行 YOLOv8-Seg，并保留模型输出的类别和置信度。由于合成纹理与 COCO 训练域存在差异，两份仿真配置使用场景中已知的实例颜色细化同名目标的分割掩码；模型未检出场景目标时，同一颜色配置可生成候选掩码。颜色分割仅用于仿真相机后端；真机配置只接受检测模型输出，模型加载失败会直接阻止运行。

## 2. 准备 PC 环境

PC 需要 x86_64 Linux 和支持 OpenGL 的桌面会话。安装窗口与渲染依赖：

```bash
sudo apt update
sudo apt install -y libglfw3-dev libgl1-mesa-dev
```

UR5e 和 Robotiq 2F-85 使用固定版本的 MuJoCo Menagerie 资源，苹果和香蕉使用固定版本的 `Piper_Mujoco_Sim` 资源。下载清单记录上游 commit 和 SHA-256，启用 MuJoCo 后端时，CMake 会自动下载并校验。也可以在配置前手动准备：

```bash
python3 scripts/prepare_mujoco_assets.py
```

离线构建前应先执行该命令。资源下载完成后，可以使用 `python3 scripts/prepare_mujoco_assets.py --check` 校验完整性。各资源的来源和许可见 [`simulation/mujoco/LICENSES.md`](../simulation/mujoco/LICENSES.md)。

项目优先使用 `MUJOCO_DIR`、`/usr/local` 或 `/opt/mujoco` 中的 MuJoCo。未找到时，CMake 会下载固定版本到 `~/.cache/thirdparty/mujoco`。配置成功时应显示：

```text
-- [Executor] mujoco ur5e simulation backend enabled
-- [Camera] mujoco rendered camera backend enabled
```

## 3. 构建 PC 仿真服务

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
cmake -S . -B build_mujoco \
  -DUSE_OPENCV_DNN=ON \
  -DUSE_MOCK_EXECUTOR=ON \
  -DENABLE_WEBRTC_AEC=OFF \
  -DENABLE_LAS2_CAMERA=OFF \
  -DENABLE_MUJOCO_EXECUTOR=ON \
  -DENABLE_REMOTE_MUJOCO=ON
cmake --build build_mujoco -j"$(nproc)"
```

`USE_MOCK_EXECUTOR` 只作用于 PC 上不使用的 pipeline 核心程序。MuJoCo 服务仍使用 `mujoco_ur5e` 执行器。

构建完成后检查仿真服务、场景资源和 pipeline 程序：

```bash
python3 scripts/check_runtime_env.py \
  --config config/grasp_pipeline_mujoco_ur5e.yaml \
  --build-dir build_mujoco \
  --skip-voice \
  --no-fix
```

最后一行应显示 `[SUMMARY] ready`。

## 4. 启动 PC 仿真服务

```bash
./build_mujoco/perceptive_grasp \
  --serve-simulation \
  --config config/grasp_pipeline_mujoco_ur5e.yaml \
  --listen 0.0.0.0 \
  --port 9090 \
  --viewer
```

服务启动后输出监听地址，并显示 `perceptive_grasp mujoco simulation` 窗口。主画面为可调第三视角，右上角为虚拟 RGB-D 相机的彩色画面。

- 左键拖动：水平旋转。
- `shift` + 左键拖动：俯仰旋转。
- 右键拖动：平移。
- 中键拖动或滚轮：缩放。

每次启动服务或显式重置场景时，程序都会生成新的单行排列顺序、位置和水平摆放角度。切换抓取目标不会重置场景，已经放入红色区域的物体和其余待抓物体会保留当前状态。服务端通过 `[MujocoSimulation] Pickup layout:` 输出当前布局的物体位置和 `yaw`，用于确认随机化结果。

## 5. 选择 pipeline 部署位置

### 5.1 在 PC 本地运行

保持仿真服务运行，在第二个终端启动抓取 pipeline：

```bash
./build_mujoco/perceptive_grasp \
  --config config/grasp_pipeline_remote_mujoco_ur5e.yaml \
  --target apple \
  --remote-host 127.0.0.1
```

PC 本地和 PC/K3 跨机部署使用同一个 `remote_mujoco` 客户端路径。区别仅为 `--remote-host` 的地址和检测器运行平台。

### 5.2 在 K3 运行

在 K3 构建远程客户端和软件 AEC：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd application/ros2/linksee/perceptive_grasp
cmake -S . -B build_remote_mujoco \
  -DENABLE_REMOTE_MUJOCO=ON \
  -DENABLE_LAS2_CAMERA=OFF \
  -DENABLE_WEBRTC_AEC=ON
cmake --build build_remote_mujoco -j"$(nproc)"
```

该构建使用 K3 的 `vision_service` 和 SpaceMIT EP 执行 YOLOv8-Seg，相机和机械臂使用远程仿真后端。

确认 PC 仿真服务、构建产物、标定配置和可选语音设备：

```bash
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py \
  --config config/grasp_pipeline_remote_mujoco_ur5e.yaml \
  --build-dir build_remote_mujoco \
  --no-fix
```

最后一行显示 `[SUMMARY] ready` 后启动抓取：

```bash
./build_remote_mujoco/perceptive_grasp \
  --config config/grasp_pipeline_remote_mujoco_ur5e.yaml \
  --target apple \
  --remote-host <pc_ip>
```

将 `--target` 设置为 `apple`、`banana` 或 `cube` 可验证顶抓。将目标改为 `cup` 可验证侧抓。

在同一个服务进程中依次验证四个物体：

```bash
for target in apple banana cup cube; do
  ./build_remote_mujoco/perceptive_grasp \
    --config config/grasp_pipeline_remote_mujoco_ur5e.yaml \
    --target "$target" \
    --remote-host <pc_ip> || break
done
```

每次命令只切换当前目标。服务端保留其余待抓物体和已经放入红色区域的物体，直到服务重启或收到显式场景重置命令。

顶层 `remote_mujoco` 配置块同时设置相机和机械臂服务地址。`--remote-host` 和 `--remote-port` 可在启动时指定 PC 地址。

## 6. 在 K3 启用语音控制

将远程仿真配置中的设备编号对应到 K3 的麦克风和扬声器：

```yaml
voice:
  echo_cancellation:
    mode: "webrtc_aec"
  asr:
    backend: "qwen3_asr"
    device: 1
    device_name: "2K USB Camera"
    rate: 16000
    channels: 1
    channel_index: -1
    mixer_volume: 40
  tts:
    playback_device: 1
    playback_device_name: "2K USB Camera"
```

以上配置适用于当前使用 2K USB Camera 录音和播放的 K3。语音桥优先按设备名称解析当前编号，避免重启或插拔设备后编号变化。使用其他音频硬件时，应根据环境检查结果更新名称、编号和声道数；只有录音设备已输出硬件回声消除结果时才使用 `hardware_aec`。

仿真语音链路与真机共用 `qwen3_asr` 和 `sensevoice` 后端。远程仿真配置默认使用 `qwen3_asr`，真机配置默认使用 `sensevoice`；两者都可通过 `--asr-backend` 在启动时选择。本机 Qwen 服务由语音桥自动管理，`remote_mujoco` 协议和抓取 pipeline 不需要调整。

启动语音控制：

```bash
source ~/.venv-grasp/bin/activate
./build_remote_mujoco/perceptive_grasp \
  --voice-control \
  --config config/grasp_pipeline_remote_mujoco_ur5e.yaml \
  --remote-host <pc_ip>
```

出现 `[VoiceBridge] Listening` 后，说“抓苹果”“抓香蕉”“抓木块”或“抓杯子”。语音桥在 K3 执行 VAD、ASR、AEC 和 TTS，并通过同一个 pipeline 状态机控制 PC 中的仿真机械臂。

K3 本地运行 MuJoCo 不属于当前支持范围。K3 使用 RISC-V 64 位架构，MuJoCo 官方未提供对应的预编译运行库；物理仿真与渲染保持在 x86_64 PC。

## 7. 验收仿真结果

一次完整任务应满足以下条件：

1. `apple`、`banana` 和 `cube` 选择 `top`，`cup` 选择 `side`。
2. 绿色区域和四个物体完整出现在相机画面中，物体之间没有互相遮挡。
3. 夹爪到达目标并产生双指接触后，目标才随夹爪移动。
4. 机械臂抬起目标并移动到桌面红色区域上方。
5. 夹爪张开后目标受重力落到桌面，服务端输出 `inside_zone_xy=true`。
6. 连续抓取后，已放置的目标保持在红色区域内，后续目标选择不会重置场景。
7. pipeline 依次完成 `APPROACHING`、`GRASPING`、`LIFTING`、`PLACING` 和 `HOMING`，最终摘要为 `result=SUCCESS`。

服务端在释放时输出物体相对红色区域中心的偏移、扣除物体尺寸后的可用半宽和 `inside_zone_xy=true`。该结果表示完整物体留有边界余量地落入放置区，不表示通过直接修改物体坐标完成任务。

发生机械臂与桌面碰撞、路径碰撞、空抓、目标落在放置区域外或通信超时时，任务必须失败并进入恢复流程，不能通过移动目标位置伪造成功。

## 8. 配置文件

- [`grasp_pipeline_mujoco_ur5e.yaml`](../config/grasp_pipeline_mujoco_ur5e.yaml)：PC 仿真服务的场景、相机、UR5e 和物理参数。
- [`grasp_pipeline_remote_mujoco_ur5e.yaml`](../config/grasp_pipeline_remote_mujoco_ur5e.yaml)：PC 或 K3 抓取 pipeline 的远程相机、机械臂和语音配置。

字段说明见[抓取配置参考](grasp_config.md)。
