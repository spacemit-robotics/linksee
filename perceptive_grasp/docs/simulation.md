# MuJoCo 仿真运行

本文用 `manipulation` 运行 UR5e、Robotiq 2F-85 与 RGB-D 抓取场景，支持 PC 或 K3 本机运行，也支持 PC 服务 + K3 客户端分开部署。安装、构建和首次抓取见 [README 的仿真运行](../README.md#7-仿真运行)。

## 1. 文件与职责

物理世界、设备控制与渲染由 SDK 的 `components/control/manipulation/` 提供；应用侧保存抓取场景、感知与规划配置。服务中的机械臂、夹爪、相机和窗口共用同一个物理世界。下表路径相对 `perceptive_grasp/`：

| 文件或目录 | 用途 |
| --- | --- |
| `scripts/run_manipulation_simulation.py` | 构建、启动场景、运行抓取 |
| `config/grasp_pipeline_manipulation_mujoco.yaml` | 本机 pipeline、仿真服务、RGB-D 相机与规划配置 |
| `config/grasp_pipeline_manipulation_remote.yaml` | 本机或远程抓取客户端配置 |
| `simulation/mujoco/manipulation/device.yaml` | 机械臂、夹爪驱动及设备限制 |
| `simulation/mujoco/manipulation/mapping.yaml` | 场景、关节和执行器映射 |
| `simulation/mujoco/manipulation_scene.xml` | 场景、初始姿态及物理参数 |
| `build_manipulation/prefix/` | 示例使用的公共组件安装目录 |
| `build_manipulation/app/` | 本机服务与抓取客户端 |
| `build_manipulation/client/` | `build --client-only` 生成的客户端 |

`manipulator.mujoco.device_plan` 是仿真执行器的必填项，指向公共设备配置；该配置选择 `mujoco_ur5e` 与 `mujoco_robotiq_2f85`，映射文件中的 `scene` 指定唯一的 MJCF 场景来源。

服务通过 `DeviceSession` 控制机械臂与夹爪，`MujocoRuntime` 维护物理世界。相机与窗口读取世界快照；逆运动学和路径检查使用状态副本，不移动场景物体。

## 2. 构建与运行

在应用目录构建本机服务和客户端：

```bash
python3 scripts/run_manipulation_simulation.py build
python3 scripts/run_manipulation_simulation.py serve
```

保持服务运行，在同一目录的另一终端抓取：

```bash
python3 scripts/run_manipulation_simulation.py grasp apple
```

目标可选 `apple`、`banana`、`cube`、`cup`，前三种顶抓、杯子侧抓。`serve` 用服务配置，`grasp` 用客户端配置；本机抓取也经回环地址连服务。

仅构建远程客户端用 `python3 scripts/run_manipulation_simulation.py build --client-only`，不需要客户端安装 MuJoCo 或图形库。PC/K3 跨机部署的 SSH 转发见 [README](../README.md#73-pc-显示k3-远程抓取)。

### 安装目录运行

按 [README](../README.md#51-构建并安装-k3-程序)安装的 K3 程序同时包含仿真服务和客户端。配置与完整场景在 `$PREFIX/share/perceptive_grasp`，可从任意目录启动。在有 X11 图形环境的终端起服务：

```bash
source ~/spacemit_robot/build/envsetup.sh
"$PREFIX/bin/mujoco_grasp_sim_server" \
  --config "$PREFIX/share/perceptive_grasp/config/grasp_pipeline_manipulation_mujoco.yaml" \
  --viewer
```

保持运行，在另一终端执行：

```bash
source ~/spacemit_robot/build/envsetup.sh
"$PREFIX/bin/perceptive_grasp" \
  --config "$PREFIX/share/perceptive_grasp/config/grasp_pipeline_manipulation_remote.yaml" \
  --target apple
```

该构建使用 VisionService，需准备[检测模型与标签](sdk_dependencies.md#5-准备检测模型)；前述源码示例脚本是 OpenCV DNN 构建，未装 YOLO 模型时可用场景颜色分割。

### 图形环境

PC 用 GLFW/OpenGL，K3 用系统镜像提供的原生 X11/gl4es 与 EGL/GLES 驱动。图形上下文在主线程创建，鼠标可旋转、平移、缩放视角。

`serve --no-viewer` 只隐藏窗口，模拟 RGB-D 相机仍需要可用的 `DISPLAY`（桌面会话、SSH X11 转发或 Xvfb 均可）；仅远程客户端不需要显示环境。启动脚本把 Mesa 软件渲染线程数默认设为 `2`，可用 `LP_NUM_THREADS` 覆盖，该设置不改变硬件渲染方式。

### 服务地址

| 服务 | 默认回环端口 | 用途 |
| --- | --- | --- |
| 应用仿真服务 | `9090` | 提供 RGB-D 输入，执行规划、抓取与放置任务 |
| `manipulation-simulation` | `9100` | 通过公共设备协议控制机械臂、末端和读取图像 |

应用服务内部使用 `manipulation`，其 `remote_mujoco` 抓取协议与组件通用设备协议不同，客户端和端口不能互换；跨机部署用 SSH 转发。

应用协议按顺序处理请求，执行期间不支持并行取消或采帧。客户端断连不代表动作停止，应用须等待动作结果并处理失败状态。

## 3. 场景与任务行为

场景的初始姿态、重力补偿和伺服参数由 MJCF 定义。设备映射设置 `synchronized_joint_point: true`，使关节点位按共同时间比例推进，保持规划路径的轴间关系。

源码示例的 OpenCV DNN 构建允许依据场景颜色生成目标掩码，未装 YOLO 模型时仍可完成几何估计和抓取；颜色分割只适用于配套合成场景。VisionService 构建和真机运行需要检测模型。

- 夹爪双指接触目标后才建立抓持约束，空抓不会附着物体；约束释放后物体继续参与物理仿真。
- 放杯时先扶正杯体，再按实际几何降低释放高度；检查姿态与桌面净空后开夹，退离并返回初始位。
- 放杯或退离失败会阻止后续动作，须重置场景后重新开始。
- 切换目标保留其余待抓物体和已放置物体的状态；重启服务恢复初始场景。

场景参数变化时，同步检查相机外参、关节映射、抓取尺寸和规划工作区。抓持约束是仿真简化，不能据此判断纯摩擦抓持或真实夹爪的抓稳能力。

## 4. 共享仿真的语音控制

先完成 [README 的远程部署](../README.md#73-pc-显示k3-远程抓取)，保持服务与 SSH 转发运行，并按[语音控制](voice_control.md)准备 Python 依赖、模型及音频设备。

`build --client-only` 关闭 WebRTC AEC，按音频设备选择回声处理方式：

| 回声处理方式 | 配置与要求 |
| --- | --- |
| 播报期间暂停录音 | 在 `config/grasp_pipeline_manipulation_remote.yaml` 中把 `voice.echo_cancellation.mode` 设为 `half_duplex` |
| 硬件回声消除 | 音频设备已提供硬件回声消除时选 `hardware_aec` |
| 软件 AEC | 另行构建 `ENABLE_WEBRTC_AEC=ON` 的客户端 |

在应用目录运行：

```bash
source ~/.venv-grasp/bin/activate
./build_manipulation/client/perceptive_grasp \
  --voice-control \
  --config config/grasp_pipeline_manipulation_remote.yaml \
  --remote-host 127.0.0.1
```

语音模型与音频设备在客户端运行。应用支持 `sensevoice` 和 `qwen3_asr`，用 `--asr-backend` 选择；语音接入不改变应用协议的执行与断连语义。

## 5. 检查运行结果

一次完整任务依次为：`apple`、`banana`、`cube` 顶抓、`cup` 侧抓；机械臂按通过碰撞检查的路径接近绿色取物区中的目标；双指接触后建立抓持，随夹爪抬升移到红色放置区；夹爪张开释放，服务端确认目标落在放置区；pipeline 完成接近、抓持、抬升、放置和归位，最终摘要显示 `result=SUCCESS`；切换目标后已完成任务的场景状态保持。

发生路径碰撞、空抓、通信超时或放置检查失败时，检查失败日志与恢复结果。字段和约束见[抓取配置参考](grasp_config.md)。
