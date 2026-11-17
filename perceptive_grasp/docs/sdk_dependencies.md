# 依赖、SDK 组件与模型

本文说明如何在 SpaceMIT K3 上准备 `perceptive_grasp` 真机部署的软件依赖、SDK 组件和模型文件。仅运行仿真时，按 [README 的仿真运行](../README.md#7-仿真运行)准备，无需安装真机相机和机械臂依赖。

检测模型、底盘、视觉和 `manipulation` 组件为必需项。`spacemit_las2` 运行时和语音模型仅在启用相应功能时安装。

## 1. 准备 SDK

已有完整的 `~/spacemit_robot` 工作区时跳过本节。

```bash
mkdir -p ~/spacemit_robot
cd ~/spacemit_robot
repo init -u https://github.com/spacemit-robotics/manifest.git \
  -b main \
  -m default.xml \
  --repo-url=https://gitee.com/spacemit-robotics/git-repo
repo sync -j4
repo start robot-dev --all
```

每次打开新终端后，先加载 SDK 环境：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
```

该脚本会把 `output/staging/bin` 和 `output/staging/lib` 加入运行环境，并提供 `mm` 等构建命令。

## 2. 安装软件依赖

### 2.1 安装系统库

安装源码管理、构建、Python 和运动学依赖：

```bash
sudo apt update
sudo apt install -y \
  git \
  repo \
  wget \
  unzip \
  patchelf \
  cmake \
  build-essential \
  meson \
  ninja-build \
  python3-pip \
  python3-venv \
  libyaml-cpp-dev \
  libeigen3-dev \
  libopencv-dev \
  libopenblas0-pthread \
  openblas-spacemit \
  spacemit-onnxruntime \
  libboost-all-dev \
  liburdfdom-dev \
  libglfw3-dev \
  libgl1-mesa-dev \
  libx11-dev
```

### 2.2 运动学后端（无需额外安装）

`manipulation` 组件内置 `eigen` 运动学后端（`MANIPULATION_ENABLE_KINEMATICS_EIGEN` 默认开启），只依赖常见的 Eigen3 头文件，不需要单独下载和编译任何运动学库。构建组件时确认 `libeigen3-dev` 已安装即可：

```bash
sudo apt install -y libeigen3-dev
```

应用配置中的 `kinematics_solver` 默认为 `eigen`，与旧的 `pinocchio` 后端在相同 URDF 和关节序列下结果一致。

如确实需要切换到 Pinocchio 后端，可单独开启该开关；不开启不影响正常使用：

```bash
cd ~/spacemit_robot/components/control/manipulation
mm -DMANIPULATION_ENABLE_KINEMATICS_PINOCCHIO=ON
```

开启后需自行编译安装 Pinocchio v3.9.0，并把配置改为 `kinematics_solver: "pinocchio"`。

## 3. 安装 Python 依赖

创建独立虚拟环境并安装环境检查、手眼标定和语音桥所需的 Python 包：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
python3 -m venv ~/.venv-grasp
source ~/.venv-grasp/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

## 4. 构建 SDK 组件

`mm` 会构建当前组件并将公共头文件和动态库安装到 `~/spacemit_robot/output/staging`。按本节顺序准备底盘、视觉与统一机械臂组件及其依赖。

### 4.1 构建底盘控制组件

底盘组件提供 UART 和 RPMsg 驱动，供 pipeline 执行短距离前进、后退和原地转向。

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd components/control/base
mm
```

确认产物：

```bash
ls ~/spacemit_robot/output/staging/include/chassis.h
ls ~/spacemit_robot/output/staging/lib/libchassis.so
```

### 4.2 构建视觉推理组件

视觉组件提供 `vision_service` 接口，并通过 SpaceMIT EP 在 X100 AI 核上执行 YOLOv8-Seg 推理。

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd components/model_zoo/vision
mm -- -C "$SROBOTIS_ROOT/application/ros2/linksee/perceptive_grasp/cmake/vision-k3-system-cpp.cmake"
```

`vision-k3-system-cpp.cmake` 由 Linksee 提供，使用现有 `vision` 组件源码构建 C++ 视觉库和测试工具。该配置使用系统 OpenCV、Eigen 和 OpenBLAS，不构建 Python 扩展、wheel 或示例程序。OpenBLAS 使用 `openblas-spacemit` 提供的同版本 LP64 头文件；配置阶段检查头文件与系统运行库是否匹配。

该配置会覆盖当前构建缓存中的依赖与功能开关，后续 `mm` 沿用该缓存。切换回全量构建时，使用新的构建目录或清理视觉组件的构建缓存。

确认产物：

```bash
ls ~/spacemit_robot/output/staging/include/vision_service.h
ls ~/spacemit_robot/output/staging/lib/libvision.so
```

### 4.3 构建机械臂与夹爪控制组件

`manipulation` 提供设备控制、关节反馈、正逆运动学和公共标定工具。内置 `eigen` 运动学后端默认开启，无需额外参数。以下配置同时构建 MuJoCo 物理世界、窗口和 RGB-D 渲染，供应用的本机仿真服务使用：

```bash
cd ~/spacemit_robot/components/control/manipulation
mm --with-deps \
  -DMANIPULATION_ENABLE_MUJOCO=ON \
  -DMANIPULATION_ENABLE_SIMULATION_RENDERER=ON
```

确认产物：

```bash
manipulation drivers --roles
manipulation-device --help
manipulation-simulation --help
```

组件安装完成后，按 [README](../README.md#51-构建并安装-k3-程序)构建并安装应用。MuJoCo 窗口与 RGB-D 渲染需要可用的 X11 图形环境；K3 使用系统提供的 EGL/GLES 驱动。

## 5. 准备检测模型

下载 YOLOv8-Seg 模型，并将 COCO 标签复制到运行时路径：

```bash
cd ~/spacemit_robot/components/model_zoo/vision/examples/yolov8_seg
bash scripts/download_models.sh

mkdir -p ~/.cache/models/vision/labels
cp ~/spacemit_robot/components/model_zoo/vision/assets/labels/coco.txt \
  ~/.cache/models/vision/labels/coco.txt
```

确认模型和标签文件：

```bash
ls ~/.cache/models/vision/yolov8_seg/yolov8n-seg.q.onnx
ls ~/.cache/models/vision/labels/coco.txt
```

## 6. 准备 RGB-D 相机后端

运行阶段会初始化 `camera.type` 选中的后端。

应用选择 `realsense` 后端时，需要安装 RealSense 依赖。使用 `spacemit_las2` 时，需要准备对应的运行时、模型和标定文件。

先安装 RGB-D 相机设备检查工具：

```bash
sudo apt install -y v4l-utils
```

### 6.1 `realsense` 后端

安装 RealSense 运行库和工具：

```bash
sudo apt install -y ros-humble-librealsense2
```

`realsense` 后端无需额外模型。安装完成后确认工具可用：

```bash
source /opt/ros/humble/setup.sh
rs-enumerate-devices --version
```

设备连接和数据流检查见[硬件部署](hardware_setup.md#5-验证-rgb-d-相机)。

### 6.2 `spacemit_las2` 后端

下载运行时压缩包和深度推理模型，解压后目录为 `~/las2_runtime`：

```bash
cd ~
wget -O las2_runtime_handoff.zip \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/libs/las2/las2_runtime_handoff.zip
unzip -oq las2_runtime_handoff.zip

mkdir -p ~/las2_runtime/models
wget -O ~/las2_runtime/models/LAS2_M_256x320.fp16.iofp32.corr_func_nhwc.gelu.onnx \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vla/libs/las2/LAS2_M_256x320.fp16.iofp32.corr_func_nhwc.gelu.onnx
```

确认接口、运行库、模型和双目标定文件：

```bash
ls ~/las2_runtime/include/las2_usb_stereo.h
ls ~/las2_runtime/lib/liblas2_usb_stereo.so
ls ~/las2_runtime/models/LAS2_M_256x320.fp16.iofp32.corr_func_nhwc.gelu.onnx
ls ~/las2_runtime/config/matlab_stereo_opencv.json
```

## 7. 准备语音模型（可选）

只使用命令行抓取时可以跳过本节。本地语音桥需要 VAD、ASR、TTS 和中文文本规范化资源。

### 7.1 准备 sensevoice（可选）

sensevoice 将语音命令识别为文本：

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget -O sensevoice.tar.gz \
  https://archive.spacemit.com/spacemit-ai/model_zoo/asr/sensevoice.tar.gz
tar -xzf sensevoice.tar.gz
```

### 7.2 准备 Qwen3-ASR

Qwen3-ASR 通过 `llama-server` 提供 OpenAI 兼容的语音识别接口。安装 K3 优化的服务程序并下载模型：

```bash
sudo apt install -y llama.cpp-tools-spacemit
dpkg-query -W llama.cpp-tools-spacemit

mkdir -p ~/.cache/models/asr/qwen3asr
cd ~/.cache/models/asr/qwen3asr
wget -O qwen3-asr-0.6B-dynq-q40.tar.gz \
  https://archive.spacemit.com/spacemit-ai/model_zoo/asr/qwen3-asr-0.6B-dynq-q40.tar.gz
tar -xzf qwen3-asr-0.6B-dynq-q40.tar.gz
rm -f qwen3-asr-0.6B-dynq-q40.tar.gz
```

Qwen3-ASR 媒体后端需要 `llama.cpp-tools-spacemit 0.1.7` 或更高版本。版本较旧时先执行 `sudo apt update` 和 `sudo apt upgrade llama.cpp-tools-spacemit`。

应用默认在选择 Qwen3-ASR 后自动启动和停止本机服务，无需单独执行 `llama-server`。以下命令仅用于独立验证模型或为其他客户端常驻提供服务：

```bash
MODEL_DIR=~/.cache/models/asr/qwen3asr/qwen3-asr-0.6B-dynq-q40
llama-server \
  -m "$MODEL_DIR/Qwen3-ASR-0.6B-text-q40.gguf" \
  --media-backend smt \
  --smt-config-dir "$MODEL_DIR" \
  --host 127.0.0.1 \
  --port 8063 \
  -t 4
```

默认配置连接 `http://127.0.0.1:8063/v1/chat/completions`。服务运行在其他设备时，将地址改为对应的局域网 ip，并关闭 `auto_start`；远程服务需要在语音桥启动前保持运行。

### 7.3 下载 VAD 模型

Silero VAD 用于识别语音开始和结束：

```bash
mkdir -p ~/.cache/models/vad/silero
wget -O ~/.cache/models/vad/silero/silero_vad.onnx \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vad/silero/silero_vad.onnx
```

### 7.4 下载 TTS 模型

matcha-tts 生成中文语音特征，vocos 声码器将其转换为可播放音频：

```bash
mkdir -p ~/.cache/models/tts/matcha-tts
cd ~/.cache/models/tts/matcha-tts
wget -O matcha-icefall-zh-baker.tar.gz \
  https://archive.spacemit.com/spacemit-ai/model_zoo/tts/matcha-tts/matcha-icefall-zh-baker.tar.gz
tar -xzf matcha-icefall-zh-baker.tar.gz

mkdir -p ~/.cache/models/tts/vocoder
wget -O ~/.cache/models/tts/vocoder/vocos-22khz-univ.q.onnx \
  https://archive.spacemit.com/spacemit-ai/model_zoo/tts/vocoder/vocos-22khz-univ.q.onnx
```

### 7.5 下载文本规范化资源

文本规范化资源用于处理日期、数字和电话号码等中文播报内容：

```bash
mkdir -p ~/.cache/models/tts/text_norm/v1/zh
cd ~/.cache/models/tts/text_norm/v1/zh
wget -O date.fst \
  https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/date.fst
wget -O new_heteronym.fst \
  https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/new_heteronym.fst
wget -O number.fst \
  https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/number.fst
wget -O phone.fst \
  https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/phone.fst
```

确认语音模型和资源：

```bash
ls ~/.cache/models/asr/sensevoice
ls ~/.cache/models/vad/silero/silero_vad.onnx
ls ~/.cache/models/tts/matcha-tts
ls ~/.cache/models/tts/vocoder/vocos-22khz-univ.q.onnx
ls ~/.cache/models/tts/text_norm/v1/zh
```

## 8. 构建并安装应用

完成依赖与模型准备后，按 [README](../README.md#51-构建并安装-k3-程序)构建安装。该配置同时包含真实硬件执行器、VisionService、本机 MuJoCo 服务与远程客户端；程序安装到 `$PREFIX/bin`，配置和资源安装到 `$PREFIX/share/perceptive_grasp`。

常用构建选项如下，默认值指未显式传入选项时的 CMake 默认值：

- `ENABLE_WEBRTC_AEC`：启用本地语音桥的软件回声消除，默认开启。
- `ENABLE_LAS2_CAMERA`：编译 `spacemit_las2` 双目相机后端，默认开启；未找到运行库时构建继续，运行时选择该后端会报错。
- `ENABLE_REMOTE_MUJOCO`：编译应用抓取协议客户端，连接本机或远程仿真服务；客户端不依赖本机 MuJoCo。
- `ENABLE_MUJOCO_EXECUTOR`：通过 `manipulation` 构建仿真服务，机械臂、夹爪、相机和窗口共用一个物理世界，支持 x86_64 PC 与 riscv64 K3。
- `USE_OPENCV_DNN`：使用 OpenCV DNN 检测器，主要用于 PC 仿真和未部署 `vision_service` 的独立构建。
- `USE_MOCK_EXECUTOR`：使用 mock 执行器，不控制真实机械臂。

共享仿真的物理世界与渲染库由 SDK 的 `components/control/manipulation/simulation/mujoco/` 提供，随 `manipulation` 一同构建、安装。

共享仿真使用独立构建目录。PC 或 K3 本机服务执行：

```bash
python3 scripts/run_manipulation_simulation.py build
```

只运行远程客户端时执行：

```bash
python3 scripts/run_manipulation_simulation.py build --client-only
```

构建产物按用途分目录保存：

| 构建方式 | 输出目录 |
| --- | --- |
| 本机服务与客户端 | 公共组件：`build_manipulation/prefix/`；应用：`build_manipulation/app/` |
| 仅客户端 | `build_manipulation/client/` |

仅客户端不需要本机 MuJoCo 或显示器。两种构建均关闭真机执行器、LAS2 和 WebRTC AEC。启用语音时须按[共享仿真的语音控制](simulation.md#4-共享仿真的语音控制)选择回声处理模式。

安装依赖与运行命令见 [README](../README.md#7-仿真运行)，场景、协议和语音配置见[仿真运行](simulation.md)。
