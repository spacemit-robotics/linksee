# 依赖、SDK 组件与模型

本文说明真机部署所需的软件依赖、SDK 组件和模型文件。只跑仿真时按 [README 的仿真运行](../README.md#7-仿真运行)准备，无需真机相机和机械臂依赖。

检测模型、视觉和 `manipulation` 组件为必需项；底盘驱动已内置于 `src/chassis/`；`spacemit_las2` 运行时和语音模型仅在启用相应功能时安装。

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

每次打开新终端先加载 SDK 环境。该脚本把 `output/staging/bin` 和 `output/staging/lib` 加入运行环境，并提供 `mm` 等构建命令：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
```

## 2. 安装软件依赖

### 2.1 安装系统库

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

`manipulation` 内置 `eigen` 运动学后端（`MANIPULATION_ENABLE_KINEMATICS_EIGEN` 默认开启），只依赖 Eigen3 头文件，与旧 `pinocchio` 后端在相同 URDF 和关节序列下结果一致。应用配置的 `kinematics_solver` 默认即 `eigen`，确认 `libeigen3-dev` 已装即可。

确实需要 Pinocchio 时单独开启该开关（不开启不影响使用），并自行编译安装 Pinocchio v3.9.0：

```bash
cd ~/spacemit_robot/components/control/manipulation
mm -DMANIPULATION_ENABLE_KINEMATICS_PINOCCHIO=ON
```

开启后把配置改为 `kinematics_solver: "pinocchio"`。

## 3. 安装 Python 依赖

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
python3 -m venv ~/.venv-grasp
source ~/.venv-grasp/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

该虚拟环境只用于应用侧的 Python 脚本。**编译 SDK 组件前必须先退出**，否则依赖检查会误判：

```bash
deactivate
```

`mm --with-deps` 用 `python3 -c 'import can'` 这类命令检查系统包。激活 venv 后 `python3` 指向 venv 解释器，而 venv 默认不继承系统 site-packages，`apt` 装的 `python3-can`、`python3-typing-extensions` 在其中不可见，检查会一直报 `still missing after installation`——`apt` 其实已经装好了。

## 4. 构建 SDK 组件

`mm` 构建当前组件并把公共头文件和动态库安装到 `~/spacemit_robot/output/staging`。编译 C++ 组件不需要虚拟环境；新开终端不会自动激活 venv，无需处理。

### 4.1 构建底盘控制组件

提供 UART 和 RPMsg 驱动，供 pipeline 执行短距离前进、后退和原地转向：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd components/control/base
mm
```

确认产物：`output/staging/include/chassis.h` 与 `output/staging/lib/libchassis.so`。

### 4.2 构建视觉推理组件

提供 `vision_service` 接口，通过 SpaceMIT EP 在 X100 AI 核上执行 YOLOv8-Seg 推理：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd components/model_zoo/vision
mm -- -C "$SROBOTIS_ROOT/application/ros2/linksee/perceptive_grasp/cmake/vision-k3-system-cpp.cmake"
```

该 cmake 文件由 Linksee 提供，用现有组件源码构建 C++ 视觉库和测试工具：使用系统 OpenCV、Eigen 和 OpenBLAS，不构建 Python 扩展、wheel 或示例；OpenBLAS 使用 `openblas-spacemit` 的同版本 LP64 头文件，配置阶段会检查头文件与系统运行库是否匹配。

注意它会覆盖当前构建缓存中的依赖与功能开关，后续 `mm` 沿用该缓存；切回全量构建时需新建构建目录或清理缓存。产物为 `output/staging/include/vision_service.h` 与 `output/staging/lib/libvision.so`。

### 4.3 构建机械臂与夹爪控制组件

`manipulation` 提供设备控制、关节反馈、正逆运动学和公共标定工具（`eigen` 后端默认开启）。以下配置同时构建 MuJoCo 物理世界、窗口和 RGB-D 渲染，供应用的本机仿真服务使用：

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

完成后按 [README](../README.md#51-构建并安装-k3-程序)构建并安装应用。MuJoCo 窗口与 RGB-D 渲染需要可用的 X11 图形环境；K3 使用系统提供的 EGL/GLES 驱动。

## 5. 准备检测模型

```bash
cd ~/spacemit_robot/components/model_zoo/vision/examples/yolov8_seg
bash scripts/download_models.sh

mkdir -p ~/.cache/models/vision/labels
cp ~/spacemit_robot/components/model_zoo/vision/assets/labels/coco.txt \
  ~/.cache/models/vision/labels/coco.txt
```

确认 `~/.cache/models/vision/yolov8_seg/yolov8n-seg.q.onnx` 与 `~/.cache/models/vision/labels/coco.txt` 存在。

## 6. 准备 RGB-D 相机后端

运行时初始化 `camera.type` 选中的后端。先装设备检查工具：

```bash
sudo apt install -y v4l-utils
```

### 6.1 `realsense` 后端

```bash
sudo apt install -y ros-humble-librealsense2
```

无需额外模型。安装后确认工具可用：

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

确认接口、运行库、模型和双目标定文件均存在：

```bash
ls ~/las2_runtime/{include/las2_usb_stereo.h,lib/liblas2_usb_stereo.so,config/matlab_stereo_opencv.json}
ls ~/las2_runtime/models/LAS2_M_256x320.fp16.iofp32.corr_func_nhwc.gelu.onnx
```

## 7. 准备语音模型（可选）

只用命令行抓取时跳过本节。本地语音桥需要 VAD、ASR、TTS 和中文文本规范化资源。

### 7.1 sensevoice（可选）

把语音命令识别为文本：

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget -O sensevoice.tar.gz \
  https://archive.spacemit.com/spacemit-ai/model_zoo/asr/sensevoice.tar.gz
tar -xzf sensevoice.tar.gz
```

### 7.2 Qwen3-ASR

Qwen3-ASR 通过 `llama-server` 提供 OpenAI 兼容的语音识别接口。需 `llama.cpp-tools-spacemit 0.1.7` 或更高版本，版本较旧时先 `sudo apt update && sudo apt upgrade llama.cpp-tools-spacemit`：

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

应用默认在选择 Qwen3-ASR 后自动启动和停止本机服务，无需手动起 `llama-server`。以下命令仅用于独立验证模型或为其他客户端常驻服务：

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

默认连接 `http://127.0.0.1:8063/v1/chat/completions`。服务在别的设备上时改成对应局域网 IP 并关闭 `auto_start`，且须在语音桥启动前保持运行。

### 7.3 VAD 模型

Silero VAD 用于识别语音开始和结束：

```bash
mkdir -p ~/.cache/models/vad/silero
wget -O ~/.cache/models/vad/silero/silero_vad.onnx \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vad/silero/silero_vad.onnx
```

### 7.4 TTS 模型

matcha-tts 生成中文语音特征，vocos 声码器转成可播放音频：

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

### 7.5 文本规范化资源

用于处理日期、数字和电话号码等中文播报内容：

```bash
mkdir -p ~/.cache/models/tts/text_norm/v1/zh
cd ~/.cache/models/tts/text_norm/v1/zh
BASE=https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh
for f in date.fst new_heteronym.fst number.fst phone.fst; do wget -O "$f" "$BASE/$f"; done
```

确认全部语音资源：

```bash
ls ~/.cache/models/asr/sensevoice
ls ~/.cache/models/vad/silero/silero_vad.onnx
ls ~/.cache/models/tts/matcha-tts
ls ~/.cache/models/tts/vocoder/vocos-22khz-univ.q.onnx
ls ~/.cache/models/tts/text_norm/v1/zh
```

## 8. 构建并安装应用

按 [README](../README.md#51-构建并安装-k3-程序)构建安装。该配置包含真实硬件执行器、VisionService、本机 MuJoCo 服务与远程客户端；程序安装到 `$PREFIX/bin`，配置和资源到 `$PREFIX/share/perceptive_grasp`。

常用构建选项（默认值指未显式传入时的 CMake 默认值）：

- `ENABLE_WEBRTC_AEC`：本地语音桥的软件回声消除，默认开启。
- `ENABLE_LAS2_CAMERA`：编译 `spacemit_las2` 双目相机后端，默认开启；未找到运行库时构建继续，运行时选该后端会报错。
- `ENABLE_REMOTE_MUJOCO`：编译抓取协议客户端，连接本机或远程仿真服务，不依赖本机 MuJoCo。
- `ENABLE_MUJOCO_EXECUTOR`：通过 `manipulation` 构建仿真服务，机械臂、夹爪、相机和窗口共用一个物理世界，支持 x86_64 PC 与 riscv64 K3。
- `USE_OPENCV_DNN`：使用 OpenCV DNN 检测器，主要用于 PC 仿真和未部署 `vision_service` 的独立构建。
- `USE_MOCK_EXECUTOR`：使用 mock 执行器，不控制真实机械臂。

共享仿真的物理世界与渲染库由 SDK 的 `components/control/manipulation/simulation/mujoco/` 提供，随 `manipulation` 一同构建安装。它使用独立构建目录：本机服务执行 `python3 scripts/run_manipulation_simulation.py build`，只跑远程客户端加 `--client-only`。

| 构建方式 | 输出目录 |
| --- | --- |
| 本机服务与客户端 | 公共组件 `build_manipulation/prefix/`；应用 `build_manipulation/app/` |
| 仅客户端 | `build_manipulation/client/` |

仅客户端不需要本机 MuJoCo 或显示器。两种构建均关闭真机执行器、LAS2 和 WebRTC AEC。启用语音时须按[共享仿真的语音控制](simulation.md#4-共享仿真的语音控制)选择回声处理模式。安装依赖与运行命令见 [README](../README.md#7-仿真运行)，场景、协议和语音配置见[仿真运行](simulation.md)。
