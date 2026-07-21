# 依赖、sdk 组件与模型

本文说明如何在 spacemit k3 上准备 perceptive grasp 的软件依赖、sdk 组件和模型文件。

检测模型和四个 sdk 组件为必需项。spacemit_las2 运行时和语音模型仅在启用相应功能时安装。

## 1. 准备 sdk

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

每次打开新终端后，先加载 sdk 环境：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
```

该脚本会把 `output/staging/bin` 和 `output/staging/lib` 加入运行环境，并提供 `mm` 等构建命令。

## 2. 安装软件依赖

### 2.1 安装系统库

安装源码管理、构建、python 和运动学依赖：

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
  libboost-all-dev \
  liburdfdom-dev
```

`meson` 和 `ninja-build` 用于构建语音桥的 webrtc 软件回声消除模块。只使用硬件回声消除或 `half_duplex` 模式时，可以在 cmake 配置阶段设置 `-DENABLE_WEBRTC_AEC=OFF`。

### 2.2 安装 pinocchio

Linksee 机械臂使用 pinocchio 完成正运动学和逆运动学计算。

```bash
cd ~
git clone --recursive \
  -b v3.9.0 \
  https://github.com/stack-of-tasks/pinocchio.git

cmake -S ~/pinocchio -B ~/pinocchio/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_WITH_COLLISION_SUPPORT=OFF
cmake --build ~/pinocchio/build -j"$(nproc)"
sudo cmake --install ~/pinocchio/build
sudo ldconfig
```

确认 cmake 能找到 pinocchio：

```bash
find /usr/local -path '*pinocchio*Config.cmake' -print
```

## 3. 安装 python 依赖

创建独立虚拟环境并安装环境检查、手眼标定和语音桥所需的 python 包：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
python3 -m venv ~/.venv-grasp
source ~/.venv-grasp/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

## 4. 构建 sdk 组件

`mm` 会构建当前组件并将公共头文件和动态库安装到 `~/spacemit_robot/output/staging`。按本节顺序构建，确保机械臂组件能够找到夹爪和运动学依赖。

### 4.1 构建底盘控制组件

底盘组件提供 uart 和 rpmsg 驱动，供 pipeline 执行短距离前进、后退和原地转向。

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

视觉组件提供 `vision_service` 接口，并通过 spacemit ep 在 x100 ai 核上执行 yolov8-seg 推理。

```bash
cd ~/spacemit_robot/components/model_zoo/vision
mm
```

确认产物：

```bash
ls ~/spacemit_robot/output/staging/include/vision_service.h
ls ~/spacemit_robot/output/staging/lib/libvision.so
```

### 4.3 构建夹爪控制组件

夹爪组件提供夹爪开合、力度控制和夹持状态读取接口。

```bash
cd ~/spacemit_robot/components/control/grasp
mm
```

确认产物：

```bash
ls ~/spacemit_robot/output/staging/include/grasp.h
ls ~/spacemit_robot/output/staging/lib/libgrasp.so
```

### 4.4 构建机械臂控制组件

机械臂组件提供 Linksee 机械臂驱动、关节运动、直线运动以及基于 pinocchio 的正逆运动学接口。

```bash
cd ~/spacemit_robot/components/control/manipulator
mm
```

确认产物：

```bash
ls ~/spacemit_robot/output/staging/include/manipulator.h
ls ~/spacemit_robot/output/staging/lib/libmanipulator.so
```

## 5. 准备检测模型

下载 yolov8-seg 模型，并将 coco 标签复制到运行时路径：

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

## 6. 准备立体相机后端

运行阶段会初始化 `camera.type` 选中的后端。

当应用选择 realsense 深度相机后端时，需要安装 realsense 依赖。使用 spacemit_las2 时，则需要准备其运行时、模型和标定文件。

先安装立体相机设备检查工具：

```bash
sudo apt install -y v4l-utils
```

### 6.1 realsense 后端

安装 realsense 运行库和工具：

```bash
sudo apt install -y ros-humble-librealsense2
```

`realsense` 后端无需额外模型。安装完成后确认工具可用：

```bash
source /opt/ros/humble/setup.sh
rs-enumerate-devices --version
```

设备连接和数据流检查见[硬件部署](hardware_setup.md#5-验证立体相机)。

### 6.2 spacemit_las2 后端

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

只使用命令行抓取时可以跳过本节。本地语音桥需要 vad、asr、tts 和中文文本规范化资源。

### 7.1 下载 asr 模型

sensevoice 将语音命令识别为文本：

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget -O sensevoice.tar.gz \
  https://archive.spacemit.com/spacemit-ai/model_zoo/asr/sensevoice.tar.gz
tar -xzf sensevoice.tar.gz
```

### 7.2 下载 vad 模型

silero vad 用于识别语音开始和结束：

```bash
mkdir -p ~/.cache/models/vad/silero
wget -O ~/.cache/models/vad/silero/silero_vad.onnx \
  https://archive.spacemit.com/spacemit-ai/model_zoo/vad/silero/silero_vad.onnx
```

### 7.3 下载 tts 模型

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

### 7.4 下载文本规范化资源

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
