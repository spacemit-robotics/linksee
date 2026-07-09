# 依赖预编译与模型下载

## 1. 安装系统依赖

安装系统依赖：

```bash
sudo apt install -y \
  ros-humble-librealsense2 \
  v4l-utils \
  cmake \
  build-essential \
  libeigen3-dev \
  libboost-all-dev \
  liburdfdom-dev
```

安装 `Pinocchio` 正逆运动学依赖。只需要 FK/IK 时，`coal` 碰撞检测库可以跳过。

```bash
# 可选：编译安装 coal（碰撞检测库）
cd ~
git clone --recursive -b v3.0.2 https://github.com/coal-library/coal.git
cd ~/coal
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF
make -j$(nproc)
sudo make install
sudo ldconfig

# 编译安装 Pinocchio
cd ~
git clone --recursive -b v3.9.0 https://github.com/stack-of-tasks/pinocchio.git
cd ~/pinocchio
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_WITH_COLLISION_SUPPORT=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

## 2. 构建依赖组件

先加载 SDK 构建环境：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
```

构建底盘控制库。`mobile_base` 模块通过它控制 Linksee 底盘短距离前进、后退和原地转向。

```bash
cd ~/spacemit_robot/components/control/base
mm
```

产物：

```bash
ls ~/spacemit_robot/output/staging/include/chassis.h
ls ~/spacemit_robot/output/staging/lib/libchassis.so
```

构建视觉推理库。`TargetDetector` 模块通过 VisionService 加载 YOLOv8-seg 模型并输出目标框、类别和 mask。

```bash
cd ~/spacemit_robot/components/model_zoo/vision
mm
```

产物：

```bash
ls ~/spacemit_robot/output/staging/include/vision_service.h
ls ~/spacemit_robot/output/staging/lib/libvision.so
```

构建夹爪控制库。`GraspExecutor` 模块通过它控制夹爪张开、闭合，并读取夹爪状态和负载来判断是否抓住物体。

```bash
cd ~/spacemit_robot/components/control/grasp
mm
```

产物：

```bash
ls ~/spacemit_robot/output/staging/include/grasp.h
ls ~/spacemit_robot/output/staging/lib/libgrasp.so
```

构建机械臂控制库。`GraspExecutor` 模块通过它控制机械臂，并调用 Pinocchio 后端做 FK/IK 求解。

```bash
cd ~/spacemit_robot/components/control/manipulator
mm
```

产物：

```bash
ls ~/spacemit_robot/output/staging/include/manipulator.h
ls ~/spacemit_robot/output/staging/lib/libmanipulator.so
```

## 3. 下载检测模型

```bash
cd ~/spacemit_robot/components/model_zoo/vision/examples/yolov8_seg
bash scripts/download_models.sh

mkdir -p ~/.cache/models/vision/labels
cp ~/spacemit_robot/components/model_zoo/vision/assets/labels/coco.txt \
  ~/.cache/models/vision/labels/coco.txt
```

确认文件：

```bash
ls ~/.cache/models/vision/yolov8_seg/yolov8n-seg.q.onnx
ls ~/.cache/models/vision/labels/coco.txt
```

## 4. 下载语音模型

下载 ASR 语音识别模型。`local_voice_bridge.py` 使用 SenseVoice 将语音命令识别为文本：

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget https://archive.spacemit.com/spacemit-ai/model_zoo/asr/sensevoice.tar.gz
tar -xzf sensevoice.tar.gz
```

下载 VAD 语音活动检测模型。语音桥用它判断一段语音何时开始、何时结束：

```bash
mkdir -p ~/.cache/models/vad/silero
cd ~/.cache/models/vad/silero
wget https://archive.spacemit.com/spacemit-ai/model_zoo/vad/silero/silero_vad.onnx
```

下载 TTS 中文声学模型。状态播报会用 Matcha-TTS 生成中文语音特征：

```bash
mkdir -p ~/.cache/models/tts/matcha-tts
cd ~/.cache/models/tts/matcha-tts
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/matcha-tts/matcha-icefall-zh-baker.tar.gz
tar -xzf matcha-icefall-zh-baker.tar.gz
```

下载 TTS 声码器模型。声码器把 TTS 语音特征转换为可播放音频：

```bash
mkdir -p ~/.cache/models/tts/vocoder
cd ~/.cache/models/tts/vocoder
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/vocoder/vocos-22khz-univ.q.onnx
```

下载中文文本规范化资源。TTS 播报日期、数字、电话等文本前会使用这些 FST 规则做规范化：

```bash
mkdir -p ~/.cache/models/tts/text_norm/v1/zh
cd ~/.cache/models/tts/text_norm/v1/zh
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/date.fst
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/new_heteronym.fst
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/number.fst
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/text_norm/v1/zh/phone.fst
```

确认文件：

```bash
ls ~/.cache/models/asr/sensevoice
ls ~/.cache/models/vad/silero/silero_vad.onnx
ls ~/.cache/models/tts/matcha-tts
ls ~/.cache/models/tts/vocoder/vocos-22khz-univ.q.onnx
ls ~/.cache/models/tts/text_norm/v1/zh
```
