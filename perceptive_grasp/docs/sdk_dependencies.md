# SDK 依赖与模型

本项目使用真实硬件链路：

```text
D435i -> VisionService(YOLOv8-seg) -> grasp_planner -> manipulator/grasp -> SO101
```

以下命令默认 SDK 位于 `~/spacemit_robot`。

## 1. 加载 SDK 环境

每个新终端先执行：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
```

## 2. 安装 Python 依赖

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
python3 -m venv ~/.venv-grasp
source ~/.venv-grasp/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 3. 安装系统依赖

```bash
sudo apt install -y ros-humble-librealsense2 v4l-utils
```

Pinocchio 按 [components/control/manipulator/README.md](../../../../../components/control/control_manipulator/README.md) 的“Pinocchio 源码编译”章节安装到 `/usr/local`。安装完成后执行：

```bash
sudo ldconfig
```

## 4. 构建 SDK 组件

```bash
cd ~/spacemit_robot
source build/envsetup.sh

cd components/model_zoo/vision
mm

cd ~/spacemit_robot/components/control/grasp
mm

cd ~/spacemit_robot/components/control/manipulator
mm
```

确认产物：

```bash
ls ~/spacemit_robot/output/staging/include/vision_service.h
ls ~/spacemit_robot/output/staging/lib/libvision.so
ls ~/spacemit_robot/output/staging/include/grasp.h
ls ~/spacemit_robot/output/staging/lib/libgrasp.so
ls ~/spacemit_robot/output/staging/include/manipulator.h
ls ~/spacemit_robot/output/staging/lib/libmanipulator.so
```

## 5. 下载检测模型

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

## 6. 下载语音模型

```bash
mkdir -p ~/.cache/models/asr
cd ~/.cache/models/asr
wget https://archive.spacemit.com/spacemit-ai/model_zoo/asr/sensevoice.tar.gz
tar -xzf sensevoice.tar.gz

mkdir -p ~/.cache/models/vad/silero
cd ~/.cache/models/vad/silero
wget https://archive.spacemit.com/spacemit-ai/model_zoo/vad/silero/silero_vad.onnx

mkdir -p ~/.cache/models/tts/matcha-tts
cd ~/.cache/models/tts/matcha-tts
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/matcha-tts/matcha-icefall-zh-baker.tar.gz
tar -xzf matcha-icefall-zh-baker.tar.gz

mkdir -p ~/.cache/models/tts/vocoder
cd ~/.cache/models/tts/vocoder
wget https://archive.spacemit.com/spacemit-ai/model_zoo/tts/vocoder/vocos-22khz-univ.q.onnx

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

## 7. 构建抓取程序

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
rm -rf build && mkdir -p build && cd build
source ~/spacemit_robot/build/envsetup.sh
cmake ..
make -j$(nproc)
```

构建摘要应包含：

```text
Detector:  VisionService
Executor:  Real hardware
Camera:    RealSense D435i
```

## 8. 运行前检查

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

检查通过时应看到：

```text
...
[SUMMARY] ready
```
