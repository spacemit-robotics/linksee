# perceptive grasp

perceptive grasp 是面向 Linksee 轮式机器人的近距离感知抓取应用。系统使用立体相机获取彩色图像和深度信息，通过 yolov8-seg 完成目标分割，结合深度反投影、固定相机手眼标定（eye-to-hand）和顶抓规划生成 Linksee 机械臂可执行的抓取位姿。目标超出机械臂舒适抓取区时，Linksee 底盘会短距离调整位置，停车后重新感知和规划。应用还支持本地自动语音识别（asr）命令和语音合成（tts）状态播报。

![perceptive grasp 系统链路](docs/assets/grasp-system-flow.svg)

## 1. 功能范围

perceptive grasp 提供以下功能：

- 支持 realsense d435i 深度相机和 spacemit_las2 双目相机两种立体相机后端。
- 使用 yolov8-seg 输出目标类别、检测框和分割掩码。
- 根据目标表面深度和手眼标定结果计算机械臂基座坐标。
- 规划并执行接近、夹取、抬升、放置和归位动作。
- 在抓取规划阶段执行底盘前后移动或原地转向，并在移动后重新检测。
- 通过本地语音桥接收抓取、取消和结束命令，并播报任务状态。

本方案面向桌面或近距离场景中的单目标顶抓。底盘辅助不包含导航、避障和全局路径规划。

## 2. 系统组成

| 模块 | 实现 | 作用 |
|---|---|---|
| 立体相机 | realsense d435i / spacemit_las2 | 输出像素对齐的彩色图像和深度信息 |
| 目标检测 | spacemit_vision_service(yolov8-seg) | 输出目标类别、检测框和分割掩码 |
| 抓取规划 | grasp planner | 计算抓取像素、三维坐标、预抓取位和夹爪方向 |
| 机械臂执行 | spacemit_manipulator/grasp | 控制 Linksee 机械臂和夹爪 |
| 底盘辅助 | spacemit_base | 调整机器人与目标的相对位置 |
| 语音交互 | spacemit_audio/vad/asr/tts | 接收语音命令并播报状态 |

## 3. 准备环境

1. 按[方案依赖](docs/sdk_dependencies.md)下载源码，安装系统依赖，构建依赖组件，并获取视觉和语音模型。
2. 按[硬件部署](docs/hardware_setup.md)连接立体相机、机械臂、底盘和音频设备。

## 4. 配置

主配置文件为 [`config/grasp_pipeline.yaml`](config/grasp_pipeline.yaml)。首次部署时至少完成以下配置：

1. 确认机械臂和底盘的稳定串口路径。
2. 选择立体相机后端，并填写后端所需的设备、模型和标定参数。
3. 完成 eye-to-hand 标定，更新 `calibration.T_base_camera`。
4. 根据实际音频设备更新 asr 和 tts 设备编号。

字段说明和配置步骤见[抓取配置参考](docs/grasp_config.md)和[手眼标定](docs/hand_eye_calibration.md)。

## 5. 构建与检查

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
rm -rf build
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

构建目录中的 `perceptive_grasp` 是结构化日志启动器，
`perceptive_grasp_core` 是实际运行抓取 pipeline 的 c++ 程序。正常运行应使用启动器。

构建完成后检查运行环境：

```bash
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

检查脚本会同时验证启动器和核心程序。环境可运行时，最后一行显示：

```text
[SUMMARY] ready
```

## 6. 运行抓取

抓取指定目标：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
./build/perceptive_grasp \
  --config config/grasp_pipeline.yaml \
  --target banana
```

排查立体相机、opencl、机械臂、运动学或底盘驱动时增加 `--debug` 来查看更多日志：

```bash
./build/perceptive_grasp \
  --debug \
  --config config/grasp_pipeline.yaml \
  --target banana
```

启动本地语音控制：

```bash
source ~/.venv-grasp/bin/activate
python3 scripts/local_voice_bridge.py \
  --config config/grasp_pipeline.yaml \
  --binary build/perceptive_grasp
```

## 7. 文档导航

- [方案依赖](docs/sdk_dependencies.md)（操作指南）：安装依赖、构建 sdk 组件、准备检测和语音模型。
- [硬件部署](docs/hardware_setup.md)（操作指南）：连接硬件、配置设备权限、验证立体相机和串口。
- [抓取配置](docs/grasp_config.md)（参考）：说明配置文件字段、默认值、单位和约束。
- [手眼标定](docs/hand_eye_calibration.md)（操作指南）：生成标定板、采集数据、求解并写回标定结果。
- [语音控制](docs/voice_control.md)（操作指南）：配置 asr/tts 并启动本地语音桥。
- [故障诊断](docs/debugging.md)（操作指南）：定位感知、规划、执行、底盘和性能问题。
- [pipeline 状态机](docs/pipeline_state_machine.md)（说明）：介绍状态转换、异步动作、输出事件和失败终态。
- [抓取方案](docs/grasping_approaches.md)（说明）：介绍感知定位、抓取规划、底盘介入条件和方案边界。
