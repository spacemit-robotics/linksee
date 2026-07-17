# 语音控制

perceptive grasp 通过本地语音桥接入自动语音识别（asr）和语音合成（tts）。语音桥将识别结果发送给抓取进程，并将 pipeline 状态转换为语音播报。

![语音控制链路](assets/voice-control-flow.svg)

## 1. 准备语音环境

开始前完成以下准备工作：

- 按[方案依赖](sdk_dependencies.md#7-准备语音模型可选)准备 vad、asr、tts和文本规范化资源。
- 完成 perceptive grasp 构建，并确认 `build/perceptive_grasp` 存在。
- 连接麦克风和扬声器。

加载运行环境并检查语音模块：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 -c "import spacemit_audio, spacemit_vad, spacemit_asr, spacemit_tts"
```

命令无输出且退出码为 `0`，表示语音模块可以正常加载。

## 2. 配置音频设备

运行环境检查并查看采集设备和播放设备：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

将检查结果中的设备编号写入 `config/grasp_pipeline.yaml`：

```yaml
voice:
  asr:
    device: 1
    rate: 16000
    channels: 1
  tts:
    engine: "matcha:zh"
    playback_device: 1
    playback_rate: 48000
    channels: 1
    speed: 1.0
    volume: 80
    mixer_volume: 80
```

- `asr.device` 使用采集设备列表中的编号。
- `tts.playback_device` 使用播放设备列表中的编号。
- `tts.mixer_volume` 设置启动时的 alsa `pcm` 音量。设为 `-1` 时不修改 mixer。

录音格式、vad 阈值和 tts 参数见[抓取配置](grasp_config.md#6-配置语音交互)。

## 3. 配置语音命令

在 `config/grasp_pipeline.yaml` 的 `voice` 配置块中维护命令词和目标别名：

- `trigger_words`：触发抓取，例如“抓”或“拿”。
- `cancel_words`：停止当前任务，例如“停止”或“取消”。
- `home_words`：机械臂归位并退出程序，例如“结束”或“回家”。
- `target_aliases`：将 asr 识别文本映射为检测模型类别名，例如将“香蕉”映射为 `banana`。

`target_aliases` 的值必须与检测模型类别名一致。完整字段和默认命令词见[抓取配置](grasp_config.md#6-配置语音交互)。

## 4. 启动语音桥

**安全提示：** 语音命令会触发真实底盘和机械臂动作。启动前清理机器人周围障碍物，并确保可以随时切断驱动电源。

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/local_voice_bridge.py \
  --config config/grasp_pipeline.yaml \
  --binary build/perceptive_grasp
```

语音桥自动使用 `--voice-stdin --status-stdout` 启动抓取进程，并在 pipeline 就绪后启动 tts 和 asr。终端出现以下日志后即可发送语音命令：

```text
[VoiceBridge] Listening: ...
```

支持的命令类型：

- `抓香蕉`：启动香蕉抓取任务。
- `停止`：取消当前任务，机械臂回到观察位并等待下一条命令。
- `结束`、`回家`或 `home`：机械臂回到 home 姿态并退出程序。

## 5. 验证命令解析

使用文本输入模式验证命令解析，无需启动录音、asr 和 tts：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
./build/perceptive_grasp \
  --config config/grasp_pipeline.yaml \
  --voice-stdin \
  --status-stdout
```

在终端输入 `抓香蕉`。程序输出以 `VOICE_STATUS` 开头的状态事件，并进入 `OBSERVING`。输入 `结束` 后，机械臂回到 home 姿态，进程正常退出。

## 6. 处理常见问题

- 未找到采集或播放设备：重新运行环境检查，确认当前用户具有音频设备访问权限，然后更新 `asr.device` 和 `tts.playback_device`。
- 语音桥提示抓取进程未就绪：先根据抓取进程输出处理相机、机械臂、底盘或模型初始化失败。
- 识别文本正确但命令未执行：检查 `trigger_words` 和 `target_aliases`，确认目标别名映射到有效的检测模型类别名。
- 没有语音播报：检查播放设备编号、`tts.volume` 和 `tts.mixer_volume`。

更多运行问题见[故障诊断](debugging.md)。
