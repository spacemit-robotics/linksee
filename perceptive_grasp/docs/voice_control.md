# 语音控制

`perceptive_grasp` 通过本地语音桥接入自动语音识别（ASR）和语音合成（TTS）。语音桥将识别结果发送给抓取进程，并将 pipeline 状态转换为语音播报。语音链路独立于相机和机械臂后端，可用于 Linksee 真机或远程 MuJoCo 仿真。

![语音控制链路](assets/voice-control-flow.svg)

## 1. 准备语音环境

开始前完成以下准备工作：

- 按[方案依赖](sdk_dependencies.md#7-准备语音模型可选)准备 VAD、ASR、TTS 和文本规范化资源。
- 完成 `perceptive_grasp` 构建，并确认 `build/perceptive_grasp` 存在。
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
  echo_cancellation:
    mode: "webrtc_aec"
    delay_ms: 50
    noise_suppression: true
    high_pass_filter: true
  asr:
    backend: "qwen3_asr"
    device: 1
    device_name: "2K USB Camera"
    rate: 16000
    channels: 1
    channel_index: -1
    mixer_volume: 40
  tts:
    enabled: true
    engine: "matcha:zh"
    playback_device: 1
    playback_device_name: "2K USB Camera"
    playback_rate: 48000
    channels: 1
    speed: 1.0
    volume: 80
    mixer_volume: 80
```

- `asr.device_name` 优先按名称匹配采集设备，`asr.device` 作为未配置名称时的编号。
- `asr.channels` 按麦克风原生采集格式填写。当前 2K USB Camera 使用 `16000 Hz/1ch`；其他设备应以环境检查和采集探测结果为准。
- `asr.channel_index` 选择送入 ASR 的物理声道。单声道设备使用 `-1`；双声道设备可选择 `0` 或 `1`，使用 `-1` 时会对两个声道求平均。
- `asr.mixer_volume` 设置启动时的 ALSA 麦克风增益。当前 Rapoo 麦克风使用 `40`，兼顾近距离输入余量和语音信噪比。
- `tts.playback_device_name` 优先按名称匹配播放设备，`tts.playback_device` 作为未配置名称时的编号。
- `tts.mixer_volume` 设置启动时的 ALSA `PCM` 音量。设为 `-1` 时不修改 mixer。
- 音频硬件已提供回声消除后的录音流时，将 `echo_cancellation.mode` 设为 `hardware_aec`；否则使用 `webrtc_aec`。两种模式在 TTS 播放期间都会继续执行 VAD 和 ASR。
- `webrtc_aec` 只在 TTS 播放和回声尾音期间使用软件 AEC 输出，随后切换回麦克风原始输入。启动播报结束前不会接收命令，后续状态播报期间仍可接收语音；单次语音会固定使用同一音频路径，路径切换和 pipeline 恢复等待命令时会重新初始化 VAD。

录音格式、VAD 阈值和 TTS 参数见[抓取配置](grasp_config.md#6-配置语音交互)。

## 3. 选择 ASR 后端

真机配置默认使用 SenseVoice，远程仿真配置默认使用 Qwen3-ASR。需要 Qwen3-ASR 时，先按[方案依赖](sdk_dependencies.md#72-准备-qwen3-asr)安装运行程序并准备模型：

```yaml
voice:
  asr:
    backend: "qwen3_asr"
    qwen3_asr:
      endpoint: "http://127.0.0.1:8063/v1/chat/completions"
      model: "qwen3-asr"
      timeout_sec: 10
      context_max_terms: 0
      max_transcript_chars: 32
      auto_start: true
      model_dir: "~/.cache/models/asr/qwen3asr/qwen3-asr-0.6B-dynq-q40"
      server_threads: 4
      startup_timeout_sec: 90
```

本机端点启用 `auto_start` 后，语音桥会自动启动 `llama-server`，等待服务就绪，并在退出时停止由本次任务启动的服务。若端点已由其他进程提供，语音桥直接复用，不会停止该进程。

`qwen3_asr.endpoint` 也可以指向局域网服务。远程端点不会自动启动，运行前应确保其 `/health` 接口可访问。

使用进程内运行的 sensevoice 时，可通过命令行选择后端：

```bash
./build/perceptive_grasp \
  --voice-control \
  --asr-backend sensevoice \
  --config config/grasp_pipeline.yaml
```

## 4. 配置语音命令

在 `config/grasp_pipeline.yaml` 的 `voice` 配置块中维护命令词和目标别名：

- `trigger_words`：触发抓取，例如“抓”或“拿”。
- `cancel_words`：停止当前任务，例如“停止”或“取消”。
- `home_words`：机械臂归位并退出程序，例如“结束”或“回家”。
- `target_aliases`：将 ASR 识别文本映射为检测模型类别名，例如将“香蕉”映射为 `banana`。

`target_aliases` 的值必须与检测模型类别名一致。完整字段和默认命令词见[抓取配置](grasp_config.md#6-配置语音交互)。

## 5. 启动语音桥

**安全提示：** 语音命令会触发真实底盘和机械臂动作。启动前清理机器人周围障碍物，并确保可以随时切断驱动电源。

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
./build/perceptive_grasp \
  --voice-control \
  --config config/grasp_pipeline.yaml
```

语音桥自动使用 `--voice-stdin --status-stdout` 启动抓取进程，并在 pipeline 就绪后启动 TTS 和 ASR。终端出现以下日志后即可发送语音命令：

```text
[VoiceBridge] Listening: ...
```

支持的命令类型：

- `抓香蕉`：启动香蕉抓取任务。
- `停止`：取消当前任务，机械臂回到观察位并等待下一条命令。
- `结束`、`回家`或 `home`：机械臂回到 home 姿态并退出程序。

### 5.1 控制远程仿真

PC 启动 MuJoCo 仿真服务后，在 K3 运行：

```bash
source ~/.venv-grasp/bin/activate
./build_remote_mujoco/perceptive_grasp \
  --voice-control \
  --config config/grasp_pipeline_remote_mujoco_ur5e.yaml \
  --remote-host <pc_ip>
```

语音设备、VAD、ASR、AEC 和 TTS 均运行在 K3。`remote_mujoco` 只传输 RGB-D 数据和机械臂命令，不传输音频。完整构建和启动步骤见[仿真运行](simulation.md#6-在-k3-启用语音控制)。

## 6. 验证命令解析

使用文本输入模式验证命令解析，无需启动录音、ASR 和 TTS：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
./build/perceptive_grasp \
  --config config/grasp_pipeline.yaml \
  --voice-stdin \
  --status-stdout
```

在终端输入 `抓香蕉`。程序输出以 `VOICE_STATUS` 开头的状态事件，并进入 `OBSERVING`。输入 `结束` 后，机械臂回到 home 姿态，进程正常退出。

## 7. 处理常见问题

- 未找到采集或播放设备：重新运行环境检查，确认当前用户具有音频设备访问权限，然后更新 `asr.device` 和 `tts.playback_device`。
- 语音桥提示抓取进程未就绪：先根据抓取进程输出处理相机、机械臂、底盘或模型初始化失败。
- 识别文本正确但命令未执行：检查 `trigger_words` 和 `target_aliases`，确认目标别名映射到有效的检测模型类别名。
- Qwen3-ASR 初始化失败：本机自动启动时检查 `model_dir`、`llama-server` 版本和日志中给出的 `/tmp/perceptive_grasp_qwen3_asr_<uid>.log`；使用远程端点时检查主机、端口和 `/health` 接口。
- TTS 播报被识别为命令：硬件没有回声消除时确认 `echo_cancellation.mode` 为 `webrtc_aec`，然后根据录音效果微调 `delay_ms`。`half_duplex` 模式会在播报期间暂停 ASR。
- VAD 检测到语音后一直不结束：检查麦克风底噪，并保留 `asr.vad_max_speech_duration_ms` 限制，达到上限后语音桥会自动提交并重置当前语音段。
- 第一次抓取后不再响应：pipeline 返回等待状态时，语音桥会强制结束旧语音段、清空过期 ASR 片段并重置 VAD。若仍未响应，使用 `--audio-debug` 检查后续语音的 `vad_max` 是否达到触发阈值。
- 双声道麦克风漏字或随机误识别：检查启动日志中的 `ASR capture channel`。麦克风阵列不应直接平均存在时延的两个通道，应通过 `asr.channel_index` 固定选择识别清晰的声道。
- 无法从日志判断误识别原因：增加 `--audio-debug --asr-audio-dir /tmp/perceptive_grasp_asr`。程序会输出 VAD 概率、ASR 请求耗时，并保存实际提交给后端的单声道 wav 片段。
- 启动后音频设备与配置不一致：确认没有运行第二个语音桥。程序使用进程锁阻止重复启动，避免已占用的设备从枚举列表中消失并改变设备编号。
- 没有语音播报：检查播放设备编号、`tts.volume` 和 `tts.mixer_volume`。

更多运行问题见[故障诊断](debugging.md)。
