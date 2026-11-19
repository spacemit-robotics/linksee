# 语音控制

`perceptive_grasp` 通过本地语音桥接入 ASR 和 TTS：语音桥把识别结果送给抓取进程，并把 pipeline 状态转成播报。语音链路独立于相机和机械臂后端，真机和远程 MuJoCo 仿真均可用。

![语音控制链路](assets/voice-control-flow.svg)

## 1. 准备语音环境

- 按[方案依赖](sdk_dependencies.md#7-准备语音模型可选)准备 VAD、ASR、TTS 和文本规范化资源；
- 按 [README](../README.md#51-构建并安装-k3-程序)构建安装，确认 `$PREFIX/bin/perceptive_grasp` 存在；
- 连接麦克风和扬声器。

按 README 的 K3 配置安装时 `ENABLE_WEBRTC_AEC=OFF`。用本地语音桥前把 `voice.echo_cancellation.mode` 设为 `half_duplex`，或音频设备自带回声消除时选 `hardware_aec`；要用下面示例中的 `webrtc_aec`，需先以 `ENABLE_WEBRTC_AEC=ON` 重新构建安装。

加载环境并确认语音模块可加载（无输出且退出码 `0` 即正常）：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 -c "import spacemit_audio, spacemit_vad, spacemit_asr, spacemit_tts"
```

## 2. 配置音频设备

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

把检查结果中的设备编号写入 `config/grasp_pipeline.yaml`：

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

- `asr.device_name` / `tts.playback_device_name` 优先按名称匹配设备，编号字段仅在未配置名称时生效。
- `asr.channels` 按麦克风原生采集格式填写，`2K USB Camera` 为 `16000 Hz/1ch`，其他设备以环境检查和采集探测结果为准。
- `asr.channel_index` 选择送入 ASR 的物理声道：单声道用 `-1`，双声道可选 `0` 或 `1`，`-1` 时对两声道求平均。
- `asr.mixer_volume` 是启动时的 ALSA 麦克风增益，Rapoo 麦克风用 `40`，兼顾输入余量和信噪比；`tts.mixer_volume` 设置 ALSA `PCM` 音量，`-1` 表示不改 mixer。
- 音频硬件已输出回声消除后的录音流时用 `hardware_aec`，否则用 `webrtc_aec`；两者在 TTS 播放期间都继续跑 VAD 和 ASR。
- `webrtc_aec` 仅在 TTS 播放和回声尾音期间用软件 AEC 输出，随后切回麦克风原始输入。启动播报结束前不接收命令，后续状态播报期间仍可接收；单次语音固定使用同一音频路径，路径切换和 pipeline 恢复等待时会重新初始化 VAD。

录音格式、VAD 阈值和 TTS 参数见[抓取配置](grasp_config.md#6-配置语音交互)。

## 3. 选择 ASR 后端

真机配置默认 SenseVoice，远程仿真配置默认 Qwen3-ASR。要用 Qwen3-ASR，先按[方案依赖](sdk_dependencies.md#72-准备-qwen3-asr)安装运行程序并准备模型：

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

本机端点启用 `auto_start` 后，语音桥自动启动 `llama-server`、等待就绪，并在退出时停掉由本次任务启动的服务；端点已由其他进程提供时直接复用，不会停止该进程。端点也可以是局域网服务，此时不会自动启动，运行前须确保 `/health` 可访问。

后端用进程内 sensevoice 时可直接在命令行选择：

```bash
"$PREFIX/bin/perceptive_grasp" \
  --voice-control \
  --asr-backend sensevoice \
  --config config/grasp_pipeline.yaml
```

## 4. 配置语音命令

在 `config/grasp_pipeline.yaml` 的 `voice` 块中维护命令词和目标别名：`trigger_words` 触发抓取（“抓”“拿”），`cancel_words` 停止当前任务（“停止”“取消”），`home_words` 归位并退出（“结束”“回家”），`target_aliases` 把 ASR 文本映射为检测类别名（“香蕉” → `banana`）。

`target_aliases` 的值必须与检测模型类别名一致。完整字段和默认命令词见[抓取配置](grasp_config.md#6-配置语音交互)。

## 5. 启动语音桥

**安全提示：** 语音命令会触发真实底盘和机械臂动作。启动前清理周围障碍物，并确保能随时切断驱动电源。

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
"$PREFIX/bin/perceptive_grasp" \
  --voice-control \
  --config config/grasp_pipeline.yaml
```

语音桥以 `--voice-stdin --status-stdout` 启动抓取进程，pipeline 就绪后启动 TTS 和 ASR。看到以下日志即可发命令：

```text
[VoiceBridge] Listening: ...
```

支持的语句：`抓香蕉` 启动抓取；`停止` 取消当前任务，机械臂回观察位等待下一条；`结束` / `回家` / `home` 回到 home 姿态并退出。

### 5.1 控制远程仿真

按 [README](../README.md#73-pc-显示k3-远程抓取)启动共享仿真服务、构建 K3 客户端并保持 SSH 转发。该客户端默认不构建 WebRTC AEC：先把 `config/grasp_pipeline_manipulation_remote.yaml` 的 `voice.echo_cancellation.mode` 改为 `half_duplex`，并按实际设备设置录音与播放参数。在应用目录运行：

```bash
source ~/.venv-grasp/bin/activate
./build_manipulation/client/perceptive_grasp \
  --voice-control \
  --config config/grasp_pipeline_manipulation_remote.yaml \
  --remote-host 127.0.0.1
```

语音设备、VAD、ASR 和 TTS 都在 K3 上运行；`remote_mujoco` 只传 RGB-D 数据和抓取任务请求，不传音频，服务通过 `manipulation` 控制仿真机械臂与夹爪。回声处理选项及协议限制见[共享仿真的语音控制](simulation.md#4-共享仿真的语音控制)。

## 6. 验证命令解析

用文本输入模式验证解析，不启动录音、ASR 和 TTS：

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
"$PREFIX/bin/perceptive_grasp" \
  --config config/grasp_pipeline.yaml \
  --voice-stdin \
  --status-stdout
```

输入 `抓香蕉`，程序输出 `VOICE_STATUS` 开头的状态事件并进入 `OBSERVING`；输入 `结束` 后机械臂回 home 姿态，进程正常退出。

## 7. 处理常见问题

| 现象 | 处理 |
| --- | --- |
| 未找到采集或播放设备 | 重跑环境检查，确认当前用户有音频设备访问权限，再更新 `asr.device` 和 `tts.playback_device` |
| 语音桥提示抓取进程未就绪 | 先按抓取进程输出处理相机、机械臂、底盘或模型初始化失败 |
| 识别正确但命令未执行 | 检查 `trigger_words` 和 `target_aliases`，确认别名映射到有效的检测类别名 |
| Qwen3-ASR 初始化失败 | 本机自动启动时检查 `model_dir`、`llama-server` 版本和 `/tmp/perceptive_grasp_qwen3_asr_<uid>.log`；远程端点检查主机、端口和 `/health` |
| TTS 播报被识别为命令 | 无硬件回声消除时确认 `echo_cancellation.mode` 为 `webrtc_aec`，再按录音效果微调 `delay_ms`；`half_duplex` 在播报期间暂停 ASR |
| VAD 检测到语音后不结束 | 检查麦克风底噪，保留 `asr.vad_max_speech_duration_ms` 上限，达到后语音桥自动提交并重置语音段 |
| 第一次抓取后不再响应 | pipeline 返回等待状态时语音桥会结束旧语音段、清空过期 ASR 片段并重置 VAD；仍未响应则用 `--audio-debug` 看后续语音的 `vad_max` 是否达到阈值 |
| 双声道麦克风漏字或随机误识别 | 看启动日志的 `ASR capture channel`；麦克风阵列不应直接平均存在时延的两通道，用 `asr.channel_index` 固定清晰声道 |
| 无法判断误识别原因 | 加 `--audio-debug --asr-audio-dir /tmp/perceptive_grasp_asr`，输出 VAD 概率、ASR 请求耗时并保存提交给后端的单声道 wav |
| 启动后音频设备与配置不一致 | 确认没有第二个语音桥在跑；程序用进程锁阻止重复启动，避免占用中的设备从枚举列表消失并改变编号 |
| 没有语音播报 | 检查播放设备编号、`tts.volume` 和 `tts.mixer_volume` |

更多运行问题见[故障诊断](debugging.md)。
