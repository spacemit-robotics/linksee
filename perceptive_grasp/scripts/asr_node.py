#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""实时录音 + VAD + ASR + ROS2 发布"""

import argparse
import collections
import queue
import signal
import sys
import threading


def import_ros2_or_exit():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        return rclpy, Node, String
    except Exception as exc:
        py_ver = f"{sys.version_info.major}.{sys.version_info.minor}"
        raise SystemExit(
            "\n[ROS2] rclpy 导入失败，当前 Python 是 "
            f"{py_ver}。\n"
            "K3 当前 ROS2 Humble 的 rclpy 使用 Python 3.14 ABI，"
            "请使用与板端 ROS2 ABI 一致的 Python 环境运行语音节点。\n"
            "请按下面方式启动：\n"
            "  cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp\n"
            "  source /opt/ros/humble/setup.zsh\n"
            "  source ~/.venv-grasp/bin/activate\n"
            "  python3 scripts/asr_node.py --config config/grasp_pipeline.yaml\n"
            f"原始错误: {type(exc).__name__}: {exc}"
        ) from exc


def _parse_scalar(value):
    value = value.strip()
    if not value:
        return ""
    if (value[0] == value[-1]) and value[0] in ("'", '"'):
        return value[1:-1]
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def _strip_comment(line):
    in_quote = None
    for i, ch in enumerate(line):
        if ch in ("'", '"'):
            if in_quote == ch:
                in_quote = None
            elif in_quote is None:
                in_quote = ch
        elif ch == "#" and in_quote is None:
            return line[:i]
    return line


def load_voice_config(path):
    try:
        import yaml
    except ImportError:
        yaml = None

    if yaml is not None:
        with open(path, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}
        return root.get("voice", {}) or {}

    voice = {}
    asr = {}
    in_voice = False
    in_asr = False
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = _strip_comment(raw.rstrip("\n"))
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(" "))
            stripped = line.strip()

            if indent == 0:
                in_voice = stripped == "voice:"
                in_asr = False
                continue
            if not in_voice or ":" not in stripped:
                continue

            key, value = stripped.split(":", 1)
            key = key.strip()
            value = value.strip()
            if indent == 2:
                in_asr = key == "asr" and value == ""
                if not in_asr:
                    voice[key] = _parse_scalar(value)
            elif indent == 4 and in_asr:
                asr[key] = _parse_scalar(value)

    if asr:
        voice["asr"] = asr
    return voice


def main():
    parser = argparse.ArgumentParser(description="实时语音识别")
    parser.add_argument("--config", help="读取 grasp_pipeline.yaml 中的 voice/asr 配置")
    parser.add_argument(
        "-d",
        "--device",
        type=int,
        default=None,
        help="音频设备索引，填写 --list-devices 输出方括号里的数字 (-1=自动选择默认输入)",
    )
    parser.add_argument("-r", "--rate", type=int, default=None, help="采集采样率")
    parser.add_argument("-c", "--channels", type=int, default=None, help="采集声道数")
    parser.add_argument("-t", "--topic", default=None, help="ROS2 std_msgs/String 发布话题")
    parser.add_argument("--vad-trigger-threshold", type=float, default=None)
    parser.add_argument("--vad-stop-threshold", type=float, default=None)
    parser.add_argument("--vad-min-speech-duration-ms", type=int, default=None)
    parser.add_argument("-l", "--list-devices", action="store_true", help="列出音频设备")
    args = parser.parse_args()

    voice_cfg = {}
    asr_cfg = {}
    if args.config:
        voice_cfg = load_voice_config(args.config)
        asr_cfg = voice_cfg.get("asr", {}) or {}

    device = args.device if args.device is not None else int(asr_cfg.get("device", -1))
    rate = args.rate if args.rate is not None else int(asr_cfg.get("rate", 16000))
    channels = args.channels if args.channels is not None else int(asr_cfg.get("channels", 2))
    topic = args.topic if args.topic is not None else voice_cfg.get("input_topic", "asr_text")
    vad_trigger_threshold = (
        args.vad_trigger_threshold if args.vad_trigger_threshold is not None
        else float(asr_cfg.get("vad_trigger_threshold", 0.4))
    )
    vad_stop_threshold = (
        args.vad_stop_threshold if args.vad_stop_threshold is not None
        else float(asr_cfg.get("vad_stop_threshold", 0.3))
    )
    vad_min_speech_duration_ms = (
        args.vad_min_speech_duration_ms
        if args.vad_min_speech_duration_ms is not None
        else int(asr_cfg.get("vad_min_speech_duration_ms", 100))
    )

    audio_queue = queue.Queue()
    running = threading.Event()
    running.set()

    def stop_handler(sig, frame):
        del sig, frame
        running.clear()
        audio_queue.put(None)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    capture = None
    asr = None
    node = None
    rclpy_initialized = False

    try:
        import numpy as np

        import spacemit_audio
        from spacemit_audio import AudioCapture
        import spacemit_vad
        import spacemit_asr

        if args.list_devices:
            for idx, name in AudioCapture.list_devices():
                print(f"  [{idx}] {name}")
            return

        rclpy, Node, String = import_ros2_or_exit()

        rclpy.init()
        rclpy_initialized = True
        node = Node("asr_publisher")
        pub = node.create_publisher(String, topic, 10)

        vad_config = (
            spacemit_vad.VadConfig.preset("silero")
            .with_trigger_threshold(vad_trigger_threshold)
            .with_stop_threshold(vad_stop_threshold)
            .with_min_speech_duration(vad_min_speech_duration_ms)
            .with_smoothing(False)
        )
        vad = spacemit_vad.VadEngine(vad_config)
        print(f"VAD initialized: {vad.engine_name}")

        asr_config = spacemit_asr.Config()
        asr_config.provider = "cpu"
        asr_config._config.num_threads = 4
        asr_config.language = spacemit_asr.Language.ZH
        asr_config.punctuation = True
        asr = spacemit_asr.Engine(asr_config).initialize()
        print(f"ASR initialized: {asr.backend_name}")

        asr.recognize(np.zeros(16000, dtype=np.float32))
        print("ASR warmup done")
        if not running.is_set():
            return

        target_rate = 16000
        need_resample = rate != target_rate
        resampler = None
        if need_resample:
            resampler = spacemit_asr.Resampler(rate, target_rate, channels=1)
            print(f"Resampler: {rate} -> {target_rate} Hz")

        state = {"in_speech": False}
        speech_buffer = []

        pre_buf_ms = 800
        pre_buf_max = target_rate * pre_buf_ms // 1000
        pre_buffer = collections.deque()

        frame_count = [0]

        def on_audio(data: bytes):
            try:
                samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                if channels > 1:
                    samples = samples.reshape(-1, channels).mean(axis=1)

                if resampler is not None:
                    samples = resampler.process(samples)

                if len(samples) == 0:
                    return

                frame_count[0] += 1
                if frame_count[0] <= 3:
                    peak = np.max(np.abs(samples))
                    print(
                        f"[DEBUG] frame {frame_count[0]}: "
                        f"{len(samples)} samples, peak={peak:.4f}"
                    )

                result = vad.detect(samples, target_rate)
                if result is None:
                    if frame_count[0] <= 3:
                        print("[DEBUG] vad.detect returned None")
                    return

                if frame_count[0] <= 3:
                    print(f"[DEBUG] vad state={result.state}, prob={result.probability:.3f}")

                if result.is_speech_start:
                    state["in_speech"] = True
                    speech_buffer.clear()

                    if pre_buffer:
                        speech_buffer.append(np.concatenate(list(pre_buffer)))

                    speech_buffer.append(samples.copy())
                    print("\r[VAD] 检测到语音...", end="", flush=True)

                elif state["in_speech"] and not result.is_speech_end:
                    speech_buffer.append(samples.copy())

                elif result.is_speech_end and state["in_speech"]:
                    speech_buffer.append(samples.copy())
                    audio = np.concatenate(speech_buffer)

                    speech_buffer.clear()
                    state["in_speech"] = False

                    dur = len(audio) / target_rate
                    print(f"\r[VAD] 语音结束 ({dur:.1f}s), 队列={audio_queue.qsize()+1}", flush=True)

                    audio_queue.put(audio)

                if not state["in_speech"]:
                    pre_buffer.append(samples.copy())
                    while sum(len(x) for x in pre_buffer) > pre_buf_max:
                        pre_buffer.popleft()

            except Exception as exc:
                print(f"\n[ERROR] callback: {exc}", flush=True)
                import traceback

                traceback.print_exc()

        spacemit_audio.init(
            sample_rate=rate,
            channels=channels,
            chunk_size=rate * channels * 2 // 25,
            capture_device=device,
        )
        if not running.is_set():
            return

        capture = AudioCapture()
        capture.set_callback(on_audio)
        capture.start()
        if not running.is_set():
            return

        print(f"正在监听 (device={device}, {rate}Hz, {channels}ch, topic={topic})... Ctrl+C 退出")

        while running.is_set():
            try:
                audio = audio_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if audio is None:
                break

            result = asr.recognize(audio)

            if result and not result.is_empty:
                text = result.text
                print(f"[ASR] {text}  (RTF={result.rtf:.2f})")

                msg = String()
                msg.data = text
                pub.publish(msg)

                rclpy.spin_once(node, timeout_sec=0)

    except ImportError as exc:
        raise SystemExit(
            "ASR 依赖未安装。\n"
            "安装: pip install -r requirements.txt\n"
            f"原始错误: {type(exc).__name__}: {exc}"
        ) from exc

    except KeyboardInterrupt:
        running.clear()

    finally:
        if (
            capture is not None
            or asr is not None
            or node is not None
            or rclpy_initialized
            or not running.is_set()
        ):
            print("\n退出中...")
        if capture is not None:
            try:
                capture.stop()
            except Exception as exc:
                print(f"[WARN] capture.stop failed: {exc}", flush=True)
        if asr is not None:
            try:
                asr.shutdown()
            except Exception as exc:
                print(f"[WARN] asr.shutdown failed: {exc}", flush=True)
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                print(f"[WARN] node.destroy_node failed: {exc}", flush=True)
        if rclpy_initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as exc:
                print(f"[WARN] rclpy.shutdown skipped: {exc}", flush=True)


if __name__ == "__main__":
    main()
