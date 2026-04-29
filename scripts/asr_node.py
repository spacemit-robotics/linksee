#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""实时录音 + VAD + ASR + ROS2 发布"""

import argparse
import collections
import queue
import signal
import threading
import numpy as np

import space_audio
from space_audio import AudioCapture
import spacemit_vad
import spacemit_asr

# --- ROS2 ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description="实时语音识别")
    parser.add_argument("-d", "--device", type=int, default=-1, help="音频设备索引 (-1=自动)")
    parser.add_argument("-r", "--rate", type=int, default=16000, help="采集采样率 (default: 16000)")
    parser.add_argument("-c", "--channels", type=int, default=2, help="采集声道数 (default: 2)")
    parser.add_argument("-l", "--list-devices", action="store_true", help="列出音频设备")
    args = parser.parse_args()

    if args.list_devices:
        for idx, name in AudioCapture.list_devices():
            print(f"  [{idx}] {name}")
        return

    # --- ROS2 init ---
    rclpy.init()
    node = Node("asr_publisher")
    pub = node.create_publisher(String, "asr_text", 10)

    # --- VAD ---
    vad_config = (spacemit_vad.VadConfig.preset("silero")
                  .with_trigger_threshold(0.4)
                  .with_stop_threshold(0.3)
                  .with_min_speech_duration(100)
                  .with_smoothing(False))
    vad = spacemit_vad.VadEngine(vad_config)
    print(f"VAD initialized: {vad.engine_name}")

    # --- ASR ---
    asr_config = spacemit_asr.Config()
    asr_config.provider = "cpu"
    asr_config._config.num_threads = 4
    asr_config.language = spacemit_asr.Language.ZH
    asr_config.punctuation = True
    asr = spacemit_asr.Engine(asr_config).initialize()
    print(f"ASR initialized: {asr.backend_name}")

    # warmup
    asr.recognize(np.zeros(16000, dtype=np.float32))
    print("ASR warmup done")

    # --- Resampler ---
    target_rate = 16000
    need_resample = args.rate != target_rate
    resampler = None
    if need_resample:
        resampler = spacemit_asr.Resampler(args.rate, target_rate, channels=1)
        print(f"Resampler: {args.rate} -> {target_rate} Hz")

    # --- 状态 ---
    audio_queue = queue.Queue()
    state = {"in_speech": False}
    speech_buffer = []

    # 环形缓冲
    pre_buf_ms = 800
    pre_buf_max = target_rate * pre_buf_ms // 1000
    pre_buffer = collections.deque()

    running = threading.Event()
    running.set()

    frame_count = [0]

    def on_audio(data: bytes):
        try:
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

            # stereo → mono
            if args.channels > 1:
                samples = samples.reshape(-1, args.channels).mean(axis=1)

            # resample → 16kHz
            if resampler is not None:
                samples = resampler.process(samples)

            if len(samples) == 0:
                return

            frame_count[0] += 1
            if frame_count[0] <= 3:
                peak = np.max(np.abs(samples))
                print(f"[DEBUG] frame {frame_count[0]}: {len(samples)} samples, peak={peak:.4f}")

            # VAD
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

            # 维护环形缓冲
            if not state["in_speech"]:
                pre_buffer.append(samples.copy())
                while sum(len(x) for x in pre_buffer) > pre_buf_max:
                    pre_buffer.popleft()

        except Exception as e:
            print(f"\n[ERROR] callback: {e}", flush=True)
            import traceback
            traceback.print_exc()

    # --- Audio capture ---
    space_audio.init(
        sample_rate=args.rate,
        channels=args.channels,
        chunk_size=args.rate * args.channels * 2 // 25,
        capture_device=args.device,
    )

    capture = AudioCapture()
    capture.set_callback(on_audio)
    capture.start()

    print(f"正在监听 (device={args.device}, {args.rate}Hz, {args.channels}ch)... Ctrl+C 退出")

    # --- 信号处理 ---
    def stop_handler(sig, frame):
        running.clear()
        audio_queue.put(None)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    # --- 主循环 ---
    try:
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

                # --- 发布 ROS2 ---
                msg = String()
                msg.data = text
                pub.publish(msg)

                # 非阻塞 spin
                rclpy.spin_once(node, timeout_sec=0)

    finally:
        print("\n退出中...")
        capture.stop()
        asr.shutdown()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
