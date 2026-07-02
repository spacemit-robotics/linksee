#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""非 ROS 本地语音桥：ASR stdin -> perceptive_grasp -> stdout status -> TTS."""

import argparse
import collections
import queue
import re
import signal
import subprocess
import threading
import time
from typing import Dict, Iterable, List, Optional

from asr_node import load_voice_config
from tts_node import (
    engine_to_preset,
    make_reverse_aliases,
    parse_status_event,
    run_tts_worker,
    status_to_speech,
)


STATUS_PREFIX = "VOICE_STATUS\t"
VOICE_BRIDGE_VERSION = "2026-07-01-asr-rate-probe-v3"
CAPTURE_RATE_CANDIDATES = (48000, 44100, 32000, 16000, 8000)
WAITING_PROMPT = "请继续说要抓取的物体。"


def build_grasp_command(binary: str, config: str,
                        extra_args: Optional[Iterable[str]] = None) -> List[str]:
    """Build the perceptive_grasp command used by the local voice bridge."""
    cmd = [
        binary,
        "--config",
        config,
        "--voice-stdin",
        "--status-stdout",
    ]
    if extra_args:
        cmd.extend(extra_args)
    return cmd


def extract_status_event(line: str) -> Optional[str]:
    """Return a status event from a prefixed stdout line."""
    if not line.startswith(STATUS_PREFIX):
        return None
    return line[len(STATUS_PREFIX):].strip()


def parse_status_fields(event: str) -> Dict[str, str]:
    return parse_status_event(event)


def status_requests_bridge_exit(event: str) -> bool:
    fields = parse_status_fields(event)
    return (
        fields.get("state") == "IDLE"
        and fields.get("message") == "Home position reached; exiting"
    )


def make_running_event() -> threading.Event:
    running = threading.Event()
    running.set()
    running.shutdown_requested = False
    return running


def _queue_stop(text_queue) -> None:
    try:
        text_queue.put_nowait(None)
    except queue.Full:
        pass


def request_shutdown(proc, running, text_queue) -> None:
    if getattr(running, "shutdown_requested", False):
        return
    running.shutdown_requested = True
    running.clear()
    if proc.poll() is None:
        proc.terminate()
    _queue_stop(text_queue)


def _clear_text_queue(text_queue) -> int:
    dropped = 0
    while True:
        try:
            text_queue.get_nowait()
            dropped += 1
        except queue.Empty:
            return dropped


def queue_tts_update(text_queue, speech: str) -> bool:
    """Queue only the latest useful status speech.

    The TTS engine is slower than the grasp state machine. Keeping every state
    causes old failures or prompts to be spoken after a later grasp has started.
    """
    if speech == WAITING_PROMPT and not text_queue.empty():
        print(f"[VoiceBridge] Drop stale waiting prompt: {speech}", flush=True)
        return False

    dropped = 0
    if speech != WAITING_PROMPT:
        dropped = _clear_text_queue(text_queue)
        if dropped:
            print(
                f"[VoiceBridge] Drop {dropped} stale TTS message(s)",
                flush=True,
            )

    try:
        text_queue.put_nowait(speech)
        return True
    except queue.Full:
        print(f"[VoiceBridge] TTS queue full, drop: {speech}", flush=True)
        return False


def _print_stream(prefix: str, stream):
    for line in iter(stream.readline, ""):
        if not line:
            break
        print(f"{prefix}{line.rstrip()}", flush=True)


def _read_grasp_stdout(proc, text_queue, running, reverse_aliases,
                       speak_all_states, status_queue=None):
    last_spoken = ""
    for line in iter(proc.stdout.readline, ""):
        if not line:
            break
        line = line.rstrip()
        event = extract_status_event(line)
        if event is None:
            print(line, flush=True)
            continue
        print(line, flush=True)
        if status_queue is not None:
            try:
                status_queue.put_nowait(event)
            except queue.Full:
                pass
        should_exit = status_requests_bridge_exit(event)
        speech = status_to_speech(event, reverse_aliases, speak_all_states)
        if not speech or speech == last_spoken:
            if should_exit:
                request_shutdown(proc, running, text_queue)
                break
            continue
        last_spoken = speech
        if queue_tts_update(text_queue, speech):
            print(f"[VoiceBridge] Queue TTS: {speech}", flush=True)
        if should_exit:
            request_shutdown(proc, running, text_queue)
            break
    running.clear()


def _send_command(proc, command_text: str) -> bool:
    if proc.poll() is not None or proc.stdin is None:
        return False
    proc.stdin.write(command_text.strip() + "\n")
    proc.stdin.flush()
    return True


def default_asr_channels(asr_cfg) -> int:
    return int(asr_cfg.get("channels", 1))


def _capture_hw_name_from_arecord(device: int,
                                  runner=subprocess.run) -> Optional[str]:
    try:
        result = runner(
            ["arecord", "-l"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"[VoiceBridge] Cannot run arecord -l: {exc}", flush=True)
        return None
    if result.returncode != 0:
        print("[VoiceBridge] arecord -l failed", flush=True)
        return None

    for line in (result.stdout or "").splitlines():
        match = re.search(r"card\s+(\d+):.*device\s+(\d+):", line)
        if not match:
            continue
        card = int(match.group(1))
        pcm_device = int(match.group(2))
        if card == device:
            return f"hw:{card},{pcm_device}"
    return None


def capture_hw_name(device: int, list_devices,
                    runner=subprocess.run) -> Optional[str]:
    if device < 0:
        return None
    try:
        devices = list_devices()
    except Exception as exc:
        print(f"[VoiceBridge] Cannot list capture devices: {exc}", flush=True)
        return None
    for index, name in devices:
        if int(index) != device:
            continue
        match = re.search(r"\((hw:\d+,\d+)\)", name)
        if match:
            return match.group(1)

    hw_name = _capture_hw_name_from_arecord(device, runner)
    if hw_name is None:
        print(
            f"[VoiceBridge] Cannot resolve ALSA hw name for capture device {device}",
            flush=True,
        )
    return hw_name


def _capture_rate_supported(hw_name: str, rate: int, channels: int,
                            runner=subprocess.run) -> bool:
    command = [
        "arecord",
        "-D",
        hw_name,
        "-f",
        "S16_LE",
        "-c",
        str(channels),
        "-r",
        str(rate),
        "-d",
        "1",
        "/dev/null",
    ]
    try:
        result = runner(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"[VoiceBridge] Capture rate probe skipped: {exc}", flush=True)
        return False
    return result.returncode == 0


def candidate_capture_rates(requested_rate: int) -> List[int]:
    rates = [requested_rate]
    rates.extend(rate for rate in CAPTURE_RATE_CANDIDATES
                 if rate != requested_rate)
    return rates


def _stop_capture_quietly(capture) -> None:
    for method_name in ("stop", "close"):
        method = getattr(capture, method_name, None)
        if method is None:
            continue
        try:
            method()
        except Exception as exc:
            print(
                f"[VoiceBridge] Ignore AudioCapture.{method_name} error: {exc}",
                flush=True,
            )


def resolve_spacemit_capture_rate(device: int, requested_rate: int,
                                  channels: int, audio_module,
                                  capture_cls) -> int:
    print(
        "[VoiceBridge] Probe SpaceMIT ASR capture rates: "
        f"device={device}, requested={requested_rate}Hz",
        flush=True,
    )
    for rate in candidate_capture_rates(requested_rate):
        capture = None
        try:
            audio_module.init(
                sample_rate=rate,
                channels=channels,
                chunk_size=rate * channels * 2 // 25,
                capture_device=device,
            )
            capture = capture_cls()
            capture.set_callback(lambda data: None)
            started = bool(capture.start())
        except Exception as exc:
            started = False
            print(
                "[VoiceBridge] Probe SpaceMIT ASR capture: "
                f"device={device}, rate={rate}Hz -> fail ({exc})",
                flush=True,
            )
        else:
            print(
                "[VoiceBridge] Probe SpaceMIT ASR capture: "
                f"device={device}, rate={rate}Hz -> "
                f"{'ok' if started else 'fail'}",
                flush=True,
            )
        finally:
            if capture is not None:
                _stop_capture_quietly(capture)

        if started:
            if rate != requested_rate:
                print(
                    "[VoiceBridge] ASR capture rate fallback: "
                    f"{requested_rate} -> {rate} Hz",
                    flush=True,
                )
            return rate

    print(
        "[VoiceBridge] No probed SpaceMIT ASR capture rate worked; "
        f"keep configured rate {requested_rate} Hz",
        flush=True,
    )
    return requested_rate


def resolve_capture_rate(device: int, requested_rate: int, channels: int,
                         list_devices, runner=subprocess.run) -> int:
    hw_name = capture_hw_name(device, list_devices, runner)
    if hw_name is None:
        print(
            "[VoiceBridge] ASR capture rate probe skipped: "
            f"device={device}, configured rate={requested_rate}Hz",
            flush=True,
        )
        return requested_rate

    print(
        "[VoiceBridge] Probe ASR capture rates: "
        f"device={device}, hw={hw_name}, requested={requested_rate}Hz",
        flush=True,
    )
    for rate in candidate_capture_rates(requested_rate):
        if _capture_rate_supported(hw_name, rate, channels, runner):
            if rate != requested_rate:
                print(
                    "[VoiceBridge] ASR capture rate fallback: "
                    f"{requested_rate} -> {rate} Hz on {hw_name}",
                    flush=True,
                )
            return rate

    print(
        "[VoiceBridge] No probed ASR capture rate worked; "
        f"keep configured rate {requested_rate} Hz on {hw_name}",
        flush=True,
    )
    return requested_rate


def wait_for_grasp_ready(proc, status_queue, running,
                         timeout_sec: float = 20.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while running.is_set() and time.monotonic() < deadline:
        if proc.poll() is not None:
            running.clear()
            return False
        try:
            event = status_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        fields = parse_status_fields(event)
        state = fields.get("state", "")
        message = fields.get("message", "")
        if state == "IDLE" and message == "Ready":
            return True
        if state == "ERROR" or message == "Stopped":
            running.clear()
            return False
    running.clear()
    return False


def run_asr_loop(args, proc, running):
    import numpy as np
    import spacemit_asr
    import spacemit_audio
    import spacemit_vad
    from spacemit_audio import AudioCapture

    voice_cfg = load_voice_config(args.config)
    asr_cfg = voice_cfg.get("asr", {}) or {}
    device = args.device if args.device is not None else int(asr_cfg.get("device", -1))
    rate = args.rate if args.rate is not None else int(asr_cfg.get("rate", 16000))
    channels = (
        args.channels if args.channels is not None
        else default_asr_channels(asr_cfg)
    )
    rate = resolve_spacemit_capture_rate(
        device,
        rate,
        channels,
        spacemit_audio,
        AudioCapture,
    )
    print(
        "[VoiceBridge] ASR config: "
        f"device={device}, {rate}Hz, channels={channels}",
        flush=True,
    )
    trigger = (
        args.vad_trigger_threshold if args.vad_trigger_threshold is not None
        else float(asr_cfg.get("vad_trigger_threshold", 0.4))
    )
    stop = (
        args.vad_stop_threshold if args.vad_stop_threshold is not None
        else float(asr_cfg.get("vad_stop_threshold", 0.3))
    )
    min_speech_ms = (
        args.vad_min_speech_duration_ms
        if args.vad_min_speech_duration_ms is not None
        else int(asr_cfg.get("vad_min_speech_duration_ms", 100))
    )

    vad_config = (
        spacemit_vad.VadConfig.preset("silero")
        .with_trigger_threshold(trigger)
        .with_stop_threshold(stop)
        .with_min_speech_duration(min_speech_ms)
        .with_smoothing(False)
    )
    vad = spacemit_vad.VadEngine(vad_config)
    print(f"[VoiceBridge] VAD initialized: {vad.engine_name}", flush=True)

    asr_config = spacemit_asr.Config()
    asr_config.provider = "cpu"
    asr_config._config.num_threads = args.asr_threads
    asr_config.language = spacemit_asr.Language.ZH
    asr_config.punctuation = True
    asr = spacemit_asr.Engine(asr_config).initialize()
    print(f"[VoiceBridge] ASR initialized: {asr.backend_name}", flush=True)
    asr.recognize(np.zeros(16000, dtype=np.float32))
    print("[VoiceBridge] ASR warmup done", flush=True)

    target_rate = 16000
    resampler = None
    if rate != target_rate:
        resampler = spacemit_asr.Resampler(rate, target_rate, channels=1)
        print(f"[VoiceBridge] Resampler: {rate} -> {target_rate} Hz", flush=True)

    audio_queue = queue.Queue()
    state = {"in_speech": False}
    speech_buffer = []
    pre_buffer = collections.deque()
    pre_buf_max = target_rate * args.pre_buffer_ms // 1000

    def on_audio(data: bytes):
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        if channels > 1:
            samples = samples.reshape(-1, channels).mean(axis=1)
        if resampler is not None:
            samples = resampler.process(samples)
        if len(samples) == 0:
            return

        result = vad.detect(samples, target_rate)
        if result is None:
            return

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
            print(f"\r[VAD] 语音结束 ({dur:.1f}s)", flush=True)
            audio_queue.put(audio)

        if not state["in_speech"]:
            pre_buffer.append(samples.copy())
            while sum(len(x) for x in pre_buffer) > pre_buf_max:
                pre_buffer.popleft()

    spacemit_audio.init(
        sample_rate=rate,
        channels=channels,
        chunk_size=rate * channels * 2 // 25,
        capture_device=device,
    )
    capture = AudioCapture()
    capture.set_callback(on_audio)
    if not capture.start():
        print(
            "[VoiceBridge] ASR capture failed to start: "
            f"device={device}, {rate}Hz, {channels}ch",
            flush=True,
        )
        running.clear()
        return
    print(
        f"[VoiceBridge] Listening: device={device}, {rate}Hz, {channels}ch",
        flush=True,
    )

    try:
        while running.is_set() and proc.poll() is None:
            try:
                audio = audio_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            result = asr.recognize(audio)
            if not result or result.is_empty:
                continue
            text = result.text.strip()
            if not text:
                continue
            print(f"[ASR] {text}  (RTF={result.rtf:.2f})", flush=True)
            if not _send_command(proc, text):
                running.clear()
                break
    finally:
        capture.stop()
        asr.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(
        description="perceptive_grasp 非 ROS 本地语音输入/输出桥"
    )
    parser.add_argument("--config", default="config/grasp_pipeline.yaml")
    parser.add_argument("--binary", default="build/perceptive_grasp")
    parser.add_argument("--grasp-arg", action="append", default=[],
                        help="透传给 perceptive_grasp 的额外参数，可重复")
    parser.add_argument("-d", "--device", type=int, default=None)
    parser.add_argument("-r", "--rate", type=int, default=None)
    parser.add_argument("-c", "--channels", type=int, choices=[1, 2],
                        default=None)
    parser.add_argument("--vad-trigger-threshold", type=float, default=None)
    parser.add_argument("--vad-stop-threshold", type=float, default=None)
    parser.add_argument("--vad-min-speech-duration-ms", type=int, default=None)
    parser.add_argument("--pre-buffer-ms", type=int, default=800)
    parser.add_argument("--asr-threads", type=int, default=4)
    parser.add_argument("--tts-engine", default=None)
    parser.add_argument("--tts-device", type=int, default=None)
    parser.add_argument("--tts-rate", type=int, default=None)
    parser.add_argument("--tts-channels", type=int, choices=[1, 2],
                        default=None)
    parser.add_argument("--tts-speed", type=float, default=None)
    parser.add_argument("--tts-volume", type=int, default=None)
    parser.add_argument("--speak-all", action="store_true")
    parser.add_argument("--no-play", action="store_true")
    parser.add_argument("--startup-timeout-sec", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    voice_cfg = load_voice_config(args.config)
    tts_cfg = voice_cfg.get("tts", {}) or {}
    reverse_aliases = make_reverse_aliases(voice_cfg.get("target_aliases", {}) or {})

    preset = engine_to_preset(args.tts_engine or tts_cfg.get("engine", "matcha:zh"))
    playback_device = (
        args.tts_device if args.tts_device is not None
        else int(tts_cfg.get("playback_device", -1))
    )
    playback_rate = (
        args.tts_rate if args.tts_rate is not None
        else int(tts_cfg.get("playback_rate", 48000))
    )
    tts_channels = (
        args.tts_channels if args.tts_channels is not None
        else int(tts_cfg.get("channels", 1))
    )
    speed = (
        args.tts_speed if args.tts_speed is not None
        else float(tts_cfg.get("speed", 1.0))
    )
    volume = (
        args.tts_volume if args.tts_volume is not None
        else int(tts_cfg.get("volume", 80))
    )
    mixer_volume = int(tts_cfg.get("mixer_volume", -1))
    speak_all_states = (
        args.speak_all or bool(tts_cfg.get("speak_all_states", False))
    )
    tts_enabled = bool(tts_cfg.get("enabled", True))

    cmd = build_grasp_command(args.binary, args.config, args.grasp_arg)
    print(f"[VoiceBridge] Version: {VOICE_BRIDGE_VERSION}", flush=True)
    print("[VoiceBridge] Start: " + " ".join(cmd), flush=True)
    print(
        "[VoiceBridge] TTS config: "
        f"enabled={tts_enabled}, no_play={args.no_play}, "
        f"device={playback_device}, {playback_rate}Hz, "
        f"channels={tts_channels}, engine={preset}",
        flush=True,
    )
    proc = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        start_new_session=True,
    )

    running = make_running_event()
    text_queue = queue.Queue(maxsize=8)
    status_queue = queue.Queue(maxsize=16)

    def stop_handler(sig, frame):
        del sig, frame
        request_shutdown(proc, running, text_queue)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    threads = [
        threading.Thread(
            target=_read_grasp_stdout,
            args=(proc, text_queue, running, reverse_aliases, speak_all_states,
                  status_queue),
            daemon=True,
        ),
        threading.Thread(
            target=_print_stream,
            args=("[perceptive_grasp stderr] ", proc.stderr),
            daemon=True,
        ),
    ]
    for thread in threads:
        thread.start()

    if not wait_for_grasp_ready(
            proc, status_queue, running, args.startup_timeout_sec):
        print("[VoiceBridge] perceptive_grasp not ready; stop voice bridge",
              flush=True)
        request_shutdown(proc, running, text_queue)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        for thread in threads:
            thread.join(timeout=2.0)
        return 1

    tts_thread = threading.Thread(
        target=run_tts_worker,
        args=(text_queue, running, preset, playback_device, playback_rate,
              tts_channels, speed, volume, args.no_play, mixer_volume),
        daemon=True,
    )
    threads.append(tts_thread)
    tts_thread.start()

    try:
        if running.is_set():
            run_asr_loop(args, proc, running)
        return 0 if proc.poll() in (None, 0) else proc.poll()
    finally:
        request_shutdown(proc, running, text_queue)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        for thread in threads:
            thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
