#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""非 ROS 本地语音桥：ASR stdin -> perceptive_grasp -> stdout status -> TTS."""

import argparse
import collections
import ctypes
import ctypes.util
import os
import queue
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
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
READY_LOG_LINE = "[Pipeline] IDLE | Ready"
READY_STATUS_EVENT = "state=IDLE;message=Ready"
VOICE_BRIDGE_VERSION = "2026-07-21-webrtc-aec-v10"
CAPTURE_RATE_CANDIDATES = (48000, 44100, 32000, 16000, 8000)
WAITING_PROMPT = "请继续说要抓取的物体。"
TTS_ECHO_GUARD_MS = 600
WEBRTC_AEC_TAIL_MS = 200
ECHO_CANCELLATION_MODES = ("hardware_aec", "webrtc_aec", "half_duplex")


def load_echo_cancellation_config(voice_cfg):
    config = voice_cfg.get("echo_cancellation", {}) or {}
    mode = str(config.get("mode", "half_duplex")).strip().lower()
    if mode not in ECHO_CANCELLATION_MODES:
        supported = ", ".join(ECHO_CANCELLATION_MODES)
        raise ValueError(
            f"unsupported voice.echo_cancellation.mode={mode!r}; "
            f"expected one of: {supported}"
        )
    return mode, config


def update_aec_tail(frame_end, playback_active, has_reference,
                    tail_until, tail_ms=WEBRTC_AEC_TAIL_MS):
    if playback_active or has_reference:
        return max(tail_until, frame_end + tail_ms / 1000.0)
    return tail_until


class PlaybackReferenceTimeline:
    """Map queued playback samples onto the microphone capture clock."""

    def __init__(self, sample_rate, delay_ms, clock=time.monotonic):
        self.sample_rate = sample_rate
        self.delay_sec = max(0, delay_ms) / 1000.0
        self.clock = clock
        self.lock = threading.Lock()
        self.segments = collections.deque()
        self.next_start_time = 0.0

    def schedule(self, audio, sample_rate, channels):
        import numpy as np

        samples = np.asarray(audio)
        if samples.ndim == 2:
            samples = samples.astype(np.float32).mean(axis=1)
        elif channels > 1:
            samples = samples.reshape(-1, channels).astype(np.float32).mean(axis=1)
        else:
            samples = samples.astype(np.float32).reshape(-1)
        if np.issubdtype(np.asarray(audio).dtype, np.integer):
            samples /= 32768.0
        if sample_rate != self.sample_rate and len(samples) > 0:
            output_length = max(
                1, int(round(len(samples) * self.sample_rate / sample_rate))
            )
            source_x = np.linspace(0.0, 1.0, len(samples), endpoint=False)
            target_x = np.linspace(0.0, 1.0, output_length, endpoint=False)
            samples = np.interp(target_x, source_x, samples).astype(np.float32)

        now = self.clock()
        with self.lock:
            start_time = max(now + self.delay_sec, self.next_start_time)
            self.segments.append((start_time, samples.copy()))
            self.next_start_time = start_time + len(samples) / self.sample_rate
        return start_time

    def read(self, frame_count, end_time=None):
        import numpy as np

        if end_time is None:
            end_time = self.clock()
        start_time = end_time - frame_count / self.sample_rate
        output = np.zeros(frame_count, dtype=np.float32)

        with self.lock:
            while self.segments:
                segment_start, segment = self.segments[0]
                segment_end = segment_start + len(segment) / self.sample_rate
                if segment_end >= start_time - 1.0:
                    break
                self.segments.popleft()

            for segment_start, segment in self.segments:
                segment_end = segment_start + len(segment) / self.sample_rate
                if segment_start >= end_time:
                    break
                overlap_start = max(start_time, segment_start)
                overlap_end = min(end_time, segment_end)
                if overlap_end <= overlap_start:
                    continue
                output_offset = int(round(
                    (overlap_start - start_time) * self.sample_rate
                ))
                segment_offset = int(round(
                    (overlap_start - segment_start) * self.sample_rate
                ))
                count = min(
                    int(round((overlap_end - overlap_start) * self.sample_rate)),
                    frame_count - output_offset,
                    len(segment) - segment_offset,
                )
                if count > 0:
                    output[output_offset:output_offset + count] = (
                        segment[segment_offset:segment_offset + count]
                    )
        return output


def find_voice_aec_library(binary_path, environ=os.environ):
    configured = environ.get("PERCEPTIVE_GRASP_VOICE_AEC_LIB")
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.extend([
        Path(binary_path).expanduser().resolve().parent /
        "libperceptive_voice_aec.so",
        Path(__file__).resolve().parents[1] / "build" /
        "libperceptive_voice_aec.so",
        Path(__file__).resolve().parents[3] / "lib" /
        "libperceptive_voice_aec.so",
    ])
    discovered = ctypes.util.find_library("perceptive_voice_aec")
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return discovered


class WebRtcAecProcessor:
    """ctypes wrapper around the native WebRTC APM frontend."""

    def __init__(self, library_path, sample_rate, noise_suppression=True,
                 high_pass_filter=True):
        self.library = ctypes.CDLL(library_path)
        self.library.voice_aec_create.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        self.library.voice_aec_create.restype = ctypes.c_void_p
        self.library.voice_aec_destroy.argtypes = [ctypes.c_void_p]
        self.library.voice_aec_frame_size.argtypes = [ctypes.c_void_p]
        self.library.voice_aec_frame_size.restype = ctypes.c_int
        self.library.voice_aec_process.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int16),
            ctypes.POINTER(ctypes.c_int16),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_int16),
        ]
        self.library.voice_aec_process.restype = ctypes.c_int
        self.handle = self.library.voice_aec_create(
            sample_rate,
            int(noise_suppression),
            int(high_pass_filter),
        )
        if not self.handle:
            raise RuntimeError(
                f"WebRTC AEC does not support sample rate {sample_rate}"
            )
        self.frame_size = self.library.voice_aec_frame_size(self.handle)

    def process(self, microphone, playback_reference):
        import numpy as np

        microphone_int16 = (
            np.clip(microphone, -1.0, 1.0) * 32767.0
        ).astype(np.int16)
        reference_int16 = (
            np.clip(playback_reference, -1.0, 1.0) * 32767.0
        ).astype(np.int16)
        output = np.empty(self.frame_size, dtype=np.int16)
        result = self.library.voice_aec_process(
            self.handle,
            microphone_int16.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)),
            reference_int16.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)),
            self.frame_size,
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_int16)),
        )
        if result != 0:
            raise RuntimeError(f"WebRTC AEC processing failed: {result}")
        return output.astype(np.float32) / 32768.0

    def close(self):
        if self.handle:
            self.library.voice_aec_destroy(self.handle)
            self.handle = None


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
    prefix_offset = line.find(STATUS_PREFIX)
    if prefix_offset < 0:
        return None
    event = line[prefix_offset + len(STATUS_PREFIX):].strip()
    return event if event.startswith("state=") else None


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
        if event is None and line == READY_LOG_LINE:
            event = READY_STATUS_EVENT
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
        if should_exit:
            request_shutdown(proc, running, text_queue)
            break
        speech = status_to_speech(event, reverse_aliases, speak_all_states)
        if not speech or speech == last_spoken:
            continue
        last_spoken = speech
        if queue_tts_update(text_queue, speech):
            print(f"[VoiceBridge] Queue TTS: {speech}", flush=True)
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


def candidate_capture_channels(device: int,
                               requested_channels: Optional[int]) -> List[int]:
    if requested_channels is not None:
        return [int(requested_channels)]
    if device == 0:
        return [2, 1]
    return [1]


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
    rate, _ = resolve_spacemit_capture_format(
        device,
        requested_rate,
        channels,
        audio_module,
        capture_cls,
    )
    return rate


def resolve_spacemit_capture_format(device: int, requested_rate: int,
                                    requested_channels: Optional[int],
                                    audio_module, capture_cls):
    print(
        "[VoiceBridge] Probe SpaceMIT ASR capture formats: "
        f"device={device}, requested={requested_rate}Hz, "
        f"channels={requested_channels or 'auto'}",
        flush=True,
    )
    for channels in candidate_capture_channels(device, requested_channels):
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
                    f"device={device}, rate={rate}Hz, channels={channels} "
                    f"-> fail ({exc})",
                    flush=True,
                )
            else:
                print(
                    "[VoiceBridge] Probe SpaceMIT ASR capture: "
                    f"device={device}, rate={rate}Hz, channels={channels} -> "
                    f"{'ok' if started else 'fail'}",
                    flush=True,
                )
            finally:
                if capture is not None:
                    _stop_capture_quietly(capture)

            if started:
                if rate != requested_rate or channels != requested_channels:
                    print(
                        "[VoiceBridge] ASR capture format selected: "
                        f"{rate} Hz, {channels} ch",
                        flush=True,
                    )
                return rate, channels

    print(
        "[VoiceBridge] No probed SpaceMIT ASR capture format worked; "
        f"keep configured {requested_rate} Hz, "
        f"{requested_channels or 1} ch",
        flush=True,
    )
    return requested_rate, requested_channels or 1


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


def run_asr_loop(args, proc, running, playback_active=None,
                 playback_reference=None):
    import numpy as np
    import spacemit_asr
    import spacemit_audio
    import spacemit_vad
    from spacemit_audio import AudioCapture

    voice_cfg = load_voice_config(args.config)
    echo_mode, echo_config = load_echo_cancellation_config(voice_cfg)
    asr_cfg = voice_cfg.get("asr", {}) or {}
    device = args.device if args.device is not None else int(asr_cfg.get("device", -1))
    rate = args.rate if args.rate is not None else int(asr_cfg.get("rate", 16000))
    configured_channels = (
        args.channels if args.channels is not None
        else asr_cfg.get("channels")
    )
    channels = int(configured_channels) if configured_channels is not None else None
    rate, channels = resolve_spacemit_capture_format(
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

    aec_processor = None
    if echo_mode == "webrtc_aec":
        library_path = find_voice_aec_library(args.binary)
        if library_path is None:
            print(
                "[VoiceBridge] WebRTC AEC library not found; rebuild with "
                "ENABLE_WEBRTC_AEC=ON",
                flush=True,
            )
            asr.shutdown()
            running.clear()
            return
        try:
            aec_processor = WebRtcAecProcessor(
                library_path,
                target_rate,
                bool(echo_config.get("noise_suppression", True)),
                bool(echo_config.get("high_pass_filter", True)),
            )
        except Exception as exc:
            print(f"[VoiceBridge] WebRTC AEC initialization failed: {exc}",
                  flush=True)
            asr.shutdown()
            running.clear()
            return
        print(
            "[VoiceBridge] WebRTC AEC initialized: "
            f"library={library_path}, frame_size={aec_processor.frame_size}",
            flush=True,
        )

    audio_queue = queue.Queue()
    state = {
        "in_speech": False,
        "playback_suppressed": False,
        "aec_pending": np.empty(0, dtype=np.float32),
        "aec_pending_start": None,
        "aec_tail_until": 0.0,
        "aec_output_active": False,
    }
    speech_buffer = []
    pre_buffer = collections.deque()
    pre_buf_max = target_rate * args.pre_buffer_ms // 1000

    def clear_pending_audio():
        dropped = 0
        while True:
            try:
                audio_queue.get_nowait()
                dropped += 1
            except queue.Empty:
                return dropped

    def reset_speech_segment():
        state["in_speech"] = False
        speech_buffer.clear()
        pre_buffer.clear()

    def is_playback_active():
        return playback_active is not None and playback_active.is_set()

    def apply_software_aec(samples):
        if aec_processor is None:
            return samples
        if playback_reference is None:
            raise RuntimeError("WebRTC AEC playback reference is unavailable")

        callback_end = time.monotonic()
        callback_start = callback_end - len(samples) / target_rate
        if len(state["aec_pending"]) == 0:
            state["aec_pending_start"] = callback_start
        state["aec_pending"] = np.concatenate(
            (state["aec_pending"], samples.astype(np.float32, copy=False))
        )

        processed_frames = []
        frame_size = aec_processor.frame_size
        while len(state["aec_pending"]) >= frame_size:
            frame = state["aec_pending"][:frame_size]
            state["aec_pending"] = state["aec_pending"][frame_size:]
            frame_end = (
                state["aec_pending_start"] + frame_size / target_rate
            )
            reference = playback_reference.read(frame_size, frame_end)
            processed = aec_processor.process(frame, reference)
            has_reference = bool(np.any(np.abs(reference) > 1e-5))
            state["aec_tail_until"] = update_aec_tail(
                frame_end,
                is_playback_active(),
                has_reference,
                state["aec_tail_until"],
            )
            use_aec_output = frame_end <= state["aec_tail_until"]
            if use_aec_output != state["aec_output_active"]:
                state["aec_output_active"] = use_aec_output
                path = "aec" if use_aec_output else "raw microphone"
                print(f"\n[VoiceBridge] ASR audio path: {path}", flush=True)
            processed_frames.append(processed if use_aec_output else frame.copy())
            state["aec_pending_start"] = frame_end
        if not processed_frames:
            return np.empty(0, dtype=np.float32)
        return np.concatenate(processed_frames)

    def on_audio(data: bytes):
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        if channels > 1:
            samples = samples.reshape(-1, channels).mean(axis=1)
        if resampler is not None:
            samples = resampler.process(samples)
        if len(samples) == 0:
            return

        if echo_mode == "webrtc_aec":
            try:
                samples = apply_software_aec(samples)
            except Exception as exc:
                print(f"\n[VoiceBridge] WebRTC AEC failed: {exc}", flush=True)
                running.clear()
                return
            if len(samples) == 0:
                return

        if echo_mode == "half_duplex" and is_playback_active():
            if not state["playback_suppressed"]:
                state["playback_suppressed"] = True
                reset_speech_segment()
                dropped = clear_pending_audio()
                print(
                    "\r[VoiceBridge] ASR paused during TTS playback"
                    f"; dropped_segments={dropped}",
                    flush=True,
                )
            return

        if echo_mode == "half_duplex" and state["playback_suppressed"]:
            state["playback_suppressed"] = False
            reset_speech_segment()
            reset_vad = getattr(vad, "reset", None)
            if callable(reset_vad):
                try:
                    reset_vad()
                except Exception as exc:
                    print(f"[VoiceBridge] VAD reset failed: {exc}", flush=True)
            print("[VoiceBridge] ASR resumed after TTS echo guard", flush=True)

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
        asr.shutdown()
        if aec_processor is not None:
            aec_processor.close()
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
            if echo_mode == "half_duplex" and is_playback_active():
                continue
            result = asr.recognize(audio)
            if not result or result.is_empty:
                continue
            if echo_mode == "half_duplex" and is_playback_active():
                print("[VoiceBridge] Drop ASR result captured during TTS", flush=True)
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
        if aec_processor is not None:
            aec_processor.close()


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
    try:
        echo_mode, echo_config = load_echo_cancellation_config(voice_cfg)
    except ValueError as exc:
        print(f"[VoiceBridge] Invalid configuration: {exc}", flush=True)
        return 2
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
    print(f"[VoiceBridge] Echo cancellation: mode={echo_mode}", flush=True)
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
    playback_active = threading.Event()
    playback_reference = None
    if echo_mode == "webrtc_aec":
        playback_reference = PlaybackReferenceTimeline(
            16000,
            int(echo_config.get("delay_ms", 50)),
        )

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

    if tts_enabled:
        tts_thread = threading.Thread(
            target=run_tts_worker,
            args=(text_queue, running, preset, playback_device, playback_rate,
                  tts_channels, speed, volume, args.no_play, mixer_volume,
                  playback_active,
                  TTS_ECHO_GUARD_MS if echo_mode == "half_duplex" else 0,
                  playback_reference.schedule if playback_reference else None),
            daemon=True,
        )
        threads.append(tts_thread)
        tts_thread.start()
    else:
        print("[VoiceBridge] TTS disabled", flush=True)

    try:
        if running.is_set():
            run_asr_loop(
                args,
                proc,
                running,
                playback_active,
                playback_reference,
            )
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
