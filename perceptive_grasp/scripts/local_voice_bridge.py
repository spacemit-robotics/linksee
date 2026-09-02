#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""非 ROS 本地语音桥：ASR stdin -> perceptive_grasp -> stdout status -> TTS."""

import argparse
import collections
import ctypes
import ctypes.util
import fcntl
import os
import queue
import re
import signal
import subprocess
import threading
import time
import wave
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from asr_backend import (
    create_asr_engine,
    normalize_asr_backend,
    select_capture_channel,
)
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
VOICE_BRIDGE_VERSION = "2026-08-13-audio-epoch-v19"
CAPTURE_RATE_CANDIDATES = (48000, 44100, 32000, 16000, 8000)
WAITING_PROMPT = "请继续说要抓取的物体。"
TTS_ECHO_GUARD_MS = 600
WEBRTC_AEC_TAIL_MS = 200
DEFAULT_MAX_SPEECH_DURATION_MS = 8000
DEFAULT_VAD_ENDPOINT_HOLD_MS = 600
ASR_HOTWORD_BOOST = 2.0
ECHO_CANCELLATION_MODES = ("hardware_aec", "webrtc_aec", "half_duplex")
MAX_PENDING_ASR_SEGMENTS = 2
MAX_ASR_SEGMENT_AGE_SEC = 5.0


class AudioEpoch:
    """Identify audio captured within the current command-ready period."""

    def __init__(self):
        self._value = 0
        self._lock = threading.Lock()

    def current(self):
        with self._lock:
            return self._value

    def advance(self):
        with self._lock:
            self._value += 1
            return self._value


def resolve_audio_device(configured_index, configured_name, list_devices, role):
    """Resolve a stable audio device name to the current numeric index."""
    name = str(configured_name or "").strip()
    if not name:
        return int(configured_index)
    normalized = name.casefold()
    matches = [
        (int(index), str(description))
        for index, description in list_devices()
        if normalized in str(description).casefold()
    ]
    if not matches:
        raise RuntimeError(f"{role} audio device not found by name: {name!r}")
    if len(matches) > 1:
        exact = [item for item in matches if item[1].casefold() == normalized]
        if len(exact) == 1:
            matches = exact
        else:
            descriptions = ", ".join(
                f"{index}:{description}" for index, description in matches
            )
            raise RuntimeError(
                f"{role} audio device name is ambiguous: {name!r}; "
                f"matches: {descriptions}"
            )
    index, description = matches[0]
    print(
        f"[VoiceBridge] {role} device resolved: "
        f"name={name!r} -> {index}: {description}",
        flush=True,
    )
    return index


def acquire_voice_bridge_lock(path=None):
    """Prevent concurrent bridges from remapping busy audio device indexes."""
    lock_path = Path(path or f"/tmp/perceptive_grasp_voice_{os.getuid()}.lock")
    lock_file = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        return None
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


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


def update_aec_tail(
    frame_end, playback_active, has_reference, tail_until, tail_ms=WEBRTC_AEC_TAIL_MS
):
    if playback_active or has_reference:
        return max(tail_until, frame_end + tail_ms / 1000.0)
    return tail_until


def should_rearm_vad_for_audio_path(previous_path, current_path, in_speech):
    """Return whether an idle VAD stream changed audio processing path."""
    return not in_speech and previous_path is not None and current_path != previous_path


def select_speech_samples(
    in_speech, raw_samples, aec_samples, current_path=None, prefer_aec=False
):
    """Keep an active utterance on one audio path.

    Switching from AEC output to raw microphone samples in the middle of an
    utterance creates a discontinuity that can split or corrupt ASR input.
    """
    if in_speech and current_path == "aec":
        return aec_samples, "aec"
    if in_speech and current_path == "raw":
        return raw_samples, "raw"
    if prefer_aec and aec_samples is not None:
        return aec_samples, "aec"
    return raw_samples, "raw"


def classify_vad_event(in_speech, result):
    """Normalize VAD events without restarting an active utterance."""
    if in_speech:
        if result is not None and result.is_speech_end:
            return "finish"
        return "append"
    if result is not None and result.is_speech_start:
        return "start"
    return "idle"


def save_asr_audio_segment(directory, sequence, audio, sample_rate=16000):
    """Save the exact mono waveform submitted to an ASR backend."""
    import numpy as np

    output_dir = Path(directory).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"segment_{int(sequence):04d}.wav"
    normalized = np.clip(np.asarray(audio, dtype=np.float32), -1.0, 1.0)
    pcm = (normalized * 32767.0).astype("<i2")
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(int(sample_rate))
        output.writeframes(pcm.tobytes())
    return path


def update_vad_endpoint_hold(
    in_speech, result, pending_samples, chunk_samples, resume_threshold=0.3
):
    """Delay utterance completion so short pauses do not split commands."""
    if not in_speech:
        return 0
    if result is not None and result.is_speech_end:
        return chunk_samples
    if pending_samples > 0:
        if result is not None and result.is_speech_start:
            probability = float(getattr(result, "probability", 1.0) or 0.0)
            if probability >= resume_threshold:
                return 0
        return pending_samples + chunk_samples
    if result is not None and result.is_speech_start:
        return 0
    return 0


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
                output_offset = int(
                    round((overlap_start - start_time) * self.sample_rate)
                )
                segment_offset = int(
                    round((overlap_start - segment_start) * self.sample_rate)
                )
                count = min(
                    int(round((overlap_end - overlap_start) * self.sample_rate)),
                    frame_count - output_offset,
                    len(segment) - segment_offset,
                )
                if count > 0:
                    output[output_offset : output_offset + count] = segment[
                        segment_offset : segment_offset + count
                    ]
        return output


def find_voice_aec_library(binary_path, environ=os.environ):
    configured = environ.get("PERCEPTIVE_GRASP_VOICE_AEC_LIB")
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.extend(
        [
            Path(binary_path).expanduser().resolve().parent
            / "libperceptive_voice_aec.so",
            Path(__file__).resolve().parents[1]
            / "build"
            / "libperceptive_voice_aec.so",
            Path(__file__).resolve().parents[3] / "lib" / "libperceptive_voice_aec.so",
        ]
    )
    discovered = ctypes.util.find_library("perceptive_voice_aec")
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return discovered


class WebRtcAecProcessor:
    """ctypes wrapper around the native WebRTC APM frontend."""

    def __init__(
        self, library_path, sample_rate, noise_suppression=True, high_pass_filter=True
    ):
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
            raise RuntimeError(f"WebRTC AEC does not support sample rate {sample_rate}")
        self.frame_size = self.library.voice_aec_frame_size(self.handle)

    def process(self, microphone, playback_reference):
        import numpy as np

        microphone_int16 = (np.clip(microphone, -1.0, 1.0) * 32767.0).astype(np.int16)
        reference_int16 = (np.clip(playback_reference, -1.0, 1.0) * 32767.0).astype(
            np.int16
        )
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


def build_grasp_command(
    binary: str, config: str, extra_args: Optional[Iterable[str]] = None
) -> List[str]:
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
    event = line[prefix_offset + len(STATUS_PREFIX) :].strip()
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


def _read_grasp_stdout(
    proc,
    text_queue,
    running,
    reverse_aliases,
    speak_all_states,
    status_queue=None,
    vad_rearm_requested=None,
    audio_epoch=None,
):
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
        fields = parse_status_fields(event)
        if (
            vad_rearm_requested is not None
            and fields.get("state") == "IDLE"
            and fields.get("message") in ("Ready", "Voice: waiting for next command")
        ):
            if audio_epoch is not None:
                epoch = audio_epoch.advance()
                print(
                    f"[VoiceBridge] Audio epoch advanced: {epoch}",
                    flush=True,
                )
            vad_rearm_requested.set()
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


def build_asr_hotwords(voice_cfg) -> List[str]:
    """Build a compact SenseVoice bias list from configured commands."""
    triggers = [str(word).strip() for word in voice_cfg.get("trigger_words", [])]
    aliases = [
        str(word).strip() for word in (voice_cfg.get("target_aliases", {}) or {}).keys()
    ]
    commands = list(triggers)
    commands.extend(str(word).strip() for word in voice_cfg.get("cancel_words", []))
    commands.extend(str(word).strip() for word in voice_cfg.get("home_words", []))
    commands.extend(aliases)
    commands.extend(
        trigger + alias
        for trigger in triggers
        for alias in aliases
        if trigger and alias and not trigger.isascii()
    )
    return list(dict.fromkeys(word for word in commands if word))


def _capture_hw_name_from_arecord(device: int, runner=subprocess.run) -> Optional[str]:
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


def capture_hw_name(device: int, list_devices, runner=subprocess.run) -> Optional[str]:
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


def configure_capture_mixer(
    device: int, mixer_volume: int, list_devices, runner=subprocess.run
) -> bool:
    """Set the ALSA microphone gain for the selected capture device."""
    if mixer_volume < 0:
        return True
    hw_name = capture_hw_name(device, list_devices, runner)
    match = re.fullmatch(r"hw:(\d+),\d+", hw_name or "")
    if match is None:
        print("[VoiceBridge] Cannot resolve capture mixer card", flush=True)
        return False

    volume = max(0, min(100, int(mixer_volume)))
    command = [
        "amixer",
        "-c",
        match.group(1),
        "sset",
        "Mic",
        f"{volume}%",
        "cap",
    ]
    try:
        result = runner(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"[VoiceBridge] Capture mixer setup failed: {exc}", flush=True)
        return False
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        print(f"[VoiceBridge] Capture mixer setup failed: {detail}", flush=True)
        return False

    print(
        f"[VoiceBridge] Capture mixer: card={match.group(1)}, Mic={volume}%",
        flush=True,
    )
    return True


def _capture_rate_supported(
    hw_name: str, rate: int, channels: int, runner=subprocess.run
) -> bool:
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
    rates.extend(rate for rate in CAPTURE_RATE_CANDIDATES if rate != requested_rate)
    return rates


def candidate_capture_channels(
    device: int, requested_channels: Optional[int]
) -> List[int]:
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


def resolve_spacemit_capture_rate(
    device: int, requested_rate: int, channels: int, audio_module, capture_cls
) -> int:
    rate, _ = resolve_spacemit_capture_format(
        device,
        requested_rate,
        channels,
        audio_module,
        capture_cls,
    )
    return rate


def resolve_spacemit_capture_format(
    device: int,
    requested_rate: int,
    requested_channels: Optional[int],
    audio_module,
    capture_cls,
):
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


def resolve_capture_rate(
    device: int, requested_rate: int, channels: int, list_devices, runner=subprocess.run
) -> int:
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


def wait_for_grasp_ready(
    proc, status_queue, running, timeout_sec: float = 20.0
) -> bool:
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


def run_asr_loop(
    args,
    proc,
    running,
    playback_active=None,
    playback_reference=None,
    startup_prompt_complete=None,
    vad_rearm_requested=None,
    audio_epoch=None,
):
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
        args.channels if args.channels is not None else asr_cfg.get("channels")
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
        f"[VoiceBridge] ASR config: device={device}, {rate}Hz, channels={channels}",
        flush=True,
    )
    channel_index = (
        getattr(args, "channel_index", None)
        if getattr(args, "channel_index", None) is not None
        else int(asr_cfg.get("channel_index", -1))
    )
    try:
        select_capture_channel(
            np.zeros(channels, dtype=np.float32), channels, channel_index
        )
    except ValueError as exc:
        print(f"[VoiceBridge] Invalid ASR channel selection: {exc}", flush=True)
        running.clear()
        return
    channel_label = "average" if channel_index < 0 else str(channel_index)
    print(f"[VoiceBridge] ASR capture channel: {channel_label}", flush=True)
    trigger = (
        args.vad_trigger_threshold
        if args.vad_trigger_threshold is not None
        else float(asr_cfg.get("vad_trigger_threshold", 0.4))
    )
    stop = (
        args.vad_stop_threshold
        if args.vad_stop_threshold is not None
        else float(asr_cfg.get("vad_stop_threshold", 0.3))
    )
    min_speech_ms = (
        args.vad_min_speech_duration_ms
        if args.vad_min_speech_duration_ms is not None
        else int(asr_cfg.get("vad_min_speech_duration_ms", 100))
    )
    max_speech_ms = int(
        asr_cfg.get("vad_max_speech_duration_ms", DEFAULT_MAX_SPEECH_DURATION_MS)
    )
    if max_speech_ms < min_speech_ms:
        max_speech_ms = DEFAULT_MAX_SPEECH_DURATION_MS
    endpoint_hold_ms = max(
        0, int(asr_cfg.get("vad_endpoint_hold_ms", DEFAULT_VAD_ENDPOINT_HOLD_MS))
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

    hotwords = build_asr_hotwords(voice_cfg)
    backend_override = getattr(args, "asr_backend", None)
    backend_name = (
        backend_override
        if backend_override is not None
        else asr_cfg.get("backend", "sensevoice")
    )
    backend_name = normalize_asr_backend(backend_name)
    effective_asr_cfg = dict(asr_cfg)
    effective_asr_cfg["backend"] = backend_name
    sensevoice_cfg = asr_cfg.get("sensevoice", {}) or {}
    asr_threads = (
        args.asr_threads
        if args.asr_threads is not None
        else int(sensevoice_cfg.get("num_threads", 4))
    )
    try:
        asr, warmup_required, hotword_count = create_asr_engine(
            spacemit_asr, effective_asr_cfg, asr_threads, hotwords
        )
    except Exception as exc:
        print(
            f"[VoiceBridge] ASR initialization failed: backend={backend_name}; {exc}",
            flush=True,
        )
        running.clear()
        return
    if backend_name == "qwen3_asr":
        bias_summary = f"context_terms={hotword_count}"
    else:
        bias_summary = f"hotwords={hotword_count}, boost={ASR_HOTWORD_BOOST:g}"
    print(
        f"[VoiceBridge] ASR initialized: {asr.backend_name}; {bias_summary}",
        flush=True,
    )
    if warmup_required:
        asr.recognize(np.zeros(16000, dtype=np.float32))
        print("[VoiceBridge] ASR warmup done", flush=True)
    else:
        print("[VoiceBridge] ASR endpoint ready", flush=True)

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
            print(f"[VoiceBridge] WebRTC AEC initialization failed: {exc}", flush=True)
            asr.shutdown()
            running.clear()
            return
        print(
            "[VoiceBridge] WebRTC AEC initialized: "
            f"library={library_path}, frame_size={aec_processor.frame_size}",
            flush=True,
        )

    audio_queue = queue.Queue(maxsize=MAX_PENDING_ASR_SEGMENTS)
    state = {
        "in_speech": False,
        "playback_suppressed": False,
        "aec_pending": np.empty(0, dtype=np.float32),
        "aec_pending_start": None,
        "aec_tail_until": 0.0,
        "aec_output_active": False,
        "speech_path": None,
        "speech_samples": 0,
        "endpoint_hold_samples": 0,
        "startup_guard_release_at": None,
        "startup_guard_active": startup_prompt_complete is not None,
        "level_samples": 0,
        "level_sum_squares": 0.0,
        "level_peak": 0.0,
        "level_reported": False,
        "debug_samples": 0,
        "debug_sum_squares": 0.0,
        "debug_peak": 0.0,
        "debug_vad_probability": 0.0,
        "vad_audio_path": None,
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
        state["speech_path"] = None
        state["speech_samples"] = 0
        state["endpoint_hold_samples"] = 0
        speech_buffer.clear()
        pre_buffer.clear()

    def reset_vad_state():
        reset_vad = getattr(vad, "reset", None)
        if callable(reset_vad):
            try:
                reset_vad()
            except Exception as exc:
                print(f"[VoiceBridge] VAD reset failed: {exc}", flush=True)

    def finish_speech(reason=None):
        if not speech_buffer:
            reset_speech_segment()
            reset_vad_state()
            return
        audio = np.concatenate(speech_buffer)
        duration = len(audio) / target_rate
        reset_speech_segment()
        # Silero keeps endpoint state internally. Rearm after every completed
        # utterance so the next command can produce a fresh speech-start event.
        reset_vad_state()
        suffix = f"; reason={reason}" if reason else ""
        print(f"\r[VAD] 语音结束 ({duration:.1f}s{suffix})", flush=True)
        item = (
            audio_epoch.current() if audio_epoch is not None else 0,
            time.monotonic(),
            audio,
        )
        try:
            audio_queue.put_nowait(item)
        except queue.Full:
            try:
                audio_queue.get_nowait()
            except queue.Empty:
                pass
            audio_queue.put_nowait(item)
            print(
                "[VoiceBridge] ASR queue full; dropped oldest segment",
                flush=True,
            )

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

        raw_frames = []
        processed_frames = []
        use_aec_for_chunk = False
        frame_size = aec_processor.frame_size
        while len(state["aec_pending"]) >= frame_size:
            frame = state["aec_pending"][:frame_size]
            state["aec_pending"] = state["aec_pending"][frame_size:]
            frame_end = state["aec_pending_start"] + frame_size / target_rate
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
            use_aec_for_chunk = use_aec_for_chunk or use_aec_output
            if use_aec_output != state["aec_output_active"]:
                state["aec_output_active"] = use_aec_output
                path = "aec" if use_aec_output else "raw microphone"
                print(f"\n[VoiceBridge] ASR audio path: {path}", flush=True)
            raw_frames.append(frame.copy())
            processed_frames.append(processed)
            state["aec_pending_start"] = frame_end
        if not processed_frames:
            empty = np.empty(0, dtype=np.float32)
            return empty, empty, False
        return (
            np.concatenate(raw_frames),
            np.concatenate(processed_frames),
            use_aec_for_chunk,
        )

    def on_audio(data: bytes):
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        samples = select_capture_channel(samples, channels, channel_index)
        if resampler is not None:
            samples = resampler.process(samples)
        if len(samples) == 0:
            return

        if not state["level_reported"]:
            state["level_samples"] += len(samples)
            state["level_sum_squares"] += float(np.dot(samples, samples))
            state["level_peak"] = max(
                state["level_peak"], float(np.max(np.abs(samples)))
            )
            if state["level_samples"] >= target_rate:
                rms = (state["level_sum_squares"] / state["level_samples"]) ** 0.5
                print(
                    "\n[VoiceBridge] Microphone input active: "
                    f"rms={rms:.4f}, peak={state['level_peak']:.4f}",
                    flush=True,
                )
                state["level_reported"] = True

        raw_samples = samples
        aec_samples = None
        use_aec_for_chunk = False
        if echo_mode == "webrtc_aec":
            try:
                raw_samples, aec_samples, use_aec_for_chunk = apply_software_aec(
                    samples
                )
            except Exception as exc:
                print(f"\n[VoiceBridge] WebRTC AEC failed: {exc}", flush=True)
                running.clear()
                return
            if len(raw_samples) == 0:
                return

        if state["startup_guard_active"]:
            now = time.monotonic()
            if (
                startup_prompt_complete is not None
                and startup_prompt_complete.is_set()
                and state["startup_guard_release_at"] is None
            ):
                state["startup_guard_release_at"] = now + TTS_ECHO_GUARD_MS / 1000.0
            release_at = state["startup_guard_release_at"]
            if release_at is None or now < release_at:
                if state["in_speech"] or speech_buffer or pre_buffer:
                    reset_speech_segment()
                    reset_vad_state()
                return
            state["startup_guard_active"] = False
            reset_speech_segment()
            reset_vad_state()
            print("[VoiceBridge] Startup prompt guard released", flush=True)

        samples, selected_path = select_speech_samples(
            state["in_speech"],
            raw_samples,
            aec_samples,
            state["speech_path"],
            use_aec_for_chunk,
        )

        path_rearm = should_rearm_vad_for_audio_path(
            state["vad_audio_path"], selected_path, state["in_speech"]
        )
        requested_rearm = (
            vad_rearm_requested is not None and vad_rearm_requested.is_set()
        )
        if path_rearm or requested_rearm:
            previous_path = state["vad_audio_path"]
            if requested_rearm:
                reset_speech_segment()
            reset_vad_state()
            state["endpoint_hold_samples"] = 0
            pre_buffer.clear()
            if vad_rearm_requested is not None:
                vad_rearm_requested.clear()
            dropped = clear_pending_audio() if requested_rearm else 0
            reason = (
                f"audio path {previous_path} -> {selected_path}"
                if path_rearm
                else "pipeline ready"
            )
            print(
                f"\n[VoiceBridge] VAD rearmed: {reason}; dropped_segments={dropped}",
                flush=True,
            )
        state["vad_audio_path"] = selected_path

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
            reset_vad_state()
            print("[VoiceBridge] ASR resumed after TTS echo guard", flush=True)

        result = vad.detect(samples, target_rate)
        if getattr(args, "audio_debug", False):
            state["debug_samples"] += len(samples)
            state["debug_sum_squares"] += float(np.dot(samples, samples))
            state["debug_peak"] = max(
                state["debug_peak"], float(np.max(np.abs(samples)))
            )
            state["debug_vad_probability"] = max(
                state["debug_vad_probability"],
                float(getattr(result, "probability", 0.0) or 0.0),
            )
            if state["debug_samples"] >= target_rate:
                rms = (state["debug_sum_squares"] / state["debug_samples"]) ** 0.5
                print(
                    "\n[VoiceBridge] Audio debug: "
                    f"rms={rms:.4f}, peak={state['debug_peak']:.4f}, "
                    "vad_max="
                    f"{state['debug_vad_probability']:.4f}",
                    flush=True,
                )
                state["debug_samples"] = 0
                state["debug_sum_squares"] = 0.0
                state["debug_peak"] = 0.0
                state["debug_vad_probability"] = 0.0
        vad_event = classify_vad_event(state["in_speech"], result)

        if vad_event == "start":
            state["in_speech"] = True
            state["speech_path"] = selected_path
            speech_buffer.clear()
            if pre_buffer:
                buffered = np.concatenate(list(pre_buffer))
                speech_buffer.append(buffered)
                state["speech_samples"] = len(buffered)
            else:
                state["speech_samples"] = 0
            speech_buffer.append(samples.copy())
            state["speech_samples"] += len(samples)
            print("\r[VAD] 检测到语音...", end="", flush=True)
        elif vad_event in ("append", "finish"):
            speech_buffer.append(samples.copy())
            state["speech_samples"] += len(samples)
            state["endpoint_hold_samples"] = update_vad_endpoint_hold(
                True,
                result,
                state["endpoint_hold_samples"],
                len(samples),
                trigger,
            )

        if (
            state["in_speech"]
            and state["endpoint_hold_samples"] > 0
            and state["endpoint_hold_samples"] >= target_rate * endpoint_hold_ms // 1000
        ):
            finish_speech()

        if (
            state["in_speech"]
            and state["speech_samples"] >= target_rate * max_speech_ms // 1000
        ):
            finish_speech("maximum duration")

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
        segment_sequence = 0
        while running.is_set() and proc.poll() is None:
            try:
                segment_epoch, captured_at, audio = audio_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            current_epoch = audio_epoch.current() if audio_epoch is not None else 0
            age_sec = time.monotonic() - captured_at
            if segment_epoch != current_epoch or age_sec > MAX_ASR_SEGMENT_AGE_SEC:
                print(
                    "[VoiceBridge] Drop stale ASR segment: "
                    f"segment_epoch={segment_epoch} "
                    f"current_epoch={current_epoch} age_ms={age_sec * 1000:.0f}",
                    flush=True,
                )
                continue
            if echo_mode == "half_duplex" and is_playback_active():
                continue
            segment_sequence += 1
            debug_path = None
            if getattr(args, "asr_audio_dir", None):
                try:
                    debug_path = save_asr_audio_segment(
                        args.asr_audio_dir,
                        segment_sequence,
                        audio,
                        target_rate,
                    )
                except OSError as exc:
                    print(
                        f"[VoiceBridge] Failed to save ASR audio: {exc}",
                        flush=True,
                    )
            request_started = time.monotonic()
            if getattr(args, "audio_debug", False):
                detail = f" file={debug_path}" if debug_path else ""
                print(
                    "[VoiceBridge] ASR request: "
                    f"segment={segment_sequence} "
                    f"duration_ms={len(audio) * 1000 // target_rate}"
                    f"{detail}",
                    flush=True,
                )
            try:
                result = asr.recognize(audio)
            except Exception as exc:
                print(f"[VoiceBridge] ASR request failed: {exc}", flush=True)
                continue
            if getattr(args, "audio_debug", False):
                elapsed_ms = int((time.monotonic() - request_started) * 1000)
                print(
                    "[VoiceBridge] ASR response: "
                    f"segment={segment_sequence} elapsed_ms={elapsed_ms}",
                    flush=True,
                )
            if audio_epoch is not None and segment_epoch != audio_epoch.current():
                print(
                    "[VoiceBridge] Drop ASR result from previous audio epoch",
                    flush=True,
                )
                continue
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
    parser.add_argument(
        "--remote-host", default=None, help="覆盖 remote_mujoco 服务地址"
    )
    parser.add_argument(
        "--remote-port", type=int, default=None, help="覆盖 remote_mujoco 服务端口"
    )
    parser.add_argument(
        "--grasp-arg",
        action="append",
        default=[],
        help="透传给 perceptive_grasp 的额外参数，可重复",
    )
    parser.add_argument("-d", "--device", type=int, default=None)
    parser.add_argument("-r", "--rate", type=int, default=None)
    parser.add_argument("-c", "--channels", type=int, choices=[1, 2], default=None)
    parser.add_argument(
        "--channel-index",
        type=int,
        choices=[0, 1],
        default=None,
        help="覆盖双声道录音中送入 asr 的物理声道",
    )
    parser.add_argument("--vad-trigger-threshold", type=float, default=None)
    parser.add_argument("--vad-stop-threshold", type=float, default=None)
    parser.add_argument("--vad-min-speech-duration-ms", type=int, default=None)
    parser.add_argument("--pre-buffer-ms", type=int, default=800)
    parser.add_argument(
        "--asr-backend",
        choices=("sensevoice", "qwen3_asr"),
        default=None,
        help="覆盖 voice.asr.backend",
    )
    parser.add_argument("--asr-threads", type=int, default=None)
    parser.add_argument(
        "--audio-debug", action="store_true", help="每秒输出麦克风电平和 vad 最大概率"
    )
    parser.add_argument(
        "--asr-audio-dir", default=None, help="保存实际提交给 asr 的单声道 wav 片段"
    )
    parser.add_argument("--tts-engine", default=None)
    parser.add_argument("--tts-device", type=int, default=None)
    parser.add_argument("--tts-rate", type=int, default=None)
    parser.add_argument("--tts-channels", type=int, choices=[1, 2], default=None)
    parser.add_argument("--tts-speed", type=float, default=None)
    parser.add_argument("--tts-volume", type=int, default=None)
    parser.add_argument("--speak-all", action="store_true")
    parser.add_argument("--no-play", action="store_true")
    parser.add_argument("--startup-timeout-sec", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    process_lock = acquire_voice_bridge_lock()
    if process_lock is None:
        print(
            "[VoiceBridge] Another voice bridge is already running; "
            "stop it before starting a new instance",
            flush=True,
        )
        return 2
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
        args.tts_device
        if args.tts_device is not None
        else int(tts_cfg.get("playback_device", -1))
    )
    playback_rate = (
        args.tts_rate
        if args.tts_rate is not None
        else int(tts_cfg.get("playback_rate", 48000))
    )
    tts_channels = (
        args.tts_channels
        if args.tts_channels is not None
        else int(tts_cfg.get("channels", 1))
    )
    speed = (
        args.tts_speed
        if args.tts_speed is not None
        else float(tts_cfg.get("speed", 1.0))
    )
    volume = (
        args.tts_volume
        if args.tts_volume is not None
        else int(tts_cfg.get("volume", 80))
    )
    mixer_volume = int(tts_cfg.get("mixer_volume", -1))
    speak_all_states = args.speak_all or bool(tts_cfg.get("speak_all_states", False))
    tts_enabled = bool(tts_cfg.get("enabled", True))

    grasp_args = list(args.grasp_arg)
    if args.remote_host:
        grasp_args.extend(["--remote-host", args.remote_host])
    if args.remote_port is not None:
        if args.remote_port < 1 or args.remote_port > 65535:
            print("[VoiceBridge] Invalid remote port: expected 1-65535", flush=True)
            return 2
        grasp_args.extend(["--remote-port", str(args.remote_port)])
    cmd = build_grasp_command(args.binary, args.config, grasp_args)
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
    playback_startup_complete = threading.Event()
    vad_rearm_requested = threading.Event()
    audio_epoch = AudioEpoch()
    playback_reference = None
    if echo_mode == "webrtc_aec":
        playback_reference = PlaybackReferenceTimeline(
            16000,
            int(echo_config.get("delay_ms", 50)),
        )
    startup_prompt_complete = None
    if tts_enabled and not args.no_play:
        startup_prompt_complete = threading.Event()

    def on_playback_complete(text):
        del text
        if startup_prompt_complete is not None and not startup_prompt_complete.is_set():
            startup_prompt_complete.set()

    def stop_handler(sig, frame):
        del sig, frame
        request_shutdown(proc, running, text_queue)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    threads = [
        threading.Thread(
            target=_read_grasp_stdout,
            args=(
                proc,
                text_queue,
                running,
                reverse_aliases,
                speak_all_states,
                status_queue,
                vad_rearm_requested,
                audio_epoch,
            ),
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

    if not wait_for_grasp_ready(proc, status_queue, running, args.startup_timeout_sec):
        print("[VoiceBridge] perceptive_grasp not ready; stop voice bridge", flush=True)
        request_shutdown(proc, running, text_queue)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        for thread in threads:
            thread.join(timeout=2.0)
        return 1

    asr_cfg = voice_cfg.get("asr", {}) or {}
    try:
        from spacemit_audio import AudioCapture, AudioPlayer

        asr_device = (
            args.device
            if args.device is not None
            else resolve_audio_device(
                int(asr_cfg.get("device", -1)),
                asr_cfg.get("device_name", ""),
                AudioCapture.list_devices,
                "capture",
            )
        )
        if args.tts_device is None:
            playback_device = resolve_audio_device(
                playback_device,
                tts_cfg.get("playback_device_name", ""),
                AudioPlayer.list_devices,
                "playback",
            )
        args.device = asr_device
    except Exception as exc:
        print(f"[VoiceBridge] Audio device resolution failed: {exc}", flush=True)
        request_shutdown(proc, running, text_queue)
        return 1
    asr_mixer_volume = int(asr_cfg.get("mixer_volume", -1))
    if asr_mixer_volume >= 0:
        try:
            configure_capture_mixer(
                asr_device,
                asr_mixer_volume,
                AudioCapture.list_devices,
            )
        except Exception as exc:
            print(
                f"[VoiceBridge] Capture mixer setup skipped: {exc}",
                flush=True,
            )

    if tts_enabled:
        tts_thread = threading.Thread(
            target=run_tts_worker,
            args=(
                text_queue,
                running,
                preset,
                playback_device,
                playback_rate,
                tts_channels,
                speed,
                volume,
                args.no_play,
                mixer_volume,
                playback_active,
                TTS_ECHO_GUARD_MS if echo_mode == "half_duplex" else 0,
                playback_reference.schedule if playback_reference else None,
                on_playback_complete,
                playback_startup_complete,
            ),
            daemon=True,
        )
        threads.append(tts_thread)
        tts_thread.start()
        if not playback_startup_complete.wait(timeout=5.0):
            print(
                "[VoiceBridge] TTS playback initialization timed out",
                flush=True,
            )
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
                startup_prompt_complete,
                vad_rearm_requested,
                audio_epoch,
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
