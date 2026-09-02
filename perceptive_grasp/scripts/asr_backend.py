#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""ASR backend selection shared by perceptive_grasp voice entry points."""

import base64
import glob
import io
import json
import os
import shutil
import signal
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request
import wave
from pathlib import Path


SUPPORTED_ASR_BACKENDS = ("sensevoice", "qwen3_asr")
DEFAULT_QWEN3_ENDPOINT = (
    "http://127.0.0.1:8063/v1/chat/completions"
)
DEFAULT_QWEN3_MODEL_DIR = (
    "~/.cache/models/asr/qwen3asr/qwen3-asr-0.6B-dynq-q40"
)
QWEN3_TEXT_MODEL = "Qwen3-ASR-0.6B-text-q40.gguf"


def select_capture_channel(samples, channels, channel_index=-1):
    """Convert interleaved capture samples to one stable ASR channel.

    A negative channel index averages all capture channels.
    Selecting one physical microphone avoids phase cancellation on USB
    microphone arrays whose channels are not sample-aligned.
    """
    import numpy as np

    channel_count = int(channels)
    selected = int(channel_index)
    array = np.asarray(samples)
    if channel_count < 1:
        raise ValueError("voice.asr.channels must be positive")
    if array.size % channel_count != 0:
        raise ValueError("capture buffer is not aligned to channel count")
    if channel_count == 1:
        if selected not in (-1, 0):
            raise ValueError("voice.asr.channel_index must be 0 for mono")
        return array.reshape(-1)
    frames = array.reshape(-1, channel_count)
    if selected < 0:
        return frames.astype(np.float32).mean(axis=1)
    if selected >= channel_count:
        raise ValueError(
            "voice.asr.channel_index must be smaller than voice.asr.channels"
        )
    return frames[:, selected]


def normalize_asr_backend(value):
    """Return the canonical backend name used by application configs."""
    name = str(value or "sensevoice").strip().lower().replace("-", "_")
    if name == "qwen3asr":
        name = "qwen3_asr"
    if name not in SUPPORTED_ASR_BACKENDS:
        supported = ", ".join(SUPPORTED_ASR_BACKENDS)
        raise ValueError(
            f"unsupported voice.asr.backend={value!r}; expected: {supported}"
        )
    return name


def _validate_qwen3_config(config):
    endpoint = str(config.get("endpoint", DEFAULT_QWEN3_ENDPOINT)).strip()
    model = str(config.get("model", "qwen3-asr")).strip()
    try:
        timeout_sec = float(config.get("timeout_sec", 60))
    except (TypeError, ValueError) as exc:
        raise ValueError("voice.asr.qwen3_asr.timeout_sec must be numeric") \
            from exc
    parsed = urllib.parse.urlsplit(endpoint)
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise ValueError(
            "voice.asr.qwen3_asr.endpoint must be an http(s) URL"
        )
    if not model:
        raise ValueError("voice.asr.qwen3_asr.model must not be empty")
    if timeout_sec <= 0:
        raise ValueError("voice.asr.qwen3_asr.timeout_sec must be positive")
    return endpoint, model, timeout_sec


def qwen3_health_url(endpoint):
    """Build the llama-server health URL from a chat endpoint."""
    parsed = urllib.parse.urlsplit(endpoint)
    return urllib.parse.urlunsplit(
        (parsed.scheme, parsed.netloc, "/health", "", "")
    )


def probe_qwen3_asr_endpoint(config, timeout_sec=3.0):
    """Raise a descriptive error unless llama-server reports healthy."""
    endpoint, _, configured_timeout = _validate_qwen3_config(config)
    health_url = qwen3_health_url(endpoint)
    request = urllib.request.Request(health_url, method="GET")
    try:
        with urllib.request.urlopen(
                request,
                timeout=min(float(timeout_sec), configured_timeout)) as response:
            status = int(getattr(response, "status", 200))
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise RuntimeError(
            f"qwen3-asr endpoint unavailable: {health_url}: {exc}"
        ) from exc
    if status < 200 or status >= 300:
        raise RuntimeError(
            f"qwen3-asr health check returned HTTP {status}: {health_url}"
        )
    return health_url


def _config_bool(config, key, default):
    value = config.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "yes", "on", "1"):
            return True
        if normalized in ("false", "no", "off", "0"):
            return False
    raise ValueError(f"voice.asr.qwen3_asr.{key} must be boolean")


def qwen3_endpoint_is_local(endpoint):
    """Return whether an endpoint belongs to the current machine."""
    host = (urllib.parse.urlsplit(endpoint).hostname or "").lower()
    return host in ("127.0.0.1", "localhost", "::1")


def qwen3_auto_start_enabled(config):
    """Return whether a missing local endpoint may be started on demand."""
    return _config_bool(config, "auto_start", True)


def _resolve_qwen3_server_binary(config):
    configured = str(config.get("server_binary", "")).strip()
    if configured:
        expanded = os.path.expanduser(configured)
        resolved = expanded if os.path.isfile(expanded) else shutil.which(
            configured
        )
        if resolved and os.access(resolved, os.X_OK):
            return os.path.abspath(resolved)
        raise RuntimeError(
            "qwen3-asr server_binary is not executable: " + configured
        )

    cached = glob.glob(
        os.path.expanduser(
            "~/.cache/llama-spacemit-*/usr/bin/llama-server"
        )
    )
    cached.sort(key=os.path.getmtime, reverse=True)
    for candidate in cached:
        if os.access(candidate, os.X_OK):
            return os.path.abspath(candidate)

    resolved = shutil.which("llama-server")
    if resolved:
        return os.path.abspath(resolved)
    raise RuntimeError(
        "llama-server was not found; install llama.cpp-tools-spacemit "
        "0.1.7 or later"
    )


def validate_qwen3_local_runtime(config):
    """Validate files required to auto-start a local qwen3-asr server."""
    endpoint, _, _ = _validate_qwen3_config(config)
    if not qwen3_endpoint_is_local(endpoint):
        raise ValueError("qwen3-asr auto-start only supports local endpoints")
    binary = _resolve_qwen3_server_binary(config)
    model_dir = Path(os.path.expanduser(str(
        config.get("model_dir", DEFAULT_QWEN3_MODEL_DIR)
    ))).resolve()
    text_model = model_dir / QWEN3_TEXT_MODEL
    required = (
        text_model,
        model_dir / "Qwen3-ASR-0.6B-encoder-backend.dynq.onnx",
        model_dir / "Qwen3-ASR-0.6B-encoder-frontend.dynq.onnx",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(
            "qwen3-asr model files are missing: " + ", ".join(missing)
        )
    return binary, model_dir, text_model


class Qwen3AsrResult:
    """Recognition result compatible with spacemit_asr.Result."""

    def __init__(self, text, audio_duration_ms, processing_time_ms):
        self.text = str(text or "").strip()
        self.audio_duration_ms = int(audio_duration_ms)
        self.processing_time_ms = int(processing_time_ms)
        self.rtf = (
            self.processing_time_ms / self.audio_duration_ms
            if self.audio_duration_ms > 0 else 0.0
        )

    @property
    def is_empty(self):
        return not self.text

    def __bool__(self):
        return not self.is_empty


class Qwen3AsrEngine:
    """Buffered Qwen3-ASR client for a llama-server HTTP endpoint."""

    backend_name = "Qwen3-ASR"

    def __init__(self, config):
        self.endpoint, self.model, self.timeout_sec = \
            _validate_qwen3_config(config)
        self.config = dict(config)
        self.context = str(config.get("context", "")).strip()
        try:
            self.max_transcript_chars = int(
                config.get("max_transcript_chars", 32)
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "voice.asr.qwen3_asr.max_transcript_chars must be integer"
            ) from exc
        if self.max_transcript_chars <= 0:
            raise ValueError(
                "voice.asr.qwen3_asr.max_transcript_chars must be positive"
            )
        self.auto_start = qwen3_auto_start_enabled(config)
        try:
            self.startup_timeout_sec = float(
                config.get("startup_timeout_sec", 90)
            )
            self.server_threads = int(config.get("server_threads", 4))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "qwen3-asr startup timeout and server threads must be numeric"
            ) from exc
        if self.startup_timeout_sec <= 0 or self.server_threads <= 0:
            raise ValueError(
                "qwen3-asr startup timeout and server threads must be positive"
            )
        self._initialized = False
        self._server_process = None
        self._server_log = None
        self.server_log_path = str(config.get(
            "server_log_path",
            f"/tmp/perceptive_grasp_qwen3_asr_{os.getuid()}.log",
        ))

    def initialize(self):
        health_config = {
            "endpoint": self.endpoint,
            "model": self.model,
            "timeout_sec": self.timeout_sec,
        }
        try:
            probe_qwen3_asr_endpoint(health_config)
        except RuntimeError:
            if not self.auto_start or not qwen3_endpoint_is_local(
                    self.endpoint):
                raise
            self._start_local_server(health_config)
        self._initialized = True
        return self

    def _start_local_server(self, health_config):
        binary, model_dir, text_model = validate_qwen3_local_runtime(
            self.config
        )
        parsed = urllib.parse.urlsplit(self.endpoint)
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        command = [
            binary,
            "-m", str(text_model),
            "--media-backend", "smt",
            "--smt-config-dir", str(model_dir),
            "--host", "127.0.0.1",
            "--port", str(port),
            "-t", str(self.server_threads),
        ]
        environment = os.environ.copy()
        binary_path = Path(binary)
        if binary_path.parent.name == "bin" and \
                binary_path.parent.parent.name == "usr":
            private_lib = str(binary_path.parent.parent / "lib")
            old_path = environment.get("LD_LIBRARY_PATH", "")
            environment["LD_LIBRARY_PATH"] = (
                private_lib + (":" + old_path if old_path else "")
            )
        log_path = Path(os.path.expanduser(self.server_log_path))
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._server_log = open(log_path, "w", encoding="utf-8")
        print(
            "[Qwen3-ASR] Starting local llama-server; "
            f"log={log_path}",
            flush=True,
        )
        try:
            self._server_process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=self._server_log,
                stderr=subprocess.STDOUT,
                env=environment,
                start_new_session=True,
            )
        except OSError as exc:
            self._close_server_log()
            raise RuntimeError(
                f"failed to start qwen3-asr server: {exc}"
            ) from exc

        deadline = time.monotonic() + self.startup_timeout_sec
        while time.monotonic() < deadline:
            return_code = self._server_process.poll()
            if return_code is not None:
                detail = self._read_server_log_tail(log_path)
                self._close_server_log()
                raise RuntimeError(
                    "qwen3-asr server exited during startup "
                    f"(code={return_code}); log={log_path}; {detail}"
                )
            try:
                probe_qwen3_asr_endpoint(health_config, timeout_sec=1.0)
                print("[Qwen3-ASR] Local endpoint ready", flush=True)
                return
            except RuntimeError:
                time.sleep(0.25)
        self._stop_local_server()
        raise RuntimeError(
            "qwen3-asr server startup timed out after "
            f"{self.startup_timeout_sec:g}s; log={log_path}"
        )

    @staticmethod
    def _read_server_log_tail(log_path):
        try:
            lines = log_path.read_text(
                encoding="utf-8", errors="replace"
            ).splitlines()
        except OSError:
            return "server log is unavailable"
        return " | ".join(lines[-4:]) or "server log is empty"

    def _close_server_log(self):
        if self._server_log is not None:
            self._server_log.close()
            self._server_log = None

    def _stop_local_server(self):
        process = self._server_process
        self._server_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                except (OSError, subprocess.TimeoutExpired):
                    pass
        self._close_server_log()

    @staticmethod
    def _encode_wav(audio, sample_rate=16000):
        import numpy as np

        samples = np.asarray(audio)
        if samples.ndim != 1:
            samples = samples.reshape(-1)
        if samples.dtype == np.int16:
            pcm = samples.astype("<i2", copy=False)
        else:
            normalized = np.clip(samples.astype(np.float32), -1.0, 1.0)
            pcm = (normalized * 32767.0).astype("<i2")
        output = io.BytesIO()
        with wave.open(output, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(pcm.tobytes())
        return output.getvalue(), len(pcm)

    @staticmethod
    def _extract_text(payload):
        try:
            content = payload["choices"][0]["message"].get("content", "")
        except (KeyError, IndexError, TypeError, AttributeError) as exc:
            raise RuntimeError(
                f"qwen3-asr returned an unexpected response: {payload!r}"
            ) from exc
        if isinstance(content, list):
            text = "".join(
                str(item.get("text", ""))
                for item in content
                if isinstance(item, dict) and item.get("type") == "text"
            ).strip()
        else:
            text = str(content or "").strip()
        if "<asr_text>" in text:
            text = text.rsplit("<asr_text>", 1)[1].strip()
        return text

    def _sanitize_transcript(self, text):
        transcript = str(text or "").strip()
        if not transcript:
            return ""
        if self.context and transcript == self.context:
            print("[Qwen3-ASR] Drop context echo", flush=True)
            return ""
        if len(transcript) > self.max_transcript_chars:
            print(
                "[Qwen3-ASR] Drop overlong transcription: "
                f"chars={len(transcript)} limit={self.max_transcript_chars}",
                flush=True,
            )
            return ""
        return transcript

    def recognize(self, audio):
        if not self._initialized:
            raise RuntimeError("Qwen3AsrEngine is not initialized")
        wav_bytes, sample_count = self._encode_wav(audio)
        encoded = base64.b64encode(wav_bytes).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [
                *([{"role": "system", "content": self.context}]
                  if self.context else []),
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "input_audio",
                            "input_audio": {
                                "data": encoded,
                                "format": "wav",
                            },
                        },
                        {
                            "type": "text",
                            "text": "language Chinese<asr_text>",
                        },
                    ],
                },
            ],
            "max_tokens": 512,
            "temperature": 0,
        }
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        started = time.monotonic()
        try:
            with urllib.request.urlopen(
                    request, timeout=self.timeout_sec) as response:
                body = response.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:300]
            raise RuntimeError(
                f"qwen3-asr HTTP {exc.code}: {detail}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise RuntimeError(f"qwen3-asr request failed: {exc}") from exc
        try:
            response_payload = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("qwen3-asr returned invalid JSON") from exc
        elapsed_ms = (time.monotonic() - started) * 1000.0
        duration_ms = sample_count / 16000.0 * 1000.0
        text = self._sanitize_transcript(
            self._extract_text(response_payload)
        )
        return Qwen3AsrResult(text, duration_ms, elapsed_ms)

    def shutdown(self):
        self._initialized = False
        self._stop_local_server()


def create_asr_engine(spacemit_asr, asr_config, threads, hotwords):
    """Create the configured ASR engine and return warmup metadata."""
    backend = normalize_asr_backend(asr_config.get("backend", "sensevoice"))
    if backend == "qwen3_asr":
        qwen_config = dict(asr_config.get("qwen3_asr", {}) or {})
        context_term_count = 0
        if not str(qwen_config.get("context", "")).strip():
            try:
                max_terms = int(qwen_config.get("context_max_terms", 0))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "voice.asr.qwen3_asr.context_max_terms must be integer"
                ) from exc
            terms = list(dict.fromkeys(
                str(word).strip() for word in hotwords if str(word).strip()
            ))[:max(0, max_terms)]
            context_term_count = len(terms)
            if terms:
                qwen_config["context"] = " ".join(terms)
        engine = Qwen3AsrEngine(qwen_config).initialize()
        return engine, False, context_term_count

    sensevoice = asr_config.get("sensevoice", {}) or {}
    model_dir = str(sensevoice.get("model_dir", "")).strip()
    config = spacemit_asr.Config(model_dir) \
        if model_dir else spacemit_asr.Config()
    config.provider = str(sensevoice.get("provider", "cpu"))
    config._config.num_threads = int(threads)
    config.language = spacemit_asr.Language.ZH
    config.punctuation = True
    config._config.hotwords = list(hotwords)
    config._config.hotword_boost = 2.0
    engine = spacemit_asr.Engine(config).initialize()
    return engine, True, len(hotwords)
