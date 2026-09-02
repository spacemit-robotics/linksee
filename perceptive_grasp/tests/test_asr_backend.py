#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for selectable perceptive_grasp ASR backends."""

import json
import inspect
import tempfile
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import asr_backend  # noqa: E402
import asr_node  # noqa: E402


class FakeResponse:
    def __init__(self, payload=None, status=200):
        self.status = status
        self.payload = payload if payload is not None else {}

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        del exc_type, exc_value, traceback

    def read(self):
        return json.dumps(self.payload).encode("utf-8")


class AsrBackendTest(unittest.TestCase):
    def test_standalone_asr_rearms_vad_after_speech_end(self):
        source = inspect.getsource(asr_node.main)
        speech_end = source.index(
            'elif result.is_speech_end and state["in_speech"]:'
        )
        reset = source.index('getattr(vad, "reset", None)', speech_end)
        enqueue = source.index("audio_queue.put(audio)", speech_end)

        self.assertLess(reset, enqueue)

    def test_backend_names_are_normalized(self):
        self.assertEqual(
            asr_backend.normalize_asr_backend("qwen3-asr"), "qwen3_asr"
        )
        self.assertEqual(
            asr_backend.normalize_asr_backend("sensevoice"), "sensevoice"
        )
        with self.assertRaisesRegex(ValueError, "unsupported"):
            asr_backend.normalize_asr_backend("unknown")

    def test_capture_channel_selection_avoids_stereo_cancellation(self):
        samples = np.array([1000, -1000, 2000, -2000], dtype=np.int16)
        np.testing.assert_array_equal(
            asr_backend.select_capture_channel(samples, 2, 0),
            np.array([1000, 2000], dtype=np.int16),
        )
        np.testing.assert_array_equal(
            asr_backend.select_capture_channel(samples, 2, 1),
            np.array([-1000, -2000], dtype=np.int16),
        )
        np.testing.assert_array_equal(
            asr_backend.select_capture_channel(samples, 2, -1),
            np.zeros(2, dtype=np.float32),
        )

    def test_invalid_capture_channel_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "channel_index"):
            asr_backend.select_capture_channel(
                np.zeros(4, dtype=np.int16), 2, 2
            )

    def test_health_url_uses_llama_server_root(self):
        endpoint = "http://10.0.0.2:8063/v1/chat/completions"
        self.assertEqual(
            asr_backend.qwen3_health_url(endpoint),
            "http://10.0.0.2:8063/health",
        )

    def test_qwen_recognize_sends_wav_and_parses_text(self):
        responses = [
            FakeResponse(),
            FakeResponse({
                "choices": [{"message": {"content": "抓香蕉。"}}],
            }),
        ]
        requests = []

        def fake_urlopen(request, timeout):
            del timeout
            requests.append(request)
            return responses.pop(0)

        config = {
            "endpoint": "http://127.0.0.1:8063/v1/chat/completions",
            "model": "qwen3-asr",
            "timeout_sec": 10,
        }
        with mock.patch.object(
                asr_backend.urllib.request, "urlopen",
                side_effect=fake_urlopen):
            engine = asr_backend.Qwen3AsrEngine(config).initialize()
            result = engine.recognize(np.zeros(16000, dtype=np.float32))

        self.assertEqual(result.text, "抓香蕉。")
        self.assertFalse(result.is_empty)
        self.assertEqual(requests[0].full_url, "http://127.0.0.1:8063/health")
        payload = json.loads(requests[1].data.decode("utf-8"))
        self.assertEqual(payload["model"], "qwen3-asr")
        audio = payload["messages"][0]["content"][0]["input_audio"]
        self.assertEqual(audio["format"], "wav")
        self.assertTrue(audio["data"].startswith("UklGR"))

    def test_qwen_parses_list_content(self):
        payload = {
            "choices": [{"message": {"content": [
                {"type": "text", "text": "抓"},
                {"type": "text", "text": "苹果"},
            ]}}],
        }
        self.assertEqual(
            asr_backend.Qwen3AsrEngine._extract_text(payload), "抓苹果"
        )

    def test_qwen_removes_prompt_prefix(self):
        payload = {
            "choices": [{"message": {
                "content": "language Chinese<asr_text>抓香蕉。",
            }}],
        }
        self.assertEqual(
            asr_backend.Qwen3AsrEngine._extract_text(payload), "抓香蕉。"
        )

    def test_qwen_factory_does_not_initialize_sensevoice(self):
        fake_asr = SimpleNamespace(
            Config=mock.Mock(side_effect=AssertionError("not expected")),
        )
        with mock.patch.object(
                asr_backend, "probe_qwen3_asr_endpoint",
                return_value="http://127.0.0.1:8063/health"):
            engine, warmup, hotword_count = asr_backend.create_asr_engine(
                fake_asr,
                {
                    "backend": "qwen3_asr",
                    "qwen3_asr": {
                        "endpoint": (
                            "http://127.0.0.1:8063/v1/chat/completions"
                        ),
                        "context_max_terms": 1,
                    },
                },
                4,
                ["抓香蕉"],
            )

        self.assertIsInstance(engine, asr_backend.Qwen3AsrEngine)
        self.assertFalse(warmup)
        self.assertEqual(hotword_count, 1)
        self.assertIn("抓香蕉", engine.context)

    def test_qwen_context_is_sent_as_system_message(self):
        responses = [
            FakeResponse(),
            FakeResponse({
                "choices": [{"message": {"content": "抓香蕉。"}}],
            }),
        ]
        requests = []

        def fake_urlopen(request, timeout):
            del timeout
            requests.append(request)
            return responses.pop(0)

        with mock.patch.object(
                asr_backend.urllib.request, "urlopen",
                side_effect=fake_urlopen):
            engine = asr_backend.Qwen3AsrEngine({
                "context": "可能出现香蕉和苹果。",
            }).initialize()
            engine.recognize(np.zeros(1600, dtype=np.float32))

        payload = json.loads(requests[1].data.decode("utf-8"))
        self.assertEqual(payload["messages"][0], {
            "role": "system",
            "content": "可能出现香蕉和苹果。",
        })
        self.assertEqual(payload["messages"][1]["role"], "user")

    def test_qwen_drops_context_echo_and_overlong_text(self):
        engine = asr_backend.Qwen3AsrEngine({
            "context": "抓 香蕉 苹果",
            "max_transcript_chars": 8,
        })

        self.assertEqual(engine._sanitize_transcript("抓 香蕉 苹果"), "")
        self.assertEqual(engine._sanitize_transcript("这是一段过长识别文本"), "")
        self.assertEqual(engine._sanitize_transcript("抓香蕉。"), "抓香蕉。")

    def test_qwen_context_is_disabled_by_default(self):
        fake_asr = SimpleNamespace()
        with mock.patch.object(
                asr_backend, "probe_qwen3_asr_endpoint",
                return_value="http://127.0.0.1:8063/health"):
            engine, _, context_terms = asr_backend.create_asr_engine(
                fake_asr,
                {"backend": "qwen3_asr", "qwen3_asr": {}},
                4,
                ["抓香蕉", "停止"],
            )

        self.assertEqual(context_terms, 0)
        self.assertEqual(engine.context, "")

    def test_qwen_starts_missing_local_endpoint_and_stops_owned_server(self):
        config = {
            "endpoint": "http://127.0.0.1:8063/v1/chat/completions",
            "auto_start": True,
        }
        process = mock.Mock()
        process.pid = 4321
        process.poll.return_value = None
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "llama-server"
            model_dir = Path(directory) / "model"
            text_model = model_dir / asr_backend.QWEN3_TEXT_MODEL
            log_path = Path(directory) / "server.log"
            binary.touch(mode=0o755)
            model_dir.mkdir()
            text_model.touch()
            health_results = [
                RuntimeError("not running"),
                "http://127.0.0.1:8063/health",
            ]
            with mock.patch.object(
                    asr_backend, "probe_qwen3_asr_endpoint",
                    side_effect=health_results), mock.patch.object(
                    asr_backend, "validate_qwen3_local_runtime",
                    return_value=(binary, model_dir, text_model)), \
                    mock.patch.object(
                        asr_backend.subprocess, "Popen",
                        return_value=process) as popen, mock.patch.object(
                        asr_backend.os, "killpg") as killpg:
                engine = asr_backend.Qwen3AsrEngine({
                    **config,
                    "server_log_path": str(log_path),
                }).initialize()
                engine.shutdown()

        popen.assert_called_once()
        command = popen.call_args.args[0]
        self.assertIn("--media-backend", command)
        self.assertIn("smt", command)
        killpg.assert_called_once_with(4321, asr_backend.signal.SIGTERM)
        process.wait.assert_called_once_with(timeout=10)

    def test_qwen_does_not_start_remote_endpoint(self):
        with mock.patch.object(
                asr_backend, "probe_qwen3_asr_endpoint",
                side_effect=RuntimeError("unavailable")), \
                mock.patch.object(
                    asr_backend.Qwen3AsrEngine,
                    "_start_local_server") as start:
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                asr_backend.Qwen3AsrEngine({
                    "endpoint": "http://10.0.0.2:8063/v1/chat/completions",
                    "auto_start": True,
                }).initialize()
        start.assert_not_called()

    def test_qwen_auto_start_can_be_disabled(self):
        with mock.patch.object(
                asr_backend, "probe_qwen3_asr_endpoint",
                side_effect=RuntimeError("unavailable")), \
                mock.patch.object(
                    asr_backend.Qwen3AsrEngine,
                    "_start_local_server") as start:
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                asr_backend.Qwen3AsrEngine({
                    "endpoint": (
                        "http://127.0.0.1:8063/v1/chat/completions"
                    ),
                    "auto_start": False,
                }).initialize()
        start.assert_not_called()

    def test_invalid_qwen_endpoint_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "http"):
            asr_backend.Qwen3AsrEngine({"endpoint": "127.0.0.1:8063"})


if __name__ == "__main__":
    unittest.main()
