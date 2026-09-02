#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for the non-ROS local voice bridge helpers."""

import contextlib
import inspect
import io
import queue
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import local_voice_bridge  # noqa: E402
import tts_node  # noqa: E402


class FakeProc:
    def __init__(self, returncode=None):
        self.returncode = returncode
        self.terminate_count = 0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminate_count += 1


class LocalVoiceBridgeTest(unittest.TestCase):
    def test_audio_epoch_invalidates_previous_command_period(self):
        epoch = local_voice_bridge.AudioEpoch()
        self.assertEqual(epoch.current(), 0)
        self.assertEqual(epoch.advance(), 1)
        self.assertEqual(epoch.current(), 1)

    def test_audio_device_name_resolves_current_index(self):
        devices = [
            (1, "Rapoo Camera: USB Audio (hw:1,0)"),
            (2, "2K USB Camera: Audio (hw:2,0)"),
        ]
        self.assertEqual(
            local_voice_bridge.resolve_audio_device(
                -1, "Rapoo Camera", lambda: devices, "capture"
            ),
            1,
        )

    def test_audio_device_name_must_be_unambiguous(self):
        devices = [(1, "USB Audio A"), (2, "USB Audio B")]
        with self.assertRaisesRegex(RuntimeError, "ambiguous"):
            local_voice_bridge.resolve_audio_device(
                -1, "USB Audio", lambda: devices, "capture"
            )

    def test_asr_loop_rearms_vad_after_every_completed_utterance(self):
        source = inspect.getsource(local_voice_bridge.run_asr_loop)
        finish_start = source.index("    def finish_speech(reason=None):")
        finish_end = source.index("    def is_playback_active():")
        finish_source = source[finish_start:finish_end]

        normal_reset = finish_source.rfind("reset_vad_state()")
        self.assertGreater(normal_reset, finish_source.index("duration ="))
        self.assertLess(normal_reset, finish_source.index("audio_queue.put"))

    def test_pipeline_ready_forces_rearm_during_active_speech(self):
        source = inspect.getsource(local_voice_bridge.run_asr_loop)
        requested_start = source.index("        requested_rearm = (")
        requested_end = source.index(
            "        if path_rearm or requested_rearm:", requested_start
        )
        requested_source = source[requested_start:requested_end]
        self.assertNotIn('not state["in_speech"]', requested_source)

        rearm_end = source.index(
            '        state["vad_audio_path"] = selected_path',
            requested_end,
        )
        rearm_source = source[requested_end:rearm_end]
        self.assertIn("reset_speech_segment()", rearm_source)
        self.assertIn("clear_pending_audio()", rearm_source)

    def test_voice_bridge_lock_rejects_second_instance(self):
        with tempfile.TemporaryDirectory() as directory:
            lock_path = Path(directory) / "voice.lock"
            first = local_voice_bridge.acquire_voice_bridge_lock(lock_path)
            self.assertIsNotNone(first)
            self.assertIsNone(local_voice_bridge.acquire_voice_bridge_lock(lock_path))

            first.close()
            third = local_voice_bridge.acquire_voice_bridge_lock(lock_path)
            self.assertIsNotNone(third)
            third.close()

    def test_build_grasp_command_uses_local_voice_transport(self):
        cmd = local_voice_bridge.build_grasp_command(
            binary="build/perceptive_grasp",
            config="config/grasp_pipeline.yaml",
            extra_args=["--step"],
        )

        self.assertEqual(cmd[0], "build/perceptive_grasp")
        self.assertIn("--voice-stdin", cmd)
        self.assertIn("--status-stdout", cmd)
        self.assertIn("--config", cmd)
        self.assertIn("config/grasp_pipeline.yaml", cmd)
        self.assertEqual(cmd[-1], "--step")

    def test_remote_endpoint_is_forwarded_to_pipeline(self):
        cmd = local_voice_bridge.build_grasp_command(
            binary="build/perceptive_grasp",
            config="config/remote.yaml",
            extra_args=["--remote-host", "10.0.91.182", "--remote-port", "9090"],
        )

        self.assertEqual(
            cmd[-4:],
            [
                "--remote-host",
                "10.0.91.182",
                "--remote-port",
                "9090",
            ],
        )

    def test_extract_status_event_accepts_only_prefixed_lines(self):
        event = "state=DONE;message=Task completed!;reason=success"

        self.assertEqual(
            local_voice_bridge.extract_status_event(
                f"{local_voice_bridge.STATUS_PREFIX}{event}"
            ),
            event,
        )
        self.assertIsNone(local_voice_bridge.extract_status_event("[Pipeline] log"))

    def test_extract_status_event_accepts_interleaved_prefix(self):
        line = (
            "[CHASSIS-UART-DIFF] RX thread started"
            f"{local_voice_bridge.STATUS_PREFIX}"
            "state=IDLE;message=Ready"
        )

        self.assertEqual(
            local_voice_bridge.extract_status_event(line),
            "state=IDLE;message=Ready",
        )

    def test_extract_status_event_rejects_corrupted_payload(self):
        line = (
            f"{local_voice_bridge.STATUS_PREFIX}[CHASSIS-UART-DIFF] RX thread started"
        )

        self.assertIsNone(local_voice_bridge.extract_status_event(line))

    def test_default_asr_channels_match_mono_usb_microphone(self):
        self.assertEqual(local_voice_bridge.default_asr_channels({}), 1)

    def test_remote_config_matches_mono_usb_microphone(self):
        config = yaml.safe_load(
            (ROOT / "config" / "grasp_pipeline_remote_mujoco_ur5e.yaml").read_text(
                encoding="utf-8"
            )
        )
        asr = config["voice"]["asr"]
        self.assertEqual(asr["device_name"], "2K USB Camera")
        self.assertEqual(asr["channels"], 1)
        self.assertEqual(asr["channel_index"], -1)
        tts = config["voice"]["tts"]
        self.assertEqual(tts["playback_device"], 1)
        self.assertEqual(tts["playback_device_name"], "2K USB Camera")

    def test_asr_debug_audio_is_mono_16k_pcm(self):
        with tempfile.TemporaryDirectory() as directory:
            path = local_voice_bridge.save_asr_audio_segment(
                directory,
                3,
                np.array([-1.0, 0.0, 1.0], dtype=np.float32),
            )
            import wave

            with wave.open(str(path), "rb") as source:
                self.assertEqual(source.getnchannels(), 1)
                self.assertEqual(source.getframerate(), 16000)
                self.assertEqual(source.getsampwidth(), 2)
                self.assertEqual(source.getnframes(), 3)

    def test_asr_hotwords_include_configured_command_phrases(self):
        hotwords = local_voice_bridge.build_asr_hotwords(
            {
                "trigger_words": ["抓", "pick"],
                "cancel_words": ["停止"],
                "home_words": ["回家"],
                "target_aliases": {"香蕉": "banana", "杯子": "cup"},
            }
        )

        self.assertIn("抓香蕉", hotwords)
        self.assertIn("抓杯子", hotwords)
        self.assertIn("香蕉", hotwords)
        self.assertIn("停止", hotwords)
        self.assertNotIn("pick香蕉", hotwords)
        self.assertEqual(len(hotwords), len(set(hotwords)))

    def test_capture_hw_name_is_parsed_from_device_list(self):
        devices = [
            (0, "snd-es8326: d4026800.i2s1-ES8326 HiFi (hw:0,0)"),
            (1, "2K USB Camera: Audio (hw:1,0)"),
        ]

        self.assertEqual(
            local_voice_bridge.capture_hw_name(1, lambda: devices),
            "hw:1,0",
        )

    def test_capture_hw_name_falls_back_to_arecord_list(self):
        arecord_output = """
card 0: sndes8326 [snd-es8326], device 0: d4026800.i2s1-ES8326 HiFi ES8326 HiFi-0 []
card 1: Camera [2K USB Camera], device 0: USB Audio [USB Audio]
"""

        def fake_runner(command, *args, **kwargs):
            del args, kwargs
            self.assertEqual(command, ["arecord", "-l"])
            return SimpleNamespace(returncode=0, stdout=arecord_output)

        self.assertEqual(
            local_voice_bridge.capture_hw_name(
                1,
                lambda: [(1, "2K USB Camera: Audio")],
                fake_runner,
            ),
            "hw:1,0",
        )

    def test_candidate_capture_rates_are_deduplicated(self):
        self.assertEqual(
            local_voice_bridge.candidate_capture_rates(48000),
            [48000, 44100, 32000, 16000, 8000],
        )
        self.assertEqual(
            local_voice_bridge.candidate_capture_rates(16000),
            [16000, 48000, 44100, 32000, 8000],
        )

    def test_capture_mixer_targets_selected_hw_card(self):
        commands = []

        def fake_runner(command, *args, **kwargs):
            del args, kwargs
            commands.append(command)
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        result = local_voice_bridge.configure_capture_mixer(
            1,
            60,
            lambda: [(1, "Rapoo Camera: USB Audio (hw:3,0)")],
            fake_runner,
        )

        self.assertTrue(result)
        self.assertEqual(
            commands,
            [
                ["amixer", "-c", "3", "sset", "Mic", "60%", "cap"],
            ],
        )

    def test_spacemit_capture_rate_probe_uses_audio_capture_start_result(self):
        class FakeAudio:
            probed_rates = []

            @classmethod
            def init(cls, sample_rate, channels, chunk_size, capture_device):
                cls.probed_rates.append(sample_rate)
                self.assertEqual(channels, 1)
                self.assertEqual(chunk_size, sample_rate * 2 // 25)
                self.assertEqual(capture_device, 1)

        class FakeCapture:
            def __init__(self):
                self.rate = FakeAudio.probed_rates[-1]
                self.callback = None
                self.stopped = False

            def set_callback(self, callback):
                self.callback = callback

            def start(self):
                return self.rate == 16000

            def stop(self):
                self.stopped = True

        rate = local_voice_bridge.resolve_spacemit_capture_rate(
            device=1,
            requested_rate=48000,
            channels=1,
            audio_module=FakeAudio,
            capture_cls=FakeCapture,
        )

        self.assertEqual(rate, 16000)
        self.assertEqual(FakeAudio.probed_rates, [48000, 44100, 32000, 16000])

    def test_spacemit_capture_format_falls_back_to_stereo(self):
        class FakeAudio:
            probed_formats = []

            @classmethod
            def init(cls, sample_rate, channels, chunk_size, capture_device):
                cls.probed_formats.append((sample_rate, channels))
                self.assertEqual(chunk_size, sample_rate * channels * 2 // 25)
                self.assertEqual(capture_device, 0)

        class FakeCapture:
            def __init__(self):
                self.rate, self.channels = FakeAudio.probed_formats[-1]
                self.stopped = False

            def set_callback(self, callback):
                self.callback = callback

            def start(self):
                return self.rate == 16000 and self.channels == 2

            def stop(self):
                self.stopped = True

        rate, channels = local_voice_bridge.resolve_spacemit_capture_format(
            device=0,
            requested_rate=16000,
            requested_channels=None,
            audio_module=FakeAudio,
            capture_cls=FakeCapture,
        )

        self.assertEqual((rate, channels), (16000, 2))
        self.assertEqual(FakeAudio.probed_formats[0], (16000, 2))

    def test_spacemit_capture_format_keeps_non_primary_device_mono(self):
        class FakeAudio:
            probed_formats = []

            @classmethod
            def init(cls, sample_rate, channels, chunk_size, capture_device):
                cls.probed_formats.append((sample_rate, channels))
                self.assertEqual(chunk_size, sample_rate * channels * 2 // 25)
                self.assertEqual(capture_device, 1)

        class FakeCapture:
            def __init__(self):
                self.rate, self.channels = FakeAudio.probed_formats[-1]

            def set_callback(self, callback):
                self.callback = callback

            def start(self):
                return True

            def stop(self):
                pass

        rate, channels = local_voice_bridge.resolve_spacemit_capture_format(
            device=1,
            requested_rate=16000,
            requested_channels=None,
            audio_module=FakeAudio,
            capture_cls=FakeCapture,
        )

        self.assertEqual((rate, channels), (16000, 1))
        self.assertEqual(FakeAudio.probed_formats, [(16000, 1)])

    def test_pipeline_config_preserves_validated_mono_sensevoice_default(self):
        config_path = ROOT / "config" / "grasp_pipeline.yaml"
        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.assertEqual(
            set(config["voice"]["asr"].keys()),
            {
                "backend",
                "device",
                "rate",
                "channels",
                "channel_index",
                "mixer_volume",
                "sensevoice",
                "qwen3_asr",
                "vad_trigger_threshold",
                "vad_stop_threshold",
                "vad_min_speech_duration_ms",
                "vad_endpoint_hold_ms",
                "vad_max_speech_duration_ms",
            },
        )
        self.assertEqual(config["voice"]["asr"]["device"], 1)
        self.assertEqual(config["voice"]["asr"]["rate"], 16000)
        self.assertEqual(config["voice"]["asr"]["channels"], 1)
        self.assertEqual(config["voice"]["asr"]["channel_index"], -1)
        self.assertEqual(config["voice"]["asr"]["mixer_volume"], 40)
        self.assertEqual(config["voice"]["asr"]["backend"], "sensevoice")
        self.assertEqual(config["voice"]["asr"]["sensevoice"]["provider"], "cpu")
        self.assertEqual(config["voice"]["asr"]["qwen3_asr"]["model"], "qwen3-asr")
        self.assertEqual(config["voice"]["asr"]["vad_endpoint_hold_ms"], 600)
        self.assertEqual(config["voice"]["asr"]["vad_max_speech_duration_ms"], 4000)

    def test_pipeline_config_enables_local_tts_on_usb_audio_output(self):
        config_path = ROOT / "config" / "grasp_pipeline.yaml"
        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.assertEqual(
            set(config["voice"]["tts"].keys()),
            {
                "enabled",
                "engine",
                "playback_device",
                "playback_rate",
                "channels",
                "speed",
                "volume",
                "mixer_volume",
                "speak_all_states",
            },
        )
        self.assertTrue(config["voice"]["tts"]["enabled"])
        self.assertEqual(config["voice"]["tts"]["playback_device"], 1)
        self.assertGreaterEqual(config["voice"]["tts"]["mixer_volume"], -1)
        self.assertLessEqual(config["voice"]["tts"]["mixer_volume"], 100)

    def test_pipeline_config_keeps_only_user_facing_voice_keys(self):
        config_path = ROOT / "config" / "grasp_pipeline.yaml"
        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.assertEqual(
            set(config["voice"].keys()),
            {
                "trigger_words",
                "cancel_words",
                "home_words",
                "split_command_timeout_ms",
                "echo_cancellation",
                "asr",
                "tts",
                "target_aliases",
            },
        )
        main_source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('voice["home_words"]', main_source)

    def test_pipeline_config_enables_webrtc_aec_for_usb_audio(self):
        config_path = ROOT / "config" / "grasp_pipeline.yaml"
        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        echo_config = config["voice"]["echo_cancellation"]
        self.assertEqual(echo_config["mode"], "webrtc_aec")
        self.assertGreaterEqual(echo_config["delay_ms"], 0)
        self.assertTrue(echo_config["noise_suppression"])

    def test_echo_cancellation_mode_is_validated(self):
        mode, config = local_voice_bridge.load_echo_cancellation_config(
            {
                "echo_cancellation": {"mode": "hardware_aec"},
            }
        )

        self.assertEqual(mode, "hardware_aec")
        self.assertEqual(config["mode"], "hardware_aec")
        with self.assertRaises(ValueError):
            local_voice_bridge.load_echo_cancellation_config(
                {
                    "echo_cancellation": {"mode": "unknown"},
                }
            )

    def test_voice_aec_library_uses_loader_discovery(self):
        with contextlib.ExitStack() as stack:
            stack.enter_context(
                mock.patch.object(Path, "is_file", return_value=False)
            )
            stack.enter_context(mock.patch.object(
                local_voice_bridge.ctypes.util,
                "find_library",
                return_value="libperceptive_voice_aec.so",
            ))
            library = local_voice_bridge.find_voice_aec_library(
                "build/perceptive_grasp",
                environ={},
            )

        self.assertEqual(library, "libperceptive_voice_aec.so")

    def test_playback_reference_timeline_applies_delay(self):
        now = 10.0
        timeline = local_voice_bridge.PlaybackReferenceTimeline(
            sample_rate=16000,
            delay_ms=50,
            clock=lambda: now,
        )
        audio = np.full(160, 16384, dtype=np.int16)

        start = timeline.schedule(audio, 16000, 1)
        before = timeline.read(160, end_time=start)
        during = timeline.read(160, end_time=start + 0.01)

        self.assertAlmostEqual(start, 10.05)
        np.testing.assert_array_equal(before, np.zeros(160, dtype=np.float32))
        np.testing.assert_allclose(during, 0.5, atol=1e-4)

    def test_webrtc_aec_output_is_limited_to_playback_and_echo_tail(self):
        tail_until = local_voice_bridge.update_aec_tail(
            frame_end=10.0,
            playback_active=True,
            has_reference=False,
            tail_until=0.0,
        )

        self.assertAlmostEqual(tail_until, 10.2)
        self.assertLessEqual(10.1, tail_until)
        self.assertGreater(10.3, tail_until)
        self.assertEqual(
            local_voice_bridge.update_aec_tail(
                frame_end=10.3,
                playback_active=False,
                has_reference=False,
                tail_until=tail_until,
            ),
            tail_until,
        )

    def test_vad_rearms_when_idle_audio_path_changes(self):
        self.assertTrue(
            local_voice_bridge.should_rearm_vad_for_audio_path("aec", "raw", False)
        )
        self.assertFalse(
            local_voice_bridge.should_rearm_vad_for_audio_path("aec", "raw", True)
        )
        self.assertFalse(
            local_voice_bridge.should_rearm_vad_for_audio_path("raw", "raw", False)
        )

    def test_active_utterance_keeps_its_audio_path(self):
        raw = np.full(160, 0.25, dtype=np.float32)
        aec = np.full(160, 0.50, dtype=np.float32)

        samples, path = local_voice_bridge.select_speech_samples(
            False, raw, aec, prefer_aec=False
        )
        self.assertEqual(path, "raw")
        np.testing.assert_array_equal(samples, raw)

        samples, path = local_voice_bridge.select_speech_samples(
            True, raw, aec, current_path="aec", prefer_aec=False
        )
        self.assertEqual(path, "aec")
        np.testing.assert_array_equal(samples, aec)

        samples, path = local_voice_bridge.select_speech_samples(
            False, raw, aec, prefer_aec=True
        )
        self.assertEqual(path, "aec")
        np.testing.assert_array_equal(samples, aec)

    def test_repeated_vad_start_does_not_restart_active_utterance(self):
        start = SimpleNamespace(is_speech_start=True, is_speech_end=False)
        end = SimpleNamespace(is_speech_start=False, is_speech_end=True)

        self.assertEqual(local_voice_bridge.classify_vad_event(False, start), "start")
        self.assertEqual(local_voice_bridge.classify_vad_event(True, start), "append")
        self.assertEqual(local_voice_bridge.classify_vad_event(True, None), "append")
        self.assertEqual(local_voice_bridge.classify_vad_event(True, end), "finish")

    def test_vad_endpoint_hold_merges_short_pause(self):
        start = SimpleNamespace(is_speech_start=True, is_speech_end=False)
        weak_start = SimpleNamespace(
            is_speech_start=True,
            is_speech_end=False,
            probability=0.15,
        )
        end = SimpleNamespace(is_speech_start=False, is_speech_end=True)
        neutral = SimpleNamespace(is_speech_start=False, is_speech_end=False)

        pending = local_voice_bridge.update_vad_endpoint_hold(True, end, 0, 640)
        self.assertEqual(pending, 640)
        pending = local_voice_bridge.update_vad_endpoint_hold(
            True, neutral, pending, 640
        )
        self.assertEqual(pending, 1280)
        pending = local_voice_bridge.update_vad_endpoint_hold(
            True, weak_start, pending, 640, resume_threshold=0.3
        )
        self.assertEqual(pending, 1920)
        pending = local_voice_bridge.update_vad_endpoint_hold(True, start, pending, 640)
        self.assertEqual(pending, 0)

    def test_asr_capture_start_failure_releases_asr_and_aec(self):
        class FakeVadConfig:
            @classmethod
            def preset(cls, name):
                del name
                return cls()

            def with_trigger_threshold(self, value):
                del value
                return self

            def with_stop_threshold(self, value):
                del value
                return self

            def with_min_speech_duration(self, value):
                del value
                return self

            def with_smoothing(self, value):
                del value
                return self

        class FakeAsrConfig:
            def __init__(self):
                self._config = SimpleNamespace(num_threads=0)

        class FakeAsrBackend:
            backend_name = "fake"

            def __init__(self):
                self.shutdown_count = 0

            def recognize(self, audio):
                del audio
                return None

            def shutdown(self):
                self.shutdown_count += 1

        asr_backend = FakeAsrBackend()

        class FakeAsrEngine:
            def __init__(self, config):
                del config

            def initialize(self):
                return asr_backend

        class FakeCapture:
            def set_callback(self, callback):
                self.callback = callback

            def start(self):
                return False

        fake_asr = SimpleNamespace(
            Config=FakeAsrConfig,
            Engine=FakeAsrEngine,
            Language=SimpleNamespace(ZH="zh"),
        )
        fake_audio = SimpleNamespace(
            AudioCapture=FakeCapture,
            init=lambda **kwargs: None,
        )
        fake_vad = SimpleNamespace(
            VadConfig=FakeVadConfig,
            VadEngine=lambda config: SimpleNamespace(engine_name="fake"),
        )
        fake_aec = SimpleNamespace(frame_size=160, close_count=0)

        def close_aec():
            fake_aec.close_count += 1

        fake_aec.close = close_aec
        args = SimpleNamespace(
            config="config/grasp_pipeline.yaml",
            binary="build/perceptive_grasp",
            device=None,
            rate=None,
            channels=None,
            vad_trigger_threshold=None,
            vad_stop_threshold=None,
            vad_min_speech_duration_ms=None,
            asr_threads=4,
            pre_buffer_ms=800,
        )
        running = threading.Event()
        running.set()

        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.dict(
                sys.modules,
                {
                    "spacemit_asr": fake_asr,
                    "spacemit_audio": fake_audio,
                    "spacemit_vad": fake_vad,
                },
            ))
            stack.enter_context(mock.patch.object(
                local_voice_bridge,
                "load_voice_config",
                return_value={
                    "echo_cancellation": {"mode": "webrtc_aec"},
                    "asr": {"device": 1, "rate": 16000, "channels": 1},
                },
            ))
            stack.enter_context(mock.patch.object(
                local_voice_bridge,
                "resolve_spacemit_capture_format",
                return_value=(16000, 1),
            ))
            stack.enter_context(mock.patch.object(
                local_voice_bridge,
                "find_voice_aec_library",
                return_value="/tmp/libperceptive_voice_aec.so",
            ))
            stack.enter_context(mock.patch.object(
                local_voice_bridge, "WebRtcAecProcessor", return_value=fake_aec
            ))
            local_voice_bridge.run_asr_loop(
                args,
                FakeProc(),
                running,
                threading.Event(),
                SimpleNamespace(),
            )

        self.assertEqual(asr_backend.shutdown_count, 1)
        self.assertEqual(fake_aec.close_count, 1)
        self.assertFalse(running.is_set())

    def test_tts_mixer_volume_targets_playback_hw_card(self):
        devices = [
            (0, "snd-es8326: d4026800.i2s1-ES8326 HiFi (hw:0,0)"),
            (1, "2K USB Camera: Audio (hw:1,0)"),
        ]
        commands = []

        def fake_runner(command, *args, **kwargs):
            del args, kwargs
            commands.append(command)
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        self.assertTrue(
            tts_node.configure_playback_mixer(
                playback_device=1,
                mixer_volume=80,
                list_devices=lambda: devices,
                runner=fake_runner,
            )
        )
        self.assertEqual(
            commands,
            [["amixer", "-c", "1", "sset", "PCM", "80%", "unmute"]],
        )

    def test_tts_audio_write_uses_checked_chunks(self):
        class FakePlayer:
            def __init__(self):
                self.chunks = []

            def write(self, data):
                self.chunks.append(data)
                return True

        player = FakePlayer()
        payload = b"x" * 20000

        self.assertTrue(tts_node.write_audio_bytes(player, payload, channels=1))
        self.assertEqual([len(chunk) for chunk in player.chunks], [8192, 8192, 3616])

    def test_tts_worker_marks_playback_active_while_audio_is_written(self):
        playback_transitions = []
        test_case = self

        class PlaybackState:
            active = False

            def set(self):
                self.active = True
                playback_transitions.append("set")

            def clear(self):
                self.active = False
                playback_transitions.append("clear")

        playback_state = PlaybackState()

        class FakePlayer:
            @staticmethod
            def list_devices():
                return []

            def __init__(self, device):
                del device

            def start(self):
                return True

            def write(self, data):
                del data
                test_case.assertTrue(playback_state.active)
                return True

            def stop(self):
                pass

            def close(self):
                pass

        fake_audio = SimpleNamespace(
            init=lambda **kwargs: None,
            AudioPlayer=FakePlayer,
        )

        class FakeTtsConfig:
            speech_rate = 1.0
            volume = 80

            @classmethod
            def preset(cls, preset):
                del preset
                return cls()

        class FakeTtsEngine:
            def __init__(self, config):
                del config

            def synthesize(self, text):
                del text
                return SimpleNamespace(
                    is_success=True,
                    audio_int16=np.zeros(128, dtype=np.int16),
                    sample_rate=48000,
                    duration_ms=10,
                    rtf=0.1,
                )

        fake_tts = SimpleNamespace(Config=FakeTtsConfig, Engine=FakeTtsEngine)
        text_queue = queue.Queue()
        text_queue.put("测试播报")
        text_queue.put(None)
        running = threading.Event()
        running.set()
        playback_startup_complete = threading.Event()

        with mock.patch.dict(
            sys.modules, {"spacemit_audio": fake_audio, "spacemit_tts": fake_tts}
        ):
            tts_node.run_tts_worker(
                text_queue,
                running,
                "matcha_zh",
                1,
                48000,
                1,
                1.0,
                80,
                False,
                -1,
                playback_state,
                0,
                None,
                None,
                playback_startup_complete,
            )

        self.assertEqual(playback_transitions, ["set", "clear", "clear"])
        self.assertFalse(playback_state.active)
        self.assertTrue(playback_startup_complete.is_set())

    def test_tts_worker_disables_speech_when_player_start_fails(self):
        class FakePlayer:
            @staticmethod
            def list_devices():
                return []

            def __init__(self, device):
                del device

            def start(self):
                return False

        fake_audio = SimpleNamespace(
            init=lambda **kwargs: None,
            AudioPlayer=FakePlayer,
        )
        synthesize = mock.Mock()

        class FakeTtsConfig:
            speech_rate = 1.0
            volume = 80

            @classmethod
            def preset(cls, preset):
                del preset
                return cls()

        fake_tts = SimpleNamespace(
            Config=FakeTtsConfig,
            Engine=lambda config: SimpleNamespace(synthesize=synthesize),
        )
        text_queue = queue.Queue()
        text_queue.put("系统已就绪。")
        text_queue.put(None)
        running = threading.Event()
        running.set()
        startup_complete = threading.Event()
        completed = []

        with mock.patch.dict(
            sys.modules, {"spacemit_audio": fake_audio, "spacemit_tts": fake_tts}
        ):
            tts_node.run_tts_worker(
                text_queue,
                running,
                "matcha_zh",
                2,
                48000,
                1,
                1.0,
                80,
                False,
                -1,
                playback_startup_complete=startup_complete,
                playback_complete_observer=completed.append,
            )

        self.assertTrue(startup_complete.is_set())
        self.assertEqual(completed, ["系统已就绪。"])
        synthesize.assert_not_called()

    def test_status_event_fields_are_parsed(self):
        event = "state=IDLE;message=Ready;target=banana"

        self.assertEqual(
            local_voice_bridge.parse_status_fields(event),
            {"state": "IDLE", "message": "Ready", "target": "banana"},
        )

    def test_status_event_fields_unescape_semicolon(self):
        event = "state=IDLE;message=Home position reached\\; exiting"

        self.assertEqual(
            local_voice_bridge.parse_status_fields(event),
            {"state": "IDLE", "message": "Home position reached; exiting"},
        )

    def test_ready_status_is_spoken_by_default(self):
        event = "state=IDLE;message=Ready"

        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, False),
            "系统已就绪。",
        )

    def test_waiting_status_requests_vad_rearm(self):
        class FakeStdout:
            def __init__(self):
                self.lines = [
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=IDLE;message=Voice: waiting for next command\n",
                    "",
                ]

            def readline(self):
                return self.lines.pop(0)

        proc = SimpleNamespace(
            stdout=FakeStdout(),
            poll=lambda: None,
            terminate=lambda: None,
        )
        running = local_voice_bridge.make_running_event()
        rearm_requested = threading.Event()

        local_voice_bridge._read_grasp_stdout(
            proc,
            queue.Queue(),
            running,
            {},
            False,
            vad_rearm_requested=rearm_requested,
        )

        self.assertTrue(rearm_requested.is_set())

    def test_home_exit_status_stops_voice_bridge_without_extra_tts(self):
        class FakeStdout:
            def __init__(self):
                self.lines = [
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=IDLE;message=Home position reached\\; exiting\n",
                    "",
                ]

            def readline(self):
                return self.lines.pop(0)

        class FakeProc:
            def __init__(self):
                self.stdout = FakeStdout()
                self.returncode = None
                self.terminate_count = 0

            def poll(self):
                return self.returncode

            def terminate(self):
                self.terminate_count += 1
                self.returncode = 0

        proc = FakeProc()
        text_queue = queue.Queue()
        running = local_voice_bridge.make_running_event()

        local_voice_bridge._read_grasp_stdout(proc, text_queue, running, {}, False)

        self.assertFalse(running.is_set())
        self.assertEqual(proc.terminate_count, 1)
        self.assertIsNone(text_queue.get_nowait())

    def test_status_reader_replaces_stale_tts_queue_with_latest_status(self):
        class FakeStdout:
            def __init__(self):
                self.lines = [
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=ERROR;message=Target not found: old\\; candidates: none;"
                    "target=old;reason=target_not_found\n",
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=OBSERVING;message=Moving to observe, target: carrot;"
                    "target=carrot\n",
                    "",
                ]

            def readline(self):
                return self.lines.pop(0)

        class FakeProc:
            stdout = FakeStdout()

            def poll(self):
                return None

            def terminate(self):
                pass

        text_queue = queue.Queue()
        running = local_voice_bridge.make_running_event()

        local_voice_bridge._read_grasp_stdout(
            FakeProc(), text_queue, running, {"carrot": "胡萝卜"}, False
        )

        self.assertEqual(text_queue.get_nowait(), "收到，准备抓取胡萝卜。")
        self.assertTrue(text_queue.empty())

    def test_waiting_prompt_does_not_displace_pending_result_speech(self):
        class FakeStdout:
            def __init__(self):
                self.lines = [
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=DONE;message=Task completed!;reason=success\n",
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=IDLE;message=Voice: waiting for next command\n",
                    "",
                ]

            def readline(self):
                return self.lines.pop(0)

        class FakeProc:
            stdout = FakeStdout()

            def poll(self):
                return None

            def terminate(self):
                pass

        text_queue = queue.Queue()
        running = local_voice_bridge.make_running_event()

        local_voice_bridge._read_grasp_stdout(
            FakeProc(), text_queue, running, {}, False
        )

        self.assertEqual(text_queue.get_nowait(), "抓取完成。")
        self.assertTrue(text_queue.empty())

    def test_status_reader_logs_queued_tts_text(self):
        class FakeStdout:
            def __init__(self):
                self.lines = [
                    f"{local_voice_bridge.STATUS_PREFIX}"
                    "state=OBSERVING;message=Moving to observe, target: banana;"
                    "target=banana\n",
                    "",
                ]

            def readline(self):
                return self.lines.pop(0)

        class FakeProc:
            stdout = FakeStdout()

        text_queue = queue.Queue()
        running = local_voice_bridge.make_running_event()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            local_voice_bridge._read_grasp_stdout(
                FakeProc(), text_queue, running, {"banana": "香蕉"}, False
            )

        self.assertIn(
            "[VoiceBridge] Queue TTS: 收到，准备抓取香蕉。", output.getvalue()
        )
        self.assertEqual(text_queue.get_nowait(), "收到，准备抓取香蕉。")

    def test_initial_detection_status_acknowledges_target(self):
        event = (
            "state=DETECTING;"
            "message=Detecting target before observation, target: cup;"
            "target=cup"
        )
        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, False),
            "收到，准备抓取cup。",
        )

    def test_strategy_observation_does_not_repeat_acknowledgement(self):
        event = (
            "state=OBSERVING;"
            "message=Strategy selected: side; moving to matching "
            "observation pose"
        )
        self.assertIsNone(
            local_voice_bridge.status_to_speech(event, {}, False),
        )

    def test_ready_log_is_used_when_ready_status_is_missing(self):
        class FakeStdout:
            def __init__(self):
                self.lines = [
                    f"{local_voice_bridge.READY_LOG_LINE}\n",
                    "",
                ]

            def readline(self):
                return self.lines.pop(0)

        class FakeProc:
            stdout = FakeStdout()

            def poll(self):
                return None

            def terminate(self):
                pass

        text_queue = queue.Queue()
        status_queue = queue.Queue()
        running = local_voice_bridge.make_running_event()

        local_voice_bridge._read_grasp_stdout(
            FakeProc(), text_queue, running, {}, False, status_queue
        )

        self.assertEqual(
            status_queue.get_nowait(),
            local_voice_bridge.READY_STATUS_EVENT,
        )
        self.assertEqual(text_queue.get_nowait(), "系统已就绪。")

    def test_cancel_status_is_spoken_without_torque_release(self):
        event = "state=IDLE;message=Cancelling; keeping observe pose"

        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, False),
            "已停止抓取。",
        )

    def test_home_status_is_spoken(self):
        event = "state=IDLE;message=Returning home\\; exiting after home"

        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, False),
            "收到，回到初始位置。",
        )

    def test_home_exit_status_is_spoken(self):
        event = "state=IDLE;message=Home position reached\\; exiting"

        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, False),
            "已回到初始位置，程序退出。",
        )

    def test_observe_return_status_is_spoken(self):
        event = "state=HOMING;message=Object released, returning to observe position..."

        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, False),
            "已释放，正在回到观察位。",
        )

    def test_homing_state_speaks_observe_return(self):
        event = "state=HOMING;message="

        self.assertEqual(
            local_voice_bridge.status_to_speech(event, {}, True),
            "已释放，正在回到观察位。",
        )

    def test_wait_for_grasp_ready_accepts_ready_status(self):
        events = queue.Queue()
        events.put("state=IDLE;message=Ready")
        running = local_voice_bridge.make_running_event()

        self.assertTrue(
            local_voice_bridge.wait_for_grasp_ready(
                FakeProc(), events, running, timeout_sec=0.1
            )
        )

    def test_wait_for_grasp_ready_rejects_startup_stop(self):
        events = queue.Queue()
        events.put("state=IDLE;message=Stopped")
        running = local_voice_bridge.make_running_event()

        self.assertFalse(
            local_voice_bridge.wait_for_grasp_ready(
                FakeProc(), events, running, timeout_sec=0.1
            )
        )
        self.assertFalse(running.is_set())

    def test_shutdown_request_is_idempotent(self):
        proc = FakeProc()
        running = local_voice_bridge.make_running_event()
        text_queue = queue.Queue(maxsize=1)

        local_voice_bridge.request_shutdown(proc, running, text_queue)
        local_voice_bridge.request_shutdown(proc, running, text_queue)

        self.assertFalse(running.is_set())
        self.assertEqual(proc.terminate_count, 1)
        self.assertIsNone(text_queue.get_nowait())


if __name__ == "__main__":
    unittest.main()
