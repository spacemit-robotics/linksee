#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for the non-ROS local voice bridge helpers."""

import unittest
import contextlib
import io
import queue
from pathlib import Path
from types import SimpleNamespace

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
            f"{local_voice_bridge.STATUS_PREFIX}"
            "[CHASSIS-UART-DIFF] RX thread started"
        )

        self.assertIsNone(local_voice_bridge.extract_status_event(line))

    def test_default_asr_channels_match_mono_usb_microphone(self):
        self.assertEqual(local_voice_bridge.default_asr_channels({}), 1)

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

    def test_pipeline_config_uses_mono_asr_input(self):
        config_path = ROOT / "config" / "grasp_pipeline.yaml"
        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.assertEqual(
            set(config["voice"]["asr"].keys()),
            {
                "device",
                "rate",
                "channels",
                "vad_trigger_threshold",
                "vad_stop_threshold",
                "vad_min_speech_duration_ms",
            },
        )
        self.assertEqual(config["voice"]["asr"]["device"], 1)
        self.assertEqual(config["voice"]["asr"]["rate"], 16000)
        self.assertEqual(config["voice"]["asr"]["channels"], 1)

    def test_pipeline_config_enables_local_tts_on_usb_audio_output(self):
        config_path = ROOT / "config" / "grasp_pipeline.yaml"
        with config_path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        self.assertEqual(
            set(config["voice"]["tts"].keys()),
            {
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
                "asr",
                "tts",
                "target_aliases",
            },
        )
        main_source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('voice["home_words"]', main_source)

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
        self.assertEqual([len(chunk) for chunk in player.chunks],
                         [8192, 8192, 3616])

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

        local_voice_bridge._read_grasp_stdout(
            proc, text_queue, running, {}, False)

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
            FakeProc(), text_queue, running, {"carrot": "胡萝卜"}, False)

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
            FakeProc(), text_queue, running, {}, False)

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
                FakeProc(), text_queue, running, {"banana": "香蕉"}, False)

        self.assertIn("[VoiceBridge] Queue TTS: 收到，准备抓取香蕉。",
                      output.getvalue())
        self.assertEqual(text_queue.get_nowait(), "收到，准备抓取香蕉。")

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
            FakeProc(), text_queue, running, {}, False, status_queue)

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
        event = (
            "state=HOMING;"
            "message=Object released, returning to observe position..."
        )

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
