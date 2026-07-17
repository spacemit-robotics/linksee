#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""ROS2 status text -> TTS playback node for perceptive_grasp."""

import argparse
import os
import queue
import re
import signal
import subprocess
import sys
import threading
from typing import Dict, Optional


DEFAULT_SPACEMIT_PYPI_INDEX = (
    "https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple"
)


def import_ros2_or_exit():
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
        return rclpy, ExternalShutdownException, Node, String
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
            "  python3 scripts/tts_node.py --config config/grasp_pipeline.yaml\n"
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
    tts = {}
    aliases = {}
    in_voice = False
    section = None

    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = _strip_comment(raw.rstrip("\n"))
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(" "))
            stripped = line.strip()

            if indent == 0:
                in_voice = stripped == "voice:"
                section = None
                continue
            if not in_voice or ":" not in stripped:
                continue

            key, value = stripped.split(":", 1)
            key = key.strip()
            value = value.strip()
            if indent == 2:
                if key in ("tts", "target_aliases") and value == "":
                    section = key
                else:
                    voice[key] = _parse_scalar(value)
                    section = None
            elif indent == 4 and section == "tts":
                tts[key] = _parse_scalar(value)
            elif indent == 4 and section == "target_aliases":
                aliases[key] = _parse_scalar(value)

    if tts:
        voice["tts"] = tts
    if aliases:
        voice["target_aliases"] = aliases
    return voice


def engine_to_preset(engine_name: str) -> str:
    normalized = (engine_name or "matcha:zh").strip().lower()
    mapping = {
        "matcha:zh": "matcha_zh",
        "matcha_zh": "matcha_zh",
        "zh": "matcha_zh",
        "matcha:en": "matcha_en",
        "matcha_en": "matcha_en",
        "en": "matcha_en",
        "matcha:zh-en": "matcha_zh_en",
        "matcha:zh_en": "matcha_zh_en",
        "matcha_zh_en": "matcha_zh_en",
        "zh-en": "matcha_zh_en",
        "zh_en": "matcha_zh_en",
        "kokoro": "kokoro",
    }
    return mapping.get(normalized, normalized.replace(":", "_").replace("-", "_"))


DEFAULT_TARGET_ALIASES = {
    "香蕉": "banana", "苹果": "apple", "红苹果": "apple",
    "青苹果": "apple", "橙子": "orange", "桔子": "orange",
    "橘子": "orange", "橙": "orange", "胡萝卜": "carrot",
    "萝卜": "carrot", "红萝卜": "carrot", "胡罗卜": "carrot",
    "中罗": "carrot", "走罗": "carrot", "瓶子": "bottle",
    "水瓶": "bottle", "矿泉水": "bottle", "杯子": "cup",
    "水杯": "cup", "碗": "bowl", "勺子": "spoon",
    "叉子": "fork", "剪刀": "scissors",
}


def make_reverse_aliases(aliases: Dict[str, str]) -> Dict[str, str]:
    aliases = aliases or DEFAULT_TARGET_ALIASES
    reverse = {}
    for spoken, label in aliases.items():
        if label not in reverse:
            reverse[label] = spoken
    return reverse


def label_to_spoken(label: str, reverse_aliases: Dict[str, str]) -> str:
    if not label:
        return ""
    return reverse_aliases.get(label, label)


def parse_status_event(message: str) -> Dict[str, str]:
    fields = {}
    key = []
    value = []
    reading_key = True
    escape = False
    current_key = ""

    def commit():
        if current_key:
            fields[current_key] = "".join(value).strip()

    for ch in message:
        if escape:
            (key if reading_key else value).append(ch)
            escape = False
            continue
        if ch == "\\":
            escape = True
            continue
        if reading_key and ch == "=":
            current_key = "".join(key).strip()
            key = []
            reading_key = False
            continue
        if not reading_key and ch == ";":
            commit()
            current_key = ""
            value = []
            reading_key = True
            continue
        (key if reading_key else value).append(ch)

    if not reading_key:
        commit()
    return fields


def _extract_target(message: str) -> str:
    match = re.search(r"target:\s*([^;,]+)", message)
    if match:
        return match.group(1).strip()
    match = re.search(r"Target not found:\s*([^;,]+)", message)
    if match:
        return match.group(1).strip()
    return ""


def status_to_speech(message: str, reverse_aliases: Dict[str, str],
                     speak_all_states: bool) -> Optional[str]:
    msg = message.strip()
    if not msg:
        return None

    event = parse_status_event(msg)
    if event.get("state"):
        state = event.get("state", "")
        reason = event.get("reason", "")
        detail = event.get("message", "")
        target = label_to_spoken(event.get("target", ""), reverse_aliases)

        if state == "OBSERVING":
            return f"收到，准备抓取{target}。" if target else "收到，准备抓取。"
        if reason == "target_not_found":
            return f"没有看到{target}。" if target else "没有看到指定物体。"
        if reason == "grasp_empty":
            return "没有抓到物体。"
        if reason == "depth_invalid":
            return "目标深度无效。"
        if reason == "out_of_workspace":
            return "目标超出机械臂工作范围。"
        if reason == "ik_failed":
            return "目标位置不可达。"
        if reason == "timeout":
            return "动作超时。"
        if reason == "cancelled":
            return "已停止抓取。"
        if reason == "success" or state == "DONE":
            return "抓取完成。"
        if state == "ERROR":
            return "抓取失败。"
        if state == "IDLE" and detail == "Ready":
            return "系统已就绪。"
        if detail.startswith("Cancelled") or detail.startswith("Cancelling"):
            return "已停止抓取。"
        if detail.startswith("Home position reached; exiting"):
            return "已回到初始位置，程序退出。"
        if detail.startswith("Returning home"):
            return "收到，回到初始位置。"
        if detail.startswith("Object released, returning to observe"):
            return "已释放，正在回到观察位。"
        if detail.startswith("Voice: waiting"):
            return "请继续说要抓取的物体。"

        if speak_all_states:
            translations = {
                "IDLE": "系统已就绪。",
                "DETECTING": "正在检测目标。",
                "PLANNING": "目标已确认，正在规划。",
                "APPROACHING": "正在接近目标。",
                "GRASPING": "正在抓取。",
                "LIFTING": "已抓住，正在抬起。",
                "PLACING": "正在放置。",
                "HOMING": "已释放，正在回到观察位。",
            }
            return translations.get(state)
        return None

    if msg.startswith("Moving to observe"):
        target = label_to_spoken(_extract_target(msg), reverse_aliases)
        return f"收到，准备抓取{target}。" if target else "收到，准备抓取。"
    if msg == "Ready":
        return "系统已就绪。"
    if msg.startswith("Target not found"):
        target = label_to_spoken(_extract_target(msg), reverse_aliases)
        return f"没有看到{target}。" if target else "没有看到指定物体。"
    if "Grasp empty" in msg:
        return "没有抓到物体。"
    if "Gripper close timeout" in msg:
        return "夹爪闭合超时。"
    if "IK failed" in msg or "move failed" in msg:
        return "目标位置不可达。"
    if "depth invalid" in msg:
        return "目标深度无效。"
    if "out of workspace" in msg:
        return "目标超出机械臂工作范围。"
    if msg.startswith("Cancelled") or msg.startswith("Cancelling"):
        return "已停止抓取。"
    if msg.startswith("Home position reached; exiting"):
        return "已回到初始位置，程序退出。"
    if msg.startswith("Returning home"):
        return "收到，回到初始位置。"
    if msg.startswith("Object released, returning to observe"):
        return "已释放，正在回到观察位。"
    if msg == "Task completed!":
        return "抓取完成。"
    if msg.startswith("Voice: waiting"):
        return "请继续说要抓取的物体。"

    if speak_all_states:
        translations = {
            "Ready": "系统已就绪。",
            "Arm retracted, detecting targets...": "正在检测目标。",
            "Target stable, planning grasp...": "目标已确认，正在规划。",
            "Moving to pre-grasp...": "正在接近目标。",
            "At pre-grasp, executing grasp...": "正在抓取。",
            "Object held, lifting...": "已抓住，正在抬起。",
            "Lift completed, placing...": "正在放置。",
            "Object released, returning to observe position...": "已释放，正在回到观察位。",
            "Stopped": "已停止。",
        }
        return translations.get(msg, msg)
    return None


def list_devices():
    try:
        from spacemit_audio import AudioPlayer
    except ImportError as exc:
        index_url = os.environ.get("SPACEMIT_PYPI_INDEX", DEFAULT_SPACEMIT_PYPI_INDEX)
        raise SystemExit(
            "spacemit_audio 未安装，无法列出播放设备。\n"
            "安装: pip install -r requirements.txt\n"
            f"SpaceMIT PyPI: {index_url}"
        ) from exc

    for idx, name in AudioPlayer.list_devices():
        print(f"  [{idx}] {name}")


def playback_hw_card(playback_device: int, list_devices) -> Optional[int]:
    if playback_device < 0:
        return None
    try:
        devices = list_devices()
    except Exception as exc:
        print(f"[TTS] 无法列出播放设备，跳过 mixer 设置: {exc}")
        return None
    for index, name in devices:
        if int(index) != playback_device:
            continue
        match = re.search(r"\(hw:(\d+),\d+\)", name)
        if match:
            return int(match.group(1))
    return None


def configure_playback_mixer(playback_device: int, mixer_volume: int,
                             list_devices, runner=subprocess.run) -> bool:
    if mixer_volume < 0:
        return True
    mixer_volume = max(0, min(100, int(mixer_volume)))
    card = playback_hw_card(playback_device, list_devices)
    if card is None:
        print("[TTS] 未找到播放设备的 ALSA hw card，跳过 mixer 设置")
        return False

    command = [
        "amixer",
        "-c",
        str(card),
        "sset",
        "PCM",
        f"{mixer_volume}%",
        "unmute",
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
        print(f"[TTS] mixer 设置失败: {exc}")
        return False
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        print(f"[TTS] mixer 设置失败: {detail}")
        return False

    print(f"[TTS] Mixer: card={card}, PCM={mixer_volume}%")
    return True


def write_audio_bytes(player, audio_bytes: bytes, channels: int,
                      frames_per_chunk: int = 4096) -> bool:
    chunk_bytes = frames_per_chunk * channels * 2
    for offset in range(0, len(audio_bytes), chunk_bytes):
        if not player.write(audio_bytes[offset:offset + chunk_bytes]):
            return False
    return True


def run_tts_worker(text_queue, running, preset, playback_device, playback_rate,
                   channels, speed, volume, no_play, mixer_volume=-1):
    try:
        import numpy as np
        import spacemit_tts
    except ImportError:
        index_url = os.environ.get("SPACEMIT_PYPI_INDEX", DEFAULT_SPACEMIT_PYPI_INDEX)
        print("TTS 依赖未安装。安装命令:")
        print("  pip install -r requirements.txt")
        print(f"  SpaceMIT PyPI: {index_url}")
        running.clear()
        return

    player = None
    if not no_play:
        try:
            import spacemit_audio
            from spacemit_audio import AudioPlayer
            configure_playback_mixer(
                playback_device,
                mixer_volume,
                AudioPlayer.list_devices,
            )
            spacemit_audio.init(
                sample_rate=playback_rate,
                channels=channels,
                player_device=playback_device,
            )
            player = AudioPlayer(playback_device)
            if not player.start():
                raise RuntimeError("AudioPlayer.start() returned false")
            print(f"[TTS] AudioPlayer started: device={playback_device}, "
                  f"{playback_rate}Hz, channels={channels}")
        except Exception as exc:
            print(f"[TTS] 播放器启动失败，改为只合成不播放: {exc}")
            player = None

    config = spacemit_tts.Config.preset(preset)
    config.speech_rate = speed
    config.volume = volume
    engine = spacemit_tts.Engine(config)
    print(f"[TTS] Engine initialized: {preset}, speed={speed}, volume={volume}")

    while running.is_set() or not text_queue.empty():
        try:
            text = text_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        if text is None:
            break

        print(f"[TTS] Speak: {text}")
        result = engine.synthesize(text)
        if not result or not result.is_success:
            message = result.message if result else "unknown error"
            print(f"[TTS] Synthesis failed: {message}")
            continue

        audio = result.audio_int16
        if result.sample_rate != playback_rate and len(audio) > 0:
            ratio = playback_rate / result.sample_rate
            out_len = max(1, int(len(audio) * ratio))
            old_x = np.linspace(0.0, 1.0, len(audio), endpoint=False)
            new_x = np.linspace(0.0, 1.0, out_len, endpoint=False)
            audio_f = audio.astype(np.float32) / 32768.0
            audio = (np.clip(np.interp(new_x, old_x, audio_f), -1.0, 1.0)
                     * 32767).astype(np.int16)

        if channels == 2 and audio.ndim == 1:
            audio = np.column_stack((audio, audio))

        if player is not None:
            if not write_audio_bytes(player, audio.tobytes(), channels):
                print("[TTS] Playback write failed")
                continue
            print(f"[TTS] Playback written: {len(audio)} samples")
        print(f"[TTS] Done: {result.duration_ms}ms, RTF={result.rtf:.3f}")

    if player is not None:
        player.stop()
        player.close()
    print("[TTS] Worker stopped")


def main():
    parser = argparse.ArgumentParser(description="perceptive_grasp TTS status node")
    parser.add_argument("--config", help="读取 grasp_pipeline.yaml 中的 voice/tts 配置")
    parser.add_argument("-t", "--topic", default=None,
                        help="ROS2 std_msgs/String 订阅话题")
    parser.add_argument("-e", "--engine", default=None,
                        help="TTS 后端: matcha:zh / matcha:en / matcha:zh-en / kokoro")
    parser.add_argument("-d", "--device", type=int, default=None,
                        help="播放设备索引，填写 --list-devices 输出方括号里的数字")
    parser.add_argument("--rate", type=int, default=None, help="播放采样率")
    parser.add_argument("-c", "--channels", type=int, choices=[1, 2], default=None)
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--volume", type=int, default=None)
    parser.add_argument("--speak-all", action="store_true", help="播报全部状态消息")
    parser.add_argument("--no-play", action="store_true", help="只合成，不播放")
    parser.add_argument("--list-devices", action="store_true", help="列出播放设备")
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        return

    voice_cfg = {}
    tts_cfg = {}
    if args.config:
        voice_cfg = load_voice_config(args.config)
        tts_cfg = voice_cfg.get("tts", {}) or {}

    topic = args.topic or voice_cfg.get("status_topic", "grasp_status_text")
    preset = engine_to_preset(args.engine or tts_cfg.get("engine", "matcha:zh"))
    playback_device = (
        args.device if args.device is not None
        else int(tts_cfg.get("playback_device", -1))
    )
    playback_rate = args.rate if args.rate is not None else int(tts_cfg.get("playback_rate", 48000))
    channels = args.channels if args.channels is not None else int(tts_cfg.get("channels", 1))
    speed = args.speed if args.speed is not None else float(tts_cfg.get("speed", 1.0))
    volume = args.volume if args.volume is not None else int(tts_cfg.get("volume", 80))
    mixer_volume = int(tts_cfg.get("mixer_volume", -1))
    speak_all_states = (
        args.speak_all or bool(tts_cfg.get("speak_all_states", False))
    )
    reverse_aliases = make_reverse_aliases(voice_cfg.get("target_aliases", {}) or {})

    running = threading.Event()
    running.set()
    text_queue = queue.Queue(maxsize=8)

    def stop_handler(sig, frame):
        del sig, frame
        running.clear()
        text_queue.put(None)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    rclpy, ExternalShutdownException, Node, String = import_ros2_or_exit()

    rclpy.init()
    node = Node("perceptive_grasp_tts")

    worker = threading.Thread(
        target=run_tts_worker,
        args=(text_queue, running, preset, playback_device, playback_rate,
              channels, speed, volume, args.no_play, mixer_volume),
        daemon=True,
    )
    worker.start()

    last_spoken = {"text": ""}

    def on_status(msg):
        raw = msg.data.strip() if msg else ""
        speech = status_to_speech(raw, reverse_aliases, speak_all_states)
        if not speech:
            print(f"[TTS] Ignore status: {raw}")
            return
        if speech == last_spoken["text"]:
            print(f"[TTS] Dedup: {speech}")
            return
        last_spoken["text"] = speech
        try:
            text_queue.put_nowait(speech)
        except queue.Full:
            print(f"[TTS] Queue full, drop: {speech}")

    node.create_subscription(String, topic, on_status, 10)
    print(f"[TTS] Listening on topic: {topic}")

    try:
        while running.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except ExternalShutdownException:
                break
    finally:
        running.clear()
        text_queue.put(None)
        worker.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
