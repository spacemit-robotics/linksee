#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for runtime environment diagnostics."""

import contextlib
import io
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import check_runtime_env  # noqa: E402


class RuntimeEnvDiagnosticsTest(unittest.TestCase):
    def test_sensevoice_backend_uses_sdk_managed_models(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = check_runtime_env.check_voice_asr_backend({
                "voice": {"asr": {"backend": "sensevoice"}},
            })

        self.assertTrue(ok)
        self.assertIn("sensevoice", output.getvalue())

    def test_qwen_backend_checks_llama_server(self):
        qwen = {
            "endpoint": "http://10.0.0.2:8063/v1/chat/completions",
            "model": "qwen3-asr",
        }
        with mock.patch.object(
                check_runtime_env,
                "probe_qwen3_asr_endpoint",
                return_value="http://10.0.0.2:8063/health") as probe:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_voice_asr_backend({
                    "voice": {
                        "asr": {
                            "backend": "qwen3_asr",
                            "qwen3_asr": qwen,
                        },
                    },
                })

        self.assertTrue(ok)
        probe.assert_called_once_with(qwen)
        self.assertIn("qwen3_asr", output.getvalue())

    def test_qwen_local_auto_start_checks_runtime_files(self):
        qwen = {
            "endpoint": "http://127.0.0.1:8063/v1/chat/completions",
            "auto_start": True,
        }
        with mock.patch.object(
                check_runtime_env, "validate_qwen3_local_runtime",
                return_value=("/usr/bin/llama-server", "/models", "/model")):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_voice_asr_backend({
                    "voice": {
                        "asr": {
                            "backend": "qwen3_asr",
                            "qwen3_asr": qwen,
                        },
                    },
                })

        self.assertTrue(ok)
        self.assertIn("local auto-start", output.getvalue())

    def test_unknown_asr_backend_is_rejected(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = check_runtime_env.check_voice_asr_backend({
                "voice": {"asr": {"backend": "whisper"}},
            })

        self.assertFalse(ok)
        self.assertIn("unsupported", output.getvalue())

    def test_remote_mujoco_uses_shared_endpoint(self):
        connection = mock.MagicMock()
        connection.__enter__.return_value = connection
        with mock.patch.object(
                check_runtime_env.socket, "create_connection",
                return_value=connection) as create_connection:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_remote_mujoco({
                    "remote_mujoco": {
                        "host": "10.0.91.182",
                        "port": 9090,
                    },
                })

        self.assertTrue(ok)
        create_connection.assert_called_once_with(
            ("10.0.91.182", 9090), timeout=2.0)
        self.assertIn("10.0.91.182:9090", output.getvalue())

    def test_remote_mujoco_reports_unreachable_endpoint(self):
        with mock.patch.object(
                check_runtime_env.socket, "create_connection",
                side_effect=OSError("connection refused")):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_remote_mujoco({
                    "remote_mujoco": {
                        "host": "127.0.0.1",
                        "port": 9090,
                    },
                })

        self.assertFalse(ok)
        self.assertIn("unreachable", output.getvalue())

    def test_local_mujoco_checks_server_and_device_plan(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            build = root / "build"
            config_dir = root / "config"
            scene = root / "simulation" / "mujoco" / "device.yaml"
            build.mkdir()
            config_dir.mkdir()
            scene.parent.mkdir(parents=True)
            server = build / "mujoco_grasp_sim_server"
            server.write_text("#!/bin/sh\n", encoding="utf-8")
            server.chmod(0o755)
            scene.write_text("device: {}\n", encoding="utf-8")
            config = config_dir / "pipeline.yaml"

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_mujoco_simulation({
                    "camera": {
                        "mujoco": {"device_plan": "../simulation/mujoco/device.yaml"},
                    },
                    "manipulator": {
                        "mujoco": {"device_plan": "../simulation/mujoco/device.yaml"},
                    },
                }, str(config), str(build))

        self.assertTrue(ok)
        self.assertIn("MuJoCo simulation server", output.getvalue())
        self.assertIn(str(scene), output.getvalue())

    def test_local_mujoco_rejects_missing_runtime_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "config" / "pipeline.yaml"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_mujoco_simulation({
                    "camera": {"mujoco": {"device_plan": "missing.yaml"}},
                    "manipulator": {"mujoco": {"device_plan": "missing.yaml"}},
                }, str(config), str(Path(tmp) / "build"))

        self.assertFalse(ok)
        self.assertIn("missing or not executable", output.getvalue())

    def test_hand_eye_calibration_selects_camera_backend(self):
        root = {
            "calibration": {
                "realsense": {
                    "T_base_camera": {
                        "translation": [0.1, 0.2, 0.3],
                        "rotation": [0.4, 0.5, 0.6],
                    },
                },
                "spacemit_las2": {
                    "T_base_camera": {
                        "translation": [1.1, 1.2, 1.3],
                        "rotation": [1.4, 1.5, 1.6],
                    },
                },
            },
        }

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = check_runtime_env.check_hand_eye_calibration(
                root, "realsense")

        self.assertTrue(ok)
        self.assertIn(
            "calibration.realsense.T_base_camera", output.getvalue())

    def test_hand_eye_calibration_rejects_missing_backend(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = check_runtime_env.check_hand_eye_calibration(
                {"calibration": {}}, "spacemit_las2")

        self.assertFalse(ok)
        self.assertIn(
            "calibration.spacemit_las2.T_base_camera",
            output.getvalue())

    def test_hand_eye_calibration_accepts_legacy_realsense_profile(self):
        root = {
            "calibration": {
                "T_base_camera": {
                    "translation": [0.1, 0.2, 0.3],
                    "rotation": [0.4, 0.5, 0.6],
                },
            },
        }

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = check_runtime_env.check_hand_eye_calibration(
                root, "realsense")

        self.assertTrue(ok)
        self.assertIn("legacy shared profile", output.getvalue())

    def test_hardware_aec_does_not_require_software_library(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = check_runtime_env.check_voice_echo_cancellation({
                "voice": {
                    "echo_cancellation": {"mode": "hardware_aec"},
                },
            })

        self.assertTrue(ok)
        self.assertIn("mode=hardware_aec", output.getvalue())

    def test_webrtc_aec_requires_native_library(self):
        with mock.patch.object(check_runtime_env.os.path, "isfile",
                               return_value=False):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_voice_echo_cancellation({
                    "voice": {
                        "echo_cancellation": {"mode": "webrtc_aec"},
                    },
                })

        self.assertFalse(ok)
        self.assertIn("libperceptive_voice_aec.so not found", output.getvalue())

    def test_webrtc_aec_accepts_installed_library(self):
        def is_installed_library(path):
            return path.endswith("/lib/libperceptive_voice_aec.so")

        ldd_result = SimpleNamespace(returncode=0, stdout="", stderr="")
        with mock.patch.object(
                check_runtime_env.os.path,
                "isfile",
                side_effect=is_installed_library), mock.patch.object(
                    check_runtime_env.subprocess,
                    "run",
                    return_value=ldd_result):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_voice_echo_cancellation({
                    "voice": {
                        "echo_cancellation": {"mode": "webrtc_aec"},
                    },
                })

        self.assertTrue(ok)
        self.assertIn("mode=webrtc_aec", output.getvalue())

    def test_config_path_expands_user_home(self):
        with mock.patch.dict(os.environ, {"HOME": "/home/annyi"}):
            resolved = check_runtime_env.expand_config_path(
                "~/las2_runtime/config/stereo.json")

        self.assertEqual(
            resolved, "/home/annyi/las2_runtime/config/stereo.json")

    def test_group_status_fails_when_group_is_configured_but_not_active(self):
        def fake_getgrgid(gid):
            names = {20: "dialout", 29: "audio", 1000: "annyi"}
            return SimpleNamespace(gr_name=names[gid])

        with mock.patch.object(check_runtime_env.grp, "getgrgid",
                               side_effect=fake_getgrgid), \
                mock.patch.object(check_runtime_env.getpass, "getuser",
                                  return_value="annyi"), \
                mock.patch.object(check_runtime_env.grp, "getgrall",
                                  return_value=[
                                      SimpleNamespace(
                                          gr_name="dialout",
                                          gr_mem=["annyi"],
                                      ),
                                  ]), \
                mock.patch.object(check_runtime_env.os, "getgid",
                                  return_value=1000), \
                mock.patch.object(check_runtime_env.os, "getgroups",
                                  return_value=[1000]):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_group_status("dialout")

        self.assertFalse(ok)
        self.assertIn("configured dialout group", output.getvalue())
        self.assertIn("active dialout group", output.getvalue())
        self.assertIn("重新登录", output.getvalue())

    def test_audio_permissions_prompt_for_audio_group(self):
        def fake_getgrgid(gid):
            return SimpleNamespace(gr_name="audio" if gid == 42 else "users")

        with mock.patch.object(check_runtime_env.glob, "glob",
                               return_value=[
                                   "/dev/snd/by-path",
                                   "/dev/snd/controlC0",
                               ]), \
                mock.patch.object(
                    check_runtime_env.os,
                    "stat",
                    side_effect=[
                        SimpleNamespace(
                            st_mode=stat.S_IFDIR | 0o755,
                            st_uid=0,
                            st_gid=0,
                        ),
                        SimpleNamespace(
                            st_mode=stat.S_IFCHR | 0o660,
                            st_uid=0,
                            st_gid=42,
                        ),
                    ],
                ), \
                mock.patch.object(check_runtime_env.grp, "getgrgid",
                                  side_effect=fake_getgrgid), \
                mock.patch.object(check_runtime_env.getpass, "getuser",
                                  return_value="annyi"), \
                mock.patch.object(check_runtime_env.grp, "getgrall",
                                  return_value=[
                                      SimpleNamespace(
                                          gr_name="audio",
                                          gr_mem=["annyi"],
                                      ),
                                  ]), \
                mock.patch.object(check_runtime_env.os, "getgid",
                                  return_value=1000), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=False):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_audio_permissions()

        self.assertFalse(ok)
        self.assertIn("configured audio group", output.getvalue())
        self.assertIn("active audio group", output.getvalue())
        self.assertNotIn("audio device read/write: /dev/snd/by-path",
                         output.getvalue())

    def test_video_permissions_prompt_for_video_group(self):
        def fake_getgrgid(gid):
            return SimpleNamespace(gr_name="video" if gid == 44 else "users")

        with mock.patch.object(check_runtime_env.glob, "glob",
                               return_value=[
                                   "/dev/video-dec0",
                                   "/dev/video1",
                               ]), \
                mock.patch.object(
                    check_runtime_env.os,
                    "stat",
                    side_effect=[
                        SimpleNamespace(
                            st_mode=stat.S_IFREG | 0o777,
                            st_uid=0,
                            st_gid=0,
                        ),
                        SimpleNamespace(
                            st_mode=stat.S_IFCHR | 0o660,
                            st_uid=0,
                            st_gid=44,
                        ),
                    ],
                ), \
                mock.patch.object(check_runtime_env.grp, "getgrgid",
                                  side_effect=fake_getgrgid), \
                mock.patch.object(check_runtime_env.getpass, "getuser",
                                  return_value="annyi"), \
                mock.patch.object(check_runtime_env.grp, "getgrall",
                                  return_value=[
                                      SimpleNamespace(
                                          gr_name="video",
                                          gr_mem=["annyi"],
                                      ),
                                  ]), \
                mock.patch.object(check_runtime_env.os, "getgid",
                                  return_value=1000), \
                mock.patch.object(check_runtime_env.os, "getgroups",
                                  return_value=[1000]), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=False):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_video_permissions()

        self.assertFalse(ok)
        self.assertIn("configured video group", output.getvalue())
        self.assertIn("active video group", output.getvalue())
        self.assertNotIn("video device read/write: /dev/video-dec0",
                         output.getvalue())

    def test_realsense_check_fails_without_d435i_device(self):
        with mock.patch.object(check_runtime_env.glob, "glob",
                               return_value=["/dev/video1", "/dev/video2"]), \
                mock.patch.object(check_runtime_env, "_udev_properties",
                                  return_value={"ID_MODEL": "2K_USB_Camera"}):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_realsense_camera()

        self.assertFalse(ok)
        self.assertIn("RealSense D435i", output.getvalue())
        self.assertIn("not found", output.getvalue())

    def test_serial_discovery_does_not_open_or_probe_candidate_ports(self):
        with mock.patch.object(check_runtime_env.glob, "glob", return_value=["/dev/ttyACM0"]), \
                mock.patch.object(check_runtime_env, "_udev_properties", return_value={
                    "ID_MODEL": "USB_Single_Serial"}), \
                mock.patch.object(check_runtime_env.subprocess, "run") as run:
            with contextlib.redirect_stdout(io.StringIO()):
                ok = check_runtime_env.check_serial_role_configuration(
                    "/dev/ttyACM0", "", False, "unused")
        self.assertTrue(ok)
        run.assert_not_called()

    def test_application_check_finds_launcher_and_core_in_named_build_directory(self):
        build_dir = str(ROOT / "build_release")
        launcher = os.path.join(build_dir, "perceptive_grasp")
        core = os.path.join(build_dir, "perceptive_grasp_core")
        with mock.patch.object(check_runtime_env.glob, "glob",
                               return_value=[build_dir]), \
                mock.patch.object(check_runtime_env.os.path, "isfile",
                                  side_effect=lambda path: path in {launcher, core}), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.shutil, "which",
                                  return_value=None):
            self.assertEqual(
                check_runtime_env._find_application_binaries(),
                (launcher, core))

    def test_application_check_rejects_missing_core(self):
        launcher = str(ROOT / "build" / "perceptive_grasp")
        with mock.patch.object(check_runtime_env,
                               "_find_application_binaries",
                               return_value=(launcher, "")), \
                mock.patch.object(check_runtime_env.os.path, "isfile",
                                  side_effect=lambda path: path == launcher), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=True):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_application_binaries()

        self.assertFalse(ok)
        self.assertIn("perceptive_grasp core", output.getvalue())

    def test_realsense_check_passes_when_d435i_video_device_exists(self):
        def fake_props(device):
            if device == "/dev/video3":
                return {
                    "ID_VENDOR_ID": "8086",
                    "ID_MODEL_ID": "0b3a",
                    "ID_MODEL": "Intel_R__RealSense_TM__Depth_Camera_435i",
                }
            return {"ID_MODEL": "2K_USB_Camera"}

        with mock.patch.object(check_runtime_env.glob, "glob",
                               return_value=["/dev/video1", "/dev/video3"]), \
                mock.patch.object(check_runtime_env, "_udev_properties",
                                  side_effect=fake_props):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_realsense_camera()

        self.assertTrue(ok)
        self.assertIn("/dev/video3", output.getvalue())
        self.assertIn("RealSense D435i", output.getvalue())

    def test_las2_check_accepts_complete_runtime(self):
        camera = {
            "type": "spacemit_las2",
            "spacemit_las2": {
                "video_device": "/dev/video1",
                "model_path": "/opt/las2/models/model.onnx",
                "calib_path": "/opt/las2/config/stereo.json",
                "core_count": 1,
                "core_affinity": "8",
            },
        }
        calibration = (
            '{"image_size":[1920,1200],"left_camera_matrix":[], '
            '"right_camera_matrix":[],"left_dist_coeffs":[], '
            '"right_dist_coeffs":[],"R":[],"T":[]}'
        )

        def fake_isfile(path):
            return path in {
                "/opt/las2/models/model.onnx",
                "/opt/las2/config/stereo.json",
                "/opt/las2/lib/liblas2_usb_stereo.so",
            }

        with mock.patch.object(check_runtime_env.os.path, "exists",
                               return_value=True), \
                mock.patch.object(check_runtime_env.os.path, "isfile",
                                  side_effect=fake_isfile), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=True), \
                mock.patch.object(check_runtime_env,
                                  "check_las2_capture_requirements",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  return_value=SimpleNamespace(
                                      returncode=0, stdout="", stderr="")), \
                mock.patch("builtins.open", return_value=io.StringIO(calibration)):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_las2_camera(camera, "/tmp/sdk")

        self.assertTrue(ok)
        self.assertIn("LAS2 video device", output.getvalue())
        self.assertIn("LAS2 calibration fields", output.getvalue())
        self.assertIn("liblas2_usb_stereo.so", output.getvalue())
        self.assertIn("LAS2 runtime libraries", output.getvalue())

    def test_las2_check_rejects_non_ai_core_affinity(self):
        camera = {
            "type": "spacemit_las2",
            "spacemit_las2": {
                "video_device": "/dev/video1",
                "model_path": "/opt/las2/models/model.onnx",
                "calib_path": "/opt/las2/config/stereo.json",
                "core_count": 2,
                "core_affinity": "7,8",
            },
        }

        with mock.patch.object(check_runtime_env.os.path, "exists",
                               return_value=False), \
                mock.patch.object(check_runtime_env.os.path, "isfile",
                                  return_value=False), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=False):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_las2_camera(camera, "/tmp/sdk")

        self.assertFalse(ok)
        self.assertIn("LAS2 core_affinity", output.getvalue())
        self.assertIn("range 8-15", output.getvalue())

    def test_las2_runtime_check_reports_missing_opencv(self):
        result = SimpleNamespace(
            returncode=0,
            stdout="libopencv_calib3d.so.413 => not found\n",
            stderr="",
        )
        with mock.patch.object(check_runtime_env.subprocess, "run",
                               return_value=result):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env._check_las2_runtime_libraries(
                    "/opt/las2/lib/liblas2_usb_stereo.so")

        self.assertFalse(ok)
        self.assertIn("libopencv_calib3d.so.413 => not found",
                      output.getvalue())
        self.assertIn("export LD_LIBRARY_PATH=", output.getvalue())

    def test_las2_runtime_check_accepts_patched_application(self):
        result = SimpleNamespace(
            returncode=0,
            stdout=(
                "liblas2_usb_stereo.so => /tmp/build/las2_runtime/"
                "liblas2_usb_stereo.so\n"
                "libopencv_calib3d.so.413 => /opt/las2/lib/opencv/"
                "libopencv_calib3d.so.413\n"
            ),
            stderr="",
        )
        with mock.patch.object(check_runtime_env.subprocess, "run",
                               return_value=result):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env._check_las2_runtime_libraries(
                    "/opt/las2/lib/liblas2_usb_stereo.so",
                    "/tmp/build/perceptive_grasp")

        self.assertTrue(ok)
        self.assertIn("via /tmp/build/perceptive_grasp", output.getvalue())

    def test_las2_capture_check_rejects_one_fps_yuyv(self):
        formats = """
            [0]: 'MJPG'
                Size: Discrete 4000x1200
                    Interval: Discrete 0.033s (30.000 fps)
            [1]: 'YUYV'
                Size: Discrete 4000x1200
                    Interval: Discrete 1.000s (1.000 fps)
        """
        result = SimpleNamespace(returncode=0, stdout=formats, stderr="")
        with mock.patch.object(check_runtime_env.subprocess, "run",
                               return_value=result), \
                mock.patch.object(check_runtime_env.os.path, "exists",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=True):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_las2_capture_requirements(
                    "/dev/video1")

        self.assertFalse(ok)
        self.assertIn("advertised max is 1 fps", output.getvalue())

    def test_las2_detector_rejects_spacemit_provider_without_free_cores(self):
        root = {
            "camera": {"spacemit_las2": {
                "core_count": 8,
                "core_affinity": "8,9,10,11,12,13,14,15",
            }},
            "detection": {"config_path": "yolov8_seg.yaml"},
        }
        detector = {
            "default_params": {
                "providers": ["SpaceMITExecutionProvider"],
            },
        }
        with mock.patch.object(check_runtime_env.os.path, "isfile",
                               return_value=True), \
                mock.patch.object(check_runtime_env, "load_yaml",
                                  return_value=detector):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_las2_detector_provider(
                    root, "/opt/grasp/config/grasp_pipeline.yaml")

        self.assertFalse(ok)
        self.assertIn("reduce LAS2 core_count/affinity", output.getvalue())

    def test_las2_detector_accepts_spacemit_provider_with_four_free_cores(self):
        root = {
            "camera": {"spacemit_las2": {
                "core_count": 4,
                "core_affinity": "8,9,10,11",
            }},
            "detection": {"config_path": "yolov8_seg.yaml"},
        }
        detector = {
            "default_params": {
                "num_threads": 4,
                "providers": ["SpaceMITExecutionProvider"],
            },
        }
        with mock.patch.object(check_runtime_env.os.path, "isfile",
                               return_value=True), \
                mock.patch.object(check_runtime_env, "load_yaml",
                                  return_value=detector):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_las2_detector_provider(
                    root, "/opt/grasp/config/grasp_pipeline.yaml")

        self.assertTrue(ok)
        self.assertIn("12,13,14,15", output.getvalue())

    def test_las2_detector_rejects_cpu_provider(self):
        root = {"detection": {"config_path": "yolov8_seg.yaml"}}
        detector = {
            "default_params": {
                "providers": ["CPUExecutionProvider"],
            },
        }
        with mock.patch.object(check_runtime_env.os.path, "isfile",
                               return_value=True), \
                mock.patch.object(check_runtime_env, "load_yaml",
                                  return_value=detector):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_las2_detector_provider(
                    root, "/opt/grasp/config/grasp_pipeline.yaml")

        self.assertFalse(ok)
        self.assertIn("YOLO must use SpaceMITExecutionProvider",
                      output.getvalue())

    def test_main_selects_las2_camera_check(self):
        config = {
            "camera": {
                "type": "spacemit_las2",
                "spacemit_las2": {},
            },
            "calibration": {
                "spacemit_las2": {
                    "T_base_camera": {
                        "translation": [0.0, 0.0, 0.0],
                        "rotation": [0.0, 0.0, 0.0],
                    },
                },
            },
            "manipulator": {"uart_device": "/dev/ttyACM1"},
            "mobile_base": {"enabled": False},
        }
        with mock.patch.object(check_runtime_env, "load_yaml",
                               return_value=config), \
                mock.patch.object(check_runtime_env,
                                  "check_application_binaries",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "report_serial_devices"), \
                mock.patch.object(check_runtime_env,
                                  "check_serial_role_configuration",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_tty",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_video_permissions",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_las2_camera",
                                  return_value=True) as check_las2, \
                mock.patch.object(check_runtime_env,
                                  "check_las2_detector_provider",
                                  return_value=True) as check_provider, \
                mock.patch.object(check_runtime_env, "check_realsense_camera",
                                  return_value=True) as check_realsense, \
                mock.patch.object(check_runtime_env, "check_kinematics_backend",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.os, "getcwd",
                                  return_value=str(ROOT)), \
                mock.patch.object(check_runtime_env.sys, "argv", [
                    "check_runtime_env.py", "--config", "cfg.yaml",
                    "--skip-voice",
                ]):
            with contextlib.redirect_stdout(io.StringIO()):
                rc = check_runtime_env.main()

        self.assertEqual(rc, 0)
        check_las2.assert_called_once_with(config["camera"], mock.ANY)
        check_provider.assert_called_once_with(config, "cfg.yaml")
        check_realsense.assert_not_called()

    def test_kinematics_check_reports_missing_backend(self):
        with mock.patch.object(check_runtime_env.os.path, "isfile", return_value=True), \
                mock.patch.object(check_runtime_env.os, "access", return_value=True), \
                mock.patch.object(check_runtime_env.subprocess, "run", return_value=SimpleNamespace(
                    returncode=1, stdout="", stderr="unknown kinematics backend: eigen")) as run:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_kinematics_backend("/tmp/sdk", "cfg.yaml", "/tmp/app")

        self.assertFalse(ok)
        self.assertIn("unknown kinematics backend", output.getvalue())
        self.assertEqual(run.call_args.args[0], [
            "/tmp/app/debug_ik", "--config", os.path.abspath("cfg.yaml"), "--check-model"])
        self.assertEqual(run.call_args.kwargs["timeout"], 10)

    def test_kinematics_check_reports_unresolved_runtime_libraries(self):
        with mock.patch.object(check_runtime_env.os.path, "isfile", return_value=True), \
                mock.patch.object(check_runtime_env.os, "access", return_value=True), \
                mock.patch.object(check_runtime_env.subprocess, "run", return_value=SimpleNamespace(
                    returncode=127, stdout="", stderr="libpinocchio_default.so => not found")):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_kinematics_backend("/tmp/sdk")

        self.assertFalse(ok)
        self.assertIn("manipulation kinematics", output.getvalue())
        self.assertIn("libpinocchio_default.so => not found",
                      output.getvalue())

    def test_device_plan_port_is_relative_to_plan_not_working_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "devices").mkdir()
            (root / "devices" / "arm.yaml").write_text(
                "device:\n  arm:\n    options:\n      port: serial/arm\n")
            config = {"manipulator": {"device_plan": "devices/arm.yaml"}}
            port = check_runtime_env.configured_arm_port(config, str(root / "pipeline.yaml"))
        self.assertEqual(port, str(root / "devices/serial/arm"))

    def test_device_plan_port_overrides_legacy_uart_field_without_device_access(self):
        config = {"manipulator": {"device_plan": "/tmp/device.yaml", "uart_device": "/dev/old"}}
        with mock.patch.object(check_runtime_env, "load_yaml", return_value={
                "device": {"options": {"port": "/dev/current"}}}), \
                mock.patch.object(check_runtime_env.subprocess, "run") as run:
            self.assertEqual(check_runtime_env.configured_arm_port(config, "/tmp/pipeline.yaml"),
                             "/dev/current")
        run.assert_not_called()

    def test_sdk_root_can_be_inferred_from_project_path(self):
        project = "/home/user/spacemit_robot/application/ros2/linksee/perceptive_grasp"

        self.assertEqual(
            check_runtime_env.infer_sdk_root(project),
            "/home/user/spacemit_robot",
        )

    def test_group_fix_runs_usermod_when_confirmed(self):
        fixer = check_runtime_env.RuntimeFixer(interactive=True)

        with mock.patch.object(check_runtime_env, "prompt_yes_no",
                               return_value=True), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  return_value=SimpleNamespace(returncode=0)) as run:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(fixer.offer_group_fix("audio"))

        run.assert_called_once_with(["sudo", "usermod", "-aG", "audio",
                                     check_runtime_env.getpass.getuser()],
                                    check=False)

    def test_missing_imports_offer_requirements_install(self):
        fixer = check_runtime_env.RuntimeFixer(interactive=True)

        with mock.patch.object(check_runtime_env, "prompt_yes_no",
                               return_value=True), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  return_value=SimpleNamespace(returncode=0)) as run:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(fixer.offer_requirements_install(["spacemit_asr"]))

        run.assert_called_once()
        command = run.call_args.args[0]
        self.assertEqual(command[:4], [sys.executable, "-m", "pip", "install"])
        self.assertIn("-r", command)
        self.assertIn("requirements.txt", command[-1])

    def test_serial_report_shows_by_id_and_config_roles(self):
        def fake_realpath(path):
            mapping = {
                "/dev/serial/by-id/usb-camera-if00": "/dev/ttyACM1",
                "/dev/serial/by-id/usb-single-serial-if00": "/dev/ttyACM2",
            }
            return mapping.get(path, path)

        def fake_info(command, *args, **kwargs):
            node = command[-1]
            model = "USB Single Serial" if node == "/dev/ttyACM2" else "2K USB Camera"
            return SimpleNamespace(
                returncode=0,
                stdout=f"ID_MODEL={model}\nID_VENDOR=Vendor\n",
                stderr="",
            )

        with mock.patch.object(check_runtime_env.glob, "glob",
                               side_effect=[
                                   ["/dev/ttyACM1", "/dev/ttyACM2"],
                                   [
                                       "/dev/serial/by-id/usb-camera-if00",
                                       "/dev/serial/by-id/usb-single-serial-if00",
                                   ],
                               ]), \
                mock.patch.object(check_runtime_env.os.path, "realpath",
                                  side_effect=fake_realpath), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  side_effect=fake_info):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                check_runtime_env.report_serial_devices(
                    manipulator_device="/dev/ttyACM2",
                    mobile_base_device="/dev/ttyACM1",
                )

        text = output.getvalue()
        self.assertIn("[INFO] serial devices:", text)
        self.assertIn("/dev/ttyACM2", text)
        self.assertIn("role=manipulator.device_plan port", text)
        self.assertIn("/dev/serial/by-id/usb-single-serial-if00", text)
        self.assertIn("ID_MODEL=USB Single Serial", text)
        self.assertIn("/dev/ttyACM1", text)
        self.assertIn("role=mobile_base.dev_path", text)
        self.assertIn("ID_MODEL=2K USB Camera", text)

    def test_main_checks_mobile_base_serial_device(self):
        config = {
            "camera": {"type": "realsense", "realsense": {}},
            "calibration": {
                "realsense": {
                    "T_base_camera": {
                        "translation": [0.0, 0.0, 0.0],
                        "rotation": [0.0, 0.0, 0.0],
                    },
                },
            },
            "manipulator": {"uart_device": "/dev/ttyACM2"},
            "mobile_base": {
                "enabled": True,
                "dev_path": "/dev/ttyACM1",
            },
        }

        with mock.patch.object(check_runtime_env, "load_yaml",
                               return_value=config), \
                mock.patch.object(check_runtime_env,
                                  "check_application_binaries",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "report_serial_devices"), \
                mock.patch.object(check_runtime_env,
                                  "check_serial_role_configuration",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_tty",
                                  return_value=True) as check_tty, \
                mock.patch.object(check_runtime_env, "check_realsense_camera",
                                  return_value=True), \
                mock.patch.object(check_runtime_env,
                                  "check_chassis_driver_registered",
                                  return_value=True) as check_driver, \
                mock.patch.object(check_runtime_env, "check_video_permissions",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_kinematics_backend",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_import",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_audio_permissions",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "list_audio_devices",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.os, "getcwd",
                                  return_value=str(ROOT)), \
                mock.patch.object(check_runtime_env.sys, "argv",
                                  ["check_runtime_env.py", "--config", "cfg.yaml"]):
            with contextlib.redirect_stdout(io.StringIO()):
                rc = check_runtime_env.main()

        self.assertEqual(rc, 0)
        checked_devices = [call.args[0] for call in check_tty.call_args_list]
        self.assertIn("/dev/ttyACM2", checked_devices)
        self.assertIn("/dev/ttyACM1", checked_devices)
        check_driver.assert_called_once_with(mock.ANY, "drv_uart_esp32")

    def test_main_checks_rpmsg_base_without_serial_base_suggestion(self):
        config = {
            "camera": {"type": "realsense", "realsense": {}},
            "calibration": {
                "realsense": {
                    "T_base_camera": {
                        "translation": [0.0, 0.0, 0.0],
                        "rotation": [0.0, 0.0, 0.0],
                    },
                },
            },
            "manipulator": {"uart_device": "/dev/ttyACM2"},
            "mobile_base": {
                "enabled": True,
                "driver": "drv_rpmsg_esos",
                "ctrl_dev": "/dev/rpmsg_ctrl0",
                "data_dev": "/dev/rpmsg0",
            },
        }

        with mock.patch.object(check_runtime_env, "load_yaml",
                               return_value=config), \
                mock.patch.object(check_runtime_env,
                                  "check_application_binaries",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "report_serial_devices") as report, \
                mock.patch.object(check_runtime_env,
                                  "check_serial_role_configuration",
                                  return_value=True) as serial_roles, \
                mock.patch.object(check_runtime_env, "check_tty",
                                  return_value=True) as check_tty, \
                mock.patch.object(check_runtime_env, "check_rpmsg_device",
                                  return_value=True) as check_rpmsg, \
                mock.patch.object(check_runtime_env,
                                  "check_chassis_driver_registered",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_realsense_camera",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_video_permissions",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_kinematics_backend",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_import",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "check_audio_permissions",
                                  return_value=True), \
                mock.patch.object(check_runtime_env, "list_audio_devices",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.os, "getcwd",
                                  return_value=str(ROOT)), \
                mock.patch.object(check_runtime_env.sys, "argv",
                                  ["check_runtime_env.py", "--config", "cfg.yaml"]):
            with contextlib.redirect_stdout(io.StringIO()):
                rc = check_runtime_env.main()

        self.assertEqual(rc, 0)
        report.assert_called_once_with("/dev/ttyACM2", "")
        serial_roles.assert_called_once_with(
            "/dev/ttyACM2", "", False, mock.ANY, "")
        check_rpmsg.assert_called_once_with(
            "/dev/rpmsg_ctrl0", "/dev/rpmsg0", mock.ANY)
        checked_devices = [call.args[0] for call in check_tty.call_args_list]
        self.assertEqual(checked_devices, ["/dev/ttyACM2"])

    def test_rpmsg_data_device_can_be_created_at_runtime(self):
        def fake_exists(path):
            return path == "/dev/rpmsg_ctrl0"

        fake_stat = os.stat_result((stat.S_IFCHR | 0o660, 0, 0, 0, 0, 20, 0,
                                    0, 0, 0))
        with mock.patch.object(check_runtime_env.os.path, "exists",
                               side_effect=fake_exists), \
                mock.patch.object(check_runtime_env.os, "stat",
                                  return_value=fake_stat), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.grp, "getgrgid",
                                  return_value=SimpleNamespace(gr_name="dialout")), \
                mock.patch.object(check_runtime_env, "check_group_status",
                                  return_value=True):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_rpmsg_device(
                    "/dev/rpmsg_ctrl0", "/dev/rpmsg0")

        self.assertTrue(ok)
        self.assertIn("will be created by chassis driver", output.getvalue())

    def test_chassis_driver_check_fails_when_driver_is_not_registered(self):
        with tempfile.TemporaryDirectory() as tmp:
            lib_dir = Path(tmp) / "output" / "staging" / "lib"
            lib_dir.mkdir(parents=True)
            (lib_dir / "libchassis.so").write_bytes(b"drv_rpmsg_esos")

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_chassis_driver_registered(
                    tmp, "drv_uart_esp32")

        self.assertFalse(ok)
        self.assertIn("chassis driver drv_uart_esp32", output.getvalue())
        self.assertIn("not registered", output.getvalue())

    def test_chassis_driver_check_passes_when_driver_is_registered(self):
        with tempfile.TemporaryDirectory() as tmp:
            lib_dir = Path(tmp) / "output" / "staging" / "lib"
            lib_dir.mkdir(parents=True)
            (lib_dir / "libchassis.so").write_bytes(b"drv_uart_esp32")

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_chassis_driver_registered(
                    tmp, "drv_uart_esp32")

        self.assertTrue(ok)
        self.assertIn("chassis driver drv_uart_esp32", output.getvalue())

    def test_serial_role_check_rejects_camera_as_manipulator(self):
        def fake_realpath(path):
            mapping = {
                "/dev/serial/by-id/usb-camera-if00": "/dev/ttyACM0",
                "/dev/serial/by-id/usb-arm-if00": "/dev/ttyACM1",
                "/dev/serial/by-id/usb-base-if00": "/dev/ttyACM2",
            }
            return mapping.get(path, path)

        def fake_props(device):
            if device == "/dev/ttyACM0":
                return {"ID_MODEL": "2K_USB_Camera"}
            return {"ID_MODEL": "USB_Single_Serial", "ID_VENDOR": "1a86"}

        with mock.patch.object(check_runtime_env.glob, "glob",
                               side_effect=[
                                   ["/dev/ttyACM0", "/dev/ttyACM1",
                                    "/dev/ttyACM2"],
                                   [
                                       "/dev/serial/by-id/usb-camera-if00",
                                       "/dev/serial/by-id/usb-arm-if00",
                                       "/dev/serial/by-id/usb-base-if00",
                                   ],
                               ]), \
                mock.patch.object(check_runtime_env.os.path, "realpath",
                                  side_effect=fake_realpath), \
                mock.patch.object(check_runtime_env, "_udev_properties",
                                  side_effect=fake_props):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_serial_role_configuration(
                    "/dev/ttyACM0",
                    "/dev/ttyACM2",
                    True,
                    str(ROOT),
                )

        text = output.getvalue()
        self.assertFalse(ok)
        self.assertIn("manipulator.device_plan port", text)
        self.assertIn("camera serial", text)
        self.assertNotIn("[SUGGEST]", text)

    def test_serial_role_check_fails_when_base_uses_detected_arm_port(self):
        def fake_realpath(path):
            mapping = {
                "/dev/serial/by-id/usb-arm-if00": "/dev/ttyACM1",
                "/dev/serial/by-id/usb-base-if00": "/dev/ttyACM2",
            }
            return mapping.get(path, path)

        with mock.patch.object(check_runtime_env.glob, "glob",
                               side_effect=[
                                   ["/dev/ttyACM1", "/dev/ttyACM2"],
                                   [
                                       "/dev/serial/by-id/usb-arm-if00",
                                       "/dev/serial/by-id/usb-base-if00",
                                   ],
                               ]), \
                mock.patch.object(check_runtime_env.os.path, "realpath",
                                  side_effect=fake_realpath), \
                mock.patch.object(check_runtime_env, "_udev_properties",
                                  return_value={
                                      "ID_MODEL": "USB_Single_Serial",
                                      "ID_VENDOR": "1a86",
                                  }):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_serial_role_configuration(
                    "/dev/ttyACM1",
                    "/dev/ttyACM1",
                    True,
                    str(ROOT),
                )

        text = output.getvalue()
        self.assertFalse(ok)
        self.assertIn("mobile_base.dev_path role", text)
        self.assertIn("configured arm port", text)
        self.assertIn(
            'mobile_base.dev_path: "/dev/serial/by-id/usb-base-if00"',
            text,
        )

    def test_serial_role_check_fails_when_configured_port_is_missing(self):
        def fake_realpath(path):
            mapping = {
                "/dev/serial/by-id/usb-arm-if00": "/dev/ttyACM1",
                "/dev/serial/by-id/usb-base-if00": "/dev/ttyACM2",
            }
            return mapping.get(path, path)

        with mock.patch.object(check_runtime_env.glob, "glob",
                               side_effect=[
                                   ["/dev/ttyACM1", "/dev/ttyACM2"],
                                   [
                                       "/dev/serial/by-id/usb-arm-if00",
                                       "/dev/serial/by-id/usb-base-if00",
                                   ],
                               ]), \
                mock.patch.object(check_runtime_env.os.path, "realpath",
                                  side_effect=fake_realpath), \
                mock.patch.object(check_runtime_env, "_udev_properties",
                                  return_value={
                                      "ID_MODEL": "USB_Single_Serial",
                                      "ID_VENDOR": "1a86",
                                  }):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_serial_role_configuration(
                    "/dev/ttyACM9",
                    "/dev/ttyACM2",
                    True,
                    str(ROOT),
                )

        text = output.getvalue()
        self.assertFalse(ok)
        self.assertIn("manipulator.device_plan port", text)
        self.assertIn("configured port not among USB serial devices", text)


if __name__ == "__main__":
    unittest.main()
