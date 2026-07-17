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

    def test_so101_probe_finds_read_joints_in_named_build_directory(self):
        tool = str(ROOT / "build_las2" / "read_joints")
        result = SimpleNamespace(
            returncode=0,
            stdout="[ReadJoints] Current joints (rad): [1, 2, 3, 4, 5]",
            stderr="",
        )
        with mock.patch.object(check_runtime_env.glob, "glob",
                               return_value=[tool]), \
                mock.patch.object(check_runtime_env.os.path, "isfile",
                                  side_effect=lambda path: path == tool), \
                mock.patch.object(check_runtime_env.os, "access",
                                  return_value=True), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  return_value=result) as run:
            ok = check_runtime_env._probe_so101_device(
                "/dev/ttyACM0", str(ROOT))

        self.assertTrue(ok)
        self.assertEqual(run.call_args.args[0][0], tool)
        self.assertEqual(run.call_args.kwargs["cwd"], str(ROOT))

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

    def test_kinematics_check_reports_missing_pinocchio_backend(self):
        cache_text = (
            "//The directory containing a CMake configuration file for pinocchio.\n"
            "pinocchio_DIR:PATH=pinocchio_DIR-NOTFOUND\n"
        )

        def fake_exists(path):
            return str(path).endswith("libmanipulator.so") \
                or str(path).endswith("CMakeCache.txt")

        def fake_open(path, *args, **kwargs):
            if str(path).endswith("CMakeCache.txt"):
                return io.StringIO(cache_text)
            return open(path, *args, **kwargs)

        def fake_run(command, *args, **kwargs):
            if command[0] == "nm":
                return SimpleNamespace(returncode=0, stdout="", stderr="")
            if command[0] == "ldd":
                return SimpleNamespace(returncode=0, stdout="", stderr="")
            raise AssertionError(command)

        with mock.patch.object(check_runtime_env.os.path, "exists",
                               side_effect=fake_exists), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  side_effect=fake_run), \
                mock.patch("builtins.open", side_effect=fake_open):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_kinematics_backend("/tmp/sdk")

        self.assertFalse(ok)
        self.assertIn("pinocchio backend in libmanipulator", output.getvalue())
        self.assertIn("pinocchio_DIR-NOTFOUND", output.getvalue())

    def test_kinematics_check_reports_unresolved_runtime_libraries(self):
        def fake_exists(path):
            return str(path).endswith("libmanipulator.so") \
                or str(path).endswith("CMakeCache.txt")

        def fake_run(command, *args, **kwargs):
            if command[0] == "nm":
                return SimpleNamespace(
                    returncode=0,
                    stdout="_ZN9pinocchio4testEv\n",
                    stderr="",
                )
            if command[0] == "ldd":
                return SimpleNamespace(
                    returncode=0,
                    stdout="libpinocchio_default.so => not found\n",
                    stderr="",
                )
            raise AssertionError(command)

        with mock.patch.object(check_runtime_env.os.path, "exists",
                               side_effect=fake_exists), \
                mock.patch.object(check_runtime_env.subprocess, "run",
                                  side_effect=fake_run), \
                mock.patch("builtins.open", return_value=io.StringIO(
                    "pinocchio_DIR:PATH=/opt/ros/humble/lib/cmake/pinocchio\n")):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_kinematics_backend("/tmp/sdk")

        self.assertFalse(ok)
        self.assertIn("libmanipulator runtime libraries", output.getvalue())
        self.assertIn("libpinocchio_default.so => not found",
                      output.getvalue())

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
        self.assertIn("role=manipulator.uart_device", text)
        self.assertIn("/dev/serial/by-id/usb-single-serial-if00", text)
        self.assertIn("ID_MODEL=USB Single Serial", text)
        self.assertIn("/dev/ttyACM1", text)
        self.assertIn("role=mobile_base.dev_path", text)
        self.assertIn("ID_MODEL=2K USB Camera", text)

    def test_main_checks_mobile_base_serial_device(self):
        config = {
            "camera": {"type": "realsense", "realsense": {}},
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
            "/dev/ttyACM2", "", False, mock.ANY)
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
                                  side_effect=fake_props), \
                mock.patch.object(check_runtime_env, "_probe_so101_device",
                                  side_effect=lambda d, r: d == "/dev/ttyACM1"):
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
        self.assertIn("manipulator.uart_device role", text)
        self.assertIn("camera serial", text)
        self.assertIn(
            'manipulator.uart_device: "/dev/serial/by-id/usb-arm-if00"',
            text,
        )
        self.assertIn(
            'mobile_base.dev_path: "/dev/serial/by-id/usb-base-if00"',
            text,
        )

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
                                  }), \
                mock.patch.object(check_runtime_env, "_probe_so101_device",
                                  side_effect=lambda d, r: d == "/dev/ttyACM1"):
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
        self.assertIn("SO101 manipulator port", text)
        self.assertIn(
            'mobile_base.dev_path: "/dev/serial/by-id/usb-base-if00"',
            text,
        )

    def test_serial_role_check_fails_when_no_so101_port_responds(self):
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
                                  }), \
                mock.patch.object(check_runtime_env, "_probe_so101_device",
                                  return_value=False):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                ok = check_runtime_env.check_serial_role_configuration(
                    "/dev/ttyACM1",
                    "/dev/ttyACM2",
                    True,
                    str(ROOT),
                )

        text = output.getvalue()
        self.assertFalse(ok)
        self.assertIn("manipulator.uart_device role", text)
        self.assertIn("SO101 did not respond", text)


if __name__ == "__main__":
    unittest.main()
