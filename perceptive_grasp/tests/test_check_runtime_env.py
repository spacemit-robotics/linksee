#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for runtime environment diagnostics."""

import contextlib
import io
import stat
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import check_runtime_env  # noqa: E402


class RuntimeEnvDiagnosticsTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
