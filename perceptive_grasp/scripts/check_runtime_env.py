#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Preflight checks for perceptive_grasp runtime dependencies."""

import argparse
import glob
import getpass
import grp
import importlib
import os
import stat
import subprocess
import sys
from typing import Any, Dict


def load_yaml(path: str) -> Dict[str, Any]:
    if not path:
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def status(ok: bool, name: str, detail: str = "") -> bool:
    mark = "OK" if ok else "FAIL"
    print(f"[{mark}] {name}" + (f": {detail}" if detail else ""))
    return ok


def prompt_yes_no(question: str, default: bool = False) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    try:
        answer = input(f"{question} {suffix} ").strip().lower()
    except EOFError:
        return False
    if not answer:
        return default
    return answer in ("y", "yes")


def current_user_groups() -> set[str]:
    user = getpass.getuser()
    groups = {g.gr_name for g in grp.getgrall() if user in g.gr_mem}
    groups.add(grp.getgrgid(os.getgid()).gr_name)
    return groups


def active_user_groups() -> set[str]:
    group_ids = set(os.getgroups())
    group_ids.add(os.getgid())
    groups = set()
    for group_id in group_ids:
        try:
            groups.add(grp.getgrgid(group_id).gr_name)
        except KeyError:
            continue
    return groups


def check_group_status(group: str, fixer: "RuntimeFixer | None" = None) -> bool:
    configured = group in current_user_groups()
    active = group in active_user_groups()
    fix = "" if configured else f"run: sudo usermod -aG {group} $USER && sudo reboot"
    ok = status(configured, f"configured {group} group", fix)
    if not configured and fixer is not None:
        fixer.offer_group_fix(group)
    if configured:
        detail = "" if active else "重新登录、重启，或打开新的登录会话后再运行"
        ok &= status(active, f"active {group} group", detail)
    return ok


class RuntimeFixer:
    def __init__(self, interactive: bool, assume_yes: bool = False) -> None:
        self.interactive = interactive
        self.assume_yes = assume_yes

    def _confirm(self, question: str) -> bool:
        if self.assume_yes:
            return True
        if not self.interactive:
            print("[INFO] non-interactive shell; skip automatic fix")
            return False
        return prompt_yes_no(question)

    def offer_group_fix(self, group: str) -> bool:
        user = getpass.getuser()
        command = ["sudo", "usermod", "-aG", group, user]
        question = f"Run 'sudo usermod -aG {group} {user}' now?"
        if not self._confirm(question):
            return False
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            status(False, f"add user to {group} group",
                   f"command failed with code {result.returncode}")
            return False
        status(True, f"add user to {group} group",
               "restart login session or reboot for the new group to take effect")
        return True

    def offer_requirements_install(self, missing_modules: list[str]) -> bool:
        project_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), os.pardir))
        requirements = os.path.join(project_root, "requirements.txt")
        if not os.path.exists(requirements):
            status(False, "install Python dependencies",
                   f"{requirements} not found")
            return False
        modules = ", ".join(missing_modules)
        question = f"Install Python dependencies for missing modules ({modules}) now?"
        if not self._confirm(question):
            return False
        command = [sys.executable, "-m", "pip", "install", "-r", requirements]
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            status(False, "install Python dependencies",
                   f"command failed with code {result.returncode}")
            return False
        return status(True, "install Python dependencies")


def check_import(module: str, required: bool) -> bool:
    try:
        importlib.import_module(module)
        return status(True, f"import {module}")
    except Exception as exc:
        ok = not required
        prefix = "optional missing" if ok else f"{type(exc).__name__}: {exc}"
        return status(ok, f"import {module}", prefix)


def check_tty(device: str, fixer: RuntimeFixer | None = None) -> bool:
    ok = True
    devices = sorted(glob.glob("/dev/ttyACM*"))
    ok &= status(bool(devices), "/dev/ttyACM*", " ".join(devices) or "not found")
    if not device:
        return ok
    if not os.path.exists(device):
        return status(False, "manipulator.uart_device", f"{device} not found")

    st = os.stat(device)
    mode = stat.filemode(st.st_mode)
    owner = st.st_uid
    group = grp.getgrgid(st.st_gid).gr_name
    can_rw = os.access(device, os.R_OK | os.W_OK)
    ok &= status(can_rw, "serial read/write", f"{device} {mode} uid={owner} group={group}")
    if group == "dialout":
        ok &= check_group_status("dialout", fixer)
    return ok


def check_audio_permissions(fixer: RuntimeFixer | None = None) -> bool:
    paths = sorted(glob.glob("/dev/snd/*"))
    ok = status(bool(paths), "/dev/snd/*", " ".join(paths) or "not found")
    if not paths:
        return False

    blocked = []
    audio_group_found = False
    for path in paths:
        st = os.stat(path)
        if not stat.S_ISCHR(st.st_mode):
            continue
        mode = stat.filemode(st.st_mode)
        group = grp.getgrgid(st.st_gid).gr_name
        if group == "audio":
            audio_group_found = True
        if not os.access(path, os.R_OK | os.W_OK):
            blocked.append(f"{path} {mode} uid={st.st_uid} group={group}")

    ok &= status(not blocked, "audio device read/write",
                 "; ".join(blocked) if blocked else "all accessible")
    if audio_group_found:
        ok &= check_group_status("audio", fixer)
    return ok


def check_video_permissions(fixer: RuntimeFixer | None = None) -> bool:
    paths = sorted(glob.glob("/dev/video*"))
    ok = status(bool(paths), "/dev/video*", " ".join(paths) or "not found")
    if not paths:
        return False

    blocked = []
    video_group_found = False
    for path in paths:
        st = os.stat(path)
        if not stat.S_ISCHR(st.st_mode):
            continue
        mode = stat.filemode(st.st_mode)
        group = grp.getgrgid(st.st_gid).gr_name
        if group == "video":
            video_group_found = True
        if not os.access(path, os.R_OK | os.W_OK):
            blocked.append(f"{path} {mode} uid={st.st_uid} group={group}")

    ok &= status(not blocked, "video device read/write",
                 "; ".join(blocked) if blocked else "all accessible")
    if video_group_found:
        ok &= check_group_status("video", fixer)
    return ok


def _read_text_file(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def infer_sdk_root(start_path: str) -> str:
    path = os.path.abspath(start_path)
    marker = os.path.join("application", "ros2", "linksee", "perceptive_grasp")
    if path.endswith(marker):
        return path[:-(len(marker) + 1)]

    parts = path.split(os.sep)
    marker_parts = marker.split(os.sep)
    for index in range(0, len(parts) - len(marker_parts) + 1):
        if parts[index:index + len(marker_parts)] == marker_parts:
            root_parts = parts[:index]
            return os.sep.join(root_parts) or os.sep
    return ""


def check_kinematics_backend(sdk_root: str) -> bool:
    if not sdk_root:
        return status(False, "SDK_ROOT", "source build/envsetup.sh first")

    lib_path = os.path.join(sdk_root, "output", "staging", "lib",
                            "libmanipulator.so")
    if not os.path.exists(lib_path):
        return status(False, "libmanipulator.so",
                      f"{lib_path} not found; build "
                      "components/control/manipulator")

    nm = subprocess.run(["nm", "-D", lib_path],
                        check=False, capture_output=True, text=True)
    symbols = (nm.stdout or "") + (nm.stderr or "")
    has_pinocchio_symbol = "pinocchio" in symbols.lower()
    ok = status(has_pinocchio_symbol, "pinocchio backend in libmanipulator",
                "" if has_pinocchio_symbol
                else "rebuild manipulator after installing pinocchio")

    ldd = subprocess.run(["ldd", lib_path],
                         check=False, capture_output=True, text=True)
    linked_libraries = (ldd.stdout or "") + (ldd.stderr or "")
    missing_libraries = [
        line.strip()
        for line in linked_libraries.splitlines()
        if "not found" in line
    ]
    ok &= status(not missing_libraries, "libmanipulator runtime libraries",
                 "; ".join(missing_libraries))

    cache_path = os.path.join(sdk_root, "output", "build", "cmake", "pkgs",
                              "components_control_manipulator", "CMakeCache.txt")
    cache = _read_text_file(cache_path)
    if "pinocchio_DIR:PATH=pinocchio_DIR-NOTFOUND" in cache:
        ok &= status(False, "pinocchio CMake package",
                     "pinocchio_DIR-NOTFOUND")
    elif cache:
        ok &= status(True, "pinocchio CMake package")
    else:
        status(False, "pinocchio CMake package",
               f"{cache_path} not found")
    return ok


def list_audio_devices() -> bool:
    ok = True
    try:
        from spacemit_audio import AudioCapture, AudioPlayer
    except Exception as exc:
        return status(False, "spacemit_audio devices", f"{type(exc).__name__}: {exc}")

    try:
        inputs = AudioCapture.list_devices()
        print("[INFO] capture devices:")
        for idx, name in inputs:
            print(f"  [{idx}] {name}")
        ok &= status(bool(inputs), "audio capture devices")
    except Exception as exc:
        ok &= status(False, "audio capture devices", str(exc))

    try:
        outputs = AudioPlayer.list_devices()
        print("[INFO] playback devices:")
        for idx, name in outputs:
            print(f"  [{idx}] {name}")
        ok &= status(bool(outputs), "audio playback devices")
    except Exception as exc:
        ok &= status(False, "audio playback devices", str(exc))
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Check perceptive_grasp runtime environment")
    parser.add_argument("--config", default="config/grasp_pipeline.yaml")
    parser.add_argument("--skip-voice", action="store_true",
                        help="只检查机械臂串口，不检查本地 ASR/TTS 依赖")
    parser.add_argument("--no-fix", action="store_true",
                        help="只检查环境，不提示执行自动修复")
    parser.add_argument("--yes", action="store_true",
                        help="自动确认可执行的修复动作")
    args = parser.parse_args()

    root = load_yaml(args.config)
    manip = root.get("manipulator", {}) if isinstance(root, dict) else {}
    device = manip.get("uart_device", "/dev/ttyACM0")

    print(f"[INFO] python={sys.version.split()[0]} executable={sys.executable}")
    fixer = RuntimeFixer(interactive=sys.stdin.isatty() and not args.no_fix,
                         assume_yes=args.yes)
    ok = True
    ok &= check_tty(device, fixer)
    ok &= check_video_permissions(fixer)
    sdk_root = os.environ.get("SDK_ROOT", "") or infer_sdk_root(os.getcwd())
    ok &= check_kinematics_backend(sdk_root)

    if not args.skip_voice:
        required_modules = [
            "spacemit_audio",
            "spacemit_vad",
            "spacemit_asr",
            "spacemit_tts",
        ]
        missing_modules = []
        imports_ok = True
        for module in required_modules:
            imported = check_import(module, required=True)
            imports_ok &= imported
            if not imported:
                missing_modules.append(module)
        if missing_modules and not args.no_fix \
                and fixer.offer_requirements_install(missing_modules):
            missing_modules = []
            imports_ok = True
            for module in required_modules:
                imported = check_import(module, required=True)
                imports_ok &= imported
                if not imported:
                    missing_modules.append(module)
        ok &= imports_ok
        ok &= check_audio_permissions(fixer)
        ok &= list_audio_devices()

    print("[SUMMARY] " + ("ready" if ok else "fix failed checks before running"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
