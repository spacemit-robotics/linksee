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
import json
import os
import re
import shutil
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


def expand_config_path(path: str) -> str:
    """Expand user and environment references in a configured path."""
    if not path:
        return ""
    return os.path.expanduser(os.path.expandvars(str(path)))


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


def _udev_properties(device: str) -> Dict[str, str]:
    result = subprocess.run(
        ["udevadm", "info", "-q", "property", "-n", device],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return {}

    properties = {}
    for line in (result.stdout or "").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        properties[key] = value
    return properties


def _serial_by_id_links() -> Dict[str, list[str]]:
    links_by_target: Dict[str, list[str]] = {}
    for link in sorted(glob.glob("/dev/serial/by-id/*")):
        target = os.path.realpath(link)
        links_by_target.setdefault(target, []).append(link)
    return links_by_target


def _preferred_device_path(device: str,
                           links_by_target: Dict[str, list[str]]) -> str:
    links = links_by_target.get(os.path.realpath(device), [])
    return links[0] if links else device


def _normalized_udev_value(value: str) -> str:
    return value.replace("_", " ").lower()


def _is_camera_serial(properties: Dict[str, str]) -> bool:
    model = _normalized_udev_value(properties.get("ID_MODEL", ""))
    serial = _normalized_udev_value(properties.get("ID_SERIAL", ""))
    return "camera" in model or "camera" in serial


def _is_usb_single_serial(properties: Dict[str, str]) -> bool:
    model = _normalized_udev_value(properties.get("ID_MODEL", ""))
    vendor = properties.get("ID_VENDOR", "").lower()
    return "usb single serial" in model or vendor == "1a86"


def _same_device(configured: str, detected: str) -> bool:
    if not configured or not detected:
        return False
    return configured == detected or os.path.realpath(configured) == os.path.realpath(detected)


def _probe_so101_device(device: str, sdk_root: str) -> bool:
    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), os.pardir))
    tool_candidates = [os.path.join(project_root, "build", "read_joints")]
    tool_candidates.extend(sorted(glob.glob(
        os.path.join(project_root, "build*", "read_joints"))))
    tool = next((path for path in tool_candidates
                 if os.path.isfile(path) and os.access(path, os.X_OK)), "")
    if not tool:
        return False

    try:
        result = subprocess.run(
            [tool, "--device", device, "--baudrate", "1000000"],
            check=False,
            capture_output=True,
            text=True,
            timeout=8,
            cwd=project_root,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False

    output = (result.stdout or "") + (result.stderr or "")
    if "read failed" in output.lower():
        return False
    match = re.search(r"Current joints \(rad\): \[([^\]]+)\]", output)
    if not match:
        return False
    try:
        joints = [float(part.strip()) for part in match.group(1).split(",")]
    except ValueError:
        return False
    if len(joints) < 5:
        return False
    return not all(value < -3.0 for value in joints[:5])


def check_serial_role_configuration(manipulator_device: str,
                                    mobile_base_device: str,
                                    mobile_base_enabled: bool,
                                    sdk_root: str) -> bool:
    devices = sorted(glob.glob("/dev/ttyACM*"))
    links_by_target = _serial_by_id_links()
    properties_by_device = {
        device: _udev_properties(device)
        for device in devices
    }
    camera_devices = [
        device for device, properties in properties_by_device.items()
        if _is_camera_serial(properties)
    ]
    serial_candidates = [
        device for device, properties in properties_by_device.items()
        if _is_usb_single_serial(properties) and device not in camera_devices
    ]

    detected_arm = ""
    for device in serial_candidates:
        if _probe_so101_device(device, sdk_root):
            detected_arm = device
            break

    inferred_base = ""
    if detected_arm:
        for device in serial_candidates:
            if not _same_device(device, detected_arm):
                inferred_base = device
                break

    ok = True
    manip_is_camera = any(_same_device(manipulator_device, device)
                          for device in camera_devices)
    if manip_is_camera:
        ok &= status(False, "manipulator.uart_device role",
                     f"{manipulator_device} is a camera serial")
    elif detected_arm and not _same_device(manipulator_device, detected_arm):
        ok &= status(False, "manipulator.uart_device role",
                     f"SO101 responds on {detected_arm}")
    elif detected_arm:
        ok &= status(True, "manipulator.uart_device role",
                     f"SO101 responds on {detected_arm}")
    elif serial_candidates:
        ok &= status(False, "manipulator.uart_device role",
                     "SO101 did not respond on USB serial candidates; "
                     "check arm power, cable, and uart_device")

    if detected_arm:
        print("[SUGGEST] manipulator.uart_device: "
              f"\"{_preferred_device_path(detected_arm, links_by_target)}\"")

    if mobile_base_enabled:
        base_is_camera = any(_same_device(mobile_base_device, device)
                             for device in camera_devices)
        if base_is_camera:
            ok &= status(False, "mobile_base.dev_path role",
                         f"{mobile_base_device} is a camera serial")
        elif detected_arm and _same_device(mobile_base_device, detected_arm):
            ok &= status(False, "mobile_base.dev_path role",
                         f"{mobile_base_device} is the SO101 manipulator port")
        elif inferred_base and not _same_device(mobile_base_device, inferred_base):
            ok &= status(False, "mobile_base.dev_path role",
                         f"expected chassis candidate {inferred_base}")
        elif inferred_base:
            ok &= status(True, "mobile_base.dev_path role",
                         f"chassis candidate {inferred_base}")

    if mobile_base_enabled and inferred_base:
        print("[SUGGEST] mobile_base.dev_path: "
              f"\"{_preferred_device_path(inferred_base, links_by_target)}\"")

    return ok


def report_serial_devices(manipulator_device: str = "",
                          mobile_base_device: str = "") -> None:
    devices = sorted(glob.glob("/dev/ttyACM*"))
    if not devices:
        print("[INFO] serial devices: none")
        return

    links_by_target = _serial_by_id_links()
    manipulator_target = os.path.realpath(manipulator_device) \
        if manipulator_device else ""
    mobile_base_target = os.path.realpath(mobile_base_device) \
        if mobile_base_device else ""

    print("[INFO] serial devices:")
    for device in devices:
        roles = []
        real_device = os.path.realpath(device)
        if manipulator_device and (
                device == manipulator_device or real_device == manipulator_target):
            roles.append("manipulator.uart_device")
        if mobile_base_device and (
                device == mobile_base_device or real_device == mobile_base_target):
            roles.append("mobile_base.dev_path")

        by_id = ", ".join(links_by_target.get(real_device, [])) or "none"
        properties = _udev_properties(device)
        details = []
        for key in ("ID_MODEL", "ID_SERIAL", "ID_VENDOR"):
            value = properties.get(key)
            if value:
                details.append(f"{key}={value}")
        role_text = ",".join(roles) if roles else "unassigned"
        detail_text = "; ".join(details) if details else "udev=unknown"
        print(f"  {device}: role={role_text}; by-id={by_id}; {detail_text}")


def check_tty(device: str, fixer: RuntimeFixer | None = None,
              label: str = "manipulator.uart_device") -> bool:
    ok = True
    devices = sorted(glob.glob("/dev/ttyACM*"))
    ok &= status(bool(devices), "/dev/ttyACM*", " ".join(devices) or "not found")
    if not device:
        return ok
    if not os.path.exists(device):
        return status(False, label, f"{device} not found")

    st = os.stat(device)
    mode = stat.filemode(st.st_mode)
    owner = st.st_uid
    group = grp.getgrgid(st.st_gid).gr_name
    can_rw = os.access(device, os.R_OK | os.W_OK)
    ok &= status(can_rw, "serial read/write", f"{device} {mode} uid={owner} group={group}")
    if group == "dialout":
        ok &= check_group_status("dialout", fixer)
    return ok


def check_rpmsg_device(ctrl_dev: str,
                       data_dev: str = "",
                       fixer: RuntimeFixer | None = None) -> bool:
    ok = True
    if not ctrl_dev or ctrl_dev == "auto":
        ok &= status(False, "mobile_base.ctrl_dev",
                     "set an explicit /dev/rpmsg_ctrl* path")
    elif not os.path.exists(ctrl_dev):
        ok &= status(False, "mobile_base.ctrl_dev", f"{ctrl_dev} not found")
    else:
        st = os.stat(ctrl_dev)
        mode = stat.filemode(st.st_mode)
        group = grp.getgrgid(st.st_gid).gr_name
        can_rw = os.access(ctrl_dev, os.R_OK | os.W_OK)
        ok &= status(can_rw, "mobile_base.ctrl_dev read/write",
                     f"{ctrl_dev} {mode} uid={st.st_uid} group={group}")
        if group == "dialout":
            ok &= check_group_status("dialout", fixer)

    if not data_dev or data_dev == "auto":
        ok &= status(False, "mobile_base.data_dev",
                     "set an explicit /dev/rpmsg* path")
    elif os.path.exists(data_dev):
        st = os.stat(data_dev)
        mode = stat.filemode(st.st_mode)
        group = grp.getgrgid(st.st_gid).gr_name
        can_rw = os.access(data_dev, os.R_OK | os.W_OK)
        ok &= status(can_rw, "mobile_base.data_dev read/write",
                     f"{data_dev} {mode} uid={st.st_uid} group={group}")
        if group == "dialout":
            ok &= check_group_status("dialout", fixer)
    else:
        ok &= status(True, "mobile_base.data_dev",
                     f"{data_dev} will be created by chassis driver")
    return ok


def check_chassis_driver_registered(sdk_root: str, driver: str) -> bool:
    if not driver or driver == "none":
        return True

    lib_path = os.path.join(sdk_root, "output", "staging", "lib",
                            "libchassis.so")
    try:
        with open(lib_path, "rb") as f:
            data = f.read()
    except OSError as exc:
        return status(False, "libchassis.so", f"{lib_path}: {exc}")

    registered = driver.encode("utf-8") in data
    detail = "registered" if registered else "not registered in libchassis.so"
    return status(registered, f"chassis driver {driver}", detail)


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


def check_realsense_camera() -> bool:
    matches = []
    for path in sorted(glob.glob("/dev/video*")):
        properties = _udev_properties(path)
        model = _normalized_udev_value(properties.get("ID_MODEL", ""))
        product = _normalized_udev_value(properties.get("ID_V4L_PRODUCT", ""))
        vendor_id = properties.get("ID_VENDOR_ID", "").lower()
        model_id = properties.get("ID_MODEL_ID", "").lower()
        is_d435i = (
            vendor_id == "8086" and model_id == "0b3a"
        ) or (
            "realsense" in model or "realsense" in product
        ) or (
            "d435i" in model or "435i" in product
        )
        if is_d435i:
            matches.append(path)

    return status(bool(matches), "RealSense D435i",
                  " ".join(matches) if matches else "not found")


def _check_readable_file(path: str, label: str) -> bool:
    exists = bool(path) and os.path.isfile(path)
    readable = exists and os.access(path, os.R_OK)
    detail = path if readable else f"{path or 'not configured'} not readable"
    return status(readable, label, detail)


def _find_las2_library(sdk_root: str, model_path: str,
                       calibration_path: str) -> str:
    roots = []
    runtime_dir = os.environ.get("LAS2_RUNTIME_DIR", "")
    if runtime_dir:
        roots.append(runtime_dir)
    if sdk_root:
        roots.append(os.path.join(sdk_root, "output", "staging"))
    for path in (model_path, calibration_path):
        if path:
            roots.append(os.path.dirname(os.path.dirname(
                os.path.abspath(path))))
    roots.extend(filter(None, os.environ.get("LD_LIBRARY_PATH", "").split(":")))

    candidates = []
    for root in roots:
        candidates.append(os.path.join(root, "lib", "liblas2_usb_stereo.so"))
        candidates.append(os.path.join(root, "liblas2_usb_stereo.so"))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    return ""


def _find_application_binaries() -> tuple[str, str]:
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    build_dirs = [os.path.join(project_root, "build")]
    build_dirs.extend(sorted(glob.glob(os.path.join(project_root, "build*"))))
    path_launcher = shutil.which("perceptive_grasp")
    if path_launcher:
        build_dirs.append(os.path.dirname(os.path.abspath(path_launcher)))

    partial_match = ("", "")
    for build_dir in dict.fromkeys(build_dirs):
        launcher = os.path.join(build_dir, "perceptive_grasp")
        core = os.path.join(build_dir, "perceptive_grasp_core")
        launcher_exists = os.path.isfile(launcher)
        core_exists = os.path.isfile(core)
        if launcher_exists and core_exists:
            return launcher, core
        if not any(partial_match) and (launcher_exists or core_exists):
            partial_match = (launcher if launcher_exists else "",
                             core if core_exists else "")
    return partial_match


def check_application_binaries() -> bool:
    launcher, core = _find_application_binaries()
    launcher_ok = bool(launcher) and os.path.isfile(launcher) \
        and os.access(launcher, os.X_OK)
    core_ok = bool(core) and os.path.isfile(core) and os.access(core, os.X_OK)
    ok = status(
        launcher_ok,
        "perceptive_grasp launcher",
        launcher if launcher_ok else "build or install perceptive_grasp",
    )
    ok &= status(
        core_ok,
        "perceptive_grasp core",
        core if core_ok else "perceptive_grasp_core missing or not executable",
    )
    return ok


def check_voice_echo_cancellation(root: Dict[str, Any]) -> bool:
    voice = root.get("voice", {}) if isinstance(root, dict) else {}
    echo_config = voice.get("echo_cancellation", {}) \
        if isinstance(voice, dict) else {}
    echo_config = echo_config if isinstance(echo_config, dict) else {}
    mode = str(echo_config.get("mode", "half_duplex")).strip().lower()
    supported_modes = ("hardware_aec", "webrtc_aec", "half_duplex")
    if mode not in supported_modes:
        return status(
            False,
            "voice echo cancellation",
            f"unsupported mode {mode!r}",
        )
    if mode != "webrtc_aec":
        return status(True, "voice echo cancellation", f"mode={mode}")

    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), os.pardir)
    )
    candidates = [
        os.environ.get("PERCEPTIVE_GRASP_VOICE_AEC_LIB", ""),
        os.path.join(project_root, "build", "libperceptive_voice_aec.so"),
        os.path.join(project_root, "build_las2", "libperceptive_voice_aec.so"),
        os.path.abspath(os.path.join(
            os.path.dirname(__file__), os.pardir, os.pardir, os.pardir,
            "lib", "libperceptive_voice_aec.so",
        )),
    ]
    library = next((path for path in candidates if path and os.path.isfile(path)), "")
    if not library:
        return status(
            False,
            "WebRTC AEC library",
            "libperceptive_voice_aec.so not found; install meson and "
            "ninja-build, then rebuild perceptive_grasp",
        )

    try:
        result = subprocess.run(
            ["ldd", library],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return status(False, "WebRTC AEC runtime libraries", str(exc))
    output = (result.stdout or "") + (result.stderr or "")
    missing = [line.strip() for line in output.splitlines() if "not found" in line]
    if missing:
        return status(False, "WebRTC AEC runtime libraries", "; ".join(missing))
    return status(True, "voice echo cancellation", f"mode={mode}; {library}")


def _check_las2_runtime_libraries(library: str,
                                  application: str = "") -> bool:
    ldd_target = application or library
    try:
        result = subprocess.run(
            ["ldd", ldd_target],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return status(False, "LAS2 runtime libraries", str(exc))

    output = (result.stdout or "") + (result.stderr or "")
    application_uses_las2 = not application \
        or "liblas2_usb_stereo.so" in output
    missing = [line.strip() for line in output.splitlines()
               if "not found" in line]
    if application and not application_uses_las2:
        return _check_las2_runtime_libraries(library)
    if missing:
        lib_dir = os.path.dirname(os.path.abspath(library))
        runtime_root = os.path.dirname(lib_dir)
        command = (
            f'export LD_LIBRARY_PATH="{lib_dir}:'
            f'{runtime_root}/lib/spacemit_ort:'
            f'{runtime_root}/lib/opencv:${{LD_LIBRARY_PATH:-}}"'
        )
        return status(False, "LAS2 runtime libraries",
                      "; ".join(missing) + f"; run: {command}")

    detail = "all dependencies resolved"
    if application and result.returncode == 0:
        detail += f" via {application}"
    return status(result.returncode == 0, "LAS2 runtime libraries",
                  detail if result.returncode == 0 else output.strip())


def check_las2_capture_requirements(device: str) -> bool:
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-formats-ext"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return status(False, "LAS2 YUYV capture format", str(exc))

    formats = (result.stdout or "") + (result.stderr or "")
    yuyv_match = re.search(
        r"\[\d+\]:\s*'YUYV'.*?(?=\n\s*\[\d+\]:|\Z)",
        formats,
        flags=re.DOTALL,
    )
    yuyv = yuyv_match.group(0) if yuyv_match else ""
    size_ok = "Size: Discrete 4000x1200" in yuyv
    fps_values = [float(value) for value in re.findall(
        r"\(([0-9.]+) fps\)", yuyv)]
    max_fps = max(fps_values, default=0.0)
    format_ok = result.returncode == 0 and size_ok and max_fps >= 29.0
    detail = "YUYV 4000x1200@30 available" if format_ok else (
        f"requires YUYV 4000x1200@30; advertised max is {max_fps:g} fps"
    )
    ok = status(format_ok, "LAS2 YUYV capture format", detail)

    heap = "/dev/dma_heap/system"
    heap_ok = os.path.exists(heap) and os.access(heap, os.R_OK | os.W_OK)
    ok &= status(heap_ok, "LAS2 DMA heap",
                 heap if heap_ok else f"{heap} not accessible")
    return ok


def check_las2_camera(camera: Dict[str, Any], sdk_root: str) -> bool:
    las2 = camera.get("spacemit_las2", {}) \
        if isinstance(camera, dict) else {}
    device = las2.get("video_device", "")
    model = expand_config_path(las2.get("model_path", ""))
    calibration = expand_config_path(las2.get("calib_path", ""))
    core_count = las2.get("core_count", 1)
    core_affinity = str(las2.get("core_affinity", "8"))
    try:
        affinity_ids = [int(value.strip()) for value in core_affinity.split(",")
                        if value.strip()]
    except ValueError:
        affinity_ids = []

    ok = True
    device_ok = bool(device) and os.path.exists(device) \
        and os.access(device, os.R_OK | os.W_OK)
    ok &= status(device_ok, "LAS2 video device",
                 device if device_ok else f"{device or 'not configured'} not accessible")
    if device_ok:
        ok &= check_las2_capture_requirements(device)
    ok &= _check_readable_file(model, "LAS2 model")
    calibration_ok = _check_readable_file(calibration, "LAS2 calibration")
    ok &= calibration_ok
    if calibration_ok:
        try:
            with open(calibration, "r", encoding="utf-8") as file:
                data = json.load(file)
            required = {
                "image_size", "left_camera_matrix", "right_camera_matrix",
                "left_dist_coeffs", "right_dist_coeffs", "R", "T",
            }
            missing = sorted(required.difference(data))
            ok &= status(not missing, "LAS2 calibration fields",
                         "complete" if not missing else "missing: " + ", ".join(missing))
        except (OSError, ValueError, TypeError) as exc:
            ok &= status(False, "LAS2 calibration JSON", str(exc))

    count_ok = isinstance(core_count, int) and 1 <= core_count <= 16
    ok &= status(count_ok, "LAS2 core_count", str(core_count))
    affinity_ok = count_ok and len(affinity_ids) == core_count \
        and len(set(affinity_ids)) == len(affinity_ids) \
        and all(8 <= core <= 15 for core in affinity_ids)
    affinity_detail = core_affinity if affinity_ok else (
        f"{core_affinity}; expected {core_count} unique X100 AI core IDs "
        "in range 8-15"
    )
    ok &= status(affinity_ok, "LAS2 core_affinity", affinity_detail)
    library = _find_las2_library(sdk_root, model, calibration)
    ok &= status(bool(library), "liblas2_usb_stereo.so",
                 library or "set LAS2_RUNTIME_DIR or LD_LIBRARY_PATH")
    if library:
        _, application = _find_application_binaries()
        ok &= _check_las2_runtime_libraries(
            library, application)
    return ok


def check_las2_detector_provider(root: Dict[str, Any],
                                 pipeline_config: str) -> bool:
    detection = root.get("detection", {}) if isinstance(root, dict) else {}
    detector_config = detection.get("config_path", "") \
        if isinstance(detection, dict) else ""
    if detector_config and not os.path.isabs(detector_config):
        detector_config = os.path.join(
            os.path.dirname(os.path.abspath(pipeline_config)),
            detector_config,
        )

    if not detector_config or not os.path.isfile(detector_config):
        return status(False, "LAS2 detector provider",
                      f"detector config not found: "
                      f"{detector_config or 'not configured'}")

    try:
        detector = load_yaml(detector_config)
        params = detector.get("default_params", {}) \
            if isinstance(detector, dict) else {}
        providers = params.get("providers", []) \
            if isinstance(params, dict) else []
        provider = providers[0] if providers else "SpaceMITExecutionProvider"
        detector_threads = int(params.get("num_threads", 4)) \
            if isinstance(params, dict) else 4
    except (OSError, ValueError, TypeError) as exc:
        return status(False, "LAS2 detector provider", str(exc))

    if provider != "SpaceMITExecutionProvider":
        return status(
            False,
            "LAS2 detector provider",
            f"{provider}; YOLO must use SpaceMITExecutionProvider",
        )

    camera = root.get("camera", {}) if isinstance(root, dict) else {}
    las2 = camera.get("spacemit_las2", {}) \
        if isinstance(camera, dict) else {}
    affinity = str(las2.get(
        "core_affinity", "8,9,10,11,12,13,14,15"))
    try:
        las2_cores = {
            int(value.strip()) for value in affinity.split(",")
            if value.strip()
        }
    except ValueError:
        las2_cores = set(range(8, 16))
    free_ai_cores = sorted(set(range(8, 16)) - las2_cores)
    required_cores = max(1, min(detector_threads, 8))
    compatible = len(free_ai_cores) >= required_cores
    if compatible:
        detail = (f"{provider}; free AI cores="
                  + ",".join(str(core) for core in free_ai_cores))
    else:
        detail = (
            f"{provider} requires {required_cores} free AI cores, "
            f"but LAS2 affinity {affinity} leaves "
            f"{len(free_ai_cores)}; reduce LAS2 core_count/affinity"
        )
    return status(compatible, "LAS2 detector provider", detail)


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
    camera = root.get("camera", {}) if isinstance(root, dict) else {}
    camera_type = str(camera.get("type", "realsense"))
    camera_type = {"d435i": "realsense"}.get(camera_type, camera_type)
    manip = root.get("manipulator", {}) if isinstance(root, dict) else {}
    device = manip.get("uart_device", "/dev/ttyACM0")
    mobile_base = root.get("mobile_base", {}) if isinstance(root, dict) else {}
    mobile_base_enabled = bool(mobile_base.get("enabled", False))
    mobile_base_driver = mobile_base.get("driver", "drv_uart_esp32")
    mobile_base_is_uart = mobile_base_enabled \
        and mobile_base_driver == "drv_uart_esp32"
    mobile_base_device = mobile_base.get("dev_path", "") \
        if mobile_base_is_uart else ""

    print(f"[INFO] python={sys.version.split()[0]} executable={sys.executable}")
    fixer = RuntimeFixer(interactive=sys.stdin.isatty() and not args.no_fix,
                         assume_yes=args.yes)
    ok = True
    supported_camera = camera_type in ("realsense", "spacemit_las2")
    ok &= status(supported_camera, "camera.type", camera_type)
    backend_config = camera.get(camera_type) if supported_camera else None
    ok &= status(isinstance(backend_config, dict),
                 f"camera.{camera_type} configuration")
    sdk_root = os.environ.get("SDK_ROOT", "") or infer_sdk_root(os.getcwd())
    ok &= check_application_binaries()
    report_serial_devices(device, mobile_base_device)
    ok &= check_serial_role_configuration(
        device, mobile_base_device, mobile_base_is_uart, sdk_root)
    ok &= check_tty(device, fixer, "manipulator.uart_device")
    if mobile_base_enabled:
        ok &= check_chassis_driver_registered(sdk_root, mobile_base_driver)
    if mobile_base_is_uart:
        ok &= check_tty(mobile_base_device, fixer, "mobile_base.dev_path")
    elif mobile_base_enabled and mobile_base_driver == "drv_rpmsg_esos":
        ok &= check_rpmsg_device(
            mobile_base.get("ctrl_dev", "/dev/rpmsg_ctrl0"),
            mobile_base.get("data_dev", "/dev/rpmsg0"),
            fixer)
    ok &= check_video_permissions(fixer)
    if camera_type == "spacemit_las2":
        ok &= check_las2_camera(camera, sdk_root)
        ok &= check_las2_detector_provider(root, args.config)
    else:
        ok &= check_realsense_camera()
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
        ok &= check_voice_echo_cancellation(root)
        ok &= check_audio_permissions(fixer)
        ok &= list_audio_devices()

    print("[SUMMARY] " + ("ready" if ok else "fix failed checks before running"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
