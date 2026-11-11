#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Build and run the manipulation-backed simulation without hardware access."""

import argparse
import os
from pathlib import Path
import subprocess
import sys


APP = Path(__file__).resolve().parents[1]
SDK = APP.parents[3]


def run(command):
    """Keep argument boundaries and report a failing subprocess unchanged."""
    subprocess.run([str(item) for item in command], cwd=APP, check=True)


def build(args):
    """Build into a dedicated tree, never the installed hardware SDK prefix."""
    prefix = args.build_dir / "prefix"
    common = SDK / "components/control/manipulation"
    if not args.client_only:
        if not (common / "CMakeLists.txt").is_file():
            raise RuntimeError("place this application inside the spacemit_robot SDK")
        command = [
            "cmake", "-S", common, "-B", args.build_dir / "sdk",
            "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_INSTALL_PREFIX={prefix}",
            "-DBUILD_TESTING=OFF", "-DMANIPULATION_ENABLE_NERO=OFF",
            "-DMANIPULATION_ENABLE_SO101=OFF", "-DMANIPULATION_ENABLE_CALIBRATION=OFF",
            "-DMANIPULATION_ENABLE_MUJOCO=ON",
            "-DMANIPULATION_ENABLE_SIMULATION_RENDERER=ON",
        ]
        if args.mujoco_dir:
            command.append(f"-DMUJOCO_DIR={args.mujoco_dir}")
        if args.gl4es:
            command.extend(["-DMUJOCO_RENDER_BACKEND=X11", "-DMUJOCO_RENDER_USE_GL4ES=ON"])
        run(command)
        run(["cmake", "--build", args.build_dir / "sdk", "-j", args.jobs])
        run(["cmake", "--install", args.build_dir / "sdk"])
    target = args.build_dir / ("client" if args.client_only else "app")
    command = [
        "cmake", "-S", APP, "-B", target, "-DCMAKE_BUILD_TYPE=Release",
        "-DUSE_OPENCV_DNN=ON", "-DUSE_MOCK_EXECUTOR=ON",
        "-DENABLE_WEBRTC_AEC=OFF", "-DENABLE_LAS2_CAMERA=OFF",
        "-DENABLE_REMOTE_MUJOCO=ON",
        f"-DENABLE_MUJOCO_EXECUTOR={'OFF' if args.client_only else 'ON'}",
        f"-DENABLE_MANIPULATION_MUJOCO={'OFF' if args.client_only else 'ON'}",
    ]
    if not args.client_only:
        command.append(f"-DCMAKE_PREFIX_PATH={prefix}")
        if args.mujoco_dir:
            command.append(f"-DMUJOCO_DIR={args.mujoco_dir}")
    run(command)
    run(["cmake", "--build", target, "-j", args.jobs])


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--build-dir", type=Path, default=APP / "build_manipulation")
    commands = result.add_subparsers(dest="command", required=True)
    command = commands.add_parser("build", parents=[common], help="prepare simulation binaries")
    command.add_argument("--client-only", action="store_true", help="no local physics or graphics")
    command.add_argument("--jobs", type=int, default=2)
    command.add_argument("--mujoco-dir", type=Path)
    command.add_argument("--gl4es", action="store_true", help="use the K3 gl4es graphics backend")
    command = commands.add_parser("serve", parents=[common], help="run the local scene and viewer")
    command.add_argument("--no-viewer", action="store_true")
    command.add_argument("--port", type=int, default=9090)
    command = commands.add_parser("grasp", parents=[common], help="grasp a simulated object")
    command.add_argument("target", choices=["apple", "banana", "cube", "cup"])
    command.add_argument("--client-only", action="store_true", help="use the client-only build")
    command.add_argument("--host", default="127.0.0.1")
    command.add_argument("--port", type=int, default=9090)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    args.build_dir = args.build_dir.resolve()
    if args.command == "build":
        if args.jobs < 1 or args.jobs > 256:
            raise ValueError("--jobs must be in 1..256")
        if args.mujoco_dir:
            args.mujoco_dir = args.mujoco_dir.resolve()
        build(args)
        return
    if not 1 <= args.port <= 65535:
        raise ValueError("--port must be in 1..65535")
    directory = args.build_dir / ("client" if getattr(args, "client_only", False) else "app")
    if args.command == "serve":
        command = [directory / "mujoco_grasp_sim_server", "--config",
                   APP / "config/grasp_pipeline_manipulation_mujoco.yaml",
                   "--listen", "127.0.0.1", "--port", str(args.port)]
        if not args.no_viewer:
            command.append("--viewer")
    else:
        command = [directory / "perceptive_grasp", "--config",
                   APP / "config/grasp_pipeline_manipulation_remote.yaml",
                   "--target", args.target, "--remote-host", args.host,
                   "--remote-port", str(args.port)]
    if not command[0].is_file():
        raise RuntimeError("simulation binary is missing; run the build command first")
    if args.command == "serve":
        # Bound Mesa software rasterizer workers so rendering leaves CPU time for physics.
        # Hardware rendering is unchanged, and explicit operator settings take precedence.
        os.environ.setdefault("LP_NUM_THREADS", "2")
    # Replace this launcher so Ctrl+C reaches the application directly.
    os.chdir(APP)
    os.execv(str(command[0]), [str(item) for item in command])


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
