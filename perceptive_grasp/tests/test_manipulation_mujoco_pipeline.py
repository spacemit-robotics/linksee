#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise rendered RGB-D, remote perception, contact grasp and placement."""

import argparse
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

import yaml


def receive(stream, count):
    data = bytearray()
    while len(data) < count:
        part = stream.recv(count - len(data))
        if not part:
            raise RuntimeError("simulation closed a partial frame")
        data.extend(part)
    return bytes(data)


def frame(port):
    with socket.create_connection(("127.0.0.1", port), timeout=10) as stream:
        stream.sendall(struct.pack("!IHHI", 0x50474D4A, 1, 1, 0))
        magic, version, _, size = struct.unpack("!IHHI", receive(stream, 12))
        if magic != 0x50474D4A or version != 1 or size > 64000000:
            raise RuntimeError("invalid frame response header")
        response = receive(stream, size)
    ok, result, length = struct.unpack_from("!Bii", response)
    detail = response[9:9 + length].decode()
    if not ok or result:
        raise RuntimeError(f"camera frame failed: {detail}")
    payload = response[9 + length:]
    sample, width, height, fx, fy, cx, cy, rgb_size, depth_size = struct.unpack_from(
        "!qiiffffii", payload)
    if width <= 0 or height <= 0 or rgb_size != width * height * 3:
        raise RuntimeError("invalid RGB image dimensions")
    if depth_size != width * height * 2 or len(payload) != 40 + rgb_size + depth_size:
        raise RuntimeError("invalid depth image dimensions")
    if not (fx > 0 and fy > 0 and 0 <= cx < width and 0 <= cy < height):
        raise RuntimeError("invalid camera intrinsics")
    rgb = payload[40:40 + rgb_size]
    depth = payload[40 + rgb_size:]
    if len(set(rgb)) < 20 or not any(depth):
        raise RuntimeError("RGB-D frame lacks rendered scene content")
    return sample, rgb, width, height


def save_image(directory, name, frame_data):
    if directory is None:
        return
    directory.mkdir(parents=True, exist_ok=True)
    _, bgr, width, height = frame_data
    rgb = bytearray(len(bgr))
    rgb[0::3], rgb[1::3], rgb[2::3] = bgr[2::3], bgr[1::3], bgr[0::3]
    (directory / name).write_bytes(f"P6\n{width} {height}\n255\n".encode() + rgb)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--client", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--xvfb", required=True)
    parser.add_argument("--target", choices=["apple", "banana", "cup", "cube"], default="apple")
    parser.add_argument("--artifacts", type=Path)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--local", action="store_true")
    parser.add_argument("--cancel-local", action="store_true")
    args = parser.parse_args()
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix="manipulation-grasp-") as temporary:
        directory = Path(temporary)
        environment = dict(os.environ, MANIPULATION_LOCK_DIRECTORY=str(directory / "locks"))
        environment.setdefault("LP_NUM_THREADS", "2")
        if args.local:
            profile = yaml.safe_load((args.source / "config/grasp_pipeline_manipulation_mujoco.yaml").read_text())
            profile["manipulator"]["mujoco"]["device_plan"] = str(
                args.source / "simulation/mujoco/manipulation/device.yaml")
            profile["detection"]["config_path"] = str(args.source / "config/yolov8_seg.yaml")
            profile["debug"] = {"save_grasp_debug": True, "output_dir": str(directory / "frames")}
            config = directory / "local.yaml"
            config.write_text(yaml.safe_dump(profile))
            command = [args.xvfb, "-a", str(args.client), "--config", str(config),
                       "--target", args.target, "--status-stdout", "--step"]
            if args.cancel_local:
                pid_path = directory / "client.pid"
                command[2:2] = ["sh", "-c", 'echo "$$" > "$1"; shift; exec "$@"',
                                "sh", str(pid_path)]
                log_path = directory / "local.log"
                with log_path.open("w") as output:
                    process = subprocess.Popen(command, cwd=directory, env=environment,
                                               stdin=subprocess.PIPE, stdout=output,
                                               stderr=subprocess.STDOUT, text=True)
                    try:
                        process.stdin.write("y\n" * 24)
                        process.stdin.close()
                        deadline = time.monotonic() + 100
                        while "grasp phase=descend_to_target" not in log_path.read_text():
                            if process.poll() is not None or time.monotonic() > deadline:
                                raise RuntimeError("local cancellation never reached the moving grasp stage")
                            time.sleep(0.02)
                        os.kill(int(pid_path.read_text()), signal.SIGINT)
                        process.wait(timeout=80)
                        output_text = log_path.read_text()
                        print(output_text)
                        if process.returncode or "Shutdown" not in output_text:
                            raise RuntimeError("local cancellation did not finish graceful shutdown")
                        if "grasp phase=close_gripper" in output_text:
                            raise RuntimeError("cancelled local grasp closed the gripper")
                    finally:
                        if process.poll() is None:
                            if pid_path.exists():
                                os.kill(int(pid_path.read_text()), signal.SIGKILL)
                            process.wait(timeout=10)
                return
            process = subprocess.run(command, cwd=directory, env=environment, input="y\n" * 24,
                                     capture_output=True, text=True, timeout=180)
            print(process.stdout)
            print(process.stderr)
            if process.returncode or "after={" not in process.stdout:
                raise RuntimeError("local RGB-D pipeline did not complete physical placement")
            capture = process.stdout.find("_grasp_pose.png")
            close = process.stdout.find("grasp phase=close_gripper")
            if capture < 0 or close < capture:
                raise RuntimeError("local step capture did not precede gripper closure")
            if not list((directory / "frames").rglob("*grasp_pose*")):
                raise RuntimeError("local step mode did not save the grasp-pose camera frame")
            return
        log_path = directory / "server.log"
        with log_path.open("w") as log:
            server = subprocess.Popen([
                args.xvfb, "-a", str(args.server), "--config",
                str(args.source / "config/grasp_pipeline_manipulation_mujoco.yaml"),
                "--listen", "127.0.0.1", "--port", str(port), "--viewer",
            ], stdout=log, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
            try:
                deadline = time.monotonic() + 30
                while True:
                    if "Listening on" in log_path.read_text():
                        break
                    if server.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError("simulation server did not become ready")
                    time.sleep(0.1)
                before_frame = frame(port)
                before, rgb_before, _, _ = before_frame
                save_image(args.artifacts, f"{args.target}_before.ppm", before_frame)
                command = [str(args.client), "--config",
                           str(args.source / "config/grasp_pipeline_manipulation_remote.yaml"),
                           "--target", args.target, "--remote-port", str(port), "--status-stdout"]
                if args.plan_only:
                    command.append("--plan-only")
                process = subprocess.run(command, cwd=directory, env=environment,
                                         capture_output=True, text=True, timeout=180)
                print(process.stdout)
                print(process.stderr)
                if process.returncode:
                    if args.artifacts:
                        try:
                            save_image(args.artifacts, f"{args.target}_failed.ppm", frame(port))
                        except (OSError, RuntimeError) as error:
                            print(f"failure frame unavailable: {error}")
                    raise RuntimeError(f"perception pipeline exited {process.returncode}")
                if args.target == "cup" and "Strategy selected: side" not in process.stdout:
                    raise RuntimeError("cup perception did not select the side-grasp strategy")
                after_frame = frame(port)
                after, rgb_after, _, _ = after_frame
                save_image(args.artifacts, f"{args.target}_after.ppm", after_frame)
                if after <= before:
                    raise RuntimeError("camera did not sample the advancing shared world")
                if not args.plan_only and rgb_after == rgb_before:
                    raise RuntimeError("grasp did not change the rendered scene")
                if not args.plan_only and "after={" not in log_path.read_text():
                    raise RuntimeError("pipeline did not execute physical placement")
            finally:
                try:
                    os.killpg(server.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    server.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
                print(log_path.read_text())


if __name__ == "__main__":
    main()
