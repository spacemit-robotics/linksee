#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise the real debug_grasp binary without opening cameras or robot devices."""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--strace", type=Path)
    parser.add_argument("--trace-dir", type=Path)
    args = parser.parse_args()
    binary = args.binary.resolve(strict=True)
    if args.strace and not args.trace_dir:
        parser.error("--strace requires --trace-dir")
    if args.trace_dir:
        args.trace_dir.mkdir(parents=True, exist_ok=True)

    cases = [
        (["--help"], 0, "never commands the arm", False),
        (["-h"], 0, "opens a RealSense camera", False),
        (["--execute"], 2, "--execute is not supported", False),
        (["--help", "--execute"], 2, "--execute is not supported", False),
        (["--execute", "--help"], 2, "--execute is not supported", False),
        (["--unknown", "--execute"], 2, "--execute is not supported", False),
        (["--config", "--execute"], 2, "--execute is not supported", False),
        (["--unknown"], 1, "unknown argument", False),
        (["--help", "--unknown"], 1, "unknown argument", False),
        (["--unknown", "--help"], 1, "unknown argument", False),
        (["positional"], 1, "unknown argument", False),
    ]
    for option in ("--config", "--target", "--output", "--warmup", "--frames"):
        cases.append(([option], 1, "missing value", False))
        cases.append(([option, "--help"], 1, "missing value", False))
        if option != "--target":
            cases.append(([option, ""], 1, "empty value", False))
    for option in ("--warmup", "--frames"):
        for value in ("-1", "abc", "1x", "1.5", " 1", "1 ", "+", "+-0",
                      "2147483648", "999999999999999999999"):
            cases.append(([option, value], 1, "requires a decimal integer", False))
    cases.append((["--frames", "0"], 1, "requires a decimal integer", False))
    # Legal options must still reach config loading. The absent config stops the
    # process before planner, VisionService or RealSense initialization.
    cases.extend([
        ([], 1, "Stage 1: load config", True),
        (["--warmup", "0", "--frames", "1", "--target", "",
          "--step", "--no-save"], 1, "Stage 1: load config", True),
        (["--warmup", "+1", "--frames", "2147483647", "--target", "apple"],
         1, "Stage 1: load config", True),
        (["--warmup", "0", "--frames", "1", "--help"],
         0, "never commands the arm", False),
    ])
    for index, (options, expected_code, expected_text, loads_config) in enumerate(cases):
        with tempfile.TemporaryDirectory(prefix="debug-grasp-cli-") as directory:
            root = Path(directory)
            output = root / "output"
            command = [str(binary), "--config", str(root / "absent.yaml"),
                       "--output", str(output), *options]
            trace = None
            if args.strace:
                trace = args.trace_dir / f"case-{index:02d}.trace"
                command = [str(args.strace), "-f", "-qq", "-o", str(trace),
                           "-e", "trace=%file,ioctl,%network", *command]
            result = subprocess.run(command, cwd=root, stdin=subprocess.DEVNULL,
                                    capture_output=True, text=True, timeout=15, check=False)
            log = result.stdout + result.stderr
            assert result.returncode == expected_code, (options, result.returncode, log)
            assert expected_text in log, (options, log)
            assert output.exists() == loads_config, (options, log)
            if not loads_config:
                assert "[debug_grasp] Stage" not in log, (options, log)
            assert "Stage 2:" not in log and "Stage 4:" not in log, (options, log)
            if trace:
                syscalls = trace.read_text()
                assert not re.search(r"/dev/(?:ttyUSB|ttyACM|ttyS|video|bus/usb|hidraw)"
                                     r"|AF_CAN|PF_CAN", syscalls), (options, trace)
    print(f"PASS: {len(cases)} real-binary CLI cases; no camera or arm initialization")


if __name__ == "__main__":
    main()
