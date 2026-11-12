#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Capture public-runtime RGB-D with configuration-relative paths from an unrelated cwd."""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--xvfb", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="manipulation-camera-tool-") as temporary:
        directory = Path(temporary)
        environment = dict(os.environ, MANIPULATION_LOCK_DIRECTORY=str(directory / "locks"))
        environment.setdefault("LP_NUM_THREADS", "2")
        result = subprocess.run([
            args.xvfb, "-a", args.binary, "--config", args.config,
            "--frames", "1", "--warmup", "0", "--no-detect", "--output", "frames",
        ], cwd=directory, env=environment, capture_output=True, text=True, timeout=30)
        print(result.stdout)
        print(result.stderr)
        if result.returncode:
            raise RuntimeError("camera-only public runtime failed")
        for suffix in ("_color.png", "_depth.png"):
            images = list((directory / "frames").glob(f"*{suffix}"))
            if len(images) != 1 or images[0].stat().st_size < 100:
                raise RuntimeError(f"camera tool did not produce {suffix}")


if __name__ == "__main__":
    main()
