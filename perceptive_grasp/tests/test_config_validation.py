#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

"""Exercise runtime YAML validation without initializing hardware."""

import argparse
import subprocess
import tempfile
from pathlib import Path

import yaml


def run(binary, config):
    return subprocess.run(
        [binary, "--config", str(config), "--validate-config"],
        check=False,
        capture_output=True,
        text=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--config", action="append", required=True)
    args = parser.parse_args()

    for config in args.config:
        result = run(args.binary, config)
        if result.returncode != 0 or "[Config] valid" not in result.stdout:
            raise RuntimeError(
                f"valid config rejected: {config}\n{result.stdout}{result.stderr}"
            )

    source = Path(args.config[0])
    root = yaml.safe_load(source.read_text(encoding="utf-8"))
    root["manipulator"]["joint_limits"] = [[-1.0, 1.0]]
    with tempfile.TemporaryDirectory() as directory:
        invalid = Path(directory) / "invalid_joint_limits.yaml"
        invalid.write_text(yaml.safe_dump(root), encoding="utf-8")
        result = run(args.binary, invalid)
    if result.returncode == 0 or "must contain joint, min and max" not in result.stderr:
        raise RuntimeError(
            "invalid joint limit schema was accepted\n" +
            result.stdout + result.stderr
        )

    root = yaml.safe_load(source.read_text(encoding="utf-8"))
    root.setdefault("grasp", {}).setdefault("top", {})[
        "sparse_projected_center_blend"
    ] = 1.1
    with tempfile.TemporaryDirectory() as directory:
        invalid = Path(directory) / "invalid_sparse_center_blend.yaml"
        invalid.write_text(yaml.safe_dump(root), encoding="utf-8")
        result = run(args.binary, invalid)
    if (
        result.returncode == 0
        or "sparse_projected_center_blend must be in [0, 1]"
        not in result.stderr
    ):
        raise RuntimeError(
            "invalid sparse projected-center blend was accepted\n"
            + result.stdout
            + result.stderr
        )


if __name__ == "__main__":
    main()
