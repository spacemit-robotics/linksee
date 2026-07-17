#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Run perceptive_grasp while hiding third-party debug output."""

import os
import re
import signal
import subprocess
import sys
from typing import Sequence


STRUCTURED_PREFIXES = (
    "[Init]",
    "[Stage ",
    "[Action]",
    "[Timing]",
    "[Step]",
    "[Voice]",
    "VOICE_STATUS",
)


def should_keep_line(line: str) -> bool:
    """Return whether one stdout line belongs to the public task log."""
    text = line.rstrip("\r\n")
    if not text:
        return True
    if text.startswith("[Timing] stage=DETECTING") and \
            "result=NOT_FOUND" in text:
        return False
    if text.startswith(STRUCTURED_PREFIXES):
        return True
    if text.startswith("=== Perceptive Grasp Demo ==="):
        return True
    if text.startswith(("Config:", "Target:")):
        return True
    if text.startswith(("Usage:", "Options:", "Examples:", "  --", "  /")):
        return True
    if text.startswith("========== PIPELINE SUMMARY") or text.startswith(
            "======================================"):
        return True
    if text.startswith(("result=", "message=")):
        return True
    if re.match(r"^\s{2}\[\d{2}\]\s", text):
        return True
    if text == "[Pipeline] IDLE | Ready":
        return True
    if text.startswith("Pipeline initialization failed!"):
        return True
    return False


def _write_if_kept(line: bytes) -> None:
    text = line.decode("utf-8", errors="replace")
    status_offset = text.find("VOICE_STATUS\t")
    if status_offset >= 0:
        text = text[status_offset:]
        line = text.encode("utf-8")
    if should_keep_line(text):
        os.write(sys.stdout.fileno(), line)


def _default_binary() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(script_dir, "perceptive_grasp_core"),
        os.path.join(os.getcwd(), "perceptive_grasp_core"),
    )
    return next((path for path in candidates if os.path.isfile(path)),
                "./perceptive_grasp_core")


def _parse_command(argv: Sequence[str]) -> tuple[list[str], bool, bool]:
    args = list(argv)
    binary = _default_binary()
    if args[:1] == ["--wrapper-help"]:
        print("Usage: perceptive_grasp [--debug] [perceptive_grasp options]")
        print("Default mode: structured Pipeline logs only")
        print("--debug: show complete SDK and module logs")
        raise SystemExit(0)
    if args[:1] == ["--binary"]:
        if len(args) < 2:
            raise SystemExit("--binary requires a path")
        binary = args[1]
        args = args[2:]
    debug_mode = "--debug" in args
    args = [arg for arg in args if arg != "--debug"]
    return [binary, *args], "--step" in args, debug_mode


def main(argv: Sequence[str] | None = None) -> int:
    command, step_mode, debug_mode = _parse_command(
        sys.argv[1:] if argv is None else argv)
    try:
        if debug_mode:
            return subprocess.call(command)
        process = subprocess.Popen(
            command,
            stdin=None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
        )
    except OSError as exc:
        print(f"Failed to start {command[0]}: {exc}", file=sys.stderr)
        return 127

    def forward_signal(signum, _frame):
        if process.poll() is None:
            process.send_signal(signum)

    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, forward_signal)

    assert process.stdout is not None
    buffer = b""
    try:
        while True:
            chunk = os.read(process.stdout.fileno(), 4096)
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                _write_if_kept(line + b"\n")
            # Step-mode prompts intentionally have no trailing newline.
            if step_mode and buffer:
                os.write(sys.stdout.fileno(), buffer)
                buffer = b""
        if buffer:
            _write_if_kept(buffer)
        return process.wait()
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
