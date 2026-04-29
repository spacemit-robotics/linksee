#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Signal-friendly launcher for the Linksee host process."""

import os
import signal
import sys

from lerobot.robots.linksee import linksee_host


def _raise_keyboard_interrupt(signum: int, _frame: object) -> None:
    del signum
    raise KeyboardInterrupt


def main() -> None:
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    sys.argv[0] = "lerobot.robots.linksee.linksee_host"
    linksee_host.main()
    os._exit(0)


if __name__ == "__main__":
    main()
