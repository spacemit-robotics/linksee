#!/bin/bash
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

set -e

HOST_PID="/tmp/linksee_host.pid"
STOP_TIMEOUT_SEC=8

if [ ! -f "${HOST_PID}" ]; then
  echo "[linksee_host] not running"
  exit 0
fi

PID="$(cat "${HOST_PID}")"
if kill -0 "${PID}" 2>/dev/null; then
  kill -INT "${PID}"
  for _ in $(seq 1 "${STOP_TIMEOUT_SEC}"); do
    if ! kill -0 "${PID}" 2>/dev/null; then
      echo "[linksee_host] stopped pid ${PID}"
      rm -f "${HOST_PID}"
      exit 0
    fi
    sleep 1
  done

  echo "[linksee_host] SIGINT timeout, forcing stop pid ${PID}"
  kill -TERM "${PID}" 2>/dev/null || true
else
  echo "[linksee_host] stale pid ${PID}"
fi

rm -f "${HOST_PID}"
