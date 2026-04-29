#!/bin/bash
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

set -e

INFER_PID="/tmp/linksee_inference.pid"
STOP_TIMEOUT_SEC=8

if [ ! -f "${INFER_PID}" ]; then
  echo "[linksee_inference] not running"
  exit 0
fi

PID="$(cat "${INFER_PID}")"
if kill -0 "${PID}" 2>/dev/null; then
  kill -INT "${PID}"
  for _ in $(seq 1 "${STOP_TIMEOUT_SEC}"); do
    if ! kill -0 "${PID}" 2>/dev/null; then
      echo "[linksee_inference] stopped pid ${PID}"
      rm -f "${INFER_PID}"
      exit 0
    fi
    sleep 1
  done

  echo "[linksee_inference] SIGINT timeout, forcing stop pid ${PID}"
  kill -TERM "${PID}" 2>/dev/null || true
else
  echo "[linksee_inference] stale pid ${PID}"
fi

rm -f "${INFER_PID}"
