#!/bin/bash
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${APP_DIR}/../../../.." && pwd)"
VENV_DIR="${REPO_ROOT}/output/envs/linksee_app"
INFER_LOG="/tmp/linksee_inference.log"
INFER_PID="/tmp/linksee_inference.pid"

if [ ! -d "${VENV_DIR}" ]; then
  bash "${APP_DIR}/scripts/setup_env.sh"
fi

if [ -f "${INFER_PID}" ] && kill -0 "$(cat "${INFER_PID}")" 2>/dev/null; then
  echo "[linksee_inference] already running with pid $(cat "${INFER_PID}")"
  exit 0
fi

source "${VENV_DIR}/bin/activate"
cd "${REPO_ROOT}"

nohup python "${APP_DIR}/scripts/evaluate.py" \
  >"${INFER_LOG}" 2>&1 &

echo $! > "${INFER_PID}"
echo "[linksee_inference] started pid $(cat "${INFER_PID}") log ${INFER_LOG}"