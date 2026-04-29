#!/bin/bash
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${APP_DIR}/../../../.." && pwd)"
VENV_DIR="${REPO_ROOT}/output/envs/linksee_app"
HOST_LOG="/tmp/linksee_host.log"
HOST_PID="/tmp/linksee_host.pid"

if [ ! -d "${VENV_DIR}" ]; then
  bash "${APP_DIR}/scripts/setup_env.sh"
fi

if [ -f "${HOST_PID}" ] && kill -0 "$(cat "${HOST_PID}")" 2>/dev/null; then
  echo "[linksee_host] already running with pid $(cat "${HOST_PID}")"
  exit 0
fi

source "${VENV_DIR}/bin/activate"
cd "${REPO_ROOT}"

nohup python "${APP_DIR}/scripts/run_host.py" \
  --robot.id=my_linksee \
  --robot.port=/dev/ttyACM0 \
  --robot.base_driver=drv_uart_esp32 \
  --robot.base_dev_path=/dev/ttyACM1 \
  --robot.base_baud=115200 \
  --robot.base_type=diff_2wd \
  --robot.base_left_wheel_gain=1.0 \
  --host.port_zmq_cmd=5565 \
  --host.port_zmq_observations=5566 \
  --host.connection_time_s=3600 \
  --robot.cameras='{
    "front": {"type":"opencv","index_or_path":"/dev/video15","width":640,"height":480,"fps":30,"fourcc":"MJPG"},
    "wrist": {"type":"opencv","index_or_path":"/dev/video13","width":640,"height":480,"fps":30,"fourcc":"MJPG"}
  }' \
  >"${HOST_LOG}" 2>&1 &

echo $! > "${HOST_PID}"
echo "[linksee_host] started pid $(cat "${HOST_PID}") log ${HOST_LOG}"
