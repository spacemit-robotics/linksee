#!/bin/bash
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${APP_DIR}/../../../.." && pwd)"
VENV_DIR="${REPO_ROOT}/output/envs/linksee_app"

source "${REPO_ROOT}/build/envsetup.sh"
m_env_build application/ros2/linksee/linksee_app

echo "[setup_env] Environment ready"
echo "[setup_env] Activate with: source ${VENV_DIR}/bin/activate"