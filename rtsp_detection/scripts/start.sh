#!/bin/bash
# RTSP Detection Application - Startup Script
# Starts the main application and FFmpeg HLS transcoder

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"

# Configuration
DEVICE="${DEVICE:-/dev/video1}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
RTSP_URL="${RTSP_URL:-rtsp://0.0.0.0:18554/live}"
# Port for readiness check / ffmpeg (keep in sync with RTSP_URL)
RTSP_PORT="$(echo "$RTSP_URL" | sed -n 's#^rtsp://[^:]*:\([0-9][0-9]*\).*#\1#p')"
RTSP_PORT="${RTSP_PORT:-18554}"
RTSP_PLAY_URL="rtsp://127.0.0.1:${RTSP_PORT}/live"
HTTP_PORT="${HTTP_PORT:-18080}"
CONFIG="${CONFIG:-${APP_DIR}/config/rtsp_detection.yaml}"
WEB_ROOT="${WEB_ROOT:-${APP_DIR}/web}"
HLS_DIR="${HLS_DIR:-/tmp/hls}"
REPO_ROOT="$(cd "${APP_DIR}/../../../../" && pwd)"
if [ -z "${MPP_STAGING_DIR+x}" ]; then
    MPP_STAGING_DIR="${REPO_ROOT}/output/staging"
fi
# Plugin: explicit env > CMake POST_BUILD copy > staging default
if [ -z "${MPP_V4L2_LINLON_PLUGIN+x}" ]; then
    _local_plugin="${APP_DIR}/build/lib/libv4l2_linlonv5v7_codec2.so"
    if [ -f "$_local_plugin" ]; then
        MPP_V4L2_LINLON_PLUGIN="$_local_plugin"
    else
        MPP_V4L2_LINLON_PLUGIN="${MPP_STAGING_DIR}/lib/libv4l2_linlonv5v7_codec2.so"
    fi
fi
FFMPEG_LOGLEVEL="${FFMPEG_LOGLEVEL:-warning}"
FFMPEG_MAX_DELAY_US="${FFMPEG_MAX_DELAY_US:-500000}"
FFMPEG_RECONNECT_DELAY_MAX="${FFMPEG_RECONNECT_DELAY_MAX:-2}"
HLS_TIME="${HLS_TIME:-2}"
HLS_LIST_SIZE="${HLS_LIST_SIZE:-5}"

# Resolve executable (only explicit APP_BIN or APP_DIR/build; no PATH fallback)
_resolve_app_bin() {
    local _c
    if [ -n "${APP_BIN+x}" ] && [ -n "$APP_BIN" ]; then
        if [ -x "$APP_BIN" ]; then
            printf '%s' "$APP_BIN"
            return 0
        fi
        echo "[start.sh] WARN: APP_BIN is not executable: $APP_BIN" >&2
    fi
    _c="${APP_DIR}/build/example_rtsp_detection"
    if [ -x "$_c" ]; then
        printf '%s' "$_c"
        return 0
    fi
    return 1
}

if ! APP_BIN="$(_resolve_app_bin)"; then
    echo "[start.sh] ERROR: example_rtsp_detection not found." >&2
    echo "[start.sh]   Tried: ${APP_DIR}/build/example_rtsp_detection" >&2
    echo "[start.sh]   Build: cd ${APP_DIR} && mkdir -p build && cd build && cmake .. && make -j" >&2
    echo "[start.sh]   Or: APP_BIN=/path/to/example_rtsp_detection bash scripts/start.sh" >&2
    exit 1
fi

echo "=== RTSP Detection Startup ==="
echo "  App bin:   $APP_BIN"
echo "  Device:    $DEVICE"
echo "  Resolution: ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo "  RTSP URL:  $RTSP_URL"
echo "  HTTP port: $HTTP_PORT"
echo "  HLS dir:   $HLS_DIR"
echo "  MPP plugin: $MPP_V4L2_LINLON_PLUGIN"
echo "  HLS cfg:   time=${HLS_TIME}s list=${HLS_LIST_SIZE}"
echo "==============================="

# Create HLS output directory
mkdir -p "$HLS_DIR"

# Start main application
if [ -f "$MPP_V4L2_LINLON_PLUGIN" ]; then
    export MPP_V4L2_LINLON_PLUGIN
else
    echo "[start.sh] WARN: MPP plugin not found at $MPP_V4L2_LINLON_PLUGIN"
    echo "[start.sh] WARN: fallback to system plugin search path"
fi

# Vision YAML uses paths relative to app dir (e.g. assets/labels/coco.txt)
cd "$APP_DIR"
"$APP_BIN" \
    --device "$DEVICE" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --fps "$FPS" \
    --rtsp-url "$RTSP_URL" \
    --http-port "$HTTP_PORT" \
    --config "$CONFIG" \
    --web-root "$WEB_ROOT" \
    --hls-dir "$HLS_DIR" &
APP_PID=$!
FFMPEG_PID=""

echo "[start.sh] App started (PID=$APP_PID)"

# Wait for RTSP server to be ready (and ensure app has not exited)
wait_rtsp_ready() {
    local host="${1:-127.0.0.1}"
    local port="${2:-18554}"
    local timeout_s="${3:-15}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if ! kill -0 "$APP_PID" 2>/dev/null; then
            echo "[start.sh] ERROR: app exited before RTSP became ready"
            return 1
        fi
        if bash -c "exec 3<>/dev/tcp/${host}/${port}" 2>/dev/null; then
            echo "[start.sh] RTSP server is ready at ${host}:${port}"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "[start.sh] ERROR: RTSP server not ready within ${timeout_s}s"
    return 1
}

if ! wait_rtsp_ready 127.0.0.1 "$RTSP_PORT" 20; then
    if kill -0 "$APP_PID" 2>/dev/null; then
        echo "[start.sh] ERROR: RTSP server did not become ready, killing app (PID=$APP_PID)" >&2
        kill "$APP_PID" 2>/dev/null || true
    fi
    wait "$APP_PID" 2>/dev/null
    _app_rc=$?
    if [ "$_app_rc" -ne 0 ]; then
        echo "[start.sh] ERROR: app exit code: ${_app_rc}" >&2
    fi
    exit 1
fi

# Start FFmpeg HLS transcoder (no re-encoding, just repackaging)
BOARD_IP=$(hostname -I | awk '{print $1}')

ffmpeg -hide_banner -loglevel "$FFMPEG_LOGLEVEL" \
    -rtsp_transport tcp \
    -fflags nobuffer \
    -flags low_delay \
    -max_delay "$FFMPEG_MAX_DELAY_US" \
    -reorder_queue_size 0 \
    -i "$RTSP_PLAY_URL" \
    -c copy \
    -f hls \
    -hls_time "$HLS_TIME" \
    -hls_list_size "$HLS_LIST_SIZE" \
    -hls_flags delete_segments+append_list \
    "${HLS_DIR}/live.m3u8" &
FFMPEG_PID=$!

echo "[start.sh] FFmpeg HLS started (PID=$FFMPEG_PID)"
echo ""
echo "Access:"
echo "  RTSP:  ffplay rtsp://${BOARD_IP}:${RTSP_PORT}/live"
echo "  Web:   http://${BOARD_IP}:${HTTP_PORT}/"
echo "  HLS:   ${HLS_DIR}/live.m3u8"
echo ""
echo "Press Ctrl+C to stop"

# Cleanup on exit
cleanup() {
    echo ""
    echo "[start.sh] Stopping..."
    if [ -n "$FFMPEG_PID" ]; then
        kill "$FFMPEG_PID" 2>/dev/null || true
    fi
    kill "$APP_PID" 2>/dev/null || true
    if [ -n "$FFMPEG_PID" ]; then
        wait "$FFMPEG_PID" 2>/dev/null || true
    fi
    wait "$APP_PID" 2>/dev/null || true
    rm -rf "$HLS_DIR"
    echo "[start.sh] Done."
}

trap cleanup SIGINT SIGTERM

# Wait for either process to exit
wait -n "$APP_PID" "$FFMPEG_PID" 2>/dev/null || true
cleanup
