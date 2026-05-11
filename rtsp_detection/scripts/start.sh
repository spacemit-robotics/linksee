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
HTTP_PORT="${HTTP_PORT:-18080}"
CONFIG="${CONFIG:-${APP_DIR}/config/rtsp_detection.yaml}"
WEB_ROOT="${WEB_ROOT:-${APP_DIR}/web}"
HLS_DIR="${HLS_DIR:-/tmp/hls}"
MPP_V4L2_LINLON_PLUGIN="${MPP_V4L2_LINLON_PLUGIN:-${APP_DIR}/build/mpp/al/vcodec/libv4l2_linlonv5v7_codec.so}"
FFMPEG_LOGLEVEL="${FFMPEG_LOGLEVEL:-warning}"
FFMPEG_MAX_DELAY_US="${FFMPEG_MAX_DELAY_US:-500000}"
FFMPEG_RECONNECT_DELAY_MAX="${FFMPEG_RECONNECT_DELAY_MAX:-2}"
HLS_TIME="${HLS_TIME:-2}"
HLS_LIST_SIZE="${HLS_LIST_SIZE:-5}"

# Find the binary
APP_BIN="${APP_BIN:-example_rtsp_detection}"
if [ ! -x "$APP_BIN" ]; then
    APP_BIN="${APP_DIR}/build/example_rtsp_detection"
fi
if [ ! -x "$APP_BIN" ]; then
    APP_BIN="${APP_DIR}/../../build/bin/example_rtsp_detection"
fi

echo "=== RTSP Detection Startup ==="
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

"$APP_BIN" \
    --device "$DEVICE" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --fps "$FPS" \
    --rtsp-url "$RTSP_URL" \
    --http-port "$HTTP_PORT" \
    --config "$CONFIG" \
    --web-root "$WEB_ROOT" &
APP_PID=$!
FFMPEG_PID=""

echo "[start.sh] App started (PID=$APP_PID)"

# Wait for RTSP server to be ready (and ensure app has not exited)
wait_rtsp_ready() {
    local host="${1:-127.0.0.1}"
    local port="${2:-8554}"
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

wait_rtsp_ready 127.0.0.1 18554 20

# Start FFmpeg HLS transcoder (no re-encoding, just repackaging)
BOARD_IP=$(hostname -I | awk '{print $1}')
RTSP_PLAY_URL="rtsp://127.0.0.1:18554/live"

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
echo "  RTSP:  ffplay rtsp://${BOARD_IP}:18554/live"
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
