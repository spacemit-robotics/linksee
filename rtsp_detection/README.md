# RTSP Detection

USB 摄像头实时 AI 检测 + RTSP 推流应用。

## 功能

- USB 摄像头采集 (MJPEG) → 硬件解码 (NV12) → 硬件编码 (H.264) → RTSP 推流
- 可选 YOLOv8 目标检测，异步推理不阻塞视频流
- HTTP API 动态开关推理、查询状态
- Web 前端页面（HLS 播放 + 控制面板）

## 架构

```
UVC(MJPEG) → VDEC(NV12) → [NV12画框] → VENC(H.264) → MUX(RTSP)
                  │
                  └─ 异步推理线程: NV12→BGR → VisionService → 缓存检测结果
```

推理与编码解耦：主线程始终保持采集帧率，推理线程异步更新检测框。

## 编译

### 1. 环境准备

切换到 root 用户并加载环境：

```bash
su root
cd /path/to/spacemit_robot
source build/envsetup.sh
```

### 2. 编译 MPP

```bash
cd components/multimedia/mpp
mm
```

`mm` 命令会自动编译 mpp 并安装硬件编解码插件到 `/usr/lib/`。

### 3. 下载模型

```bash
cd components/model_zoo/vision/examples/yolov8
bash scripts/download_models.sh
```

模型会下载到 `~/.cache/models/vision/yolov8/yolov8n.q.onnx`（root 用户则是 `/root/.cache/...`）。

### 4. 编译 rtsp_detection

```bash
cd application/ros2/linksee/rtsp_detection
mkdir -p build && cd build
cmake .. && make -j
```

## 运行

**重要**：运行前请根据实际摄像头设备修改 `scripts/start.sh` 中的 `DEVICE` 变量（默认 `/dev/video1`）。

查看可用设备：

```bash
v4l2-ctl --list-devices
```

在 `application/ros2/linksee/rtsp_detection` 目录执行：

```bash
bash scripts/start.sh
```

脚本会自动启动应用并配置 FFmpeg HLS 转封装。

### 手动运行（不使用脚本）

```bash
./build/example_rtsp_detection \
    --device /dev/video1 \
    --config ./config/rtsp_detection.yaml \
    --width 1280 --height 720 --fps 30
```

## 访问

- **RTSP 直连**：`ffplay rtsp://<board_ip>:18554/live`
- **Web 界面**：`http://<board_ip>:18080/`
- **HLS 播放列表**：`http://<board_ip>:18080/stream/live.m3u8`

## HTTP API

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/inference/enable` | 开启 AI 推理 |
| POST | `/api/inference/disable` | 关闭 AI 推理 |
| GET | `/api/status` | 查询状态 |

状态响应示例：

```json
{
    "inference_enabled": true,
    "capture_fps": 30.0,
    "inference_fps": 12.5,
    "detection_count": 3,
    "rtsp_clients": 1,
    "total_frames": 1500
}
```

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--device` | `/dev/video0` | UVC 设备节点 |
| `--width` | 1280 | 采集宽度 |
| `--height` | 720 | 采集高度 |
| `--fps` | 30 | 帧率 |
| `--rtsp-url` | `rtsp://0.0.0.0:18554/live` | RTSP 输出地址 |
| `--http-port` | 18080 | HTTP API 端口 |
| `--config` | (必填) | VisionService YAML 配置 |
| `--model` | (可选) | 模型路径覆盖 |
| `--web-root` | `./web` | 前端静态文件目录 |
| `--hls-dir` | `/tmp/hls` | HLS 输出目录 |

## 常见问题

### 1. `Permission denied` 访问 `/dev/video*` 或 `/dev/dma_heap/*`

**原因**：当前用户不在 `video` 组。

**解决**：用 root 运行，或配置 udev 规则：

```bash
sudo tee /etc/udev/rules.d/99-spacemit-mpp.rules > /dev/null <<'EOF'
KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
KERNEL=="dma_heap/*",  GROUP="video", MODE="0660"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
# 重新登录或 newgrp video
```

### 2. `can not find v4l2_linlonv5v7 plugin`

**原因**：mpp codec 插件未安装。

**解决**：确认已用 root 执行 `mm` 编译 mpp，插件会自动安装到 `/usr/lib/libv4l2_linlonv5v7_codec2.so`。

### 3. `VB_CreatePool: no free pool slot, max=16`

**原因**：上次进程残留在共享内存。

**解决**：

```bash
sudo pkill -9 -f example_rtsp_detection
sudo rm -f /dev/shm/tcm_sync_standalone /dev/shm/mpp_* /dev/shm/vb_*
```

### 4. 网页有画面但没有检测框

**原因**：推理未运行（`inference_fps: 0.0`）。

**排查**：

```bash
# 查看状态
curl http://127.0.0.1:18080/api/status

# 确认模型文件存在（root 用户注意路径是 /root/.cache/...）
ls -lh ~/.cache/models/vision/yolov8/yolov8n.q.onnx

# 查看启动日志里的 VisionService 错误
```

### 5. httplib.h 下载失败

**原因**：网络不通或 GitHub 访问受限。

**解决**：手动下载 https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h 并放到：

- `third_party/cpp_httplib/httplib.h`，或
- `build/_deps/cpp_httplib_header-src/httplib.h`

或使用本地文件：

```bash
cmake .. -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h
```

### 6. 摄像头设备号不对

**原因**：`scripts/start.sh` 默认使用 `/dev/video1`。

**解决**：编辑 `scripts/start.sh`，修改 `DEVICE` 变量为实际设备节点。
