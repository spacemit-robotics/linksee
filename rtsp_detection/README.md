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

先准备本地 Vision SDK 依赖（默认路径）：

```bash
cp ../../include/vision_service.h third_party/vision/include/
cp /path/to/libvision.so third_party/vision/lib/
```

```bash
cd applications/rtsp_detection
cmake -S . -B build \
    -DOpenCV_DIR=/path/to/opencv4/cmake
cmake --build build -j
```

如需覆盖默认依赖路径：

```bash
cmake -S . -B build \
    -DOpenCV_DIR=/opt/opencv-spacemit/lib/cmake/opencv4 \
    -DVISION_INCLUDE_DIR=/path/to/vision/include \
    -DVISION_LIBRARY=/path/to/libvision.so
```

## 运行

```bash
./build/example_rtsp_detection \
    --device /dev/video12 \
    --config ./config/rtsp_detection.yaml \
    --width 1280 --height 720 --fps 30
```

或使用启动脚本（含 FFmpeg HLS 转封装）：

```bash
bash applications/rtsp_detection/scripts/start.sh
```

可选稳流参数（不降分辨率时建议）：

```bash
HLS_TIME=2 HLS_LIST_SIZE=5 \
FFMPEG_MAX_DELAY_US=500000 FFMPEG_RW_TIMEOUT_US=5000000 \
bash applications/rtsp_detection/scripts/start.sh
```

## 查看

- RTSP 直连：`ffplay rtsp://<board_ip>:8554/live`
- 浏览器：`http://<board_ip>:8080/`

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
| `--rtsp-url` | `rtsp://0.0.0.0:8554/live` | RTSP 输出地址 |
| `--http-port` | 8080 | HTTP API 端口 |
| `--config` | (必填) | VisionService YAML 配置 |
| `--model` | (可选) | 模型路径覆盖 |
| `--web-root` | `./web` | 前端静态文件目录 |

## 依赖

- MPP 库 (UVC, VDEC, VENC, MUX)
- `vision_service.h` + `libvision.so` (VisionService)
- [cpp-httplib](https://github.com/yhirose/cpp-httplib)（本目录已放置头文件）
- FFmpeg (HLS 转封装，可选)
