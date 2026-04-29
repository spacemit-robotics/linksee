# RTSP Tracking

USB 摄像头实时目标追踪 + RTSP 推流应用。

## 功能

- USB 摄像头采集 (MJPEG) -> 硬件解码 (NV12) -> ByteTrack 追踪 -> 硬件编码 (H.264) -> RTSP 推流
- HTTP API 控制追踪开关、ROI 选中目标
- Web 前端页面（MJPEG 预览 + ROI 框选）
- 状态接口返回采集 FPS、推理 FPS、追踪数、RTSP 客户端数等

## 架构

```text
UVC(MJPEG) -> VDEC(NV12) -> [ByteTrack + NV12画框] -> VENC(H.264) -> MUX(RTSP)
                  |
                  +-> NV12->BGR 推理线程
                  +-> NV12->BGR->JPEG (HTTP /stream)
```

推理线程与主视频链路解耦，尽量不阻塞采集和推流。

## 目录结构

```text
applications/rtsp_tracking/
├── cpp/
├── config/rtsp_tracking.yaml
├── web/index.html
└── third_party/vision/
    ├── include/vision_service.h
    └── lib/libvision.so
```

## 依赖

- MPP 库 (`mpp_uvc`, `mpp_vdec`, `mpp_venc`, `mpp_mux`, `mpp_sys`)
- VisionService：
  - 头文件：`third_party/vision/include/vision_service.h`
  - 动态库：`third_party/vision/lib/libvision.so`
- OpenCV（默认 `OpenCV_DIR=/opt/opencv-spacemit/lib/cmake/opencv4`）

## 编译

在 `applications/rtsp_tracking` 目录执行：

```bash
cd applications/rtsp_tracking
cmake -S . -B build \
    -DOpenCV_DIR=/opt/opencv-spacemit/lib/cmake/opencv4
cmake --build build -j
```

如果 `libvision.so` 不在默认位置：

```bash
cmake -S . -B build \
    -DOpenCV_DIR=/opt/opencv-spacemit/lib/cmake/opencv4 \
    -DVISION_INCLUDE_DIR=/path/to/vision/include \
    -DVISION_LIBRARY=/path/to/libvision.so
```

## 运行

在 `applications/rtsp_tracking` 目录：

```bash
./build/example_rtsp_tracking \
    --device /dev/video0 \
    --config ./config/rtsp_tracking.yaml \
    --width 1280 --height 720 --fps 30
```

常用参数：

- `--device`：摄像头节点（默认 `/dev/video0`）
- `--rtsp-url`：RTSP 输出地址（默认 `rtsp://0.0.0.0:8554/live`）
- `--http-port`：HTTP 端口（默认 `8080`）
- `--web-root`：Web 静态资源目录（默认 `./web`）
- `--model`：覆盖配置文件中的模型路径

## 访问地址

- Web UI：`http://<board_ip>:8080/`
- MJPEG：`http://<board_ip>:8080/stream`
- RTSP：`ffplay rtsp://<board_ip>:8554/live`

## HTTP API

### 1) 开始全目标追踪

```bash
curl -X POST http://127.0.0.1:8080/api/tracking/start
```

### 2) 停止追踪

```bash
curl -X POST http://127.0.0.1:8080/api/tracking/stop
```

### 3) ROI 选中单目标追踪

`x1/y1/x2/y2` 为 0~1 归一化坐标：

```bash
curl -X POST http://127.0.0.1:8080/api/tracking/select \
  -H "Content-Type: application/json" \
  -d '{"x1":0.20,"y1":0.20,"x2":0.50,"y2":0.60}'
```

### 4) 查询状态

```bash
curl http://127.0.0.1:8080/api/status
```

示例返回：

```json
{
  "mode": "track_single",
  "capture_fps": 30.0,
  "inference_fps": 12.5,
  "track_count": 3,
  "rtsp_clients": 1,
  "total_frames": 1500,
  "selected_track_id": 7,
  "track_lost": false
}
```

## 配置说明

默认配置文件：`config/rtsp_tracking.yaml`。

关键字段：

- `model_path`：模型路径（可被 `--model` 覆盖）
- `label_file_path`：标签路径
- `class`：后处理类（默认 `deploy.bytetrack.ByteTrackTracker`）
- `default_params`：阈值、track_buffer、providers 等

## 常见问题

1. **找不到 `libvision.so`**
   - 检查 `third_party/vision/lib/libvision.so` 是否存在，或使用 `-DVISION_LIBRARY=...`。

2. **启动时报 `--config is required`**
   - 运行时必须传 `--config`。

3. **配置里 `label_file_path` 路径不对**
   - `rtsp_tracking.yaml` 默认是 `assets/labels/coco.txt`。
   - 请确保工作目录能解析到该路径，或改成绝对路径/正确相对路径。

4. **没有视频输出**
   - 确认摄像头节点正确（`--device`），分辨率和帧率被设备支持。
   - 检查 RTSP 地址和端口是否被占用。

