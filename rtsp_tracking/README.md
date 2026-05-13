# RTSP Tracking

USB 摄像头实时目标追踪 + RTSP 推流应用。

## 功能

- USB 摄像头采集 (MJPEG) → 硬件解码 (NV12) → ByteTrack 追踪 → 硬件编码 (H.264) → RTSP 推流
- HTTP API 控制追踪开关、ROI 选中目标
- Web 前端页面（MJPEG 预览 + ROI 框选）
- 状态接口返回采集 FPS、推理 FPS、追踪数、RTSP 客户端数等

## 架构

```
UVC(MJPEG) → VDEC(NV12) → [ByteTrack + NV12画框] → VENC(H.264) → MUX(RTSP)
                  │
                  ├─ 异步推理线程: NV12→BGR → VisionService
                  └─ MJPEG 流: NV12→BGR→JPEG (HTTP /stream)
```

推理线程与主视频链路解耦，尽量不阻塞采集和推流。

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

### 4. 编译 rtsp_tracking

```bash
cd application/ros2/linksee/rtsp_tracking
mkdir -p build && cd build
cmake .. && make -j
```

## 运行

在 `application/ros2/linksee/rtsp_tracking` 目录执行：

```bash
./build/example_rtsp_tracking \
    --device /dev/video1 \
    --config ./config/rtsp_tracking.yaml \
    --width 1280 --height 720 --fps 30
```

**注意**：请根据实际摄像头设备修改 `--device` 参数。查看可用设备：

```bash
v4l2-ctl --list-devices
```

### 常用参数

- `--device`：摄像头节点（默认 `/dev/video0`）
- `--rtsp-url`：RTSP 输出地址（默认 `rtsp://0.0.0.0:18554/live`）
- `--http-port`：HTTP 端口（默认 `18080`）
- `--web-root`：Web 静态资源目录（默认 `./web`）
- `--model`：覆盖配置文件中的模型路径

## 访问

- **Web UI**：`http://<board_ip>:18080/`
- **MJPEG 流**：`http://<board_ip>:18080/stream`
- **RTSP 流**：`ffplay rtsp://<board_ip>:18554/live`

## HTTP API

### 1) 开始全目标追踪

```bash
curl -X POST http://127.0.0.1:18080/api/tracking/start
```

### 2) 停止追踪

```bash
curl -X POST http://127.0.0.1:18080/api/tracking/stop
```

### 3) ROI 选中单目标追踪

`x1/y1/x2/y2` 为 0~1 归一化坐标：

```bash
curl -X POST http://127.0.0.1:18080/api/tracking/select \
  -H "Content-Type: application/json" \
  -d '{"x1":0.20,"y1":0.20,"x2":0.50,"y2":0.60}'
```

### 4) 查询状态

```bash
curl http://127.0.0.1:18080/api/status
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
sudo pkill -9 -f example_rtsp_tracking
sudo rm -f /dev/shm/tcm_sync_standalone /dev/shm/mpp_* /dev/shm/vb_*
```

### 4. 找不到 `libvision.so`

**原因**：`third_party/vision/lib/libvision.so` 不存在或路径不对。

**解决**：检查文件是否存在，或使用 `-DVISION_LIBRARY=/path/to/libvision.so`。

### 5. 启动时报 `--config is required`

**原因**：运行时必须传 `--config`。

**解决**：

```bash
./build/example_rtsp_tracking --config ./config/rtsp_tracking.yaml
```

### 6. 配置里 `label_file_path` 路径不对

**原因**：`rtsp_tracking.yaml` 默认是 `assets/labels/coco.txt`。

**解决**：确保工作目录能解析到该路径，或改成绝对路径。

### 7. 没有视频输出

**原因**：摄像头节点错误、分辨率不支持、端口被占用。

**解决**：
- 确认摄像头节点：`v4l2-ctl --list-devices`
- 确认分辨率支持：`v4l2-ctl -d /dev/video1 --list-formats-ext`
- 确认端口未占用：`sudo lsof -i :18554` / `sudo lsof -i :18080`

### 8. httplib.h 下载失败

**原因**：网络不通或 GitHub 访问受限。

**解决**：手动下载 https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.15.3/httplib.h 并放到：

- `third_party/cpp_httplib/httplib.h`，或
- `build/_deps/cpp_httplib_header-src/httplib.h`

或使用本地文件：

```bash
cmake .. -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h
```
