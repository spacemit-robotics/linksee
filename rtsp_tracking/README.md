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

## 依赖

- **MPP 库**：`components/multimedia/mpp`（UVC, VDEC, VENC, MUX, SYS）
- **VisionService**：
  - 头文件：`third_party/vision/include/vision_service.h`
  - 动态库：`third_party/vision/lib/libvision.so`
- **OpenCV**：默认 `/opt/opencv-spacemit/lib/cmake/opencv4`
- **cpp-httplib**：自动下载（或手动放到 `third_party/cpp_httplib/httplib.h`）

## 编译

### 1. 准备 VisionService 依赖

```bash
# 拷贝头文件和库到 third_party/vision/
mkdir -p third_party/vision/{include,lib}
cp /path/to/vision_service.h third_party/vision/include/
cp /path/to/libvision.so third_party/vision/lib/
```

### 2. 编译 mpp（首次需要）

```bash
cd ../../../../components/multimedia/mpp
mkdir -p build && cd build
cmake .. && make -j
```

**插件安装**：编译过程中会生成硬件编解码插件 `libv4l2_linlonv5v7_codec2.so`，并在链接后自动尝试安装：能写入 `/usr/lib/` 则拷到系统目录，否则落到当前用户的 `~/.mpp/plugins/`（见 `components/multimedia/mpp/al/vcodec/install_plugin.sh`）。**用 root 用户执行上述 `cmake` / `make`**（例如在 `mpp/build` 下 `sudo cmake .. && sudo make -j`）时，一般会直接装进 `/usr/lib/`，下面「第 4 步」通常可跳过；普通用户编译时请看构建日志里的 `Installed ... to ...` 路径。

### 3. 编译 rtsp_tracking

在 `application/ros2/linksee/rtsp_tracking` 目录：

```bash
mkdir -p build && cd build
cmake .. && make -j
```

如需覆盖默认路径：

```bash
cmake .. \
    -DOpenCV_DIR=/opt/opencv-spacemit/lib/cmake/opencv4 \
    -DVISION_INCLUDE_DIR=/path/to/vision/include \
    -DVISION_LIBRARY=/path/to/libvision.so
```

### 4. 安装 mpp codec plugin（按需）

若第 2 步已用 **root 编译 mpp**，且日志显示插件已安装到 `/usr/lib/`，可先确认：

```bash
ls /usr/lib/libv4l2_linlonv5v7_codec2.so
```

文件存在则**无需再拷贝**。否则把插件放到 mpp 的搜索路径之一：

```bash
# 插件 .so 位置：在 mpp 的 CMake 构建目录下相对路径为 al/vcodec/libv4l2_linlonv5v7_codec2.so
# （若把 mpp 作为子目录编进本应用，则在应用 build/mpp/al/vcodec/ 下。）
# 方法 1：用户目录（无需 sudo）。例：当前目录为 components/multimedia/mpp/build
mkdir -p ~/.mpp/plugins
cp al/vcodec/libv4l2_linlonv5v7_codec2.so ~/.mpp/plugins/

# 方法 2：系统目录（需 sudo，所有用户共用）
sudo cp al/vcodec/libv4l2_linlonv5v7_codec2.so /usr/lib/
```

### 5. 配置模型路径

编辑 `config/rtsp_tracking.yaml`，确保 `model_path` 指向实际模型文件：

```yaml
model_path: /home/user/.cache/models/vision/yolov8/yolov8n.q.onnx
```

**注意**：如果用 `sudo` 运行，`~` 会解析为 `/root`，建议用绝对路径。

### 6. 配置设备权限（推荐，避免 sudo）

```bash
sudo tee /etc/udev/rules.d/99-spacemit-mpp.rules > /dev/null <<'EOF'
KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
KERNEL=="dma_heap/*",  GROUP="video", MODE="0660"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
# 重新登录或 newgrp video
```

## 运行

在 `application/ros2/linksee/rtsp_tracking` 目录：

```bash
./build/example_rtsp_tracking \
    --device /dev/video1 \
    --config ./config/rtsp_tracking.yaml \
    --width 1280 --height 720 --fps 30
```

常用参数：

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

**解决**：按"编译"第 6 步配置 udev 规则，或临时用 `sudo` 运行。

### 2. `can not find v4l2_linlonv5v7 plugin`

**原因**：mpp codec 插件未安装到搜索路径（`/usr/lib/` 或 `~/.mpp/plugins/` 等）。

**解决**：若曾用 root 编译 mpp，先确认 `/usr/lib/libv4l2_linlonv5v7_codec2.so` 是否存在；否则按「编译」第 4 步拷贝 `libv4l2_linlonv5v7_codec2.so`，或用 root 重新编译 mpp 让自动安装脚本写入 `/usr/lib/`。

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

**解决**：手动下载放到 `third_party/cpp_httplib/httplib.h`，或：

```bash
cmake .. -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h
```
