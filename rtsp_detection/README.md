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

## 依赖

- **MPP 库**：`components/multimedia/mpp`（UVC, VDEC, VENC, MUX, SYS）
- **VisionService**：
  - 头文件：`third_party/vision/include/vision_service.h`
  - 动态库：`third_party/vision/lib/libvision.so`
- **OpenCV**：默认 `/opt/opencv-spacemit/lib/cmake/opencv4`
- **cpp-httplib**：自动下载（或手动放到 `third_party/cpp_httplib/httplib.h`）
- **FFmpeg**（可选）：用于 HLS 转封装，网页播放需要

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

### 3. 编译 rtsp_detection

在 `application/ros2/linksee/rtsp_detection` 目录：

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

编辑 `config/rtsp_detection.yaml`，确保 `model_path` 指向实际模型文件：

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

在 `application/ros2/linksee/rtsp_detection` 目录：

```bash
# 使用启动脚本（推荐，含 FFmpeg HLS 转封装）
bash scripts/start.sh

# 或直接运行可执行文件
./build/example_rtsp_detection \
    --device /dev/video1 \
    --config ./config/rtsp_detection.yaml \
    --width 1280 --height 720 --fps 30
```

环境变量覆盖（可选）：

```bash
DEVICE=/dev/video1 \
WIDTH=1280 HEIGHT=720 FPS=30 \
bash scripts/start.sh
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

**解决**：按"编译"第 6 步配置 udev 规则，或临时用 `sudo` 运行。

### 2. `can not find v4l2_linlonv5v7 plugin`

**原因**：mpp codec 插件未安装到搜索路径（`/usr/lib/` 或 `~/.mpp/plugins/` 等）。

**解决**：若曾用 root 编译 mpp，先确认 `/usr/lib/libv4l2_linlonv5v7_codec2.so` 是否存在；否则按「编译」第 4 步拷贝 `libv4l2_linlonv5v7_codec2.so`，或用 root 重新编译 mpp 让自动安装脚本写入 `/usr/lib/`。

### 3. `VB_CreatePool: no free pool slot, max=16`

**原因**：上次进程残留在共享内存。

**解决**：

```bash
sudo pkill -9 -f example_rtsp_detection
sudo rm -f /dev/shm/tcm_sync_standalone /dev/shm/mpp_* /dev/shm/vb_*
```

### 4. 网页有画面但没有检测框

**原因**：`inference_fps: 0.0`，推理未运行。

**排查**：

```bash
# 查看状态
curl http://127.0.0.1:18080/api/status

# 确认模型文件存在
ls -lh ~/.cache/models/vision/yolov8/yolov8n.q.onnx
# 如果用 sudo 跑，检查 /root/.cache/... 或改 yaml 用绝对路径

# 查看启动日志里有没有 VisionService 错误
```

### 5. ffmpeg HLS 转码失败，网页无画面

**原因**：ffmpeg 版本不支持某些选项，或 RTSP 未就绪。

**解决**：start.sh 已适配 ffmpeg 8.0，如果还有问题，手动测试：

```bash
ffmpeg -rtsp_transport tcp -i rtsp://127.0.0.1:18554/live \
    -c copy -f hls -hls_time 2 -hls_list_size 5 \
    -hls_flags delete_segments+append_list /tmp/hls/live.m3u8
```

### 6. httplib.h 下载失败

**原因**：网络不通或 GitHub 访问受限。

**解决**：手动下载放到 `third_party/cpp_httplib/httplib.h`，或：

```bash
cmake .. -DHTTPLIB_LOCAL_PATH=/path/to/httplib.h
```
