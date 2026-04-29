# RTSP Detection 设计与实现说明（对齐当前代码）

## 1. 概述

`applications/rtsp_detection` 实现了 USB 摄像头实时视频采集、可选 AI 检测叠加、RTSP 推流，并提供 HTTP 控制接口。

当前实现目标：

- 视频链路稳定优先，主路径不被推理阻塞
- AI 可运行时开关，默认关闭
- 独立目录构建和启动（不依赖根工程应用目标）

## 2. 当前架构

```text
UVC(MJPEG) -> VDEC(NV12) -> [可选: NV12画框] -> VENC(H.264) -> MUX(RTSP)
                            |
                            +-> 推理线程: NV12->BGR -> VisionService::InferImage
                                 (仅更新缓存结果，不阻塞主路径)

HTTP(cpp-httplib):
  POST /api/inference/enable
  POST /api/inference/disable
  GET  /api/status

可选 HLS:
  ffmpeg -i rtsp://127.0.0.1:8554/live -c copy -f hls ...
```

## 3. 与早期方案的关键差异

以下为当前代码行为（非计划态）：

1. 不使用 `SYS_Bind` 做 AI 开关切换  
   当前始终走手动主循环：`UVC -> VDEC -> VENC -> MUX`。
2. AI 开关只切 `ai_enabled` 原子变量  
   推理线程常驻，关闭后不再更新检测框。
3. 默认无检测框  
   需调用 `POST /api/inference/enable` 开启推理。
4. `status` 返回字段以代码为准  
   包含 `inference_enabled/capture_fps/inference_fps/detection_count/rtsp_clients/total_frames`。

## 4. 数据流细节

### 4.1 主线程（采集/编码/推流）

每轮处理流程：

1. `UVC_GetFrame` 获取 MJPEG 帧
2. `VDEC_SendStream` 送解码器
3. `VDEC_RecvFrame` 获取 NV12 帧
4. 若 AI 开启：
   - 尝试将最新帧转 BGR 投递给推理线程（非阻塞，线程忙则丢弃）
   - 使用缓存检测框在 NV12 上直接画框和文字
5. `VENC_SendFrame` 编码 H.264
6. `VENC_RecvStream` + `MUX_SendPacket` 推 RTSP

### 4.2 推理线程（异步旁路）

1. 等待最新 BGR 帧
2. 调用 `VisionService::InferImage`
3. 用互斥锁更新 `cached_results`
4. 更新 `infer_fps/detect_count`

策略：最新帧优先，允许丢帧，避免队列积压带来的时延扩散。

## 5. 线程与共享状态

线程模型：

- 采集线程：主视频链路（高频、实时）
- 推理线程：异步推理（可开关）
- HTTP 线程：控制接口与静态文件服务

关键共享状态：

- `ai_enabled`：推理开关
- `running`：程序生命周期
- `cached_results`：检测结果缓存（推理写、采集读）
- `capture_fps/infer_fps/detect_count/total_frames`：运行统计

## 6. HTTP API（当前实现）

### 6.1 接口

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/inference/enable` | 开启推理 |
| POST | `/api/inference/disable` | 关闭推理 |
| GET | `/api/status` | 查询状态 |

### 6.2 `/api/status` 示例

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

## 7. 构建与运行（独立目录）

### 7.1 编译

```bash
cd applications/rtsp_detection
cmake -S . -B build
cmake --build build -j
```

### 7.2 运行主程序

```bash
./build/example_rtsp_detection \
  --device /dev/video12 \
  --config ./config/rtsp_detection.yaml \
  --width 1280 --height 720 --fps 30 \
  --rtsp-url rtsp://0.0.0.0:8554/live \
  --http-port 8080 \
  --web-root ./web
```

### 7.3 启动脚本

```bash
bash scripts/start.sh
```

脚本会：

- 启动 `example_rtsp_detection`
- 启动 ffmpeg HLS 转封装
- 优先设置本地插件路径 `MPP_V4L2_LINLON_PLUGIN`（避免误用 `/usr/lib` 旧插件）

## 8. 调试与排障

### 8.1 有画面但没检测框

默认 AI 关闭，执行：

```bash
curl -X POST http://127.0.0.1:8080/api/inference/enable
```

### 8.2 `plugin must export al_dec_request_output_frame_2`

表示加载到了旧插件。检查：

- 是否设置了 `MPP_V4L2_LINLON_PLUGIN`
- 日志中插件路径是否指向 `applications/rtsp_detection/build/mpp/al/vcodec/libv4l2_linlonv5v7_codec.so`

### 8.3 `/dev/videoX is not a video capture device`

设备节点选错，需选择具备 `Video Capture` 能力的节点。

### 8.4 `cannot read CMA base_pfn ... Permission denied`

通常不是致命错误，只要 DMA heap 可用且链路可跑可先忽略。

## 9. 现状与后续优化

已实现：

- 异步推理解耦
- NV12 原地画框
- RTSP 推流 + HTTP 控制 + HLS 脚本

可继续优化：

- 推理结果增加 TTL，避免旧框残留
- 降低 `MIN` 宏重定义 warning
- 启动脚本改为健康检查替代固定 `sleep`
- 引入更完整的运行时指标（丢帧率、端到端时延）
