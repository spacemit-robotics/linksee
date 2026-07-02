# 抓取方案说明

Perceptive Grasp 采用 RGB-D 顶抓流程：

```text
RealSense D435i -> YOLOv8-seg -> 深度定位 -> 顶抓规划 -> SO101 执行
```

## 当前方案

- 使用 RealSense D435i 获取 RGB 和 Depth。
- 使用 YOLOv8-seg 检测目标并生成 mask。
- 在 mask 内选择抓取像素并读取深度。
- 通过手眼标定把相机坐标转换到机械臂基座坐标。
- 使用 Pinocchio IK 求解 pre-grasp 和 grasp 位姿。
- 使用 SO101 和夹爪完成顶抓、抬起、放置，并回到观察姿态等待下一条命令。

## 部署定位

K3 端侧部署以稳定、可解释和可复盘为目标。当前工程只提供确定性的 RGB-D 顶抓链路，所有关键阶段都会输出日志和调试文件，便于定位检测、深度、标定、IK 和执行器问题。
