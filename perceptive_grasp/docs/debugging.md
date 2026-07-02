# 调试指南

按下面顺序定位问题：

```text
check_runtime_env -> debug_view -> debug_localize -> debug_grasp -> read_joints -> debug_execute_safe
```

## 1. 运行前诊断

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh
source ~/.venv-grasp/bin/activate
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml
```

结果应为：

```text
[SUMMARY] ready
```

## 2. 相机取图

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp/build
source ~/spacemit_robot/build/envsetup.sh
./debug_view --no-detect --frames 3 --output /tmp/debug_camera
```

`/tmp/debug_camera` 下应生成 RGB 和 Depth 图片。

## 3. 目标定位

```bash
./debug_localize --config ../config/grasp_pipeline.yaml --target banana --frames 5
```

重点看输出中的 `base_point_m`。该点应落在机械臂前方工作空间内。

## 4. 抓取规划

```bash
./debug_grasp --config ../config/grasp_pipeline.yaml --target banana --output /tmp/debug_grasp
```

该命令默认 dry-run，不移动机械臂。重点检查：

- 标注图中 bbox、mask 和黄色抓取线是否覆盖目标。
- `pixel_grasp` 是否落在目标可夹取区域。
- `pre_grasp_m` 和 `grasp_m` 是否合理。
- `ik_pre_grasp` 和 `ik_grasp` 是否成功。

## 5. 读取关节角

```bash
./read_joints --device /dev/ttyACM0
```

输出包含 rad、deg 和可复制到 YAML 的关节数组。记录 `home_joints`、`observe_joints`
和 `place_joints` 时，先手动移动到目标姿态，再执行该命令。

## 6. 安全执行验证

确认 dry-run 正常后，再执行小范围真实运动：

```bash
./debug_execute_safe --config ../config/grasp_pipeline.yaml \
  --pose 0.28 0.00 0.10 \
  --yaw 90
```

执行前确认机械臂周围无障碍。

## 7. 调试产物

`config/grasp_pipeline.yaml` 中保持：

```yaml
debug:
  save_grasp_debug: true
  output_dir: "../debug_grasp_runs"
```

规划成功后保存：

- `grasp_YYYYMMDD_HHMMSS_mmm.png`
- `grasp_YYYYMMDD_HHMMSS_mmm.json`

任务结束后保存：

- `grasp_YYYYMMDD_HHMMSS_mmm_result.json`

失败时先查看 `*_result.json` 的 `terminal_state`、`message`、`last_executor_action`、
`last_executor_detail` 和 `last_executor_result`。
