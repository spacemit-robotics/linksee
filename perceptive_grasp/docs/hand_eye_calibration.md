# 手眼标定

本项目使用 Eye-to-Hand 标定：D435i 固定在机身上，ChArUco 标定板固定在夹爪末端。

## 1. 安装依赖

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/.venv-grasp/bin/activate
pip install -r requirements.txt
```

## 2. 标定板参数

采集命令中的参数必须和实际打印板一致：

- `--charuco-squares-x`：横向方格数量。
- `--charuco-squares-y`：纵向方格数量。
- `--charuco-square-length`：黑白方格边长，单位米。
- `--charuco-marker-length`：ArUco marker 边长，单位米。

## 3. 采集数据

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/.venv-grasp/bin/activate
python3 scripts/calibrate_hand_eye.py \
  --charuco-squares-x 4 \
  --charuco-squares-y 5 \
  --charuco-square-length 0.020 \
  --charuco-marker-length 0.014 \
  --num-poses 15 \
  --dataset-dir "config/hand_eye_datasets/handeye_$(date +%Y%m%d_%H%M%S)"
```

采集时让标定板覆盖左、中、右、高、低和不同腕部角度。每次移动机械臂后等待画面稳定再采集。

## 4. 写回配置

采集完成后执行：

```bash
python3 scripts/calibrate_hand_eye.py \
  --solve-only \
  --dataset-dir config/hand_eye_datasets/handeye_YYYYMMDD_HHMMSS \
  --apply-config config/grasp_pipeline.yaml
```

写回后运行抓取前检查：

```bash
python3 scripts/check_runtime_env.py --config config/grasp_pipeline.yaml --no-fix
```

相机位置或安装角度变化后，重新采集并写回配置。
