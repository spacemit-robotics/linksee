# 手眼重采清单（目标：回到 5 mm / 2° 默认门槛）

## 现状

当前数据集（15 个样本）在放宽门槛 10 mm / 4° 下通过（`quality_passed: True`）；默认 5 mm / 2° 不通过，最差 8.05 mm / 3.40°。

已排除的原因：

- **外参无系统性偏差**：残差分解后三轴均值都在 1 mm 内，剩下全是随机噪声。
- **不是离群点**：逐个删最差样本重解，最差残差始终卡在 6.5–8 mm；删到第 4 个时样本数已低于 12 张下限。
- **求解稳定**：自助重采样 40 次全部通过 15 mm / 5°，同一数据换样本组合解出的外参平移标准差 3.3 mm。问题在单帧位姿质量。

## 根因

单帧位姿精度不够，集中在**远景 + 部分标定板**的组合上。

| 距离区间 | 样本数 | 平均位置残差 |
| --- | --- | --- |
| 0.20–0.30 m | 4 | 3.32 mm |
| 0.30–0.45 m | 6 | 4.67 mm |
| >0.45 m | 5 | 4.98 mm |

| 角点数 | 样本数 | 平均位置残差 |
| --- | --- | --- |
| 少于 9 | 3 | 6.58 mm |
| 9–10 | 3 | 3.60 mm |
| 11 及以上 | 9 | 3.97 mm |

角点 <9 的 3 个样本（#9 / #1 / #4）残差为 5.18 / 7.37 / 7.21 mm，明显拖后腿；但最差的单个样本是 #10（8.05 mm），它有 11 个角点、距离 0.364 m，属于角点够但姿态仍偏。因此**角点少是主因但不是唯一原因**，中距样本同样要挑姿态好的。

角点少也不总在远景：#9 距离只有 0.248 m 却只看到 6 个角点——近距下板可能超出画面边缘。摆位时近距也要留够边距，不能只靠"拉近"。

**重投影误差不是问题**：全部样本都低于 0.35 px，角点检测很准，缺的是角点数量。

## 重采目标

1. **近距样本占多数**：0.20–0.30 m 至少 9 个（当前仅 4 个），远景控制在 3 个以内。
2. **每帧角点不少于 11 个**：命令加 `--min-charuco-corners 11`，屏幕出现 `charuco corners N < 11` 提示就重摆。
3. **距离上界收到 0.45 m**：脚本默认 0.40 m，上次放宽到 0.60 m 以上（数据集最远样本 0.527 m），板在远端超出视野，这是角点少的主因。

## 采集命令

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
source ~/spacemit_robot/build/envsetup.sh

python3 scripts/calibrate_hand_eye.py \
  --config config/grasp_pipeline.yaml \
  --installation-id linksee_site \
  --charuco-squares-x 4 \
  --charuco-squares-y 5 \
  --charuco-square-length 0.0215 \
  --charuco-marker-length 0.015 \
  --min-charuco-corners 11 \
  --max-distance 0.45 \
  --num-poses 15 \
  --dataset-dir output/hand_eye_tight
```

板尺寸实测值就是 0.0215 / 0.015 m（见 `result.json` 的 `board` 字段），不用改；真正要改的只有 `--max-distance` 与 `--min-charuco-corners`。

`--min-charuco-corners` 目前只是**提示**，不会拒收样本——看到提示要自己重摆，采进去仍然拖低精度。

## 摆位要求

每个姿态都要满足：

- 整块板（4×5）完整在画面内，四个边都留余量；
- 板面正对相机，倾斜不超过约 30°；
- 相邻姿态至少改变两个关节。

15 个样本的覆盖分配：

| 位置 | 样本数 | 距离 |
| --- | --- | --- |
| 左下 / 中下 / 右下 | 3 | 0.22–0.28 m |
| 左中 / 中 / 右中 | 3 | 0.22–0.28 m |
| 左上 / 中上 / 右上 | 3 | 0.22–0.28 m |
| 腕部旋转 ±30° 的近距姿态 | 3 | 0.25–0.30 m |
| 中距补覆盖 | 3 | 0.35–0.45 m |

最后 3 个落在 0.30–0.45 m，仍要保证整块板进画面；板进不全就放弃这个姿态、把名额给近距。0.45 m 之外不再采——那个距离板必然进不全。

## 采完先自检

```bash
python3 scripts/calibrate_hand_eye.py \
  --solve-only \
  --installation-id linksee_site \
  --dataset-dir output/hand_eye_tight \
  --apply-config config/grasp_pipeline.yaml
```

不加门槛参数，直接用默认 5 mm / 2°：成功打印 `quality_passed: True`，失败给出实际最差残差与所用门槛。

若最差残差仍在 5–7 mm：

1. 先查是不是又有角点少的样本，用下面的脚本看每帧角点数与残差。
2. 角点都够但残差仍偏大时，检查 `T_tcp_board` 的平移模长。当前是 **63.6 mm**，应等于标定板安装面到法兰的实际距离，可用尺子核对；模长明显不符说明标定板固定有问题（松动、粘贴倾斜）。
3. 确认标定板尺寸用的是打印后实测值，不是设计值。

```bash
cd ~/spacemit_robot/application/ros2/linksee/perceptive_grasp
python3 scripts/report_hand_eye_samples.py --dataset-dir output/hand_eye_tight
```

输出按距离（<0.20 / 0.20–0.30 / 0.30–0.45 / >0.45 m）和角点数（<9 / 9–10 / ≥11）分组给出平均残差，并列出建议重摆的姿态。距离分组边界固定，只有近距上界可用 `--near-upper` 下调（如 `--near-upper 0.25`）；远景边界固定 0.45 m——超过这个距离整块板进不了画面，再细分没有意义。

## 回退

`--apply-config` 每次生成带时间戳的备份：

```bash
ls -1t config/grasp_pipeline.yaml.bak.* | head -3
cp config/grasp_pipeline.yaml.bak.<时间戳> config/grasp_pipeline.yaml
```

当前可用的外参（10 mm / 4° 门槛下）：

```
translation: [-0.025813, 0.043775, 0.304165]
rotation:    [-2.068931, 0.010174, -1.549350]
```

## 如果重采仍不达标

按代价从低到高：

1. **接受放宽门槛**：10 mm / 4° 在合同上限（15 mm / 5°）内，当前数据集已在该门槛下通过，抓取任务对绝对精度要求不高时够用。
2. **换更大的标定板**：角点不足的根源是板相对视野偏小。换 6×7 的板，同距离下角点数约翻倍，最直接。
3. **改用 LAS2 相机**：当前用 RealSense D435i（序列号 346222071954）。`camera.type` 切到 `spacemit_las2` 后重采；两者外参不通用，须重新标定。
