#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Report per-sample residual against distance and ChArUco corner count.

Residual is dominated by how many corners a pose actually saw, not by the detector: a
partially visible board still solves a pose, just a noisier one. Grouping the residuals by
corner count and distance shows which poses to replace before spending a capture session.
"""

import argparse
import json
from pathlib import Path


NEAR = (0.20, 0.30)
FAR = 0.45
# Accumulated residuals separate cleanly at 11 corners: below that the pose is solved from
# too small a patch of the board to be worth keeping.
RECOMMENDED_CORNERS = 11
WEAK_CORNERS = 9


def band_of(distance, near_upper=NEAR[1]):
    """Classify a pose by distance.

    The 0.45 m boundary is fixed because beyond it the whole board cannot fit in frame;
    only the near/mid split is adjustable, and only downwards.
    """
    if distance < NEAR[0]:
        return "too-near"
    if distance <= near_upper:
        return "near"
    if distance <= FAR:
        return "mid"
    return "far"


def near_bound(argument):
    """Accept a near upper bound no larger than the documented 0.30 m.

    Raising it would relabel ordinary mid-range poses as near and flatten the very
    comparison this report exists to show.
    """
    value = float(argument)
    if value <= NEAR[0] or value > NEAR[1]:
        raise argparse.ArgumentTypeError(
            f"近距上界须落在 ({NEAR[0]:.2f}, {NEAR[1]:.2f}] m，收到 {value:.2f}"
        )
    return value


def load_rows(directory):
    result = directory / "result.json"
    if not result.is_file():
        raise SystemExit(f"{result} 不存在，先对这个数据集跑一次 --solve-only")
    report = json.loads(result.read_text())
    errors = report.get("all_sample_errors_m_deg", [])
    rows = []
    for index, path in enumerate(sorted(directory.glob("sample_*.json"))):
        quality = json.loads(path.read_text())["quality"]
        error = errors[index] if index < len(errors) else [float("nan")] * 2
        rows.append(
            dict(
                index=index,
                distance_m=quality["distance_m"],
                corners=quality["num_charuco_corners"],
                marker_px=quality["marker_size_px"],
                reprojection_px=quality["reprojection_error_px"],
                position_mm=error[0] * 1000,
                angle_deg=error[1],
            )
        )
    return report, rows


def average(values):
    return sum(values) / len(values) if values else float("nan")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", default="output/hand_eye")
    parser.add_argument("--near-upper", type=near_bound, default=NEAR[1],
                        help=f"近距上界(m),默认 {NEAR[1]:.2f};可调小,不大于该值")
    args = parser.parse_args()

    directory = Path(args.dataset_dir)
    report, rows = load_rows(directory)
    if not rows:
        raise SystemExit(f"{directory} 下没有样本")

    limits = (NEAR[0], args.near_upper, FAR)
    labels = {
        "too-near": f"<{limits[0]:.2f} m",
        "near": f"{limits[0]:.2f}-{limits[1]:.2f} m",
        "mid": f"{limits[1]:.2f}-{limits[2]:.2f} m",
        "far": f">{limits[2]:.2f} m",
    }

    print(f"门槛 {report.get('thresholds')}  quality_passed={report.get('quality_passed')}")
    print(f"留出样本 {report.get('holdout_indices')}\n")

    print(f"{'#':>3} {'距离m':>7} {'角点':>5} {'marker_px':>9} {'重投影':>7} "
          f"{'位置mm':>7} {'角度°':>7}  区间")
    for row in sorted(rows, key=lambda item: item["distance_m"]):
        print(f"{row['index']:3d} {row['distance_m']:7.3f} {row['corners']:5d} "
              f"{row['marker_px']:9.1f} {row['reprojection_px']:7.3f} "
              f"{row['position_mm']:7.2f} {row['angle_deg']:7.2f}  "
              f"{band_of(row['distance_m'], args.near_upper)}")

    print()
    for name in ("too-near", "near", "mid", "far"):
        group = [row for row in rows if band_of(row["distance_m"], args.near_upper) == name]
        if not group:
            continue
        print(f"{labels[name]}: n={len(group)} 平均位置残差 "
              f"{average([r['position_mm'] for r in group]):.2f} mm")

    for label, predicate in (
        (f"角点<{WEAK_CORNERS}", lambda row: row["corners"] < WEAK_CORNERS),
        (f"角点{WEAK_CORNERS}-{RECOMMENDED_CORNERS - 1}",
         lambda row: WEAK_CORNERS <= row["corners"] < RECOMMENDED_CORNERS),
        (f"角点>={RECOMMENDED_CORNERS}",
         lambda row: row["corners"] >= RECOMMENDED_CORNERS),
    ):
        group = [row for row in rows if predicate(row)]
        if group:
            print(f"{label}: n={len(group)} 平均位置残差 "
                  f"{average([r['position_mm'] for r in group]):.2f} mm")

    weak = [row for row in rows if row["corners"] < RECOMMENDED_CORNERS]
    if weak:
        print()
        print(f"建议重摆这些姿态（角点少于 {RECOMMENDED_CORNERS}）：")
        for row in sorted(weak, key=lambda item: item["position_mm"], reverse=True):
            print(f"  #{row['index']} 角点 {row['corners']} 距离 {row['distance_m']:.3f} m "
                  f"残差 {row['position_mm']:.2f} mm")

    worst = max(rows, key=lambda item: item["position_mm"])
    if report.get("quality_passed") is not True:
        print()
        print(f"当前未通过质量门槛，最差样本 #{worst['index']} "
              f"{worst['position_mm']:.2f} mm / {worst['angle_deg']:.2f}°")


if __name__ == "__main__":
    main()
