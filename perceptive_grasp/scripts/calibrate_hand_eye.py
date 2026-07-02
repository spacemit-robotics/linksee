#!/usr/bin/env python3
"""
手眼标定脚本 (Eye-to-Hand)

Linksee 结构:
  - 相机 (RealSense D435i) 固定在机身顶部，不随机械臂运动 → Eye-to-Hand 问题
  - 机械臂 (SO101 5-DOF) 在机身前方
  - ChArUco 标定板或 ArUco 标记固定在机械臂末端（夹爪上）

原理:
  Eye-to-Hand: 相机固定不动，标定板随机械臂运动。
  已知: 末端在基座系的位姿 T_base_gripper (FK 算出), 标定板在相机系的位姿 T_cam_marker (ChArUco/ArUco 检测)
  求解: T_base_camera (相机在基座系的位姿)
  约束: T_base_gripper[i] * T_gripper_marker = T_base_camera * T_cam_marker[i]

使用流程:
  1. 打印一个 ChArUco 标定板（推荐）或 ArUco 标记, 固定到夹爪末端
  2. 连接 D435i 和 SO101
  3. 手动掰动机械臂到不同姿态, 每次按回车采集一组数据
  4. 采集 >= 8 组后, 按 q 计算标定结果
  5. 将结果写入 grasp_pipeline.yaml

用法:
  # 基本使用 (K3 板端, 无 GUI)
  python3 calibrate_hand_eye.py

  # 自定义参数
  python3 calibrate_hand_eye.py --charuco-square-length 0.02 --num-poses 15

  # 指定串口
  python3 calibrate_hand_eye.py --device /dev/ttyACM1

  # 不连接机械臂, 手动输入关节角
  python3 calibrate_hand_eye.py --manual-joints

依赖:
  pip install -r requirements.txt
"""

import argparse
import shutil
import json
import os
import serial
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

# ============================================================
# 路径与常量
# ============================================================
SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH = SCRIPT_DIR / ".." / "urdf" / "so101.urdf"

SO101_JOINT_IDS = [1, 2, 3, 4, 5]
SO101_NUM_JOINTS = 5
URDF_DOF = 6          # URDF 有 6 个 DOF (含 gripper)
TIP_LINK = "gripper_frame_link"
DEFAULT_DATASET_ROOT = SCRIPT_DIR / ".." / "config" / "hand_eye_datasets"
ARUCO_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_50": cv2.aruco.DICT_6X6_50,
    "6x6_100": cv2.aruco.DICT_6X6_100,
}


# ============================================================
# 数据集与 SE(3) 工具
# ============================================================
def _json_dump(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _json_load(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _make_dataset_dir(root=None):
    root = Path(root) if root else DEFAULT_DATASET_ROOT
    root.mkdir(parents=True, exist_ok=True)
    stem = "handeye_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_dir = root / stem
    suffix = 1
    while dataset_dir.exists():
        dataset_dir = root / f"{stem}_{suffix:02d}"
        suffix += 1
    dataset_dir.mkdir(parents=True, exist_ok=False)
    (dataset_dir / "images").mkdir()
    (dataset_dir / "samples").mkdir()
    return dataset_dir


def _T_from_rt(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _invert_T(T):
    T = np.asarray(T, dtype=np.float64)
    inv = np.eye(4, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def _T_to_dict(T):
    return np.asarray(T, dtype=np.float64).tolist()


def _T_from_dict(value):
    T = np.asarray(value, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"expected 4x4 transform, got {T.shape}")
    return T


def _rotation_angle_deg(R_a, R_b):
    R = np.asarray(R_a) @ np.asarray(R_b).T
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(angle))


def _average_transforms(transforms):
    """Average a list of transforms with mean translation + quaternion mean."""
    if not transforms:
        return np.eye(4, dtype=np.float64)

    translations = np.array([T[:3, 3] for T in transforms], dtype=np.float64)
    rotations = Rotation.from_matrix([T[:3, :3] for T in transforms])
    try:
        R_mean = rotations.mean().as_matrix()
    except AttributeError:
        quats = rotations.as_quat()
        q0 = quats[0]
        for i in range(1, len(quats)):
            if np.dot(quats[i], q0) < 0:
                quats[i] = -quats[i]
        q = np.mean(quats, axis=0)
        q /= np.linalg.norm(q)
        R_mean = Rotation.from_quat(q).as_matrix()

    return _T_from_rt(R_mean, np.mean(translations, axis=0))


def _estimate_gripper_marker(T_base_camera, R_g2b, t_g2b, R_t2c, t_t2c):
    """Estimate constant T_gripper_marker for a fixed T_base_camera."""
    samples = []
    for i in range(len(R_g2b)):
        T_bg = _T_from_rt(R_g2b[i], t_g2b[i])
        T_cm = _T_from_rt(R_t2c[i], t_t2c[i])
        samples.append(_invert_T(T_bg) @ T_base_camera @ T_cm)
    return _average_transforms(samples)


def _compute_residuals(T_base_camera, T_gripper_marker,
                       R_g2b, t_g2b, R_t2c, t_t2c):
    rot_errs, trans_errs = [], []
    for i in range(len(R_g2b)):
        T_bg = _T_from_rt(R_g2b[i], t_g2b[i])
        T_cm = _T_from_rt(R_t2c[i], t_t2c[i])
        lhs = T_base_camera @ T_cm
        rhs = T_bg @ T_gripper_marker
        rot_errs.append(_rotation_angle_deg(lhs[:3, :3], rhs[:3, :3]))
        trans_errs.append(float(np.linalg.norm(lhs[:3, 3] - rhs[:3, 3]) * 1000.0))
    return rot_errs, trans_errs


def _save_sample(dataset_dir, idx, image, vis, joints, T_bg, T_cm, quality):
    image_rel = f"images/pose_{idx:03d}.png"
    vis_rel = f"images/pose_{idx:03d}_vis.png"
    cv2.imwrite(str(Path(dataset_dir) / image_rel), image)
    cv2.imwrite(str(Path(dataset_dir) / vis_rel), vis)

    sample = {
        "index": idx,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "image": image_rel,
        "visualization": vis_rel,
        "joints_rad": np.asarray(joints, dtype=np.float64).tolist(),
        "T_base_gripper": _T_to_dict(T_bg),
        "T_camera_marker": _T_to_dict(T_cm),
        "quality": quality,
    }
    _json_dump(Path(dataset_dir) / "samples" / f"pose_{idx:03d}.json", sample)
    return sample


def _load_dataset(dataset_dir):
    dataset_dir = Path(dataset_dir)
    manifest = _json_load(dataset_dir / "manifest.json")
    sample_files = sorted((dataset_dir / "samples").glob("pose_*.json"))
    if not sample_files:
        raise RuntimeError(f"数据集没有样本: {dataset_dir}")

    samples = [_json_load(p) for p in sample_files]
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for s in samples:
        T_bg = _T_from_dict(s["T_base_gripper"])
        T_cm = _T_from_dict(s["T_camera_marker"])
        R_g2b.append(T_bg[:3, :3])
        t_g2b.append(T_bg[:3, 3])
        R_t2c.append(T_cm[:3, :3])
        t_t2c.append(T_cm[:3, 3])
    return manifest, samples, R_g2b, t_g2b, R_t2c, t_t2c


def _apply_to_grasp_config(config_path, t_vec, rpy):
    """Patch only calibration.T_base_camera lines and preserve the rest."""
    config_path = Path(config_path)
    text = config_path.read_text(encoding="utf-8")
    backup = config_path.with_suffix(
        config_path.suffix + ".bak." + datetime.now().strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(config_path, backup)

    lines = text.splitlines()
    in_calibration = False
    in_tbc = False
    replaced_t = False
    replaced_r = False
    out_lines = []

    t_line = f"    translation: [{t_vec[0]:.6f}, {t_vec[1]:.6f}, {t_vec[2]:.6f}]"
    r_line = f"    rotation: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]"

    for line in lines:
        stripped = line.strip()
        if stripped == "calibration:" and not line.startswith(" "):
            in_calibration = True
            in_tbc = False
        elif in_calibration and line and not line.startswith(" "):
            in_calibration = False
            in_tbc = False

        if in_calibration and stripped == "T_base_camera:":
            in_tbc = True
        elif in_tbc and line.startswith("  ") and not line.startswith("    ") and stripped:
            in_tbc = False

        if in_tbc and stripped.startswith("translation:") and not stripped.startswith("#"):
            out_lines.append(t_line)
            replaced_t = True
            continue
        if in_tbc and stripped.startswith("rotation:") and not stripped.startswith("#"):
            out_lines.append(r_line)
            replaced_r = True
            continue
        out_lines.append(line)

    if not (replaced_t and replaced_r):
        out_lines.append("")
        out_lines.append("calibration:")
        out_lines.append("  T_base_camera:")
        out_lines.append(t_line)
        out_lines.append(r_line)

    config_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
    return backup


# ============================================================
# 相机
# ============================================================
def init_realsense(width=640, height=480, fps=30):
    """初始化 RealSense D435i, 返回 (pipeline, camera_matrix, dist_coeffs)"""
    if rs is None:
        raise RuntimeError("pyrealsense2 未安装: pip install -r requirements.txt")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)

    # 等待自动曝光稳定
    for _ in range(30):
        pipeline.wait_for_frames()

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1],
    ], dtype=np.float64)
    dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    return pipeline, camera_matrix, dist_coeffs


def capture_frame(pipeline):
    """采集一帧彩色图"""
    frames = pipeline.wait_for_frames()
    return np.asanyarray(frames.get_color_frame().get_data())


# ============================================================
# ArUco / ChArUco 检测
# ============================================================
def _get_aruco_dictionary(name_or_id="4x4_50"):
    if isinstance(name_or_id, int):
        return cv2.aruco.getPredefinedDictionary(name_or_id)
    key = str(name_or_id).lower().replace("dict_", "")
    if key not in ARUCO_DICTS:
        raise ValueError(f"Unsupported ArUco dictionary: {name_or_id}")
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[key])


def _create_charuco_board(squares_x, squares_y, square_length,
                          marker_length, dictionary_name="4x4_50"):
    aruco_dict = _get_aruco_dictionary(dictionary_name)
    size = (int(squares_x), int(squares_y))
    try:
        return cv2.aruco.CharucoBoard(size, square_length, marker_length, aruco_dict)
    except TypeError:
        return cv2.aruco.CharucoBoard_create(
            int(squares_x), int(squares_y), square_length, marker_length, aruco_dict)


def _draw_charuco_axes(vis, board, camera_matrix, dist_coeffs, rvec, tvec):
    try:
        cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
    except cv2.error:
        pass


def _format_detect_info(info):
    if not info:
        return "no diagnostic info"
    parts = []
    if info.get("failure_stage"):
        parts.append(f"stage={info['failure_stage']}")
    for key in (
        "num_markers", "num_charuco_corners", "valid_frames",
        "num_frames", "max_markers", "max_charuco_corners"):
        if info.get(key) is not None:
            parts.append(f"{key}={info[key]}")
    if info.get("board_type"):
        parts.append(f"board_type={info['board_type']}")
    return ", ".join(parts) if parts else "no diagnostic info"


def detect_charuco(image, camera_matrix, dist_coeffs,
                   squares_x=5, squares_y=7,
                   square_length=0.025, marker_length=0.018,
                   dictionary_name="4x4_50", return_info=False):
    """
    检测 ChArUco 标定板，返回板坐标系到相机系的 4x4 矩阵。
    ChArUco 比单 ArUco marker 有更多亚像素角点，姿态估计更稳定。
    """
    board = _create_charuco_board(
        squares_x, squares_y, square_length, marker_length, dictionary_name)
    aruco_dict = _get_aruco_dictionary(dictionary_name)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    vis = image.copy()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        marker_corners, marker_ids, _ = detector.detectMarkers(image)
    else:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            image, aruco_dict, parameters=params)

    empty_info = {
        "detected": False,
        "board_type": "charuco",
        "failure_stage": "detect_markers",
        "num_markers": 0,
        "num_charuco_corners": 0,
        "reprojection_error_px": None,
        "marker_size_px": None,
        "distance_m": None,
    }
    if marker_ids is None or len(marker_ids) == 0:
        return (None, vis, empty_info) if return_info else (None, vis)

    cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)
    ok, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, image, board, camera_matrix, dist_coeffs)
    if not ok or charuco_ids is None or len(charuco_ids) < 4:
        info = dict(empty_info)
        info["failure_stage"] = "interpolate_charuco_corners"
        info["num_markers"] = int(len(marker_ids))
        info["num_charuco_corners"] = int(len(charuco_ids)) if charuco_ids is not None else 0
        return (None, vis, info) if return_info else (None, vis)

    try:
        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
    except cv2.error:
        pass

    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)
    pose_ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs,
        rvec, tvec)
    if not pose_ok:
        info = dict(empty_info)
        info["failure_stage"] = "estimate_pose_charuco_board"
        info["num_markers"] = int(len(marker_ids))
        info["num_charuco_corners"] = int(len(charuco_ids))
        return (None, vis, info) if return_info else (None, vis)

    _draw_charuco_axes(vis, board, camera_matrix, dist_coeffs, rvec, tvec)

    R, _ = cv2.Rodrigues(rvec)
    T = _T_from_rt(R, tvec.flatten())

    # Reprojection error on detected ChArUco corners.
    if hasattr(board, "getChessboardCorners"):
        board_corners = board.getChessboardCorners()
    else:
        board_corners = board.chessboardCorners
    obj_points = np.asarray(
        [board_corners[int(i)] for i in charuco_ids.flatten()],
        dtype=np.float32)
    projected, _ = cv2.projectPoints(
        obj_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    detected = charuco_corners.reshape(-1, 2)
    reproj_error = float(np.mean(np.linalg.norm(projected - detected, axis=1)))

    side_lengths = []
    for c in marker_corners:
        c2 = c.reshape(4, 2)
        side_lengths.extend(
            [np.linalg.norm(c2[(i + 1) % 4] - c2[i]) for i in range(4)])

    info = {
        "detected": True,
        "board_type": "charuco",
        "failure_stage": None,
        "num_markers": int(len(marker_ids)),
        "num_charuco_corners": int(len(charuco_ids)),
        "reprojection_error_px": reproj_error,
        "marker_size_px": float(np.mean(side_lengths)) if side_lengths else None,
        "distance_m": float(np.linalg.norm(T[:3, 3])),
        "dictionary": dictionary_name,
        "squares_x": int(squares_x),
        "squares_y": int(squares_y),
        "square_length_m": float(square_length),
        "marker_length_m": float(marker_length),
    }
    return (T, vis, info) if return_info else (T, vis)


def detect_aruco(image, camera_matrix, dist_coeffs, marker_length,
                 dictionary=cv2.aruco.DICT_4X4_50, return_info=False):
    """
    检测 ArUco 标记, 返回标记在相机系的 4x4 齐次矩阵 T_cam_marker (或 None)
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
    params = cv2.aruco.DetectorParameters()
    # 提高角点精度
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(image)
    vis = image.copy()

    empty_info = {
        "detected": False,
        "num_markers": 0,
        "reprojection_error_px": None,
        "marker_size_px": None,
        "distance_m": None,
    }
    if ids is None or len(ids) == 0:
        return (None, vis, empty_info) if return_info else (None, vis)

    # OpenCV 4.7+ 移除了 cv2.aruco.estimatePoseSingleMarkers,
    # 用官方推荐的 solvePnP + SOLVEPNP_IPPE_SQUARE 等价替代。
    # 物点角点顺序必须与 detectMarkers 输出一致: 左上→右上→右下→左下 (顺时针)。
    half = marker_length / 2.0
    obj_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    rvecs, tvecs = [], []
    for c in corners:
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, c.reshape(4, 2), camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return (None, vis, empty_info) if return_info else (None, vis)
        rvecs.append(rvec)
        tvecs.append(tvec)

    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    for rvec, tvec in zip(rvecs, tvecs):
        cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

    R, _ = cv2.Rodrigues(rvecs[0].flatten())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvecs[0].flatten()

    # 质量指标: 重投影误差 + marker 在图像中的平均边长
    projected, _ = cv2.projectPoints(
        obj_points, rvecs[0], tvecs[0], camera_matrix, dist_coeffs)
    projected = projected.reshape(4, 2)
    detected = corners[0].reshape(4, 2)
    reproj_error = float(np.mean(np.linalg.norm(projected - detected, axis=1)))
    side_lengths = [
        np.linalg.norm(detected[(i + 1) % 4] - detected[i])
        for i in range(4)
    ]
    info = {
        "detected": True,
        "num_markers": int(len(ids)),
        "reprojection_error_px": reproj_error,
        "marker_size_px": float(np.mean(side_lengths)),
        "distance_m": float(np.linalg.norm(T[:3, 3])),
        "marker_id": int(ids[0][0]) if ids is not None else None,
    }
    return (T, vis, info) if return_info else (T, vis)


def detect_aruco_averaged(pipeline, camera_matrix, dist_coeffs, marker_length,
                          num_frames=10, dictionary=cv2.aruco.DICT_4X4_50,
                          return_info=False):
    """
    多帧平均 ArUco 检测, 提高位姿估计精度。
    返回 (T_cam_marker_avg, vis_image) 或 (None, vis_image)
    """
    rvecs_all, tvecs_all = [], []
    reproj_errors, marker_sizes = [], []
    vis = None
    image = None

    for _ in range(num_frames):
        img = capture_frame(pipeline)
        image = img
        T, v, info = detect_aruco(
            img, camera_matrix, dist_coeffs, marker_length, dictionary,
            return_info=True)
        if T is not None:
            rvec, _ = cv2.Rodrigues(T[:3, :3])
            rvecs_all.append(rvec.flatten())
            tvecs_all.append(T[:3, 3])
            if info["reprojection_error_px"] is not None:
                reproj_errors.append(info["reprojection_error_px"])
            if info["marker_size_px"] is not None:
                marker_sizes.append(info["marker_size_px"])
            vis = v
        time.sleep(0.05)

    if len(rvecs_all) < num_frames // 2:
        failed_vis = vis if vis is not None else image if image is not None else capture_frame(pipeline)
        info = {
            "detected": False,
            "valid_frames": len(rvecs_all),
            "num_frames": num_frames,
            "reprojection_error_px": None,
            "marker_size_px": None,
            "distance_m": None,
        }
        return (None, failed_vis, image, info) if return_info else (None, failed_vis)

    # 平均平移
    t_avg = np.mean(tvecs_all, axis=0)

    # 平均旋转 (用四元数平均)
    quats = [Rotation.from_rotvec(rv).as_quat() for rv in rvecs_all]
    # 确保四元数符号一致 (避免 q 和 -q 平均为 0)
    q0 = quats[0]
    for i in range(1, len(quats)):
        if np.dot(quats[i], q0) < 0:
            quats[i] = -quats[i]
    q_avg = np.mean(quats, axis=0)
    q_avg /= np.linalg.norm(q_avg)
    R_avg = Rotation.from_quat(q_avg).as_matrix()

    T_avg = np.eye(4)
    T_avg[:3, :3] = R_avg
    T_avg[:3, 3] = t_avg
    info = {
        "detected": True,
        "valid_frames": len(rvecs_all),
        "num_frames": num_frames,
        "reprojection_error_px": float(np.mean(reproj_errors)) if reproj_errors else None,
        "marker_size_px": float(np.mean(marker_sizes)) if marker_sizes else None,
        "distance_m": float(np.linalg.norm(T_avg[:3, 3])),
    }
    return (T_avg, vis, image, info) if return_info else (T_avg, vis)


def detect_board(image, camera_matrix, dist_coeffs, args, return_info=False):
    if args.board_type == "charuco":
        return detect_charuco(
            image, camera_matrix, dist_coeffs,
            squares_x=args.charuco_squares_x,
            squares_y=args.charuco_squares_y,
            square_length=args.charuco_square_length,
            marker_length=args.charuco_marker_length,
            dictionary_name=args.aruco_dict,
            return_info=return_info)

    dictionary = ARUCO_DICTS[args.aruco_dict]
    return detect_aruco(
        image, camera_matrix, dist_coeffs, args.marker_length,
        dictionary=dictionary, return_info=return_info)


def detect_board_averaged(pipeline, camera_matrix, dist_coeffs, args,
                          num_frames=10, return_info=False):
    """
    多帧平均标定板位姿。支持 ChArUco 和单 ArUco marker。
    """
    rvecs_all, tvecs_all = [], []
    reproj_errors, marker_sizes = [], []
    charuco_counts, marker_counts = [], []
    vis = None
    image = None
    last_info = None

    for _ in range(num_frames):
        img = capture_frame(pipeline)
        image = img
        T, v, info = detect_board(
            img, camera_matrix, dist_coeffs, args, return_info=True)
        last_info = info
        if info.get("num_markers") is not None:
            marker_counts.append(info["num_markers"])
        if info.get("num_charuco_corners") is not None:
            charuco_counts.append(info["num_charuco_corners"])
        if T is not None:
            rvec, _ = cv2.Rodrigues(T[:3, :3])
            rvecs_all.append(rvec.flatten())
            tvecs_all.append(T[:3, 3])
            if info.get("reprojection_error_px") is not None:
                reproj_errors.append(info["reprojection_error_px"])
            if info.get("marker_size_px") is not None:
                marker_sizes.append(info["marker_size_px"])
            vis = v
        time.sleep(0.05)

    if len(rvecs_all) < num_frames // 2:
        failed_vis = vis if vis is not None else image if image is not None else capture_frame(pipeline)
        info = {
            "detected": False,
            "board_type": args.board_type,
            "failure_stage": last_info.get("failure_stage") if last_info else None,
            "valid_frames": len(rvecs_all),
            "num_frames": num_frames,
            "num_markers": last_info.get("num_markers") if last_info else None,
            "num_charuco_corners": last_info.get("num_charuco_corners") if last_info else None,
            "max_markers": int(max(marker_counts)) if marker_counts else None,
            "max_charuco_corners": int(max(charuco_counts)) if charuco_counts else None,
            "reprojection_error_px": None,
            "marker_size_px": None,
            "distance_m": None,
        }
        return (None, failed_vis, image, info) if return_info else (None, failed_vis)

    t_avg = np.mean(tvecs_all, axis=0)
    quats = [Rotation.from_rotvec(rv).as_quat() for rv in rvecs_all]
    q0 = quats[0]
    for i in range(1, len(quats)):
        if np.dot(quats[i], q0) < 0:
            quats[i] = -quats[i]
    q_avg = np.mean(quats, axis=0)
    q_avg /= np.linalg.norm(q_avg)
    R_avg = Rotation.from_quat(q_avg).as_matrix()

    T_avg = _T_from_rt(R_avg, t_avg)
    info = {
        "detected": True,
        "board_type": args.board_type,
        "valid_frames": len(rvecs_all),
        "num_frames": num_frames,
        "reprojection_error_px": float(np.mean(reproj_errors)) if reproj_errors else None,
        "marker_size_px": float(np.mean(marker_sizes)) if marker_sizes else None,
        "distance_m": float(np.linalg.norm(T_avg[:3, 3])),
        "num_charuco_corners": (
            float(np.mean(charuco_counts)) if charuco_counts else None),
        "num_markers": float(np.mean(marker_counts)) if marker_counts else None,
    }
    return (T_avg, vis, image, info) if return_info else (T_avg, vis)


# ============================================================
# Pure Python FK (URDF parsing, no pinocchio dependency)
# ============================================================


def _rpy_to_matrix(rpy):
    """RPY (XYZ extrinsic) → 3x3 rotation matrix"""
    return Rotation.from_euler('xyz', rpy).as_matrix()


def _make_transform(xyz, rpy):
    """Create 4x4 homogeneous transform from xyz + rpy"""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _rot_z(angle):
    """Rotation about Z axis"""
    T = np.eye(4)
    c, s = np.cos(angle), np.sin(angle)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


class URDFForwardKinematics:
    """Minimal URDF FK solver for serial chains with revolute joints (axis=Z)"""

    def __init__(self, urdf_path, tip_link="gripper_frame_link"):
        tree = ET.parse(str(urdf_path))
        root = tree.getroot()

        # Parse all joints
        self.chain = []  # list of (origin_T, joint_type, joint_name)
        parent_map = {}  # child_link -> (joint_name, parent_link)
        joints_by_name = {}

        for j in root.findall('.//joint'):
            name = j.get('name')
            jtype = j.get('type')
            parent_el = j.find('parent')
            child_el = j.find('child')
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.get('link')
            child = child_el.get('link')
            if parent is None or child is None:
                continue
            origin = j.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0,0,0]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0,0,0]
            parent_map[child] = (name, parent)
            joints_by_name[name] = {
                'type': jtype, 'xyz': xyz, 'rpy': rpy,
                'parent': parent, 'child': child
            }

        # Build chain from base to tip
        # Find path: tip_link -> ... -> base_link
        path = []
        current = tip_link
        while current in parent_map:
            jname, parent = parent_map[current]
            path.append(jname)
            current = parent
        path.reverse()

        # Build kinematic chain
        self.joint_names = []
        self.transforms = []  # static origin transforms
        self.joint_types = []  # 'revolute' or 'fixed'

        for jname in path:
            info = joints_by_name[jname]
            T_origin = _make_transform(info['xyz'], info['rpy'])
            self.transforms.append(T_origin)
            self.joint_types.append(info['type'])
            self.joint_names.append(jname)

        # Revolute joint indices (for mapping q vector)
        self.revolute_indices = [i for i, t in enumerate(self.joint_types) if t == 'revolute']
        self.num_revolute = len(self.revolute_indices)

    def forward(self, q):
        """
        Compute FK. q is array of joint angles for all revolute joints in chain order.
        Returns 4x4 T_base_tip.
        """
        T = np.eye(4)
        q_idx = 0
        for i, (T_origin, jtype) in enumerate(zip(self.transforms, self.joint_types)):
            T = T @ T_origin
            if jtype == 'revolute':
                if q_idx < len(q):
                    T = T @ _rot_z(q[q_idx])
                q_idx += 1
        return T


def init_fk(urdf_path):
    """Initialize FK solver, returns URDFForwardKinematics instance"""
    fk = URDFForwardKinematics(urdf_path, TIP_LINK)
    print(f"  FK chain: {fk.num_revolute} revolute joints")
    print(f"  Joints: {[fk.joint_names[i] for i in fk.revolute_indices]}")
    return fk


def compute_fk(fk, joints_5dof):
    """5 joint angles → 4x4 T_base_gripper (6th joint=gripper set to 0)"""
    q = np.zeros(fk.num_revolute)
    q[:len(joints_5dof)] = joints_5dof
    return fk.forward(q)


# ============================================================
# SO101 关节读取 (纯 serial 协议, 绕过 libmanipulator.so)
# ============================================================

# STS3215 位置分辨率: 0-4095 对应 0-2π (motor rad)
STS3215_POS_MAX = 4095.0


def _sts_read_pos(ser, motor_id):
    """读取单个 STS3215 舵机的 Present Position (寄存器 56, 2 bytes)"""
    # 飞特协议: FF FF ID LEN INST PARAMS... CHECKSUM
    inst = 0x02  # READ
    start_addr = 56
    length = 2
    pkt_len = 4  # len field = inst(1) + addr(1) + len(1) + checksum(1)
    packet = bytearray([0xFF, 0xFF, motor_id, pkt_len, inst, start_addr, length])
    checksum = (~(motor_id + pkt_len + inst + start_addr + length)) & 0xFF
    packet.append(checksum)

    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    # 响应: FF FF ID LEN(4) ERR POS_L POS_H CHECKSUM
    resp = ser.read(8)
    if len(resp) < 8 or resp[0] != 0xFF or resp[1] != 0xFF:
        return None
    err = resp[4]
    if err != 0:
        return None
    return resp[5] | (resp[6] << 8)


def _sts_write_byte(ser, motor_id, addr, value):
    """写入单个字节到 STS3215 寄存器"""
    inst = 0x03  # WRITE
    pkt_len = 4
    packet = bytearray([0xFF, 0xFF, motor_id, pkt_len, inst, addr, value])
    checksum = (~(motor_id + pkt_len + inst + addr + value)) & 0xFF
    packet.append(checksum)
    ser.write(packet)
    ser.flush()
    ser.read(6)  # 读取响应
    time.sleep(0.002)


def _ticks_to_rad(ticks):
    """STS3215 ticks → joint rad (与 C++ motor_rad_to_joint_rad 一致)
    motor_rad = ticks / 4095.0 * 2π
    joint_rad = motor_rad - π  (中位 2048 ticks ≈ 0 rad)
    """
    motor_rad = (ticks / STS3215_POS_MAX) * (2.0 * np.pi)
    joint_rad = motor_rad - np.pi
    # 归一化到 [-π, π]
    while joint_rad < -np.pi:
        joint_rad += 2.0 * np.pi
    while joint_rad > np.pi:
        joint_rad -= 2.0 * np.pi
    return joint_rad


def init_so101(device="/dev/ttyACM0", baudrate=1000000):
    """
    通过纯串口连接 SO101, 关闭扭矩进入示教模式
    返回 serial.Serial 对象或 None
    """
    try:
        ser = serial.Serial(device, baudrate, timeout=0.1)
        time.sleep(0.1)
    except Exception as e:
        print(f"  [WARN] 无法打开串口 {device}: {e}")
        return None

    # 关闭所有舵机扭矩 (REG_TORQUE_ENABLE=40, value=0)
    for mid in SO101_JOINT_IDS:
        _sts_write_byte(ser, mid, 40, 0)

    # 验证: 读取一次位置确认通信正常
    test_pos = _sts_read_pos(ser, SO101_JOINT_IDS[0])
    if test_pos is None:
        print("  [WARN] 无法读取舵机位置, 请检查连接")
        ser.close()
        return None

    print("  示教模式已开启 (扭矩关闭, 可自由掰动)")
    return ser


def read_joints(ser):
    """从 SO101 读取 5 个关节角 (rad)"""
    joints = []
    for mid in SO101_JOINT_IDS:
        ticks = _sts_read_pos(ser, mid)
        if ticks is None:
            return None
        joints.append(_ticks_to_rad(ticks))
    return np.array(joints)


def close_so101(ser):
    """关闭串口 (不恢复扭矩, 标定后手动操作)"""
    if ser and ser.is_open:
        ser.close()


# ============================================================
# 标定求解
# ============================================================
def solve_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c):
    """
    用多种方法求解 Eye-to-Hand, 返回:
      {method_name: {"T_base_camera": T_bc, "T_gripper_marker": T_gm, ...}}

    约束方程: T_base_camera @ T_cam_marker = T_base_gripper @ T_gripper_marker
    """
    n = len(R_g2b)
    results = {}

    # ===== 方法组 1: calibrateHandEye (Eye-to-Hand) =====
    # 传入 inv(T_base_gripper) 和 T_cam_marker
    R_b2g_list, t_b2g_list = [], []
    for i in range(n):
        R = np.array(R_g2b[i])
        t = np.array(t_g2b[i]).flatten()
        R_inv = R.T
        t_inv = -R_inv @ t
        R_b2g_list.append(R_inv)
        t_b2g_list.append(t_inv.reshape(3, 1))

    R_t2c_list = [np.array(r) for r in R_t2c]
    t_t2c_list = [np.array(t).reshape(3, 1) for t in t_t2c]

    he_methods = {
        "TSAI":       cv2.CALIB_HAND_EYE_TSAI,
        "PARK":       cv2.CALIB_HAND_EYE_PARK,
        "HORAUD":     cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF":    cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    for name, method in he_methods.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_b2g_list, t_b2g_list, R_t2c_list, t_t2c_list, method=method)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t.flatten()
            T_gm = _estimate_gripper_marker(
                T, R_g2b, t_g2b, R_t2c, t_t2c)
            results[name] = {
                "T_base_camera": T,
                "T_gripper_marker": T_gm,
                "solver": "cv2.calibrateHandEye",
            }
        except cv2.error as e:
            print(f"  [WARN] {name} 失败: {e}")

    # ===== 方法组 2: 非线性优化 (同时估计 T_base_cam 和 T_gripper_marker) =====
    try:
        opt = _optimize_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c)
        if opt is not None:
            results["OPTIM"] = opt
    except Exception as e:
        print(f"  [WARN] OPTIM 失败: {e}")

    return results


def _optimize_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c):
    """
    非线性优化: 同时求解 T_base_camera 和 T_gripper_marker
    约束: T_base_cam @ T_cam_marker[i] = T_base_gripper[i] @ T_gripper_marker
    最小化: sum || T_bc @ T_cm[i] - T_bg[i] @ T_gm ||^2 (位置+旋转)
    """
    from scipy.optimize import least_squares

    n = len(R_g2b)

    # 初始估计: 用 PARK 方法的结果作为 T_bc 初值, T_gm 设为小平移
    R_b2g_list, t_b2g_list = [], []
    for i in range(n):
        R = np.array(R_g2b[i])
        t = np.array(t_g2b[i]).flatten()
        R_b2g_list.append(R.T)
        t_b2g_list.append((-R.T @ t).reshape(3, 1))
    R_t2c_list = [np.array(r) for r in R_t2c]
    t_t2c_list = [np.array(t).reshape(3, 1) for t in t_t2c]

    try:
        R_init, t_init = cv2.calibrateHandEye(
            R_b2g_list, t_b2g_list, R_t2c_list, t_t2c_list,
            method=cv2.CALIB_HAND_EYE_PARK)
    except cv2.error:
        R_init = np.eye(3)
        t_init = np.zeros((3, 1))

    T_init = _T_from_rt(R_init, t_init.flatten())
    T_gm_init = _estimate_gripper_marker(T_init, R_g2b, t_g2b, R_t2c, t_t2c)

    # 参数化: T_bc 和 T_gm 都用 rodrigues(3) + translation(3)
    rvec_bc, _ = cv2.Rodrigues(R_init)
    rvec_gm, _ = cv2.Rodrigues(T_gm_init[:3, :3])
    x0 = np.zeros(12)
    x0[0:3] = rvec_bc.flatten()
    x0[3:6] = t_init.flatten()
    x0[6:9] = rvec_gm.flatten()
    x0[9:12] = T_gm_init[:3, 3]

    def residuals(x):
        rvec_bc = x[0:3]
        t_bc = x[3:6]
        rvec_gm = x[6:9]
        t_gm = x[9:12]

        R_bc, _ = cv2.Rodrigues(rvec_bc.reshape(3, 1))
        T_bc = np.eye(4)
        T_bc[:3, :3] = R_bc
        T_bc[:3, 3] = t_bc
        R_gm, _ = cv2.Rodrigues(rvec_gm.reshape(3, 1))
        T_gm = np.eye(4)
        T_gm[:3, :3] = R_gm
        T_gm[:3, 3] = t_gm

        res = []
        for i in range(n):
            T_bg = np.eye(4)
            T_bg[:3, :3] = R_g2b[i]
            T_bg[:3, 3] = np.asarray(t_g2b[i]).flatten()
            T_cm = np.eye(4)
            T_cm[:3, :3] = R_t2c[i]
            T_cm[:3, 3] = np.asarray(t_t2c[i]).flatten()

            # 左: T_bc @ T_cm (标记在基座系的位姿, 通过相机)
            lhs = T_bc @ T_cm
            # 右: T_bg @ T_gm (标记在基座系的位姿, 通过FK)
            rhs = T_bg @ T_gm

            # 位置残差按 5mm 归一化，旋转残差按 2° 归一化。
            res.extend(((lhs[:3, 3] - rhs[:3, 3]) / 0.005).tolist())
            R_diff = lhs[:3, :3] @ rhs[:3, :3].T
            rvec_diff, _ = cv2.Rodrigues(R_diff)
            res.extend((rvec_diff.flatten() / np.deg2rad(2.0)).tolist())

        return np.array(res)

    result = least_squares(
        residuals, x0, method='trf', loss='soft_l1',
        f_scale=1.0, max_nfev=2000)

    R_bc, _ = cv2.Rodrigues(result.x[0:3].reshape(3, 1))
    T_bc = np.eye(4)
    T_bc[:3, :3] = R_bc
    T_bc[:3, 3] = result.x[3:6]
    R_gm, _ = cv2.Rodrigues(result.x[6:9].reshape(3, 1))
    T_gm = np.eye(4)
    T_gm[:3, :3] = R_gm
    T_gm[:3, 3] = result.x[9:12]

    t_gm = T_gm[:3, 3]
    print(f"  [OPTIM] T_gripper_marker 偏移: [{t_gm[0]:.4f}, {t_gm[1]:.4f}, {t_gm[2]:.4f}] m")
    print(f"  [OPTIM] 残差 cost: {result.cost:.6f}")

    return {
        "T_base_camera": T_bc,
        "T_gripper_marker": T_gm,
        "solver": "scipy.least_squares",
        "cost": float(result.cost),
        "success": bool(result.success),
        "message": result.message,
    }


def evaluate(results, R_g2b, t_g2b, R_t2c, t_t2c):
    """
    评估各方法残差, 返回 (最佳方法名, 评估详情)。
    所有方法统一比较:
      T_base_camera @ T_camera_marker[i]
        vs
      T_base_gripper[i] @ T_gripper_marker
    """
    n = len(R_g2b)
    print(f"\n  评估 ({n} 组数据):")
    print(f"  {'方法':<12} {'旋转均值(°)':<13} {'旋转最大(°)':<13} "
          f"{'平移均值(mm)':<14} {'平移最大(mm)':<14}")
    print(f"  {'-' * 72}")

    best, best_score, best_rot = None, float('inf'), float('inf')
    details = {}
    for name, result in results.items():
        T_bc = result["T_base_camera"]
        T_gm = result.get("T_gripper_marker")
        if T_gm is None:
            T_gm = _estimate_gripper_marker(
                T_bc, R_g2b, t_g2b, R_t2c, t_t2c)
            result["T_gripper_marker"] = T_gm

        rot_errs, trans_errs = _compute_residuals(
            T_bc, T_gm, R_g2b, t_g2b, R_t2c, t_t2c)
        mr = float(np.mean(rot_errs))
        xr = float(np.max(rot_errs))
        mt = float(np.mean(trans_errs))
        xt = float(np.max(trans_errs))
        score = mt + 5.0 * mr + 0.25 * xt + 2.0 * xr
        if mt > 80.0 or mr > 15.0:
            score += 1e6

        details[name] = {
            "rotation_error_deg_mean": mr,
            "rotation_error_deg_max": xr,
            "translation_error_mm_mean": mt,
            "translation_error_mm_max": xt,
            "score": score,
            "per_sample_rotation_error_deg": rot_errs,
            "per_sample_translation_error_mm": trans_errs,
        }
        print(f"  {name:<12} {mr:<13.2f} {xr:<13.2f} {mt:<14.2f} {xt:<14.2f}")

        if score < best_score:
            best_score = score
            best = name
            best_rot = mr

    # 质量警告: 旋转误差偏高 = 标定姿态旋转多样性不足 (通常是腕滚转 joint5 没转)
    if best_rot > 5.0:
        print(f"\n  ⚠ 警告: 最佳方法旋转误差 {best_rot:.1f}° 偏高 (理想 <2°)。")
        print("    多半是采集时腕部 (joint4/joint5) 姿态变化太小, 标记朝向不够多样。")
        print("    建议重新采集: 每组大幅转动腕滚转, 让标记正面/侧面/俯仰都覆盖到。")
    return best, details


def check_capture_quality(T_bg, T_cm, info, accepted_T_bg, args):
    """Return (ok, quality dict)."""
    warnings = []
    reject_reasons = []

    distance = info.get("marker_distance_m", info.get("distance_m"))
    reproj = info.get("reprojection_error_px")
    marker_size = info.get("marker_size_px")
    detected_frames = info.get("detected_frames", info.get("valid_frames", 0))
    num_frames = info.get("num_frames", 0)

    if distance is not None:
        if distance < args.min_distance:
            reject_reasons.append(
                f"marker too close: {distance:.3f}m < {args.min_distance:.3f}m")
        if distance > args.max_distance:
            reject_reasons.append(
                f"marker too far: {distance:.3f}m > {args.max_distance:.3f}m")
    if reproj is not None and reproj > args.max_reproj_error:
        reject_reasons.append(
            f"reprojection error too high: {reproj:.2f}px > {args.max_reproj_error:.2f}px")
    if marker_size is not None and marker_size < args.min_marker_size_px:
        reject_reasons.append(
            f"marker too small: {marker_size:.1f}px < {args.min_marker_size_px:.1f}px")
    if num_frames and detected_frames < max(3, num_frames // 2):
        reject_reasons.append(
            f"unstable detection: {detected_frames}/{num_frames} frames")

    max_relative_rot = None
    if accepted_T_bg:
        rel_rots = [
            _rotation_angle_deg(T_bg[:3, :3], prev[:3, :3])
            for prev in accepted_T_bg
        ]
        max_relative_rot = max(rel_rots)
        if max_relative_rot < args.min_relative_rotation_deg:
            reject_reasons.append(
                f"pose not informative: max Δrot {max_relative_rot:.1f}° "
                f"< {args.min_relative_rotation_deg:.1f}°")

    if len(accepted_T_bg) + 1 < args.min_poses:
        warnings.append(
            f"only {len(accepted_T_bg) + 1}/{args.min_poses} poses accepted")

    quality = {
        "marker_distance_m": distance,
        "reprojection_error_px": reproj,
        "marker_size_px": marker_size,
        "detected_frames": detected_frames,
        "num_frames": num_frames,
        "max_relative_rotation_deg": max_relative_rot,
        "warnings": warnings,
        "reject_reasons": reject_reasons,
    }
    return len(reject_reasons) == 0 or args.no_quality_gate, quality


def save_result(dataset_dir, results, best, eval_details, count,
                marker_length, cam_mtx):
    dataset_dir = Path(dataset_dir) if dataset_dir else None
    T_best = results[best]["T_base_camera"]
    T_gm_best = results[best]["T_gripper_marker"]
    t_vec = T_best[:3, 3]
    rpy = Rotation.from_matrix(T_best[:3, :3]).as_euler('xyz')

    all_res = {}
    for name, result in results.items():
        T = result["T_base_camera"]
        T_gm = result["T_gripper_marker"]
        t = T[:3, 3]
        r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
        t_gm = T_gm[:3, 3]
        r_gm = Rotation.from_matrix(T_gm[:3, :3]).as_euler('xyz')
        all_res[name] = {
            "translation": t.tolist(),
            "rotation_rpy": r.tolist(),
            "matrix_4x4": _T_to_dict(T),
            "T_gripper_marker": {
                "translation": t_gm.tolist(),
                "rotation_rpy": r_gm.tolist(),
                "matrix_4x4": _T_to_dict(T_gm),
            },
            "solver": result.get("solver"),
            "cost": result.get("cost"),
            "evaluation": eval_details.get(name, {}),
        }

    out = {
        "best_method": best,
        "T_base_camera": {
            "translation": t_vec.tolist(),
            "rotation": rpy.tolist(),
            "matrix_4x4": _T_to_dict(T_best),
        },
        "T_gripper_marker": {
            "translation": T_gm_best[:3, 3].tolist(),
            "rotation": Rotation.from_matrix(T_gm_best[:3, :3]).as_euler('xyz').tolist(),
            "matrix_4x4": _T_to_dict(T_gm_best),
        },
        "all_methods": all_res,
        "num_poses": count,
        "marker_length_m": marker_length,
        "camera_matrix": cam_mtx.tolist() if cam_mtx is not None else None,
    }

    if dataset_dir:
        _json_dump(dataset_dir / "result.json", out)
        lines = [
            f"best_method: {best}",
            f"T_base_camera.translation: [{t_vec[0]:.6f}, {t_vec[1]:.6f}, {t_vec[2]:.6f}]",
            f"T_base_camera.rotation: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]",
            "",
            "method residuals:",
        ]
        for name, d in eval_details.items():
            lines.append(
                f"  {name}: rot_mean={d['rotation_error_deg_mean']:.2f}deg, "
                f"trans_mean={d['translation_error_mm_mean']:.2f}mm")
        (dataset_dir / "report.txt").write_text("\n".join(lines) + "\n",
                                                 encoding="utf-8")
    return out


# ============================================================
# 主流程
# ============================================================
def main():
    ap = argparse.ArgumentParser(
        description="手眼标定 (Eye-to-Hand): 相机固定, ChArUco/ArUco 贴在夹爪上",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 calibrate_hand_eye.py                           # 默认参数
  python3 calibrate_hand_eye.py --charuco-square-length 0.02 --num-poses 15
  python3 calibrate_hand_eye.py --device /dev/ttyACM1     # 指定串口
  python3 calibrate_hand_eye.py --manual-joints            # 手动输入关节角
""")
    ap.add_argument("--board-type", choices=["charuco", "aruco"], default="charuco",
                    help="标定板类型: charuco(推荐) 或 aruco(兼容旧单码), 默认 charuco")
    ap.add_argument("--aruco-dict", type=str, default="4x4_50",
                    choices=sorted(ARUCO_DICTS.keys()),
                    help="ArUco 字典, 默认 4x4_50")
    ap.add_argument("--marker-length", type=float, default=0.037,
                    help="单 ArUco 模式的标记边长 (米), 默认 0.037")
    ap.add_argument("--charuco-squares-x", type=int, default=5,
                    help="ChArUco 横向棋盘格数量, 默认 5")
    ap.add_argument("--charuco-squares-y", type=int, default=7,
                    help="ChArUco 纵向棋盘格数量, 默认 7")
    ap.add_argument("--charuco-square-length", type=float, default=0.025,
                    help="ChArUco 单个棋盘格边长 (米), 默认 0.025")
    ap.add_argument("--charuco-marker-length", type=float, default=0.018,
                    help="ChArUco 内部 ArUco marker 边长 (米), 默认 0.018")
    ap.add_argument("--num-poses", type=int, default=12,
                    help="采集姿态数, 默认 12 (最少 3, 建议 >=10)")
    ap.add_argument("--device", type=str, default="/dev/ttyACM0")
    ap.add_argument("--baudrate", type=int, default=1000000)
    ap.add_argument("--manual-joints", action="store_true",
                    help="手动输入关节角 (不连接机械臂)")
    ap.add_argument("--output", type=str, default=None,
                    help="输出路径, 默认 config/hand_eye_result.json")
    ap.add_argument("--save-images", type=str, default=None,
                    help="兼容旧参数: 额外保存采集图像的目录")
    ap.add_argument("--dataset-dir", type=str, default=None,
                    help="数据集目录。采集模式下可指定输出目录；--solve-only 下表示输入目录")
    ap.add_argument("--solve-only", action="store_true",
                    help="只从 --dataset-dir 读取已有数据集并重新求解")
    ap.add_argument("--apply-config", type=str, default=None,
                    help="将最佳 T_base_camera 写回指定 grasp_pipeline.yaml (会自动备份)")
    ap.add_argument("--min-poses", type=int, default=10,
                    help="正式求解建议的最少有效姿态数, 默认 10")
    ap.add_argument("--min-relative-rotation-deg", type=float, default=15.0,
                    help="新姿态与历史姿态至少需要形成的最大相对旋转, 默认 15°")
    ap.add_argument("--max-reproj-error", type=float, default=2.0,
                    help="单次标定板平均重投影误差上限(px), 默认 2.0")
    ap.add_argument("--min-marker-size-px", type=float, default=40.0,
                    help="marker 图像平均边长下限(px), 默认 40")
    ap.add_argument("--min-distance", type=float, default=0.10,
                    help="marker 距相机最小距离(m), 默认 0.10")
    ap.add_argument("--max-distance", type=float, default=0.40,
                    help="marker 距相机最大距离(m), 默认 0.40")
    ap.add_argument("--no-quality-gate", action="store_true",
                    help="只提示质量问题但不拒收样本")
    args = ap.parse_args()

    if args.output is None:
        args.output = str(SCRIPT_DIR / ".." / "config" / "hand_eye_result.json")

    if args.solve_only:
        if not args.dataset_dir:
            sys.exit("错误: --solve-only 需要指定 --dataset-dir")
        dataset_dir = Path(args.dataset_dir)
        manifest, samples, R_g2b, t_g2b, R_t2c, t_t2c = _load_dataset(dataset_dir)
        count = len(samples)
        if count < 3:
            sys.exit(f"错误: 至少需要 3 组, 当前 {count} 组")
        if count < args.min_poses:
            print(f"  [WARN] 当前只有 {count} 组, 建议至少 {args.min_poses} 组")

        print("=" * 60)
        print("  手眼标定离线求解")
        print(f"  数据集: {dataset_dir}")
        print(f"  样本数: {count}")
        print("=" * 60)

        results = solve_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c)
        if not results:
            sys.exit("错误: 所有方法都失败")
        best, eval_details = evaluate(results, R_g2b, t_g2b, R_t2c, t_t2c)
        cam_mtx = np.asarray(manifest.get("camera_matrix"), dtype=np.float64) \
            if manifest.get("camera_matrix") is not None else None
        marker_length = manifest.get(
            "marker_length_m",
            manifest.get("charuco", {}).get("marker_length_m", args.marker_length))
        out = save_result(dataset_dir, results, best, eval_details, count,
                          marker_length, cam_mtx)
        _json_dump(args.output, out)

        t_vec = np.asarray(out["T_base_camera"]["translation"])
        rpy = np.asarray(out["T_base_camera"]["rotation"])
        print(f"\n  ★ 最佳方法: {best}")
        print(f"    translation: [{t_vec[0]:.4f}, {t_vec[1]:.4f}, {t_vec[2]:.4f}]")
        print(f"    rotation:    [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")
        print(f"  结果已保存: {args.output}")
        print(f"  数据集结果: {dataset_dir / 'result.json'}")
        if args.apply_config:
            backup = _apply_to_grasp_config(args.apply_config, t_vec, rpy)
            print(f"  已写回配置: {args.apply_config}")
            print(f"  备份文件: {backup}")
        return

    print("=" * 60)
    print("  手眼标定 (Eye-to-Hand)")
    print(f"  相机: D435i (固定)  标定板: {args.board_type} (贴在夹爪)")
    print("=" * 60)
    if args.board_type == "charuco":
        print(f"  ChArUco: {args.charuco_squares_x}x{args.charuco_squares_y}, "
              f"square={args.charuco_square_length * 100:.1f}cm, "
              f"marker={args.charuco_marker_length * 100:.1f}cm")
    else:
        print(f"  ArUco: dict={args.aruco_dict}, marker={args.marker_length * 100:.1f}cm")
    print(f"  目标采集: {args.num_poses} 组")
    print()

    # ---- 1. FK solver ----
    print("[1/4] 加载 URDF ...")
    if not URDF_PATH.exists():
        sys.exit(f"  错误: URDF 不存在: {URDF_PATH}")
    fk_solver = init_fk(URDF_PATH)
    print(f"  OK: {URDF_PATH.name}")

    # ---- 2. 机械臂 ----
    arm = None
    if not args.manual_joints:
        print("\n[2/4] 连接 SO101 ...")
        arm = init_so101(args.device, args.baudrate)
        if arm:
            test = read_joints(arm)
            if test is not None:
                print(f"  OK: 当前关节角 = [{', '.join(f'{j:.3f}' for j in test)}]")
            else:
                print("  WARN: 读取失败, 退回手动模式")
                close_so101(arm)
                arm = None
        else:
            print("  WARN: 无法连接, 退回手动模式")
    else:
        print("\n[2/4] 手动输入模式")

    # ---- 3. 相机 ----
    print("\n[3/4] 初始化 D435i ...")
    try:
        pipeline, cam_mtx, cam_dist = init_realsense()
        print(f"  OK: fx={cam_mtx[0,0]:.1f}, fy={cam_mtx[1,1]:.1f}")
    except Exception as e:
        if arm:
            close_so101(arm)
        sys.exit(f"  错误: {e}")

    if args.save_images:
        os.makedirs(args.save_images, exist_ok=True)

    dataset_dir = Path(args.dataset_dir) if args.dataset_dir else _make_dataset_dir()
    dataset_dir.mkdir(parents=True, exist_ok=True)
    (dataset_dir / "images").mkdir(exist_ok=True)
    (dataset_dir / "samples").mkdir(exist_ok=True)
    if list((dataset_dir / "samples").glob("pose_*.json")):
        sys.exit(f"错误: 数据集目录已有样本, 为避免混入旧数据请换一个目录: {dataset_dir}")
    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "mode": "eye_to_hand_marker_on_gripper",
        "board_type": args.board_type,
        "aruco_dict": args.aruco_dict,
        "marker_type": args.board_type,
        "marker_length_m": args.marker_length,
        "charuco": {
            "squares_x": args.charuco_squares_x,
            "squares_y": args.charuco_squares_y,
            "square_length_m": args.charuco_square_length,
            "marker_length_m": args.charuco_marker_length,
        },
        "camera": {
            "model": "RealSense D435i",
            "width": 640,
            "height": 480,
            "fps": 30,
        },
        "camera_matrix": cam_mtx.tolist(),
        "dist_coeffs": cam_dist.tolist(),
        "urdf_path": str(URDF_PATH),
        "tip_link": TIP_LINK,
        "quality_thresholds": {
            "min_poses": args.min_poses,
            "min_relative_rotation_deg": args.min_relative_rotation_deg,
            "max_reproj_error_px": args.max_reproj_error,
            "min_marker_size_px": args.min_marker_size_px,
            "min_distance_m": args.min_distance,
            "max_distance_m": args.max_distance,
        },
        "samples": [],
    }
    _json_dump(dataset_dir / "manifest.json", manifest)
    print(f"\n  数据集目录: {dataset_dir}")

    # ---- 4. 采集 ----
    print("\n[4/4] 开始采集")
    print("-" * 60)
    print("操作步骤:")
    print("  1. 确认 ChArUco 标定板/ArUco 标记刚性固定在夹爪末端")
    print("  2. 掰动机械臂到新姿态 (标定板要对着相机)")
    print("  3. 按回车采集 | 输入 s 预览 | 输入 q 结束")
    print()
    print("★ 标定质量要求:")
    print("  - 标定板距相机 15~30cm (图像中 marker ≥40px)")
    print("  - 标定板尽量在图像中心区域")
    print("  - 每次至少变化 2 个关节, 旋转 >20°")
    print("  - 覆盖: 左/中/右 × 高/低 × 不同腕部角度")
    print("  - 标定板正面朝向相机 (倾斜 <45°)")
    print("-" * 60)

    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    count = 0

    while count < args.num_poses:
        if arm:
            prompt = f"[{count+1}/{args.num_poses}] 掰好后按回车"
        else:
            prompt = f"[{count+1}/{args.num_poses}] 输入 5 个关节角(rad, 空格分隔)"

        user = input(f"\n{prompt}: ").strip()

        if user.lower() == 'q':
            break

        if user.lower() == 's':
            img = capture_frame(pipeline)
            T_cm, vis, info = detect_board(
                img, cam_mtx, cam_dist, args, return_info=True)
            path = "/tmp/calibrate_preview.png"
            cv2.imwrite(path, vis)
            if T_cm is not None:
                print(f"  ✓ 检测到标定板  位置: [{T_cm[0,3]:.4f}, {T_cm[1,3]:.4f}, {T_cm[2,3]:.4f}]")
                if info.get("reprojection_error_px") is not None:
                    print(f"    reproj={info['reprojection_error_px']:.2f}px, "
                          f"size={info['marker_size_px']:.1f}px")
            else:
                print("  ✗ 未检测到标定板")
                print(f"    诊断: {_format_detect_info(info)}")
                if args.board_type == "charuco" and info.get("num_markers", 0) > 0:
                    print("    提示: 已检测到 ArUco marker，但没有匹配成 ChArUco 板。")
                    print("          优先检查 --charuco-squares-x/--charuco-squares-y 是否和打印板一致。")
            print(f"  图像: {path}")
            continue

        # 获取关节角
        if arm:
            joints = read_joints(arm)
            if joints is None:
                print("  ✗ 读取失败, 重试")
                continue
        else:
            if not user:
                print("  ✗ 手动模式需输入关节角")
                continue
            try:
                joints = np.array([float(v) for v in user.split()])
                if len(joints) != SO101_NUM_JOINTS:
                    print(f"  ✗ 需要 {SO101_NUM_JOINTS} 个值")
                    continue
            except ValueError:
                print("  ✗ 格式错误")
                continue

        # FK
        T_bg = compute_fk(fk_solver, joints)

        # 标定板检测 (多帧平均提高精度)
        T_cm, vis, raw_img, detect_info = detect_board_averaged(
            pipeline, cam_mtx, cam_dist, args, num_frames=15, return_info=True)
        if T_cm is None:
            print("  ✗ 未检测到标定板, 调整姿态使标定板朝向相机")
            print(f"    诊断: {_format_detect_info(detect_info)}")
            if args.board_type == "charuco" and detect_info.get("max_markers", 0) > 0:
                print("    提示: 已检测到 ArUco marker，但没有匹配成 ChArUco 板。")
                print("          优先检查 --charuco-squares-x/--charuco-squares-y 是否和打印板一致。")
            continue

        ok, quality_info = check_capture_quality(
            T_bg, T_cm, detect_info,
            [_T_from_rt(R_g2b[i], t_g2b[i]) for i in range(len(R_g2b))],
            args)
        if not ok:
            print("  ✗ 样本质量不足，未收录:")
            for reason in quality_info["reject_reasons"]:
                print(f"    - {reason}")
            print("    可用 --no-quality-gate 临时允许低质量样本。")
            continue

        R_g2b.append(T_bg[:3, :3])
        t_g2b.append(T_bg[:3, 3])
        R_t2c.append(T_cm[:3, :3])
        t_t2c.append(T_cm[:3, 3])
        count += 1

        if args.save_images:
            cv2.imwrite(os.path.join(args.save_images, f"pose_{count:03d}.png"), vis)

        sample = _save_sample(dataset_dir, count, raw_img, vis, joints,
                              T_bg, T_cm, quality_info)
        manifest["samples"].append({
            "index": count,
            "sample_file": f"samples/pose_{count:03d}.json",
            "image": sample["image"],
            "visualization": sample["visualization"],
        })
        manifest["num_samples"] = count
        _json_dump(dataset_dir / "manifest.json", manifest)

        marker_dist = quality_info.get("marker_distance_m")
        reproj = quality_info.get("reprojection_error_px")
        marker_size = quality_info.get("marker_size_px")
        print(f"  ✓ 采集成功 ({count}/{args.num_poses})")
        print(f"    关节: [{', '.join(f'{j:.3f}' for j in joints)}]")
        print(f"    末端(基座系): [{T_bg[0,3]:.4f}, {T_bg[1,3]:.4f}, {T_bg[2,3]:.4f}]")
        print(f"    标定板(相机系): [{T_cm[0,3]:.4f}, {T_cm[1,3]:.4f}, {T_cm[2,3]:.4f}]")
        print(f"    质量: dist={marker_dist*100:.1f}cm, "
              f"reproj={reproj:.2f}px, size={marker_size:.1f}px")
        for warning in quality_info.get("warnings", []):
            print(f"    WARN: {warning}")

    # 清理
    pipeline.stop()
    if arm:
        close_so101(arm)

    if count < 3:
        sys.exit(f"\n错误: 至少需要 3 组, 当前 {count} 组")
    if count < args.min_poses:
        print(f"\n  [WARN] 当前只有 {count} 组, 建议至少 {args.min_poses} 组。")

    # ---- 求解 ----
    print(f"\n{'=' * 60}")
    print(f"  标定求解 ({count} 组数据, 5 种方法)")
    print(f"{'=' * 60}")

    results = solve_hand_eye(R_g2b, t_g2b, R_t2c, t_t2c)
    if not results:
        sys.exit("错误: 所有方法都失败")

    best, eval_details = evaluate(results, R_g2b, t_g2b, R_t2c, t_t2c)
    out = save_result(dataset_dir, results, best, eval_details, count,
                      args.marker_length, cam_mtx)
    _json_dump(args.output, out)

    t_vec = np.asarray(out["T_base_camera"]["translation"])
    rpy = np.asarray(out["T_base_camera"]["rotation"])

    print(f"\n  ★ 最佳方法: {best}")
    print(f"    translation: [{t_vec[0]:.4f}, {t_vec[1]:.4f}, {t_vec[2]:.4f}]")
    print(f"    rotation:    [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")
    print(f"\n  结果已保存: {args.output}")
    print(f"  数据集结果: {dataset_dir / 'result.json'}")
    print(f"  标定报告: {dataset_dir / 'report.txt'}")
    if args.apply_config:
        backup = _apply_to_grasp_config(args.apply_config, t_vec, rpy)
        print(f"  已写回配置: {args.apply_config}")
        print(f"  备份文件: {backup}")

    # ---- 配置片段 ----
    print(f"\n{'=' * 60}")
    print("  复制以下内容到 grasp_pipeline.yaml:")
    print(f"{'=' * 60}")
    print(f"""
calibration:
  T_base_camera:
    translation: [{t_vec[0]:.4f}, {t_vec[1]:.4f}, {t_vec[2]:.4f}]
    rotation: [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]
""")


if __name__ == "__main__":
    main()
