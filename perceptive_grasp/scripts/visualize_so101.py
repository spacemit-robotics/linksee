#!/usr/bin/env python3
"""
SO-101 机械臂 Pinocchio + Meshcat 可视化

功能：
  - 加载 SO-101 URDF
  - 在浏览器中渲染机械臂 3D 模型
  - 高亮标注 base_link（蓝色坐标轴）和 gripper_frame_link（绿色坐标轴 = TCP）
  - 支持手动输入关节角，实时看 FK 结果

用法：
  python3 visualize_so101.py [--joints 0,0,0,0,0,0]

依赖：
  pip3 install meshcat
  需要系统已安装 pinocchio (本项目已有)
"""

import argparse
import os
import sys
import time

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# ============================================================
# 路径
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "..", "urdf", "so101.urdf")

# SO-101 定义
BASE_LINK = "base_link"
TIP_LINK = "gripper_frame_link"


def load_model(urdf_path):
    """加载 URDF 并返回 model + data"""
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    return model, data


def create_visualizer(model, urdf_path):
    """创建 Meshcat 可视化器"""
    # 构建几何模型（用于渲染）
    geom_model = pin.buildGeomFromUrdf(
        model, urdf_path, pin.GeometryType.VISUAL,
        package_dirs=[os.path.dirname(urdf_path)]
    )

    viz = MeshcatVisualizer(model, geom_model, geom_model)
    viz.initViewer(open=True)
    viz.loadViewerModel(rootNodeName="so101")
    return viz


def add_frame_marker(viz, model, data, frame_name, color, scale=0.08):
    """在指定 frame 位置画一个坐标轴标记"""
    import meshcat.geometry as g
    import meshcat.transformations as tf

    frame_id = model.getFrameId(frame_name)
    oMf = data.oMf[frame_id]

    # 4x4 齐次变换
    T = np.eye(4)
    T[:3, :3] = oMf.rotation
    T[:3, 3] = oMf.translation

    # 画三个轴 (RGB = XYZ)
    axes = [
        ("x", [1, 0, 0], [0.8, 0.1, 0.1]),  # 红
        ("y", [0, 1, 0], [0.1, 0.8, 0.1]),  # 绿
        ("z", [0, 0, 1], [0.1, 0.1, 0.8]),  # 蓝
    ]

    for axis_name, direction, axis_color in axes:
        # 用圆柱代表轴
        cyl = g.Cylinder(height=scale, radius=scale * 0.06)
        mat = g.MeshPhongMaterial(color=int(
            axis_color[0] * 255) << 16 |
            int(axis_color[1] * 255) << 8 |
            int(axis_color[2] * 255),
            opacity=0.9
        )

        # 旋转圆柱到对应轴方向
        direction = np.array(direction, dtype=float)
        # 默认圆柱沿 Y 轴，需要旋转
        if axis_name == "x":
            rot = tf.rotation_matrix(np.pi / 2, [0, 0, 1])
        elif axis_name == "z":
            rot = tf.rotation_matrix(-np.pi / 2, [1, 0, 0])
        else:
            rot = np.eye(4)

        # 平移到轴的一半处
        trans = np.eye(4)
        trans[:3, 3] = direction * scale / 2.0

        local_T = trans @ rot
        world_T = T @ local_T

        path = f"frames/{frame_name}/{axis_name}"
        viz.viewer[path].set_object(cyl, mat)
        viz.viewer[path].set_transform(world_T)

    # 画一个球表示原点
    sphere = g.Sphere(radius=scale * 0.15)
    sphere_color = color
    sphere_mat = g.MeshPhongMaterial(
        color=int(sphere_color[0] * 255) << 16 |
              int(sphere_color[1] * 255) << 8 |
              int(sphere_color[2] * 255),
        opacity=0.85
    )
    viz.viewer[f"frames/{frame_name}/origin"].set_object(sphere, sphere_mat)
    viz.viewer[f"frames/{frame_name}/origin"].set_transform(T)

    # 添加文字标签（用小球旁边的偏移）
    label_T = T.copy()
    label_T[:3, 3] += np.array([0, 0, scale * 1.5])


def main():
    parser = argparse.ArgumentParser(description="SO-101 Meshcat 可视化")
    parser.add_argument("--joints", type=str, default="0,0,0,0,0,0",
                        help="逗号分隔的6个关节角(rad), 如 '0,0,0,0,0,0'")
    args = parser.parse_args()

    # 解析关节角
    q_list = [float(x) for x in args.joints.split(",")]

    urdf_path = os.path.abspath(URDF_PATH)
    if not os.path.exists(urdf_path):
        print(f"[错误] URDF 不存在: {urdf_path}")
        sys.exit(1)

    print(f"[1/4] 加载 URDF: {urdf_path}")
    model, data = load_model(urdf_path)
    print(f"       DOF = {model.nv}, joints = {model.njoints - 1}")

    # 确保关节角数量匹配
    nq = model.nq
    if len(q_list) < nq:
        q_list.extend([0.0] * (nq - len(q_list)))
    q = np.array(q_list[:nq])

    print(f"[2/4] 正运动学 (q = {q})")
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # 打印 base_link 和 TCP 位姿
    base_id = model.getFrameId(BASE_LINK)
    tcp_id = model.getFrameId(TIP_LINK)

    print(f"\n  ● {BASE_LINK} 位姿:")
    print(f"    位置: {data.oMf[base_id].translation}")
    print(f"    旋转:\n{data.oMf[base_id].rotation}")

    print(f"\n  ● {TIP_LINK} (TCP) 位姿:")
    print(f"    位置: {data.oMf[tcp_id].translation}")
    print(f"    旋转:\n{data.oMf[tcp_id].rotation}")

    print("\n[3/4] 启动 Meshcat 可视化...")
    viz = create_visualizer(model, urdf_path)
    viz.display(q)

    print("[4/4] 标注坐标系...")
    # base_link: 蓝色球
    add_frame_marker(viz, model, data, BASE_LINK,
                     color=[0.2, 0.4, 1.0], scale=0.06)
    # TCP: 绿色球
    add_frame_marker(viz, model, data, TIP_LINK,
                     color=[0.1, 0.9, 0.2], scale=0.06)

    print("\n" + "=" * 50)
    print("  可视化已启动！请在浏览器中查看。")
    print("  蓝色球 = base_link (基座原点)")
    print("  绿色球 = gripper_frame_link (TCP)")
    print("  RGB 轴 = XYZ 坐标方向")
    print("=" * 50)
    print("\n按 Ctrl+C 退出...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出。")


if __name__ == "__main__":
    main()
