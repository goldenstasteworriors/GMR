#!/usr/bin/env python3
"""
缩放对比可视化工具 - 简化版

显示第一帧的2D投影对比图：
- 蓝色：BVH原始位置
- 绿色：缩放后位置
- 红色：机器人实际位置
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# 导入项目模块
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_lafan1_file
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh
import general_motion_retargeting.utils.lafan_vendor.utils as utils
import mujoco as mj

def load_bvh_raw_data(bvh_file):
    """加载BVH原始数据"""
    data = read_bvh(bvh_file)
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)

    frame = 0  # 只取第一帧
    result = {}
    for i, bone in enumerate(data.bones):
        position = global_data[1][frame, i] / 100  # cm to m
        orientation = global_data[0][frame, i]
        result[bone] = (position, orientation)

    return result, data.bones

def get_robot_joint_positions(retargeter, human_data):
    """获取机器人关节的实际位置"""
    qpos = retargeter.retarget(human_data)
    retargeter.configuration.data.qpos[:] = qpos
    mj.mj_forward(retargeter.configuration.model, retargeter.configuration.data)

    robot_positions = {}
    for body_id in range(retargeter.configuration.model.nbody):
        body_name = mj.mj_id2name(retargeter.configuration.model, mj.mjtObj.mjOBJ_BODY, body_id)
        if body_name:
            position = retargeter.configuration.data.xpos[body_id].copy()
            robot_positions[body_name] = position

    return robot_positions

def visualize_scaling(bvh_file, robot_type="unitree_g1", save_path=None):
    """生成缩放对比的2D投影图"""

    print(f"加载BVH文件: {bvh_file}")

    # 1. 加载原始BVH数据
    raw_frame, bone_names = load_bvh_raw_data(bvh_file)
    print(f"BVH数据: {len(bone_names)} 个关节")

    # 2. 加载经过坐标变换的数据
    transformed_frames, human_height = load_lafan1_file(bvh_file)
    human_data = transformed_frames[0]  # 第一帧

    # 3. 初始化重定向器
    print(f"初始化机器人: {robot_type}")
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=robot_type,
        actual_human_height=human_height,
        verbose=False
    )

    # 4. 计算缩放和机器人数据
    print("计算缩放后数据...")
    retargeter.update_targets(human_data)
    scaled_data = retargeter.scaled_human_data
    robot_positions_dict = get_robot_joint_positions(retargeter, human_data)

    # 5. 准备数据
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # Y-up到Z-up

    # 原始BVH数据（变换后）
    raw_positions = []
    for bone_name in bone_names:
        if bone_name in raw_frame:
            pos = raw_frame[bone_name][0]
            transformed_pos = pos @ rotation_matrix.T
            raw_positions.append(transformed_pos)

    # 缩放后数据
    scaled_positions = []
    for bone_name, (pos, _) in scaled_data.items():
        scaled_positions.append(pos)

    # 机器人关节位置
    robot_positions = []
    for body_name, pos in robot_positions_dict.items():
        robot_positions.append(pos)

    raw_positions = np.array(raw_positions) if raw_positions else np.empty((0, 3))
    scaled_positions = np.array(scaled_positions) if scaled_positions else np.empty((0, 3))
    robot_positions = np.array(robot_positions) if robot_positions else np.empty((0, 3))

    # 6. 绘制2D投影图
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    views = [
        ("Front View (X-Z)", 0, 2, "X (m)", "Z (m)"),
        ("Side View (Y-Z)", 1, 2, "Y (m)", "Z (m)"),
        ("Top View (X-Y)", 0, 1, "X (m)", "Y (m)")
    ]

    for i, (title, x_idx, y_idx, xlabel, ylabel) in enumerate(views):
        # 第一行：原始BVH数据
        ax_raw = axes[0, i]
        ax_raw.set_title(f"Original BVH - {title}")
        if len(raw_positions) > 0:
            ax_raw.scatter(raw_positions[:, x_idx], raw_positions[:, y_idx],
                          c='blue', s=30, alpha=0.7, label='Original BVH')
        ax_raw.set_xlabel(xlabel)
        ax_raw.set_ylabel(ylabel)
        ax_raw.grid(True, alpha=0.3)
        ax_raw.legend()

        # 第二行：三种数据叠加对比
        ax_overlay = axes[1, i]
        ax_overlay.set_title(f"Comparison - {title}")

        if len(raw_positions) > 0:
            ax_overlay.scatter(raw_positions[:, x_idx], raw_positions[:, y_idx],
                              c='blue', s=20, alpha=0.5, label='Original BVH')
        if len(scaled_positions) > 0:
            ax_overlay.scatter(scaled_positions[:, x_idx], scaled_positions[:, y_idx],
                              c='green', s=20, alpha=0.7, label='Scaled')
        if len(robot_positions) > 0:
            ax_overlay.scatter(robot_positions[:, x_idx], robot_positions[:, y_idx],
                              c='red', s=20, alpha=0.7, label='Robot')

        ax_overlay.set_xlabel(xlabel)
        ax_overlay.set_ylabel(ylabel)
        ax_overlay.grid(True, alpha=0.3)
        ax_overlay.legend()

    plt.suptitle(f'Scaling Comparison - Frame 0', fontsize=14)
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"图像已保存到: {save_path}")
    else:
        plt.show()

def main():
    # 支持BVH的机器人列表
    supported_robots = [
        'unitree_g1',
        'unitree_g1_with_hands',
        'booster_t1_29dof',
        'fourier_n1',
        'stanford_toddy',
        'engineai_pm01'
    ]

    parser = argparse.ArgumentParser(description='缩放对比可视化工具')
    parser.add_argument('--bvh_file', help='BVH文件路径')
    parser.add_argument('--robot', default='unitree_g1',
                       help=f'机器人类型，支持: {", ".join(supported_robots)}')
    parser.add_argument('--save', help='保存图像的路径')
    parser.add_argument('--list', action='store_true', help='显示支持的机器人列表')

    args = parser.parse_args()

    if args.list:
        print("支持BVH的机器人列表:")
        for robot in supported_robots:
            print(f"  - {robot}")
        return

    if not args.bvh_file:
        print("错误: 需要指定 --bvh_file 参数")
        parser.print_help()
        return

    if args.robot not in supported_robots:
        print(f"错误: 不支持的机器人 '{args.robot}'")
        print(f"支持的机器人: {', '.join(supported_robots)}")
        print("使用 --list 查看完整列表")
        return

    if not Path(args.bvh_file).exists():
        print(f"错误: BVH文件不存在: {args.bvh_file}")
        return

    try:
        visualize_scaling(args.bvh_file, args.robot, args.save)
    except Exception as e:
        print(f"错误: {e}")
        print(f"机器人 '{args.robot}' 可能有配置问题")
        print("请尝试使用 'unitree_g1'（最稳定）")

if __name__ == '__main__':
    main()