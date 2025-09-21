#!/usr/bin/env python3
"""
测试缩放机制的具体影响

验证问题：手臂的缩放系数是否会影响手腕位置？
"""

import numpy as np
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_lafan1_file

def test_scaling_impact():
    """测试缩放参数对不同关节位置的影响"""

    # 加载BVH数据
    bvh_file = "lafan1/walk1_subject1.bvh"
    lafan1_data_frames, actual_human_height = load_lafan1_file(bvh_file)
    human_data = lafan1_data_frames[0]  # 使用第一帧

    print("=== 缩放机制影响测试 ===\n")

    # 测试1: 基准情况
    print("1. 基准情况（原始缩放参数）:")
    retargeter1 = GMR(
        src_human="bvh",
        tgt_robot="unitree_g1",
        actual_human_height=actual_human_height,
        verbose=False
    )
    retargeter1.update_targets(human_data)
    scaled_data1 = retargeter1.scaled_human_data

    print("   原始缩放参数:")
    for joint, scale in retargeter1.human_scale_table.items():
        print(f"     {joint:15s}: {scale:.3f}")

    print("\n   关键关节位置:")
    key_joints = ['Hips', 'LeftArm', 'LeftForeArm', 'LeftHand']
    for joint in key_joints:
        if joint in scaled_data1:
            pos = scaled_data1[joint][0]
            print(f"     {joint:15s}: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}]")

    # 测试2: 只修改LeftArm缩放参数
    print("\n2. 只修改LeftArm缩放参数 (0.729 -> 1.0):")
    retargeter2 = GMR(
        src_human="bvh",
        tgt_robot="unitree_g1",
        actual_human_height=actual_human_height,
        verbose=False
    )
    # 修改手臂缩放参数
    retargeter2.human_scale_table['LeftArm'] = 1.0
    retargeter2.update_targets(human_data)
    scaled_data2 = retargeter2.scaled_human_data

    print("   修改后的关键关节位置:")
    for joint in key_joints:
        if joint in scaled_data2:
            pos = scaled_data2[joint][0]
            pos_old = scaled_data1[joint][0] if joint in scaled_data1 else np.zeros(3)
            diff = pos - pos_old
            print(f"     {joint:15s}: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}] (Δ: [{diff[0]:6.3f}, {diff[1]:6.3f}, {diff[2]:6.3f}])")

    # 测试3: 只修改LeftForeArm缩放参数
    print("\n3. 只修改LeftForeArm缩放参数 (0.729 -> 1.0):")
    retargeter3 = GMR(
        src_human="bvh",
        tgt_robot="unitree_g1",
        actual_human_height=actual_human_height,
        verbose=False
    )
    # 修改前臂缩放参数
    retargeter3.human_scale_table['LeftForeArm'] = 1.0
    retargeter3.update_targets(human_data)
    scaled_data3 = retargeter3.scaled_human_data

    print("   修改后的关键关节位置:")
    for joint in key_joints:
        if joint in scaled_data3:
            pos = scaled_data3[joint][0]
            pos_old = scaled_data1[joint][0] if joint in scaled_data1 else np.zeros(3)
            diff = pos - pos_old
            print(f"     {joint:15s}: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}] (Δ: [{diff[0]:6.3f}, {diff[1]:6.3f}, {diff[2]:6.3f}])")

    # 分析缩放机制
    print("\n=== 缩放机制分析 ===")
    print("从上面的结果可以看出:")
    print("1. 修改LeftArm缩放参数会影响:")
    for joint in ['LeftArm', 'LeftForeArm', 'LeftHand']:
        if joint in scaled_data1 and joint in scaled_data2:
            pos1 = scaled_data1[joint][0]
            pos2 = scaled_data2[joint][0]
            changed = not np.allclose(pos1, pos2, atol=1e-6)
            print(f"   - {joint}: {'会变化' if changed else '不变化'}")

    print("\n2. 修改LeftForeArm缩放参数会影响:")
    for joint in ['LeftArm', 'LeftForeArm', 'LeftHand']:
        if joint in scaled_data1 and joint in scaled_data3:
            pos1 = scaled_data1[joint][0]
            pos3 = scaled_data3[joint][0]
            changed = not np.allclose(pos1, pos3, atol=1e-6)
            print(f"   - {joint}: {'会变化' if changed else '不变化'}")

if __name__ == "__main__":
    test_scaling_impact()