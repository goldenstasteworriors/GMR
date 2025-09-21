# 缩放参数可视化工具

简化版可视化工具，用于快速检查BVH动作重定向的缩放效果。

## 安装依赖

```bash
pip install matplotlib
```

## 使用方法

### 基本用法

```bash
# 查看G1机器人的缩放效果
python visualize_scaling.py --bvh_file lafan1/walk1_subject1.bvh --robot unitree_g1

# 查看其他机器人
python visualize_scaling.py --bvh_file lafan1/dance1_subject1.bvh --robot booster_t1

# 保存图像
python visualize_scaling.py --bvh_file lafan1/walk1_subject1.bvh --robot unitree_g1 --save comparison.png
```

## 图像说明

工具会生成6个子图的2D对比图：

**上排**：原始BVH数据的三个视角
- 前视图 (X-Z)
- 侧视图 (Y-Z)
- 俯视图 (X-Y)

**下排**：三种数据的叠加对比
- **蓝色点**：BVH原始关节位置
- **绿色点**：缩放后的目标位置
- **红色点**：机器人实际关节位置

## 如何分析

**理想情况**：绿色点和红色点应该尽可能接近

**如果差距很大**：
1. 检查配置文件中的缩放参数
2. 调整对应关节的缩放系数
3. 重新运行查看效果

## 支持的机器人

```bash
# 查看支持的机器人列表
python visualize_scaling.py --list
```

**当前支持BVH的机器人**：
- `unitree_g1` (默认，最稳定)
- `unitree_g1_with_hands`
- `booster_t1_29dof` (注意：不是booster_t1)
- `fourier_n1`
- `stanford_toddy`
- `engineai_pm01`

**注意**：只有这些机器人有BVH配置文件，其他机器人不支持BVH可视化

## 配置文件位置

缩放参数位于：`general_motion_retargeting/ik_configs/bvh_to_{robot}.json`

调整 `human_scale_table` 中的数值来优化缩放效果。