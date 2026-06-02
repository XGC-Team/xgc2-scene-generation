#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
偏航角转四元数工具

输入偏航角（默认角度），输出可以直接复制到 YAML 文件的四元数格式
只绕Z轴旋转，roll和pitch为0

四元数公式：
q = [0, 0, sin(yaw/2), cos(yaw/2)]
"""

import math
import sys

def yaw_to_quaternion(yaw_rad):
    """
    将偏航角转换为四元数

    Args:
        yaw_rad: 偏航角（弧度）

    Returns:
        四元数 [x, y, z, w] (YAML格式)
    """
    x = 0.0
    y = 0.0
    z = math.sin(yaw_rad / 2.0)
    w = math.cos(yaw_rad / 2.0)

    return [x, y, z, w]

def main():
    if len(sys.argv) < 2:
        print("用法: python yaw_to_quaternion.py <偏航角> [单位]")
        print("示例:")
        print("  python yaw_to_quaternion.py 90              # 角度制（默认）")
        print("  python yaw_to_quaternion.py 90 deg         # 角度制（显式指定）")
        print("  python yaw_to_quaternion.py 1.5708 rad    # 弧度制")
        sys.exit(1)

    yaw = float(sys.argv[1])
    unit = sys.argv[2].lower() if len(sys.argv) > 2 else 'deg'

    # 转换为弧度
    if unit == 'deg' or unit == 'd':
        yaw_rad = math.radians(yaw)
    elif unit == 'rad' or unit == 'r':
        yaw_rad = yaw
    else:
        yaw_rad = math.radians(yaw)

    # 计算四元数
    q = yaw_to_quaternion(yaw_rad)

    # 输出可以直接复制到 YAML 的格式
    print(f"  orientation: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")

if __name__ == "__main__":
    main()