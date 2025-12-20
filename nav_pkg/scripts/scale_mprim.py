#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
缩放 mprim 文件脚本
功能：
1. 将 mprim 文件中的 resolution_m 修改为 0.05
2. 等比例缩放每个 primID 中的 intermediateposes (x, y)
"""

import os
import glob
import sys


def process_file(filepath, target_resolution=0.05):
    try:
        with open(filepath, "r") as f:
            lines = f.readlines()
    except Exception as e:
        print("无法读取文件 {}: {}".format(filepath, e))
        return

    new_lines = []
    old_resolution = None

    # 第一遍扫描，找到旧的分辨率
    for line in lines:
        if line.strip().startswith("resolution_m:"):
            parts = line.split()
            if len(parts) > 1:
                try:
                    old_resolution = float(parts[1])
                except ValueError:
                    pass
            break

    if old_resolution is None:
        print("跳过 {}: 未找到 resolution_m".format(os.path.basename(filepath)))
        return

    # 如果分辨率已经很接近目标值，则不处理（或者是为了强制更新数值？）
    # 用户要求“改成0.05...也要等比例改变”，暗示这是一个转换过程
    # 如果已经是0.05，scale_factor=1.0，不会改变数值，所以也是安全的
    if abs(old_resolution - target_resolution) < 1e-9:
        print(
            "跳过 {}: 分辨率已经是 {}".format(
                os.path.basename(filepath), target_resolution
            )
        )
        return

    scale_factor = target_resolution / old_resolution
    print(
        "处理 {}: 分辨率 {:.6f} -> {:.6f} (缩放比例: {:.4f})".format(
            os.path.basename(filepath), old_resolution, target_resolution, scale_factor
        )
    )

    intermediate_poses_count = 0

    for line in lines:
        stripped_line = line.strip()

        # 处理 resolution_m 行
        if stripped_line.startswith("resolution_m:"):
            new_lines.append("resolution_m: {:.6f}\n".format(target_resolution))
            continue

        # 处理 intermediateposes 计数行
        if stripped_line.startswith("intermediateposes:"):
            new_lines.append(line)
            parts = stripped_line.split()
            if len(parts) > 1:
                try:
                    intermediate_poses_count = int(parts[1])
                except ValueError:
                    intermediate_poses_count = 0
            continue

        # 处理 intermediateposes 数据行
        if intermediate_poses_count > 0:
            parts = line.split()
            # 确保是数据行（通常是3个浮点数）
            if len(parts) >= 3:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    theta = float(parts[2])  # theta 通常不需要缩放，它是角度

                    new_x = x * scale_factor
                    new_y = y * scale_factor

                    # 保持与其他行类似的格式，这里使用 .4f
                    new_lines.append(
                        "{:.4f} {:.4f} {:.4f}\n".format(new_x, new_y, theta)
                    )
                except ValueError:
                    # 如果解析失败，保留原样
                    new_lines.append(line)
            else:
                new_lines.append(line)

            intermediate_poses_count -= 1
        else:
            new_lines.append(line)

    try:
        with open(filepath, "w") as f:
            f.writelines(new_lines)
        print("成功更新 {}".format(os.path.basename(filepath)))
    except Exception as e:
        print("写入文件 {} 失败: {}".format(filepath, e))


def main():
    target_dir = "/home/zhang/catkin_ws/src/nav_pkg/mprim2"

    if not os.path.exists(target_dir):
        print("目录不存在: {}".format(target_dir))
        sys.exit(1)

    mprim_files = glob.glob(os.path.join(target_dir, "*.mprim"))

    if not mprim_files:
        print("在 {} 中未找到 .mprim 文件".format(target_dir))
        sys.exit(0)

    print("找到 {} 个 .mprim 文件，开始处理...".format(len(mprim_files)))
    for f in mprim_files:
        process_file(f)
    print("处理完成。")


if __name__ == "__main__":
    main()
