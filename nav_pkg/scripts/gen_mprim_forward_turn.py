#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成仅包含“向前走 + 转弯/原地转向”的 motion primitive 文件。
参考 mprim2/genmprim.m 与 genmprim_unicycle.m，针对 sbpl 格式。

特性：
1. 分辨率 resolution = 0.05 m
2. 角度离散：16
3. 每个角度 6 个 primitive（仅直进/后退 + 原地转向）：
   - 前进 1 格
   - 前进 8 格
   - 后退 1 格
   - 后退 8 格
   - 原地左转（+1 角度步长）
   - 原地右转（-1 角度步长）

输出文件格式与 sbpl 通用 mprim 文件一致。
"""

import math
import sys
from pathlib import Path


def generate(output_path: str, resolution: float = 0.05) -> None:
    number_of_angles = 16
    num_prims_per_angle = 6
    num_samples = 10

    # 基础模板：在 0 度下的 (dx, dy, dtheta, costmult)
    # 仅前进/转向，不含后退
    base_prims = [
        (1, 0, 0, 1),    # 短前进
        (8, 0, 0, 1),    # 长前进
        (-1, 0, 0, 5),   # 短后退
        (-8, 0, 0, 5),   # 长后退
        (0, 0, 1, 5),    # 原地左转
        (0, 0, -1, 5),   # 原地右转
    ]

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    def write_line(fh, line: str) -> None:
        fh.write(line + "\n")

    with out.open("w") as fh:
        write_line(fh, f"resolution_m: {resolution:.6f}")
        write_line(fh, f"numberofangles: {number_of_angles}")
        write_line(fh, f"totalnumberofprimitives: {number_of_angles * num_prims_per_angle}")

        for angle_ind in range(number_of_angles):
            current_angle = angle_ind * 2 * math.pi / number_of_angles

            for prim_id, (dx, dy, dtheta, costmult) in enumerate(base_prims):
                write_line(fh, f"primID: {prim_id}")
                write_line(fh, f"startangle_c: {angle_ind}")

                # 旋转末端栅格坐标
                endx_c = round(dx * math.cos(current_angle) - dy * math.sin(current_angle))
                endy_c = round(dx * math.sin(current_angle) + dy * math.cos(current_angle))
                endtheta_c = (angle_ind + dtheta) % number_of_angles

                # 生成中间姿态
                startpt = (0.0, 0.0, current_angle)
                endpt = (
                    endx_c * resolution,
                    endy_c * resolution,
                    endtheta_c * 2 * math.pi / number_of_angles,
                )

                intermcells = []
                if (endx_c == 0 and endy_c == 0) or dtheta == 0:
                    # 纯转向或直线
                    rot_angle = dtheta * 2 * math.pi / number_of_angles
                    for i in range(num_samples):
                        t = i / (num_samples - 1)
                        x = startpt[0] + (endpt[0] - startpt[0]) * t
                        y = startpt[1] + (endpt[1] - startpt[1]) * t
                        theta = (startpt[2] + rot_angle * t) % (2 * math.pi)
                        intermcells.append((x, y, theta))
                else:
                    # 前进并转弯：为稳健起见，直接线性插值姿态（避免奇异导致数值爆炸）
                    for i in range(num_samples):
                        t = i / (num_samples - 1)
                        x = startpt[0] + (endpt[0] - startpt[0]) * t
                        y = startpt[1] + (endpt[1] - startpt[1]) * t
                        theta = (startpt[2] + (endpt[2] - startpt[2]) * t) % (2 * math.pi)
                        intermcells.append((x, y, theta))

                write_line(fh, f"endpose_c: {endx_c} {endy_c} {endtheta_c}")
                write_line(fh, f"additionalactioncostmult: {costmult}")
                write_line(fh, f"intermediateposes: {len(intermcells)}")
                for x, y, th in intermcells:
                    write_line(fh, f"{x:.4f} {y:.4f} {th:.4f}")


if __name__ == "__main__":
    out_file = "mprim_forward_turn.mprim"
    if len(sys.argv) > 1:
        out_file = sys.argv[1]
    generate(out_file)
    print(f"mprim 已生成: {out_file}")

