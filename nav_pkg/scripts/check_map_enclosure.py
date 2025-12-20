#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
从地图原点检测地图封闭性
从原点开始广度优先搜索，检查是否能到达地图边界（未知区域）
"""

import cv2
import numpy as np
import yaml
import os
import sys
from collections import deque


def load_map_yaml(yaml_path):
    """加载地图配置文件"""
    try:
        with open(yaml_path, "r") as f:
            map_config = yaml.safe_load(f)
        return map_config
    except Exception as e:
        print(f"错误: 无法读取 yaml 文件: {e}")
        return None


def world_to_map(wx, wy, origin, resolution):
    """世界坐标转换为地图坐标"""
    mx = int((wx - origin[0]) / resolution)
    my = int((wy - origin[1]) / resolution)
    return mx, my


def filter_isolated_unknown_cells(unknown_cells, ros_map, min_cluster_size=10):
    """
    过滤孤立的未知区域点
    使用连通性分析，只保留大的未知区域集群

    Args:
        unknown_cells: 发现的未知区域像素列表 [(x, y), ...]
        ros_map: ROS格式的地图
        min_cluster_size: 最小集群大小，小于此值的集群会被过滤

    Returns:
        filtered_cells: 过滤后的未知区域像素列表
    """
    if len(unknown_cells) == 0:
        return []

    # 创建未知区域的查找集合
    unknown_set = set(unknown_cells)

    # 使用BFS对未知区域进行聚类
    visited = set()
    clusters = []

    for start_cell in unknown_cells:
        if start_cell in visited:
            continue

        # 从这个点开始BFS，找到所有连通的未知区域
        cluster = []
        queue = deque([start_cell])
        visited.add(start_cell)

        while queue:
            x, y = queue.popleft()
            cluster.append((x, y))

            # 检查8个方向的邻居（包括对角线）
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue

                    nx, ny = x + dx, y + dy

                    # 检查是否在未知区域集合中且未访问
                    if (nx, ny) in unknown_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))

        clusters.append(cluster)

    # 过滤掉小的集群
    filtered_cells = []
    for cluster in clusters:
        if len(cluster) >= min_cluster_size:
            filtered_cells.extend(cluster)

    print(f"\n聚类分析结果:")
    print(f"  发现 {len(clusters)} 个未知区域集群")
    large_clusters = [c for c in clusters if len(c) >= min_cluster_size]
    small_clusters = [c for c in clusters if len(c) < min_cluster_size]
    print(f"  保留 {len(large_clusters)} 个大集群 (>= {min_cluster_size} 像素)")
    print(f"  过滤掉 {len(small_clusters)} 个小集群 (< {min_cluster_size} 像素)")

    if large_clusters:
        cluster_sizes = [len(c) for c in large_clusters]
        print(f"  最大集群: {max(cluster_sizes)} 像素")
        print(f"  平均集群大小: {np.mean(cluster_sizes):.1f} 像素")

    return filtered_cells


def check_map_enclosure_from_origin(pgm_path):
    """从原点检测地图封闭性"""

    # 1. 读取 yaml 配置文件
    yaml_path = pgm_path.replace(".pgm", ".yaml")
    use_default_origin = False

    if not os.path.exists(yaml_path):
        print(f"警告: 找不到配置文件 {yaml_path}")
        print(f"将使用默认配置，原点设置在地图中心")
        use_default_origin = True
        map_config = None
    else:
        map_config = load_map_yaml(yaml_path)
        if map_config is None:
            print(f"警告: 配置文件读取失败，将使用默认配置")
            use_default_origin = True

    if use_default_origin:
        # 使用默认配置
        print(f"\n使用默认配置:")
        resolution = 0.05
        origin = None  # 稍后根据地图尺寸计算
        occ_th = 0.65
        free_th = 0.196
        negate = 0
        print(f"  分辨率: {resolution} m/pixel")
        print(f"  原点: 地图中心（稍后计算）")
        print(f"  占用阈值: {occ_th}")
        print(f"  自由阈值: {free_th}")
        print(f"  negate: {negate}")
    else:
        print(f"地图配置:")
        print(f"  图像文件: {map_config.get('image', 'N/A')}")
        print(f"  分辨率: {map_config.get('resolution', 'N/A')} m/pixel")
        print(f"  原点: {map_config.get('origin', 'N/A')}")
        print(f"  占用阈值: {map_config.get('occupied_thresh', 0.65)}")
        print(f"  自由阈值: {map_config.get('free_thresh', 0.196)}")
        print(f"  negate: {map_config.get('negate', 0)}")

        resolution = map_config.get("resolution", 0.05)
        origin = map_config.get("origin", [0.0, 0.0, 0.0])
        occ_th = map_config.get("occupied_thresh", 0.65)
        free_th = map_config.get("free_thresh", 0.196)
        negate = map_config.get("negate", 0)

    # 2. 读取 PGM 文件
    try:
        raw_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if raw_img is None:
            print(f"错误: 无法读取地图文件 {pgm_path}")
            return False
    except Exception as e:
        print(f"读取地图时出错: {e}")
        return False

    h, w = raw_img.shape
    print(f"\n地图尺寸: {w} x {h} 像素")

    # 如果没有yaml文件，计算地图中心作为原点
    if use_default_origin or origin is None:
        # 地图中心在像素坐标系中的位置
        center_px = w // 2
        center_py = h // 2

        # 将地图中心设置为世界坐标原点 (0, 0)
        # 因此 origin 应该是地图左下角在世界坐标系中的位置
        # world_x = (pixel_x - center_px) * resolution
        # world_y = (pixel_y - center_py) * resolution
        # 所以 origin = [-center_px * resolution, -center_py * resolution]

        origin = [-center_px * resolution, -center_py * resolution, 0.0]
        print(f"  计算得到的原点: {origin}")
        print(f"  地图中心像素坐标: ({center_px}, {center_py})")

    # 3. 将原始像素值转换为 ROS 地图格式
    # 模拟 map_server 的处理逻辑
    if negate:
        color_avg = 255 - raw_img.astype(np.float64)
    else:
        color_avg = raw_img.astype(np.float64)

    occ = (255 - color_avg) / 255.0

    ros_map = np.zeros_like(raw_img, dtype=np.int8)
    ros_map[occ > occ_th] = 100  # 障碍物
    ros_map[occ < free_th] = 0  # 自由空间
    ros_map[(occ >= free_th) & (occ <= occ_th)] = -1  # 未知区域

    # 统计地图内容
    free_count = np.sum(ros_map == 0)
    obstacle_count = np.sum(ros_map == 100)
    unknown_count = np.sum(ros_map == -1)

    print(f"\n地图内容统计:")
    print(f"  自由空间: {free_count} 像素")
    print(f"  障碍物: {obstacle_count} 像素")
    print(f"  未知区域: {unknown_count} 像素")

    # 4. 计算原点在地图中的位置
    origin_mx, origin_my = world_to_map(0, 0, origin, resolution)

    # 注意：图像坐标系Y轴向下，地图坐标系Y轴向上，需要翻转
    origin_my = h - origin_my - 1

    print(f"\n原点在地图中的位置: ({origin_mx}, {origin_my})")

    # 检查原点是否在地图范围内
    if origin_mx < 0 or origin_mx >= w or origin_my < 0 or origin_my >= h:
        print(f"错误: 原点不在地图范围内")
        return False

    # 检查原点是否在自由空间
    if ros_map[origin_my, origin_mx] != 0:
        origin_value = ros_map[origin_my, origin_mx]
        if origin_value == 100:
            print(f"警告: 原点位置是障碍物！")
        elif origin_value == -1:
            print(f"警告: 原点位置是未知区域！")
        else:
            print(f"警告: 原点位置值异常: {origin_value}")

        # 尝试在原点附近查找自由空间
        print(f"尝试在原点附近查找自由空间...")
        found = False
        for radius in range(1, 20):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    ny, nx = origin_my + dy, origin_mx + dx
                    if 0 <= nx < w and 0 <= ny < h and ros_map[ny, nx] == 0:
                        origin_mx, origin_my = nx, ny
                        print(f"找到附近的自由空间: ({origin_mx}, {origin_my})")
                        found = True
                        break
                if found:
                    break
            if found:
                break

        if not found:
            print(f"错误: 无法在原点附近找到自由空间")
            return False

    # 5. 从原点开始广度优先搜索
    print(f"\n开始从原点进行广度优先搜索...")

    visited = np.zeros((h, w), dtype=bool)
    queue = deque([(origin_my, origin_mx)])
    visited[origin_my, origin_mx] = True

    explored_count = 0
    reached_unknown = False
    unknown_cells = []

    while queue:
        y, x = queue.popleft()
        explored_count += 1

        # 检查四个方向
        for dy, dx in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            ny, nx = y + dy, x + dx

            # 检查边界
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue

            # 如果已访问，跳过
            if visited[ny, nx]:
                continue

            cell_value = ros_map[ny, nx]

            # 如果是障碍物，跳过
            if cell_value == 100:
                continue

            # 如果是未知区域，记录并跳过
            if cell_value == -1:
                if not reached_unknown:
                    reached_unknown = True
                    print(f"  发现未知区域: ({nx}, {ny})")
                unknown_cells.append((nx, ny))
                visited[ny, nx] = True
                continue

            # 如果是自由空间，继续探索
            if cell_value == 0:
                visited[ny, nx] = True
                queue.append((ny, nx))

    # 6. 过滤孤立的未知区域点
    # 对未知区域点进行连通性分析，只保留大的连通区域
    filtered_unknown_cells = filter_isolated_unknown_cells(
        unknown_cells, ros_map, min_cluster_size=10
    )

    print(f"\n未知区域过滤:")
    print(f"  原始发现: {len(unknown_cells)} 个未知区域像素")
    print(f"  过滤后: {len(filtered_unknown_cells)} 个未知区域像素")
    print(
        f"  已过滤掉 {len(unknown_cells) - len(filtered_unknown_cells)} 个孤立/稀疏的未知点"
    )

    # 判断是否真的到达了有效的未知区域
    reached_valid_unknown = len(filtered_unknown_cells) > 0

    # 7. 保存可视化结果
    # 创建彩色可视化地图
    vis_map = np.zeros((h, w, 3), dtype=np.uint8)

    # 自由空间 = 白色
    vis_map[ros_map == 0] = [255, 255, 255]
    # 障碍物 = 黑色
    vis_map[ros_map == 100] = [0, 0, 0]
    # 未知区域 = 灰色
    vis_map[ros_map == -1] = [128, 128, 128]

    # 标记已探索的区域为绿色
    vis_map[visited & (ros_map == 0)] = [0, 255, 0]

    # 标记原点为红色十字
    cv2.drawMarker(
        vis_map, (origin_mx, origin_my), (0, 0, 255), cv2.MARKER_CROSS, 20, 2
    )

    # 标记过滤后的未知区域为蓝色点
    for nx, ny in filtered_unknown_cells[:100]:  # 只标记前100个
        cv2.circle(vis_map, (nx, ny), 2, (255, 0, 0), -1)

    # 标记被过滤掉的孤立点为黄色（用于调试）
    filtered_out = set(unknown_cells) - set(filtered_unknown_cells)
    for nx, ny in list(filtered_out)[:100]:  # 只标记前100个
        cv2.circle(vis_map, (nx, ny), 1, (0, 255, 255), -1)

    # 保存可视化地图（带标记）
    output_vis_path = pgm_path.replace(".pgm", "_enclosure_check.png")
    cv2.imwrite(output_vis_path, vis_map)

    # 保存转换后的ROS地图（无标记）
    ros_map_vis = np.zeros((h, w, 3), dtype=np.uint8)
    ros_map_vis[ros_map == 0] = [255, 255, 255]  # 自由空间 = 白色
    ros_map_vis[ros_map == 100] = [0, 0, 0]  # 障碍物 = 黑色
    ros_map_vis[ros_map == -1] = [205, 205, 205]  # 未知区域 = 浅灰色

    output_ros_path = pgm_path.replace(".pgm", "_ros_format.png")
    cv2.imwrite(output_ros_path, ros_map_vis)

    print(f"\n已保存可视化结果:")
    print(f"  1. 封闭性检查图: {output_vis_path}")
    print(f"     - 绿色 = 从原点可达的自由空间")
    print(f"     - 白色 = 不可达的自由空间")
    print(f"     - 黑色 = 障碍物")
    print(f"     - 灰色 = 未知区域")
    print(f"     - 红色十字 = 原点位置")
    print(f"     - 蓝色点 = 有效的未知区域边界（大集群）")
    print(f"     - 黄色点 = 被过滤的孤立未知点")
    print(f"  2. ROS格式地图: {output_ros_path}")
    print(f"     - 白色 = 自由空间 (0)")
    print(f"     - 黑色 = 障碍物 (100)")
    print(f"     - 浅灰色 = 未知区域 (-1)")

    # 8. 分析结果
    print(f"\n搜索完成!")
    print(f"  探索的自由空间像素: {explored_count}")
    print(f"  发现的有效未知区域像素: {len(filtered_unknown_cells)}")

    print(f"\n封闭性检查结果:")
    if reached_valid_unknown:
        print(f"  ✅ 地图不是封闭的")
        print(f"  从原点出发可以到达 {len(filtered_unknown_cells)} 个有效未知区域单元")
        print(f"  这意味着机器人可以探索地图边界")
    else:
        print(f"  ⚠️  地图是封闭的!")
        print(f"  从原点出发无法到达任何有效的未知区域")
        print(f"  机器人被完全困在已知的自由空间内")

        # 计算封闭空间的大小
        total_reachable = explored_count
        total_area = total_reachable * (resolution**2)
        print(f"  可达区域大小: {total_area:.2f} 平方米")

    # 计算可达性统计
    reachable_free = explored_count
    total_free = free_count
    reachability_ratio = reachable_free / total_free if total_free > 0 else 0

    print(f"\n可达性统计:")
    print(
        f"  可达的自由空间: {reachable_free} / {total_free} ({reachability_ratio:.1%})"
    )
    unreachable_free = total_free - reachable_free
    if unreachable_free > 0:
        print(f"  不可达的自由空间: {unreachable_free} 像素")
        print(f"  这些区域可能是孤立的房间或被障碍物分隔的区域")

    return not reached_valid_unknown  # 返回True表示封闭，False表示不封闭


def main():
    # 默认地图路径
    default_map = "/home/zhang/catkin_ws/src/wpr_simulation/maps/map.pgm"

    if len(sys.argv) > 1:
        map_path = sys.argv[1]
    else:
        map_path = default_map

    print("=" * 70)
    print("地图封闭性检查工具")
    print("=" * 70)
    print(f"地图文件: {map_path}\n")

    is_enclosed = check_map_enclosure_from_origin(map_path)

    print("\n" + "=" * 70)
    if is_enclosed:
        print("结论: 地图是封闭的")
        sys.exit(1)
    else:
        print("结论: 地图不是封闭的")
        sys.exit(0)


if __name__ == "__main__":
    main()
