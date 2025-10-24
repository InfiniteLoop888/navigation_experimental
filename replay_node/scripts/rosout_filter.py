#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
过滤特定publisher的rosout消息并保存到JSON文件
"""

import rosbag
import json
import sys
import os
from datetime import datetime
from collections import defaultdict


def filter_rosout_by_publisher(
    bag_file, target_publishers, output_file=None, format_type="json"
):
    """
    过滤特定publisher的rosout消息

    Args:
        bag_file: rosbag文件路径
        target_publishers: 目标publisher列表
        output_file: 输出文件路径
    """

    if not os.path.exists(bag_file):
        print(f"错误: bag文件不存在: {bag_file}")
        return False

    if output_file is None:
        base_name = os.path.splitext(os.path.basename(bag_file))[0]
        publisher_names = "_".join([p.replace("/", "") for p in target_publishers])
        output_file = f"{base_name}_rosout_{publisher_names}.json"

    print(f"正在处理bag文件: {bag_file}")
    print(f"目标publishers: {target_publishers}")
    print(f"输出文件: {output_file}")

    try:
        bag = rosbag.Bag(bag_file)

        filtered_messages = []
        found_publishers = set()
        publishers_stats = defaultdict(int)

        for topic, msg, t in bag.read_messages(topics=["/rosout"]):
            if msg.name in target_publishers:
                found_publishers.add(msg.name)
                publishers_stats[msg.name] += 1

                message_data = {
                    "time": t.secs + t.nsecs / 1e9,
                    "node": msg.name,
                    "level": get_level_name(msg.level),
                    "msg": msg.msg,
                }

                filtered_messages.append(message_data)

        if not filtered_messages:
            print("警告: 没有找到匹配的publisher消息")
            bag.close()
            return False

        # 根据格式类型保存数据
        if format_type == "log":
            # 保存为简洁的日志格式
            with open(output_file, "w", encoding="utf-8") as f:
                for msg in filtered_messages:
                    timestamp = msg["time"]
                    node = msg["node"]
                    level = msg["level"]
                    message = msg["msg"]
                    f.write(f"[{timestamp:.3f}][{node}] {message}\n")
        else:
            # 保存为JSON格式
            json_data = {
                "bag_file": bag_file,
                "extraction_time": datetime.now().isoformat(),
                "target_publishers": target_publishers,
                "found_publishers": list(found_publishers),
                "total_messages": len(filtered_messages),
                "publishers_stats": dict(publishers_stats),
                "messages": filtered_messages,
            }

            with open(output_file, "w", encoding="utf-8") as f:
                json.dump(json_data, f, ensure_ascii=False, indent=2)

        print(f"成功保存 {len(filtered_messages)} 条过滤后的rosout消息到 {output_file}")

        # 显示统计信息
        print("\n过滤结果:")
        for pub in target_publishers:
            count = publishers_stats.get(pub, 0)
            status = "✓" if count > 0 else "✗"
            print(f"  {status} {pub}: {count} 条消息")

        bag.close()
        return True

    except Exception as e:
        print(f"处理过程中出错: {str(e)}")
        return False


def get_level_name(level):
    """将数字级别转换为可读的级别名称"""
    level_names = {1: "FATAL", 2: "ERROR", 4: "WARN", 8: "INFO", 16: "DEBUG"}
    return level_names.get(level, f"LEVEL_{level}")


def main():
    if len(sys.argv) < 3:
        print(
            "使用方法: python3 rosout_filter.py <bag_file> <publisher1> [publisher2] ... [output_file]"
        )
        print("示例: python3 rosout_filter.py file.bag /move_base /gazebo")
        print("示例: python3 rosout_filter.py file.bag /move_base output.json")
        sys.exit(1)

    bag_file = sys.argv[1]

    # 解析参数
    args = sys.argv[2:]
    target_publishers = []
    output_file = None

    for arg in args:
        if arg.endswith(".json"):
            output_file = arg
        else:
            target_publishers.append(arg)

    if not target_publishers:
        print("错误: 必须指定至少一个publisher")
        sys.exit(1)

    success = filter_rosout_by_publisher(bag_file, target_publishers, output_file)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
