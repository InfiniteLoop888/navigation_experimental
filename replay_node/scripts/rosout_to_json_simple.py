#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简化版：将rosbag文件中的rosout消息提取并保存到JSON文件
只保存关键信息，文件更小
"""

import rosbag
import json
import sys
import os
from datetime import datetime
from collections import defaultdict


def rosout_to_json_simple(bag_file, output_file=None):
    """
    从bag文件中提取rosout消息并保存到简化的JSON文件
    """

    if not os.path.exists(bag_file):
        print(f"错误: bag文件不存在: {bag_file}")
        return False

    if output_file is None:
        base_name = os.path.splitext(os.path.basename(bag_file))[0]
        output_file = f"{base_name}_rosout_simple.json"

    print(f"正在处理bag文件: {bag_file}")
    print(f"输出文件: {output_file}")

    try:
        bag = rosbag.Bag(bag_file)

        rosout_messages = []
        publishers_stats = defaultdict(int)

        for topic, msg, t in bag.read_messages(topics=["/rosout"]):
            publishers_stats[msg.name] += 1

            # 简化的消息数据
            message_data = {
                "time": t.secs + t.nsecs / 1e9,  # 总秒数
                "node": msg.name,
                "level": get_level_name(msg.level),
                "msg": msg.msg,
            }

            rosout_messages.append(message_data)

        # 简化的JSON结构
        json_data = {
            "bag_file": bag_file,
            "extraction_time": datetime.now().isoformat(),
            "total_messages": len(rosout_messages),
            "publishers": dict(publishers_stats),
            "messages": rosout_messages,
        }

        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(json_data, f, ensure_ascii=False, indent=2)

        print(f"成功保存 {len(rosout_messages)} 条rosout消息到 {output_file}")

        # 显示统计信息
        print("\n统计信息:")
        for pub, count in sorted(publishers_stats.items()):
            print(f"  {pub}: {count} 条消息")

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
    if len(sys.argv) < 2:
        print("使用方法: python3 rosout_to_json_simple.py <bag_file> [output_file]")
        sys.exit(1)

    bag_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    success = rosout_to_json_simple(bag_file, output_file)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
