#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS节点版本的rosout过滤器
"""

import rospy
import os
import sys
import subprocess
from rosout_filter import filter_rosout_by_publisher


def main():
    rospy.init_node("rosout_filter_node")

    # 获取参数
    bag_file = rospy.get_param("~bag_file", "")
    filter_publishers_str = rospy.get_param("~filter_publishers", "move_base")
    format_type = rospy.get_param("~format_type", "log")

    if not bag_file:
        rospy.logerr("必须提供bag_file参数")
        return

    # 解析publisher列表
    filter_publishers = [
        p.strip() for p in filter_publishers_str.split(",") if p.strip()
    ]

    if not filter_publishers:
        rospy.logerr("必须提供至少一个filter_publishers参数")
        return

    rospy.loginfo(f"开始过滤rosout消息，bag文件: {bag_file}")
    rospy.loginfo(f"目标publishers: {filter_publishers}")

    # 构建输出文件路径（保存在脚本同一目录下）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bag_basename = os.path.splitext(os.path.basename(bag_file))[0]
    publisher_names = "_".join([p.replace("/", "") for p in filter_publishers])
    file_extension = "log" if format_type == "log" else "json"
    output_file = os.path.join(
        script_dir, f"{bag_basename}_rosout_{publisher_names}.{file_extension}"
    )

    rospy.loginfo(f"输出文件路径: {output_file}")

    # 调用过滤函数
    success = filter_rosout_by_publisher(
        bag_file, filter_publishers, output_file, format_type
    )

    if success:
        rospy.loginfo("rosout过滤完成")
    else:
        rospy.logerr("rosout过滤失败")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
