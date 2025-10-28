#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
正确的rostopic过滤示例
展示如何正确使用rostopic echo的过滤功能
"""

import rospy
from rosgraph_msgs.msg import Log


def callback(msg):
    """rosout消息回调函数"""
    # 正确的字段访问方式
    print(f"[{msg.level}] [{msg.name}]: {msg.msg}")


def main():
    rospy.init_node("rostopic_filter_example")

    # 订阅/rosout话题
    sub = rospy.Subscriber("/rosout", Log, callback)

    print("等待rosout消息...")
    print("按Ctrl+C退出")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == "__main__":
    main()
