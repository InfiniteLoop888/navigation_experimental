#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import time
from datetime import datetime
import signal
from rosgraph_msgs.msg import Log


class NavigationLogger:
    def __init__(self):
        rospy.init_node("navigation_logger", anonymous=True)

        # 获取参数
        self.log_file_path = rospy.get_param(
            "~log_file_path", "/tmp/navigation_system.log"
        )
        self.log_level = rospy.get_param("~log_level", "INFO")

        # 日志级别映射
        self.level_map = {"DEBUG": 1, "INFO": 2, "WARN": 4, "ERROR": 8, "FATAL": 16}
        self.level_names = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}
        self.min_level = self.level_map.get(self.log_level, 2)  # 默认INFO级别

        # 创建日志目录
        log_dir = os.path.dirname(self.log_file_path)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 初始化状态
        self.shutdown_flag = False
        self.log_count = 0

        # 订阅 /rosout 话题
        self.rosout_sub = rospy.Subscriber("/rosout", Log, self.log_callback)

        rospy.loginfo(
            f"[NavigationLogger] 日志收集器已启动，日志文件: {self.log_file_path}"
        )
        rospy.loginfo(
            f"[NavigationLogger] 日志级别过滤: {self.log_level} (级别值: {self.min_level})"
        )

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def log_callback(self, log_msg):
        """处理 /rosout 话题的日志消息"""
        try:
            # 提取关键信息
            level = log_msg.level
            msg = log_msg.msg
            name = log_msg.name
            time_sec = log_msg.header.stamp.secs
            time_nsec = log_msg.header.stamp.nsecs

            # 过滤导航相关的日志
            navigation_keywords = [
                "move_base",
                "teb",
                "sbpl",
                "amcl",
                "map_server",
                "teb_local_planner",
                "sbpl_lattice_planner",
                "costmap",
                "global_planner",
                "local_planner",
                "navigation",
                "base_local_planner",
                "base_global_planner",
                "recovery",
            ]

            # 检查是否包含导航相关关键词
            is_navigation_log = any(
                keyword in name.lower() for keyword in navigation_keywords
            )

            # 也检查消息内容
            if not is_navigation_log:
                is_navigation_log = any(
                    keyword in msg.lower()
                    for keyword in [
                        "planning",
                        "path",
                        "goal",
                        "trajectory",
                        "obstacle",
                        "recovery",
                        "navigation",
                        "teb",
                        "sbpl",
                        "costmap",
                        "optimization",
                        "homotopy",
                        "oscillation",
                        "velocity",
                    ]
                )

            # 检查日志级别是否满足要求
            if level >= self.min_level and is_navigation_log:
                # 使用当前时间而不是消息时间戳，因为消息时间戳可能有问题
                current_time = datetime.now()
                timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S")

                # 将数字级别转换为英文级别
                level_name = self.level_names.get(level, f"LEVEL_{level}")

                log_line = f"[{timestamp}] [{level_name}] [{name}] {msg}\n"
                self.write_to_log(log_line)
                self.log_count += 1

                # 每10条日志显示一次统计信息
                if self.log_count % 100 == 0:
                    rospy.loginfo(
                        f"[NavigationLogger] 已收集 {self.log_count} 条导航日志"
                    )

        except Exception as e:
            rospy.logwarn(f"[NavigationLogger] 处理日志消息时出错: {e}")

    def write_to_log(self, message):
        """写入日志文件"""
        if self.shutdown_flag:
            return

        try:
            # 使用追加模式，每次都打开和关闭文件
            with open(self.log_file_path, "a") as f:
                f.write(message)
                f.flush()
        except Exception as e:
            rospy.logerr(f"[NavigationLogger] 写入日志文件错误: {e}")

    def signal_handler(self, signum, frame):
        """信号处理函数"""
        if not self.shutdown_flag:
            self.shutdown_flag = True
            rospy.loginfo(
                f"[NavigationLogger] 收到信号 {signum}，正在关闭日志收集器..."
            )
            # 写入结束标记
            self.write_to_log(
                f"\n=== Navigation System Log Ended at {datetime.now()} ===\n"
            )
            self.write_to_log(f"=== 总共收集了 {self.log_count} 条导航日志 ===\n")
            sys.exit(0)

    def run(self):
        """主运行循环"""
        try:
            # 写入开始标记
            self.write_to_log(
                f"=== Navigation System Log Started at {datetime.now()} ===\n"
            )
            self.write_to_log(f"=== 日志收集器: NavigationLogger ===\n")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[NavigationLogger] 收到中断信号，正在关闭...")
        except Exception as e:
            rospy.logerr(f"[NavigationLogger] 运行时错误: {e}")
        finally:
            if not self.shutdown_flag:
                self.write_to_log(
                    f"\n=== Navigation System Log Ended at {datetime.now()} ===\n"
                )
                self.write_to_log(f"=== 总共收集了 {self.log_count} 条导航日志 ===\n")


if __name__ == "__main__":
    try:
        logger = NavigationLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[NavigationLogger] 启动失败: {e}")
