#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS消息记录节点
功能：
1. 持续记录过去30秒的ROS消息
2. 自动删除超过30秒的消息
3. 提供保存功能（保存为rosbag）
注意：回放功能由独立的replay_player_node.py提供
"""

import rospy
import rosbag
import threading
import os
import time
from collections import deque
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import rostopic


class RosRecordingNode:
    def __init__(self):
        rospy.init_node("ros_recording_node", anonymous=False)

        # 参数配置
        self.buffer_duration = rospy.get_param(
            "~buffer_duration", 30.0
        )  # 缓冲时长（秒）

        # 话题列表文件路径
        self.topics_file = rospy.get_param("~topics_file", "")

        # 录制模式
        self.record_all = rospy.get_param("~record_all", True)

        # 从文件或参数读取话题列表
        if self.record_all:
            # 录制所有话题模式
            self.topics_to_record = []
            rospy.loginfo("录制模式: 所有话题")
            # 启动话题发现线程
            self.topic_discovery_thread = threading.Thread(target=self.discover_topics)
            self.topic_discovery_thread.daemon = True
            self.topic_discovery_thread.start()
        elif self.topics_file and os.path.exists(self.topics_file):
            self.topics_to_record = self.load_topics_from_file(self.topics_file)
            rospy.loginfo("从文件加载话题列表: {}".format(self.topics_file))
        else:
            # 如果没有指定话题文件，使用默认话题列表
            self.topics_to_record = ["/rosout", "/tf"]
            if self.topics_file:
                rospy.logwarn(
                    "话题文件不存在: {}，使用默认话题列表".format(self.topics_file)
                )

        self.save_directory = rospy.get_param(
            "~save_directory", "/tmp/ros_replay"
        )  # 保存目录

        # 创建保存目录
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        # 消息缓冲区：存储(topic, msg, timestamp)
        self.message_buffer = deque()
        self.buffer_lock = threading.Lock()

        # 订阅者字典
        self.subscribers = {}

        # 服务
        self.save_service = rospy.Service(
            "~save_buffer", Trigger, self.save_buffer_callback
        )
        self.clear_service = rospy.Service(
            "~clear_buffer", Trigger, self.clear_buffer_callback
        )

        # 状态发布者
        self.status_pub = rospy.Publisher("~status", String, queue_size=10)

        # 启动订阅
        if not self.record_all:
            self.setup_subscribers()

        # 启动清理线程
        self.cleanup_thread = threading.Thread(target=self.cleanup_old_messages)
        self.cleanup_thread.daemon = True
        self.cleanup_thread.start()

        rospy.loginfo("ROS消息记录节点已启动")
        rospy.loginfo("缓冲时长: {} 秒".format(self.buffer_duration))
        if self.record_all:
            rospy.loginfo("录制模式: 所有话题 (动态发现)")
        else:
            rospy.loginfo("记录话题数量: {}".format(len(self.topics_to_record)))
        rospy.loginfo("保存目录: {}".format(self.save_directory))
        rospy.loginfo("提示：使用 replay_player_node.py 来回放保存的bag文件")

    def load_topics_from_file(self, filepath):
        """从文件加载话题列表

        Args:
            filepath: 话题列表文件路径，每行一个话题

        Returns:
            话题列表
        """
        topics = []
        try:
            with open(filepath, "r") as f:
                for line in f:
                    # 去除空白字符和注释
                    line = line.strip()
                    if line and not line.startswith("#"):
                        topics.append(line)

            rospy.loginfo("从 {} 加载了 {} 个话题".format(filepath, len(topics)))
            return topics

        except Exception as e:
            rospy.logerr("读取话题文件失败: {}".format(str(e)))
            return []

    def setup_subscribers(self):
        """设置话题订阅者"""
        for topic in self.topics_to_record:
            try:
                # 获取话题类型
                msg_class, _, _ = rostopic.get_topic_class(topic, blocking=True)
                if msg_class is None:
                    rospy.logwarn(
                        "话题 {} 不存在，将在话题出现时自动订阅".format(topic)
                    )
                    # 启动一个线程来等待话题出现
                    threading.Thread(target=self.wait_for_topic, args=(topic,)).start()
                else:
                    # 创建订阅者
                    self.subscribers[topic] = rospy.Subscriber(
                        topic, msg_class, self.message_callback, callback_args=topic
                    )
                    rospy.loginfo("已订阅话题: {}".format(topic))
            except Exception as e:
                rospy.logerr("订阅话题 {} 失败: {}".format(topic, str(e)))

    def wait_for_topic(self, topic):
        """等待话题出现并订阅"""
        rate = rospy.Rate(1)  # 1Hz检查
        while not rospy.is_shutdown() and topic not in self.subscribers:
            try:
                msg_class, _, _ = rostopic.get_topic_class(topic, blocking=False)
                if msg_class is not None:
                    self.subscribers[topic] = rospy.Subscriber(
                        topic, msg_class, self.message_callback, callback_args=topic
                    )
                    rospy.loginfo("已订阅话题: {}".format(topic))
                    break
            except:
                pass
            rate.sleep()

    def discover_topics(self):
        """发现并订阅所有话题"""
        rate = rospy.Rate(2)  # 2Hz检查新话题
        known_topics = set()

        while not rospy.is_shutdown():
            try:
                # 获取当前所有话题
                current_topics = rospy.get_published_topics()

                for topic_name, topic_type in current_topics:
                    if topic_name not in known_topics:
                        known_topics.add(topic_name)
                        self.subscribe_to_topic(topic_name, topic_type)

            except Exception as e:
                rospy.logwarn("发现话题时出错: {}".format(str(e)))

            rate.sleep()

    def subscribe_to_topic(self, topic_name, topic_type):
        """订阅指定话题"""
        try:
            # 获取消息类型
            msg_class = rospy.get_message_class(topic_type)
            if msg_class is not None:
                # 创建订阅者
                self.subscribers[topic_name] = rospy.Subscriber(
                    topic_name,
                    msg_class,
                    self.message_callback,
                    callback_args=topic_name,
                )
                rospy.loginfo("已订阅话题: {} ({})".format(topic_name, topic_type))
            else:
                rospy.logwarn(
                    "无法获取话题 {} 的消息类型: {}".format(topic_name, topic_type)
                )
        except Exception as e:
            rospy.logwarn("订阅话题 {} 失败: {}".format(topic_name, str(e)))

    def message_callback(self, msg, topic):
        """消息回调函数"""
        current_time = rospy.Time.now()

        with self.buffer_lock:
            # 添加消息到缓冲区
            self.message_buffer.append((topic, msg, current_time))

            # 限制缓冲区大小（防止内存溢出）
            if len(self.message_buffer) > 100000:  # 最多10万条消息
                self.message_buffer.popleft()

    def cleanup_old_messages(self):
        """清理超过缓冲时长的旧消息"""
        rate = rospy.Rate(10)  # 10Hz清理频率

        # 统计每个话题的消息数量
        topic_counts = {}

        while not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now()
                cutoff_time = current_time - rospy.Duration(self.buffer_duration)

                with self.buffer_lock:
                    # 统计当前缓冲区中每个话题的消息数量
                    topic_counts.clear()
                    for _, topic, _ in self.message_buffer:
                        topic_counts[topic] = topic_counts.get(topic, 0) + 1

                    # 从队列前端删除过期消息，但保留只有一条消息的话题
                    while (
                        self.message_buffer and self.message_buffer[0][2] < cutoff_time
                    ):
                        topic = self.message_buffer[0][0]
                        # 如果该话题只有一条消息，跳过删除
                        if topic_counts.get(topic, 0) <= 1:
                            break
                        self.message_buffer.popleft()
                        # 更新计数
                        if topic in topic_counts:
                            topic_counts[topic] -= 1

                # 发布状态
                buffer_size = len(self.message_buffer)
                if buffer_size > 0:
                    oldest_age = (current_time - self.message_buffer[0][2]).to_sec()
                    status_msg = "缓冲区: {} 条消息, 最旧: {:.1f}秒".format(
                        buffer_size, oldest_age
                    )
                else:
                    status_msg = "缓冲区: 空"

                self.status_pub.publish(String(data=status_msg))

            except Exception as e:
                rospy.logerr("清理消息时出错: {}".format(str(e)))

            rate.sleep()

    def save_buffer_callback(self, req):
        """保存缓冲区到rosbag文件"""
        try:
            # 生成文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            bag_filename = os.path.join(
                self.save_directory, "replay_{}.bag".format(timestamp)
            )

            with self.buffer_lock:
                if len(self.message_buffer) == 0:
                    return TriggerResponse(
                        success=False, message="缓冲区为空，无法保存"
                    )

                # 创建rosbag
                bag = rosbag.Bag(bag_filename, "w")

                try:
                    # 写入所有消息
                    for topic, msg, timestamp in self.message_buffer:
                        bag.write(topic, msg, timestamp)

                    message_count = len(self.message_buffer)

                finally:
                    bag.close()

            success_msg = "已保存 {} 条消息到 {}".format(message_count, bag_filename)
            rospy.loginfo(success_msg)
            return TriggerResponse(success=True, message=success_msg)

        except Exception as e:
            error_msg = "保存失败: {}".format(str(e))
            rospy.logerr(error_msg)
            return TriggerResponse(success=False, message=error_msg)

    def clear_buffer_callback(self, req):
        """清空缓冲区"""
        try:
            with self.buffer_lock:
                count = len(self.message_buffer)
                self.message_buffer.clear()

            msg = "已清空 {} 条消息".format(count)
            rospy.loginfo(msg)
            return TriggerResponse(success=True, message=msg)

        except Exception as e:
            error_msg = "清空缓冲区失败: {}".format(str(e))
            rospy.logerr(error_msg)
            return TriggerResponse(success=False, message=error_msg)

    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RosRecordingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
