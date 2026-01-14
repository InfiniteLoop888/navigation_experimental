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

import threading
import os
import time
import traceback
from collections import deque
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import yaml
import rospy
import rosbag
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
            # 确保包含 /rosout 用于自动保存触发
            if "/rosout" not in self.topics_to_record:
                self.topics_to_record.append("/rosout")
                rospy.loginfo("自动添加 /rosout 到订阅列表")
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
        self.single_message_buffer = (
            {}
        )  # 存储只有一个消息的话题 {topic: (msg, timestamp)}
        self.buffer_lock = threading.Lock()

        # 录制开始时间（使用第一个话题的时间）
        self.recording_start_time = None

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

        # 上次自动保存时间
        self.last_auto_save_time = 0
        self.auto_save_interval = 10.0  # 自动保存间隔（秒）

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
        """从YAML文件加载话题列表

        Args:
            filepath: 话题列表YAML文件路径，支持两种格式：
                      1. 简单列表格式: [- /odom, - /cmd_vel]
                      2. 字典格式: {topics: [/odom, /cmd_vel]}

        Returns:
            话题列表
        """
        topics = []
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

                # 如果是列表格式
                if isinstance(data, list):
                    topics = [str(topic).strip() for topic in data if topic]
                # 如果是字典格式，尝试获取 topics 键
                elif isinstance(data, dict):
                    if "topics" in data:
                        topics = [
                            str(topic).strip() for topic in data["topics"] if topic
                        ]
                    else:
                        rospy.logwarn("YAML文件中未找到 'topics' 键，尝试使用所有值")
                        # 尝试将所有值作为话题
                        for _, value in data.items():
                            if isinstance(value, list):
                                topics.extend(
                                    [str(topic).strip() for topic in value if topic]
                                )
                            elif isinstance(value, str) and value:
                                topics.append(value.strip())
                # 如果是None或空文件
                elif data is None:
                    rospy.logwarn("YAML文件为空")
                else:
                    rospy.logwarn("不支持的YAML格式: {}".format(type(data)))

            # 过滤空字符串和注释
            topics = [topic for topic in topics if topic and not topic.startswith("#")]

            rospy.loginfo("从 {} 加载了 {} 个话题".format(filepath, len(topics)))
            return topics

        except yaml.YAMLError as e:
            rospy.logerr("YAML解析失败: {}".format(str(e)))
            return []
        except (IOError, OSError) as e:
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
            except Exception as e:  # pylint: disable=broad-except
                rospy.logerr("订阅话题 {} 失败: {}".format(topic, str(e)))
                rospy.logerr("异常堆栈: {}".format(traceback.format_exc()))

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
            except Exception as ex:  # pylint: disable=broad-except
                # 在等待话题出现的过程中，可能会遇到各种异常（如话题类型变化等），记录日志以便排查
                rospy.logdebug("等待话题 {} 时出现异常: {}".format(topic, str(ex)))
                rospy.logdebug("异常堆栈: {}".format(traceback.format_exc()))
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

            except Exception as e:  # pylint: disable=broad-except
                rospy.logwarn("发现话题时出错: {}".format(str(e)))
                rospy.logwarn("异常堆栈: {}".format(traceback.format_exc()))

            rate.sleep()

    def subscribe_to_topic(self, topic_name, topic_type):
        """订阅指定话题"""
        try:
            # 使用rostopic.get_topic_class替代rospy.get_message_class
            msg_class, _, _ = rostopic.get_topic_class(topic_name, blocking=False)
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
        except Exception as e:  # pylint: disable=broad-except
            rospy.logwarn("订阅话题 {} 失败: {}".format(topic_name, str(e)))
            rospy.logwarn("异常堆栈: {}".format(traceback.format_exc()))

    def message_callback(self, msg, topic):
        """消息回调函数"""
        current_time = rospy.Time.now()

        with self.buffer_lock:
            # 添加消息到缓冲区
            self.message_buffer.append((topic, msg, current_time))

            # 如果是第一个话题，设置录制开始时间（只设置一次）
            if self.recording_start_time is None:
                self.recording_start_time = current_time

            # 限制缓冲区大小（防止内存溢出）
            if len(self.message_buffer) > 100000:  # 最多10万条消息
                self.message_buffer.popleft()

        # 检查 /rosout 消息触发自动保存
        if topic == "/rosout":
            try:
                # 获取日志内容，兼容 msg.msg 和 str(msg)
                log_content = getattr(msg, "msg", str(msg))
                if "Solution not found" in log_content:
                    now_sec = current_time.to_sec()
                    if now_sec - self.last_auto_save_time > self.auto_save_interval:
                        self.last_auto_save_time = now_sec
                        rospy.loginfo("检测到 'Solution not found'，触发自动保存")
                        # 启动线程进行保存，避免阻塞回调
                        save_thread = threading.Thread(
                            target=self.save_buffer_callback, args=(None,)
                        )
                        save_thread.daemon = True
                        save_thread.start()
            except Exception as e:  # pylint: disable=broad-except
                rospy.logwarn("自动保存检查失败: {}".format(str(e)))

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
                    for topic, _, _ in self.message_buffer:
                        topic_counts[topic] = topic_counts.get(topic, 0) + 1

                    # 从队列前端删除过期消息，将单消息话题移到专门缓冲区
                    while self.message_buffer:
                        topic, msg, timestamp = self.message_buffer[0]
                        if timestamp >= cutoff_time:
                            # 未过期的消息，停止删除
                            break
                        elif topic_counts.get(topic, 0) <= 1:
                            # 过期的消息，但如果该话题只有一条消息，移到单消息缓冲区
                            self.message_buffer.popleft()
                            # 直接存储或更新该话题（字典会自动覆盖）
                            self.single_message_buffer[topic] = (msg, timestamp)
                        else:
                            # 过期的多消息话题，删除第一条消息
                            self.message_buffer.popleft()
                            topic_counts[topic] -= 1

                    # 获取状态信息（在锁内）
                    buffer_size = len(self.message_buffer)
                    single_buffer_size = len(self.single_message_buffer)
                    total_buffer_size = buffer_size + single_buffer_size

                    # 计算最旧消息年龄（只考虑多消息话题）
                    multi_message_topics = []
                    for topic, _, timestamp in self.message_buffer:
                        if topic_counts.get(topic, 0) > 1:
                            multi_message_topics.append(timestamp)

                    if multi_message_topics:
                        oldest_age = (current_time - min(multi_message_topics)).to_sec()
                    else:
                        oldest_age = 0.0  # 没有多消息话题

                    # 统计只有1条消息的话题数量
                    single_message_topics = sum(
                        1 for count in topic_counts.values() if count == 1
                    ) + len(
                        self.single_message_buffer
                    )  # 加上单消息缓冲区中的话题
                    total_topics = len(topic_counts) + len(self.single_message_buffer)

                # 发布状态（在锁外）
                # 计算录制时长（使用第一个话题的时间）
                recording_duration = (current_time - self.recording_start_time).to_sec()

                if total_buffer_size > 0:
                    status_msg = "录制时长: {:.1f}秒 | 缓冲区: {} 条消息 (多消息: {}, 单消息: {}) | \
                        最旧: {:.1f}秒前 | 话题: {}个 (单条消息: {}个)".format(
                        recording_duration,
                        total_buffer_size,
                        buffer_size,
                        single_buffer_size,
                        oldest_age,
                        total_topics,
                        single_message_topics,
                    )
                else:
                    status_msg = "录制时长: {:.1f}秒 | 缓冲区: 空".format(
                        recording_duration
                    )

                rospy.loginfo(status_msg)

            except Exception as e:  # pylint: disable=broad-except
                rospy.logerr("清理消息时出错: {}".format(str(e)))
                rospy.logerr("异常堆栈: {}".format(traceback.format_exc()))

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
                if (
                    len(self.message_buffer) == 0
                    and len(self.single_message_buffer) == 0
                ):
                    return TriggerResponse(
                        success=False, message="缓冲区为空，无法保存"
                    )

                # 统计话题信息
                topic_counts = {}
                for topic, _, _ in self.message_buffer:
                    topic_counts[topic] = topic_counts.get(topic, 0) + 1

                total_topics = len(topic_counts) + len(self.single_message_buffer)

                # 计算最旧消息年龄（只考虑多消息话题）
                current_time = rospy.Time.now()
                multi_message_topics = []
                for topic, _, timestamp in self.message_buffer:
                    if topic_counts.get(topic, 0) > 1:
                        multi_message_topics.append(timestamp)

                if multi_message_topics:
                    oldest_age = (current_time - min(multi_message_topics)).to_sec()
                else:
                    oldest_age = 0.0  # 没有多消息话题

                # 创建rosbag
                bag = rosbag.Bag(bag_filename, "w")
                all_messages = []
                try:
                    # 写入所有消息（多消息缓冲区和单消息缓冲区）
                    for topic, msg, timestamp in self.message_buffer:
                        all_messages.append((topic, msg, timestamp))

                    earliest_time = self.message_buffer[0][2]
                    for topic, (
                        msg,
                        _,
                    ) in self.single_message_buffer.items():
                        all_messages.append((topic, msg, earliest_time))

                    all_messages.sort(key=lambda x: x[2])

                    message_count = len(all_messages)
                    for topic, msg, timestamp in all_messages:
                        bag.write(topic, msg, timestamp)
                finally:
                    bag.close()

            success_msg_cn = (
                "已保存 {} 条消息到 {}, 录制时长: {:.1f}秒, 话题: {}个".format(
                    message_count,
                    bag_filename,
                    oldest_age,
                    total_topics,
                )
            )
            success_msg_en = "Saved to {} ({} msgs, {:.1f}s, {} topics)".format(
                bag_filename,  # 只显示文件名，不显示完整路径
                message_count,
                oldest_age,
                total_topics,
            )
            rospy.loginfo(success_msg_cn)
            return TriggerResponse(success=True, message=success_msg_en)

        except Exception as e:  # pylint: disable=broad-except
            error_msg_cn = "保存失败: {}".format(str(e))
            error_msg_en = "Save failed: {}".format(str(e))
            rospy.logerr(error_msg_cn)
            rospy.logerr("异常堆栈: {}".format(traceback.format_exc()))
            return TriggerResponse(success=False, message=error_msg_en)

    def clear_buffer_callback(self, req):
        """清空缓冲区"""
        try:
            with self.buffer_lock:
                count = len(self.message_buffer) + len(self.single_message_buffer)
                self.message_buffer.clear()
                self.single_message_buffer.clear()

            msg_cn = "已清空 {} 条消息".format(count)
            msg_en = "Cleared {} messages".format(count)
            rospy.loginfo(msg_cn)
            return TriggerResponse(success=True, message=msg_en)

        except Exception as e:  # pylint: disable=broad-except
            error_msg_cn = "清空缓冲区失败: {}".format(str(e))
            error_msg_en = "Clear buffer failed: {}".format(str(e))
            rospy.logerr(error_msg_cn)
            rospy.logerr("异常堆栈: {}".format(traceback.format_exc()))
            return TriggerResponse(success=False, message=error_msg_en)

    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RosRecordingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
