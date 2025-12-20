#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
靠近目标点脚本 (增强版)
功能：
1. 模式1: 给定目标点 (x, y)。检查机器人到该点的路径，若有障碍则停在障碍前。
2. 模式2: 给定起点 (x, y) 和方向 yaw。沿该方向发射射线，若check_dist内检测到障碍，则以最远障碍外 stop_dist 为目标。
3. 支持设置朝向策略：面向障碍物 或 背对障碍物。
4. 支持垂直于障碍物表面或保持连线方向。

用法：
    from approach import approach
    # 模式1: 尝试去 (5, 2)
    approach(5.0, 2.0)

    # 模式2: 从 (5, 2) 向东 (yaw=0) 探测，停在检测到的最远障碍物后方（远离起点）
    approach(5.0, 2.0, yaw=0.0, check_dist=2.0, facing_obstacle=True)
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_srvs.srv import Empty
import math
import sys
import numpy as np

# 全局单例实例
_sender_instance = None


class RobotGoalSender:
    def __init__(self):
        # 检查节点是否已初始化，如果没有则初始化
        if rospy.get_name() == "/unnamed":
            rospy.init_node("robot_goal_sender", anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.local_costmap_sub = rospy.Subscriber(
            "/move_base/local_costmap/costmap", OccupancyGrid, self.local_costmap_cb
        )
        self.local_costmap = None

        self.global_costmap_sub = rospy.Subscriber(
            "/move_base/global_costmap/costmap", OccupancyGrid, self.global_costmap_cb
        )
        self.global_costmap = None

        self.status_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_cb
        )
        self.latest_status = None

        # 等待 costmap 数据
        rospy.sleep(0.5)

    def status_cb(self, msg):
        if msg.status_list:
            # 获取最后一个目标的状态
            # 注意：status_list 可能包含多个 goal 的状态，我们需要找到我们刚发的那个
            # 简单起见，取最后一个
            status = msg.status_list[-1].status
            # 状态码: 1=ACTIVE, 3=SUCCEEDED, 4=ABORTED
            if status == GoalStatus.SUCCEEDED:
                self.latest_status = status
            elif status == GoalStatus.ACTIVE:
                self.latest_status = status

    def local_costmap_cb(self, msg):
        self.local_costmap = msg

    def global_costmap_cb(self, msg):
        self.global_costmap = msg

    def get_transform(self, target_frame, source_frame):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logerr("获取TF失败 (%s -> %s): %s", target_frame, source_frame, e)
            return None

    def transform_point(self, x, y, transform):
        """转换 2D 点"""
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        rx = transform.transform.rotation.x
        ry = transform.transform.rotation.y
        rz = transform.transform.rotation.z
        rw = transform.transform.rotation.w

        yaw = math.atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz))

        final_x = x * math.cos(yaw) - y * math.sin(yaw) + tx
        final_y = x * math.sin(yaw) + y * math.cos(yaw) + ty
        return final_x, final_y

    def get_tf_yaw(self, transform):
        q = transform.transform.rotation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def check_approach(self, x, y, max_dist=2.0, angle_range=90.0):
        """
        检查点 (x, y) 是否在机器人 base_link 坐标面前指定角度范围内、指定距离内
        
        :param x: 目标点 x 坐标 (Map Frame)
        :param y: 目标点 y 坐标 (Map Frame)
        :param max_dist: 最大距离（米），默认 2.0
        :param angle_range: 角度范围（度），默认 90.0（即 ±90 度，前方 180 度扇形）
        :return: bool，如果在范围内返回 True，否则返回 False
        """
        # 获取 base_link 到 map 的变换（用于反向计算）
        base_to_map_tf = self.get_transform('map', 'base_link')
        if not base_to_map_tf:
            rospy.logwarn("无法获取 base_link -> map 变换")
            return False
        
        # 提取 base_link 在 map 中的位置和朝向
        robot_x = base_to_map_tf.transform.translation.x
        robot_y = base_to_map_tf.transform.translation.y
        robot_yaw = self.get_tf_yaw(base_to_map_tf)
        
        # 将点从 map 坐标系转换到 base_link 坐标系（反向变换）
        # 先平移到以机器人为原点
        dx = x - robot_x
        dy = y - robot_y
        
        # 再旋转到 base_link 坐标系（反向旋转）
        base_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        base_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
        
        # 计算距离
        dist = math.sqrt(base_x * base_x + base_y * base_y)
        
        # 计算角度（相对于 base_link 的 x 轴正方向，即机器人前方）
        angle_rad = math.atan2(base_y, base_x)
        angle_deg = math.degrees(angle_rad)
        
        # 检查是否在距离范围内
        if dist > max_dist:
            rospy.logwarn("check_approach: 目标点距离机器人超过 %d 米", max_dist)
            return False
        
        # 检查是否在角度范围内（±angle_range/2 度）
        angle_threshold = angle_range / 2.0
        if abs(angle_deg) > angle_threshold:
            rospy.logwarn("check_approach: 目标点角度超过 %d 度", angle_threshold)
            return False
        
        rospy.loginfo("check_approach: 目标点在机器人前方 %d 米、±%d 度范围内", dist, angle_threshold)
        return True

    def get_gradient_yaw(self, costmap, gx, gy):
        """计算 (gx, gy) 处最外层边界的梯度方向 (指向障碍物)"""
        width = costmap.info.width
        height = costmap.info.height
        data = np.array(costmap.data, dtype=np.int8).reshape((height, width))

        def get_val(x, y):
            if x < 0 or x >= width or y < 0 or y >= height:
                return 0
            v = int(data[y][x])
            return 0 if v == -1 else v  # 未知视为0

        # gx,gy本身就在最外层边界，直接使用它计算梯度
        x_left = max(0, gx - 1)
        x_right = min(width - 1, gx + 1)
        y_down = max(0, gy - 1)
        y_up = min(height - 1, gy + 1)

        val_left = get_val(x_left, gy)
        val_right = get_val(x_right, gy)
        val_down = get_val(gx, y_down)
        val_up = get_val(gx, y_up)

        grad_x = val_right - val_left
        grad_y = val_up - val_down

        if grad_x == 0 and grad_y == 0:
            return None

        return math.atan2(grad_y, grad_x)

    def check_obstacle_in_costmap(
        self, data, width, height, resolution, origin_x, origin_y, px, py
    ):
        """检查点(px, py)在costmap中是否有障碍物
        data: 预处理好的numpy数组 (height, width)
        """
        gx = int((px - origin_x) / resolution)
        gy = int((py - origin_y) / resolution)

        if 0 <= gx < width and 0 <= gy < height:
            val = data[gy][gx]
            return val >= 10  # 致命障碍
        return False

    def execute(
        self,
        target_wx,
        target_wy,
        yaw=None,
        stop_dist=0.3,
        check_dist=2.0,
        facing_obstacle=True,
        vertical=False,
    ):
        rospy.loginfo("开始执行接近任务 (持续监测模式)")
        rate = rospy.Rate(0.5)

        last_sent_x = None
        last_sent_y = None
        last_sent_yaw = None

        # 阈值：位置差异小于 0.1m 且 角度差异小于 0.1 rad 不更新
        pos_threshold = 0.2
        yaw_threshold = 0.1

        # 清除之前的状态
        self.latest_status = None

        while not rospy.is_shutdown():
            if self.latest_status == GoalStatus.SUCCEEDED:
                rospy.loginfo("已到达目标点，任务完成")
                break

            if self.local_costmap is None or self.global_costmap is None:
                rospy.logwarn_throttle(2.0, "等待 Costmap...")
                rate.sleep()
                continue

            # 使用local costmap的frame_id和分辨率（通常local和global使用相同的frame_id）
            local_costmap = self.local_costmap
            global_costmap = self.global_costmap
            frame_id = local_costmap.header.frame_id  # usually odom
            resolution = local_costmap.info.resolution

            # 1. 准备坐标转换
            map_to_costmap_tf = self.get_transform(frame_id, "map")
            if not map_to_costmap_tf:
                rospy.logerr("无法获取 map -> %s 变换", frame_id)
                rate.sleep()
                continue

            # 转换输入点到 Costmap Frame
            start_x, start_y = self.transform_point(
                target_wx, target_wy, map_to_costmap_tf
            )

            ray_start_x = 0.0
            ray_start_y = 0.0
            ray_angle = 0.0
            max_ray_len = 0.0

            # 2. 确定射线参数（统一从目标点发出）
            ray_start_x = start_x
            ray_start_y = start_y

            if yaw is not None:
                # 模式2: 从目标点沿指定方向(yaw)发射射线
                tf_yaw_offset = self.get_tf_yaw(map_to_costmap_tf)
                ray_angle = yaw + tf_yaw_offset
                max_ray_len = check_dist
            else:
                # 模式1: 从目标点指向机器人当前位置
                robot_tf = self.get_transform(frame_id, "base_link")
                if not robot_tf:
                    rate.sleep()
                    continue

                rx = robot_tf.transform.translation.x
                ry = robot_tf.transform.translation.y

                dx = rx - start_x
                dy = ry - start_y
                max_ray_len = math.sqrt(dx * dx + dy * dy)
                ray_angle = math.atan2(dy, dx)

            # 3. 射线检测（同时检查local和global costmap）
            steps = int(max_ray_len / (resolution * 0.5))
            if steps == 0:
                steps = 1
            # 限制最大步数，避免过长射线导致性能问题
            max_steps = 10000
            if steps > max_steps:
                rospy.logwarn("射线长度过长，限制步数从 %d 到 %d", steps, max_steps)
                steps = max_steps

            # 预处理costmap数据（避免在循环中重复创建数组）
            local_width = local_costmap.info.width
            local_height = local_costmap.info.height
            local_resolution = local_costmap.info.resolution
            local_origin_x = local_costmap.info.origin.position.x
            local_origin_y = local_costmap.info.origin.position.y
            local_data = np.array(local_costmap.data, dtype=np.int8).reshape(
                (local_height, local_width)
            )

            global_width = global_costmap.info.width
            global_height = global_costmap.info.height
            global_resolution = global_costmap.info.resolution
            global_origin_x = global_costmap.info.origin.position.x
            global_origin_y = global_costmap.info.origin.position.y
            global_data = np.array(global_costmap.data, dtype=np.int8).reshape(
                (global_height, global_width)
            )

            # 射线宽度检测参数
            ray_width = 0.1  # 射线宽度（米）
            # 计算垂直于射线方向的单位向量（用于宽度检测）
            perp_angle = ray_angle + math.pi / 2.0
            perp_dx = math.cos(perp_angle)
            perp_dy = math.sin(perp_angle)
            # 计算宽度方向需要检查的步数（使用较小的分辨率以确保覆盖）
            width_resolution = min(local_resolution, global_resolution)
            width_steps = int(ray_width / (2.0 * width_resolution)) + 1

            hit_obstacle = False
            hit_x = 0.0
            hit_y = 0.0
            hit_gx = 0
            hit_gy = 0

            for i in range(steps):
                ratio = i / float(steps)
                dist_current = max_ray_len * ratio

                # 射线中心点
                px_center = ray_start_x + dist_current * math.cos(ray_angle)
                py_center = ray_start_y + dist_current * math.sin(ray_angle)

                # 在垂直于射线的方向上检查宽度范围内的点
                found_obstacle = False
                for w in range(-width_steps, width_steps + 1):
                    offset = w * width_resolution
                    px = px_center + offset * perp_dx
                    py = py_center + offset * perp_dy

                    # 同时检查local和global costmap（使用预处理好的数据）
                    local_obstacle = self.check_obstacle_in_costmap(
                        local_data,
                        local_width,
                        local_height,
                        local_resolution,
                        local_origin_x,
                        local_origin_y,
                        px,
                        py,
                    )
                    global_obstacle = self.check_obstacle_in_costmap(
                        global_data,
                        global_width,
                        global_height,
                        global_resolution,
                        global_origin_x,
                        global_origin_y,
                        px,
                        py,
                    )

                    if local_obstacle or global_obstacle:
                        found_obstacle = True
                        break

                if found_obstacle:
                    hit_obstacle = True
                    # 记录最远障碍物（循环会继续，hit_x/hit_y 会更新为更远的点）
                    # 使用射线中心线上的点作为击中点
                    hit_x = px_center
                    hit_y = py_center
                    # 使用local costmap的坐标来计算hit_gx和hit_gy（用于梯度计算）
                    hit_gx = int((px_center - local_origin_x) / local_resolution)
                    hit_gy = int((py_center - local_origin_y) / local_resolution)

            # 4. 计算最终目标点 (Costmap Frame)
            final_x = 0.0
            final_y = 0.0
            final_valid = False

            if hit_obstacle:
                # 统一计算：从目标点发出射线，遇到障碍物后停在障碍物之后（远离目标点方向）
                final_x = hit_x + stop_dist * math.cos(ray_angle)
                final_y = hit_y + stop_dist * math.sin(ray_angle)
                final_valid = True
            else:
                # 无论是模式1还是模式2，如果没检测到障碍物，都直接去原始目标点
                # 对于模式2，这意味着如果 check_dist 内无障碍，则认为目标点安全或直接前往目标
                final_x = start_x
                final_y = start_y
                final_valid = True

            if not final_valid:
                rate.sleep()
                continue

            # 计算朝向
            target_yaw = ray_angle

            if hit_obstacle and vertical:
                obstacle_yaw = self.get_gradient_yaw(local_costmap, hit_gx, hit_gy)
                if obstacle_yaw is not None:
                    target_yaw = obstacle_yaw

            # 当vertical=False时：facing_obstacle=False是面对障碍物，True是背对障碍物
            if not vertical:
                if facing_obstacle:
                    target_yaw += math.pi  # 背对
            else:
                # 当vertical=True时，保持原有逻辑
                if not facing_obstacle:
                    target_yaw += math.pi  # 背对

            # 检查是否需要更新目标
            should_update = False
            if last_sent_x is None:
                should_update = True
            else:
                d_pos = math.sqrt(
                    (final_x - last_sent_x) ** 2 + (final_y - last_sent_y) ** 2
                )
                # 简单角度差
                d_yaw = abs(target_yaw - last_sent_yaw)
                while d_yaw > math.pi:
                    d_yaw -= 2 * math.pi
                d_yaw = abs(d_yaw)

                if d_pos > pos_threshold or d_yaw > yaw_threshold:
                    should_update = True

            # 5. 发布目标 (转换为 Map Frame)
            costmap_to_map_tf = self.get_transform("map", frame_id)

            pub_frame = frame_id
            pub_x = final_x
            pub_y = final_y
            pub_yaw = target_yaw

            if costmap_to_map_tf:
                pub_frame = "map"
                pub_x, pub_y = self.transform_point(final_x, final_y, costmap_to_map_tf)
                tf_yaw = self.get_tf_yaw(costmap_to_map_tf)
                pub_yaw = target_yaw + tf_yaw
            else:
                rospy.logwarn(
                    "无法获取 %s -> map 变换，将使用 %s 发布", frame_id, frame_id
                )

            if should_update:
                goal = PoseStamped()
                goal.header.frame_id = pub_frame
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = pub_x
                goal.pose.position.y = pub_y
                goal.pose.position.z = 0.0

                goal.pose.orientation.z = math.sin(pub_yaw / 2.0)
                goal.pose.orientation.w = math.cos(pub_yaw / 2.0)

                self.pub.publish(goal)
                rospy.loginfo(
                    "目标更新 (Frame: %s): (%.2f, %.2f) Yaw: %.2f",
                    pub_frame,
                    pub_x,
                    pub_y,
                    pub_yaw,
                )

                # 发布目标后调用/move_base/clear_costmaps服务
                try:
                    rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
                    clear_costmaps = rospy.ServiceProxy(
                        "/move_base/clear_costmaps", Empty
                    )
                    clear_costmaps()
                    rospy.loginfo("调用/move_base/clear_costmaps服务成功")
                except rospy.ROSException as e:
                    rospy.logwarn("等待/move_base/clear_costmaps服务超时: %s", str(e))
                except Exception as e:
                    rospy.logwarn("调用/move_base/clear_costmaps服务出错: %s", str(e))

                last_sent_x = (
                    final_x  # 依然使用 Costmap 坐标做阈值判断比较方便（因为都在动）
                )
                last_sent_y = final_y
                last_sent_yaw = target_yaw

                # 每次发布新目标后，重置状态为 ACTIVE，这样才能正确捕捉到下一次的 SUCCEEDED
                self.latest_status = GoalStatus.ACTIVE

                # break

            rate.sleep()


def approach(
    x, y, yaw=None, stop_dist=0.3, check_dist=2.0, facing_obstacle=True, vertical=False
):
    """
    外部调用接口
    :param x: 目标 x (Map Frame)
    :param y: 目标 y (Map Frame)
    :param yaw: 射线方向 (Map Frame, 弧度)。若为None，则使用机器人到(x,y)的连线。
    :param stop_dist: 遇障停止距离
    :param check_dist: 射线检测长度 (仅当 yaw 不为 None 时有效)
    :param facing_obstacle: True=面向障碍物, False=背对障碍物
    :param vertical: 是否垂直于障碍物表面 (True) 或 保持连线方向 (False)
    """
    global _sender_instance
    if _sender_instance is None:
        _sender_instance = RobotGoalSender()

    _sender_instance.execute(
        x, y, yaw, stop_dist, check_dist, facing_obstacle, vertical
    )


def check_approach(x, y, max_dist=2.0, angle_range=90.0):
    """
    检查点 (x, y) 是否在机器人 base_link 坐标面前指定角度范围内、指定距离内
    
    :param x: 目标点 x 坐标 (Map Frame)
    :param y: 目标点 y 坐标 (Map Frame)
    :param max_dist: 最大距离（米），默认 2.0
    :param angle_range: 角度范围（度），默认 90.0（即 ±90 度，前方 180 度扇形）
    :return: bool，如果在范围内返回 True，否则返回 False
    
    示例：
        from approach import check_approach
        if check_approach(5.0, 2.0):
            print("目标点在机器人前方 2m、±90 度范围内")
    """
    global _sender_instance
    if _sender_instance is None:
        _sender_instance = RobotGoalSender()
    
    return _sender_instance.check_approach(x, y, max_dist, angle_range)


def approach_forward(
    distance=1.0,
    stop_dist=0.3,
    check_dist=2.0,
    facing_obstacle=True,
    vertical=False,
    node_name="approach_forward",
    init_node=True,
):
    """
    外部可调用函数：向机器人正前方指定距离处执行approach

    功能：
    1. 获取机器人当前位置和朝向
    2. 计算正前方指定距离的目标点
    3. 调用approach函数执行接近任务

    :param distance: 正前方距离（米），默认1.0米
    :param stop_dist: 遇障停止距离，默认0.3米
    :param check_dist: 射线检测长度，默认2.0米
    :param facing_obstacle: True=面向障碍物, False=背对障碍物，默认True
    :param vertical: 是否垂直于障碍物表面，默认False
    :param node_name: ROS节点名称，默认'approach_forward'
    :param init_node: 是否初始化ROS节点，默认True。如果节点已初始化，设置为False
    :return: None
    """
    # 初始化ROS节点（如果需要）
    if init_node and rospy.get_name() == "/unnamed":
        rospy.init_node(node_name, anonymous=True)

    # 创建TF buffer和listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 等待TF数据
    rospy.sleep(0.5)

    # 获取机器人当前位置和朝向（从base_link到map的变换）
    try:
        transform = tf_buffer.lookup_transform(
            "map", "base_link", rospy.Time(0), rospy.Duration(2.0)
        )
    except Exception as e:
        rospy.logerr("无法获取机器人位置 (map -> base_link): %s" % str(e))
        return

    # 提取位置
    robot_x = transform.transform.translation.x
    robot_y = transform.transform.translation.y

    # 提取朝向（yaw角）
    q = transform.transform.rotation
    robot_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

    # 计算正前方目标点
    target_x = robot_x + distance * math.cos(robot_yaw)
    target_y = robot_y + distance * math.sin(robot_yaw)

    rospy.loginfo(
        "机器人当前位置: (%.2f, %.2f), 朝向: %.2f rad (%.2f deg)",
        robot_x,
        robot_y,
        robot_yaw,
        math.degrees(robot_yaw),
    )
    rospy.loginfo("正前方 %.2f 米目标点: (%.2f, %.2f)", distance, target_x, target_y)

    # 调用approach函数
    approach(
        target_x,
        target_y,
        yaw=None,
        stop_dist=stop_dist,
        check_dist=check_dist,
        facing_obstacle=facing_obstacle,
        vertical=vertical,
    )


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "forward":
        # 模式：向正前方approach
        # python approach.py forward [distance] [stop_dist]
        distance = 1.0
        if len(sys.argv) > 2:
            distance = float(sys.argv[2])

        stop = 0.3
        if len(sys.argv) > 3:
            stop = float(sys.argv[3])

        approach_forward(distance=distance, stop_dist=stop)
    elif len(sys.argv) > 2:
        # 模式：指定坐标approach
        # python approach.py x y [yaw] [stop_dist]
        tx = float(sys.argv[1])
        ty = float(sys.argv[2])

        target_yaw = None
        if len(sys.argv) > 3 and sys.argv[3] != "None":
            # 输入为角度，转换为弧度
            target_yaw = math.radians(float(sys.argv[3]))

        stop = 1.0
        if len(sys.argv) > 4:
            stop = float(sys.argv[4])

        rospy.init_node("approach_test", anonymous=True)
        approach(
            tx,
            ty,
            yaw=target_yaw,
            stop_dist=stop,
            check_dist=2.0,
            facing_obstacle=True,
            vertical=True,
        )
        rospy.sleep(0.5)
    else:
        print("用法:")
        print("  向正前方approach:")
        print("    python approach.py forward [distance] [stop_dist]")
        print("  指定坐标approach:")
        print("    python approach.py x y [yaw] [stop_dist]")
        print("")
        print("外部调用示例:")
        print("  from approach import approach_forward")
        print("  approach_forward(distance=1.0)")
        print("  approach_forward(distance=1.5, stop_dist=0.5)")
