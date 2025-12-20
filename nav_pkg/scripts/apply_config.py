#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
应用 YAML 配置脚本
功能：从 YAML 文件读取参数配置，并应用到 ROS 系统（支持 dynamic_reconfigure）

用法：
    python apply_config.py default
    python apply_config.py narrow
    python apply_config.py narrow --file my_config.yaml
"""

import rospy
import yaml
import sys
import os
import ast
from dynamic_reconfigure.client import Client


def load_yaml(filepath):
    """读取 YAML 文件"""
    if not os.path.exists(filepath):
        rospy.logerr("配置文件不存在: %s" % filepath)
        return None
    try:
        with open(filepath, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr("读取配置文件失败: %s" % str(e))
        return None


def parse_param_path(full_path):
    """
    解析参数路径，分离节点名和参数名
    例如: /move_base/global_costmap/inflation/inflation_radius
    返回: (/move_base/global_costmap/inflation, inflation_radius)
    注意：inflation 是插件层，不支持 dynamic_reconfigure，会使用 rosparam 设置
    """
    if not full_path.startswith("/"):
        full_path = "/" + full_path

    parts = full_path.split("/")
    if len(parts) < 3:
        return None, None

    param_name = parts[-1]
    node_name = "/".join(parts[:-1])
    return node_name, param_name


def check_dynamic_reconfigure_available(node_name, timeout=1.0):
    """
    检查指定节点是否支持 dynamic_reconfigure
    
    :param node_name: 节点名称，例如 '/move_base/local_costmap/inflation'
    注意：插件层（如 inflation）不是独立节点，不支持 dynamic_reconfigure
    :param timeout: 超时时间（秒），默认1.0秒
    :return: bool，如果服务可用返回True，否则返回False
    """
    try:
        service_name = node_name + "/set_parameters"
        rospy.wait_for_service(service_name, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def apply_config(config_name, yaml_file=None, node_name="apply_config_node", init_node=True):
    """
    外部可调用函数：应用指定配置
    
    :param config_name: 配置名称（在YAML文件中的key）
    :param yaml_file: YAML配置文件路径。如果为None，则使用默认路径
    :param node_name: ROS节点名称，默认'apply_config_node'
    :param init_node: 是否初始化ROS节点，默认True。如果节点已初始化，设置为False
    :return: bool，成功返回True，失败返回False
    """
    # 如果未提供yaml_file，使用默认路径
    if yaml_file is None:
        yaml_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "../config/dynamic_params.yaml"
        )
    
    # 如果配置文件不存在，尝试当前目录
    if not os.path.exists(yaml_file):
        current_dir_yaml = os.path.join(os.getcwd(), "dynamic_params.yaml")
        if os.path.exists(current_dir_yaml):
            yaml_file = current_dir_yaml
        else:
            if init_node and rospy.get_name() == '/unnamed':
                rospy.logerr("配置文件不存在: %s" % yaml_file)
            else:
                print("配置文件不存在: %s" % yaml_file)
            return False
    
    # 初始化ROS节点（如果需要）
    if init_node and rospy.get_name() == '/unnamed':
        rospy.init_node(node_name, anonymous=True)
    
    data = load_yaml(yaml_file)
    if not data:
        return False

    if config_name not in data:
        rospy.logerr("在配置文件中找不到配置项: %s" % config_name)
        print("可用配置: %s" % ", ".join(data.keys()))
        return False

    params = data[config_name]
    rospy.loginfo("正在应用配置 '%s'..." % config_name)

    # 按节点分组参数，以便批量更新
    node_params = {}

    for full_path, value in params.items():
        node_name, param_name = parse_param_path(full_path)
        if not node_name:
            rospy.logwarn("无效的参数路径: %s" % full_path)
            continue

        if node_name not in node_params:
            node_params[node_name] = {}

        # 特殊处理 footprint，确保它是字符串格式
        if "footprint" in param_name and isinstance(value, list):
            value = str(value)

        node_params[node_name][param_name] = value

    # 对每个节点执行参数设置
    success_count = 0
    for target_node_name, config_dict in node_params.items():
        rospy.loginfo("更新节点: %s -> %s" % (target_node_name, config_dict))
        
        # 先检查 dynamic_reconfigure 是否可用
        use_dynamic_reconfigure = check_dynamic_reconfigure_available(target_node_name, timeout=1.0)
        
        if use_dynamic_reconfigure:
            try:
                client = Client(target_node_name, timeout=2.0)
                client.update_configuration(config_dict)
                # rospy.loginfo("使用 dynamic_reconfigure 更新节点: %s" % target_node_name)
                success_count += 1
            except Exception as e:
                rospy.logwarn(
                    "dynamic_reconfigure 更新失败 %s: %s (改用 rosparam)" % (target_node_name, str(e))
                )
                # 如果 dynamic_reconfigure 失败，改用 rosparam set
                for p_name, p_val in config_dict.items():
                    param_path = target_node_name + "/" + p_name
                    try:
                        rospy.set_param(param_path, p_val)
                        rospy.loginfo("设置参数: %s = %s" % (param_path, p_val))
                    except Exception as e:
                        rospy.logerr("设置参数失败 %s: %s" % (param_path, str(e)))
                success_count += 1
        else:
            # 直接使用 rosparam set
            rospy.loginfo("节点 %s 不支持 dynamic_reconfigure，使用 rosparam 设置" % target_node_name)
            for p_name, p_val in config_dict.items():
                param_path = target_node_name + "/" + p_name
                try:
                    rospy.set_param(param_path, p_val)
                    # rospy.loginfo("设置参数: %s = %s" % (param_path, p_val))
                except Exception as e:
                    rospy.logerr("设置参数失败 %s: %s" % (param_path, str(e)))
            success_count += 1

    rospy.loginfo("配置应用完成。更新了 %d 个节点。" % success_count)
    return True


def main():
    """
    主函数：命令行入口，解析参数并调用apply_config()
    """
    # 解析参数
    args = sys.argv[1:]
    config_name = "default"
    yaml_file = None

    if len(args) > 0 and args[0] not in ["-h", "--help"]:
        config_name = args[0]

    if "--file" in args:
        idx = args.index("--file")
        if idx + 1 < len(args):
            yaml_file = args[idx + 1]

    if "-h" in args or "--help" in args or len(args) == 0:
        print("用法:")
        print("  python apply_config.py <配置名> [--file <yaml文件>]")
        print("")
        print("示例:")
        print("  python apply_config.py default")
        print("  python apply_config.py narrow")
        print("  python apply_config.py narrow --file /path/to/my_params.yaml")
        print("")
        print("外部调用示例:")
        print("  from apply_config import apply_config")
        print("  apply_config('default')")
        print("  apply_config('narrow', yaml_file='/path/to/config.yaml')")
        sys.exit(0)

    # 调用外部可用的apply_config函数
    if yaml_file:
        print("使用配置文件: %s" % yaml_file)
    else:
        default_yaml = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "../config/dynamic_params.yaml"
        )
        print("使用配置文件: %s" % default_yaml)
    
    apply_config(config_name, yaml_file=yaml_file)


if __name__ == "__main__":
    main()
