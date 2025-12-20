# 同时运行两个机器人环境的方法

## 方法一：使用命名空间（推荐，单ROS Master）

使用合并的launch文件，通过命名空间隔离两个环境：

```bash
roslaunch nav_pkg dual_robot_env.launch
```

### 控制机器人1：
```bash
# 发送速度指令（持续发布，10Hz）
rostopic pub -r 10 /robot1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 发送速度指令（单次发布）
rostopic pub /robot1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 发送导航目标
rostopic pub /robot1/move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 控制机器人2：
```bash
# 发送速度指令（持续发布，10Hz）
rostopic pub -r 10 /robot2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 发送速度指令（单次发布）
rostopic pub /robot2/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 发送导航目标
rostopic pub /robot2/move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 查看话题：
```bash
# 查看所有话题
rostopic list

# 查看机器人1的话题
rostopic list | grep robot1

# 查看机器人2的话题
rostopic list | grep robot2
```

---

## 方法二：使用不同的ROS Master（完全隔离）

在两个不同的终端中分别运行，使用不同的ROS_MASTER_URI：

### 终端1 - 机器人1环境：
```bash
# 设置ROS Master端口
export ROS_MASTER_URI=http://localhost:11311

# 启动roscore（如果还没有运行）
roscore

# 启动机器人1环境
roslaunch wpr_simulation wpb_stage_robocup.launch
roslaunch nav_pkg nav_daily_logs.launch
```

### 终端2 - 机器人2环境：
```bash
# 设置不同的ROS Master端口
export ROS_MASTER_URI=http://localhost:11312

# 启动第二个roscore
roscore -p 11312

# 启动机器人2环境
roslaunch wpr_simulation wpb_stage_robocup.launch
roslaunch nav_pkg nav_daily_logs.launch
```

### 控制机器人1（在终端1或新终端）：
```bash
export ROS_MASTER_URI=http://localhost:11311
rostopic pub /cmd_vel geometry_msgs/Twist ...
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...
```

### 控制机器人2（在终端2或新终端）：
```bash
export ROS_MASTER_URI=http://localhost:11312
rostopic pub /cmd_vel geometry_msgs/Twist ...
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...
```

---

## 注意事项

1. **Gazebo冲突**：如果两个环境都使用Gazebo，可能会有冲突。建议：
   - 方法一：两个环境共享同一个Gazebo实例（可能有问题）
   - 方法二：使用不同的ROS Master，但Gazebo仍然可能冲突

2. **TF树冲突**：两个机器人会有不同的TF树，需要确保：
   - 方法一：使用命名空间隔离TF树（robot1/tf, robot2/tf）
   - 方法二：完全隔离，不会有冲突

3. **RViz可视化**：
   - 方法一：需要配置RViz订阅正确的话题（robot1/* 或 robot2/*）
   - 方法二：可以分别启动两个RViz实例

4. **资源占用**：同时运行两个仿真环境会占用较多系统资源

---

## 推荐方案

**推荐使用方法一（命名空间）**，因为：
- 更简单，只需一个命令启动
- 共享同一个ROS Master，便于监控和调试
- 可以使用同一个RViz查看两个机器人（需要配置）

如果遇到Gazebo冲突或其他问题，再考虑使用方法二（不同的ROS Master）。

