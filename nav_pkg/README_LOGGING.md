# 导航系统日志记录功能

## 概述
本系统提供了完整的导航日志记录功能，可以收集 move_base、TEB 规划器、SBPL 规划器、AMCL 和 map_server 的所有日志信息。

## 使用方法

### 方法1: 使用带日志的启动脚本（推荐）
```bash
# 使用默认日志路径
./scripts/start_navigation_with_logging.sh

# 指定自定义日志路径
./scripts/start_navigation_with_logging.sh /path/to/your/logfile.log
```

### 方法2: 使用 launch 文件
```bash
# 使用默认日志路径
roslaunch nav_pkg nav_daily_logs.launch

# 指定自定义日志路径
roslaunch nav_pkg nav_daily_logs.launch log_file_path:=/path/to/your/logfile.log
```

### 方法3: 使用带日志配置的 launch 文件
```bash
roslaunch nav_pkg nav_with_logging.launch log_file_path:=/path/to/your/logfile.log
```

## 日志内容

系统会记录以下组件的所有日志：
- **move_base**: 路径规划、目标设置、状态信息
- **TEB Local Planner**: 局部路径规划、优化过程、障碍物处理
- **SBPL Global Planner**: 全局路径规划、搜索过程
- **AMCL**: 定位信息、粒子滤波状态
- **map_server**: 地图加载、服务状态

## 日志格式

日志文件包含以下信息：
- 时间戳
- 日志级别 (INFO, WARN, ERROR, DEBUG)
- 节点名称
- 日志消息内容

示例：
```
[2024-01-15 10:30:45.123] [INFO] [move_base] Goal received
[2024-01-15 10:30:45.124] [INFO] [TebLocalPlanner] TEB Local Planner initialized
[2024-01-15 10:30:45.125] [WARN] [TebLocalPlanner] Oscillation detected
```

## 配置参数

### launch 文件参数
- `log_file_path`: 日志文件路径（默认: `/tmp/navigation_system.log`）
- `log_retention_days`: 日志保留天数（默认: 7天）

### 环境变量
- `ROSCONSOLE_CONFIG_FILE`: ROS 控制台配置文件路径

## 日志文件管理

- 日志文件会自动创建在指定路径
- 支持日志轮转，避免文件过大
- 可以通过 `log_retention_days` 参数设置保留天数

## 故障排除

1. **权限问题**: 确保对日志文件路径有写权限
2. **磁盘空间**: 定期清理旧日志文件
3. **日志级别**: 可以通过修改 `rosconsole.conf` 调整日志级别

## 注意事项

- 日志文件会持续增长，建议定期清理
- 在高频日志输出时，可能影响系统性能
- 建议在生产环境中使用日志轮转功能
