#!/bin/bash

# ROS bag播放脚本
# 从ROS参数读取配置并执行rosbag play

# 获取ROS参数
BAG_FILE=$(rosparam get /rosbag_play/bag_file)
RATE=$(rosparam get /rosbag_play/rate)
LOOP=$(rosparam get /rosbag_play/loop)
TOPICS=$(rosparam get /rosbag_play/topics)
START_TIME=$(rosparam get /rosbag_play/start_time)
DURATION=$(rosparam get /rosbag_play/duration)

# 新增参数
PREFIX=$(rosparam get /rosbag_play/prefix 2>/dev/null || echo "")
QUIET=$(rosparam get /rosbag_play/quiet 2>/dev/null || echo "false")
IMMEDIATE=$(rosparam get /rosbag_play/immediate 2>/dev/null || echo "false")
PAUSE=$(rosparam get /rosbag_play/pause 2>/dev/null || echo "false")
QUEUE_SIZE=$(rosparam get /rosbag_play/queue_size 2>/dev/null || echo "100")
CLOCK_HZ=$(rosparam get /rosbag_play/clock_hz 2>/dev/null || echo "100")
DELAY=$(rosparam get /rosbag_play/delay 2>/dev/null || echo "0")
SKIP_EMPTY=$(rosparam get /rosbag_play/skip_empty 2>/dev/null || echo "0")
KEEP_ALIVE=$(rosparam get /rosbag_play/keep_alive 2>/dev/null || echo "false")
WAIT_FOR_SUBSCRIBERS=$(rosparam get /rosbag_play/wait_for_subscribers 2>/dev/null || echo "false")

# 调试信息
echo "参数检查:"
echo "  BAG_FILE: $BAG_FILE"
echo "  RATE: $RATE"
echo "  LOOP: $LOOP"
echo "  TOPICS: '$TOPICS'"
echo "  START_TIME: $START_TIME"
echo "  DURATION: $DURATION"
echo "  PREFIX: '$PREFIX'"
echo "  QUIET: $QUIET"
echo "  IMMEDIATE: $IMMEDIATE"
echo "  PAUSE: $PAUSE"
echo "  QUEUE_SIZE: $QUEUE_SIZE"
echo "  CLOCK_HZ: $CLOCK_HZ"
echo "  DELAY: $DELAY"
echo "  SKIP_EMPTY: $SKIP_EMPTY"
echo "  KEEP_ALIVE: $KEEP_ALIVE"
echo "  WAIT_FOR_SUBSCRIBERS: $WAIT_FOR_SUBSCRIBERS"

# 检查bag文件是否存在
if [ ! -f "$BAG_FILE" ]; then
    echo "错误: bag文件不存在: $BAG_FILE"
    exit 1
fi

# 构建rosbag play命令
ROSBAG_CMD="rosbag play $BAG_FILE"

# 基本参数
ROSBAG_CMD="$ROSBAG_CMD --rate=$RATE --clock"

# 添加前缀
if [ -n "$PREFIX" ] && [ "$PREFIX" != "" ]; then
    echo "添加话题前缀: $PREFIX"
    ROSBAG_CMD="$ROSBAG_CMD --prefix=$PREFIX"
fi

# 添加安静模式
if [ "$QUIET" = "true" ]; then
    echo "启用安静模式"
    ROSBAG_CMD="$ROSBAG_CMD --quiet"
fi

# 添加立即播放
if [ "$IMMEDIATE" = "true" ]; then
    echo "启用立即播放"
    ROSBAG_CMD="$ROSBAG_CMD --immediate"
fi

# 添加暂停模式
if [ "$PAUSE" = "true" ]; then
    echo "启用暂停模式"
    ROSBAG_CMD="$ROSBAG_CMD --pause"
fi

# 添加队列大小
if [ "$QUEUE_SIZE" != "100" ]; then
    echo "设置队列大小: $QUEUE_SIZE"
    ROSBAG_CMD="$ROSBAG_CMD --queue=$QUEUE_SIZE"
fi

# 添加时钟频率
if [ "$CLOCK_HZ" != "100" ]; then
    echo "设置时钟频率: $CLOCK_HZ Hz"
    ROSBAG_CMD="$ROSBAG_CMD --hz=$CLOCK_HZ"
fi

# 添加延迟
if [ "$DELAY" != "0" ]; then
    echo "设置延迟: $DELAY 秒"
    ROSBAG_CMD="$ROSBAG_CMD --delay=$DELAY"
fi

# 添加循环选项
if [ "$LOOP" = "true" ]; then
    echo "启用循环播放"
    ROSBAG_CMD="$ROSBAG_CMD --loop"
fi

# 添加话题过滤
if [ -n "$TOPICS" ] && [ "$TOPICS" != "" ] && [ "$TOPICS" != "''" ]; then
    echo "添加话题过滤: $TOPICS"
    ROSBAG_CMD="$ROSBAG_CMD --topics=$TOPICS"
else
    echo "跳过话题过滤（参数为空）"
fi

# 添加开始时间
if [ "$START_TIME" != "0" ]; then
    echo "设置开始时间: $START_TIME 秒"
    ROSBAG_CMD="$ROSBAG_CMD --start=$START_TIME"
fi

# 添加持续时间
if [ "$DURATION" != "0" ]; then
    echo "设置持续时间: $DURATION 秒"
    ROSBAG_CMD="$ROSBAG_CMD --duration=$DURATION"
fi

# 添加跳过空区域
if [ "$SKIP_EMPTY" != "0" ]; then
    echo "跳过空区域: $SKIP_EMPTY 秒"
    ROSBAG_CMD="$ROSBAG_CMD --skip-empty=$SKIP_EMPTY"
fi

# 添加保持活跃
if [ "$KEEP_ALIVE" = "true" ]; then
    echo "启用保持活跃"
    ROSBAG_CMD="$ROSBAG_CMD --keep-alive"
fi

# 添加等待订阅者
if [ "$WAIT_FOR_SUBSCRIBERS" = "true" ]; then
    echo "等待订阅者"
    ROSBAG_CMD="$ROSBAG_CMD --wait-for-subscribers"
fi

echo "执行rosbag命令: $ROSBAG_CMD"

# 执行rosbag play命令
eval $ROSBAG_CMD

echo "rosbag播放完成"
