#!/bin/bash
set -e # 遇到错误立即退出

xhost + # 允许 Docker 容器访问 X11 显示服务器

# 自动检测Windows客户端IP地址（SSH连接来源）
WINDOWS_IP=$(who am i | awk '{print $5}' | sed 's/[()]//g' | cut -d: -f1)
if [ -z "$WINDOWS_IP" ]; then
    # 备选方法：从SSH_CLIENT环境变量获取
    WINDOWS_IP=$(echo $SSH_CLIENT | cut -d' ' -f1)
fi
echo "检测到的Windows IP: $WINDOWS_IP"

# 导出环境变量供docker-compose使用
export DISPLAY_IP="$WINDOWS_IP:0.0"

# 构建并启动 ROS Noetic 容器
docker compose up -d --build learn_ros-noetic