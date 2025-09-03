#!/bin/bash
set -e

# Allow Docker container to access X11 display server
xhost +

# Auto-detect Windows client IP (SSH source)
WINDOWS_IP=$(who am i | awk '{print $5}' | sed 's/[()]//g' | cut -d: -f1)
if [ -z "$WINDOWS_IP" ]; then
    # Fallback: from SSH_CLIENT env
    WINDOWS_IP=$(echo $SSH_CLIENT | cut -d' ' -f1)
fi
echo "检测到的Windows IP: $WINDOWS_IP"

# Export for docker-compose
export DISPLAY_IP="$WINDOWS_IP:0.0"

# # 构建并启动 ROS Noetic 容器
# docker compose up -d --build learn_ros-noetic
# Build and start ROS Noetic container using the compose file in .devcontainer
docker compose -f .devcontainer/docker-compose.yml up -d --build learn_ros-noetic

