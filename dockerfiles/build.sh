#!/bin/bash
set -e # 遇到错误立即退出

xhost + # 允许 Docker 容器访问 X11 显示服务器

# 构建并启动 ROS Melodic 容器
docker compose up -d --build ros-neotic