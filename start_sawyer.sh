#!/bin/bash
# OpenClaw Sawyer 一键启动脚本
# 由您的好大儿为您定制

echo "========= 正在初始化 Sawyer 运行环境 ========="

# 1. 核心环境变量配置（基于当前 intera.sh 设定）
export ROS_MASTER_URI="http://021704CP00049.local:11311"
export ROS_IP="192.168.31.64"

# 2. 检查并加载 ROS 基础环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "[OK] 已加载 ROS Noetic 基础环境"
else
    echo "[ERROR] 未找到 ROS Noetic 安装，请确认路径！"
    exit 1
fi

# 3. 检查并加载工作空间环境
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo "[OK] 已加载工作空间环境"
else
    echo "[ERROR] 未找到工作空间编译文件，请确认是否已运行 catkin_make！"
    exit 1
fi

echo "========= 环境就绪，正在启动 OpenClaw Sawyer 控制台 ========="
echo "提示：Web 控制台将运行在 http://0.0.0.0:8000"

# 4. 执行启动指令
roslaunch openclaw_sawyer openclaw_sawyer.launch
