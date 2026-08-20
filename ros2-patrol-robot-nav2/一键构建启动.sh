#!/bin/bash
# 一键构建 + 启动 ros2-patrol-robot-nav2（用我们的地图）
# 用法: bash 一键构建启动.sh
set -e
cd "$(dirname "$0")"

echo "=== 1. 构建 ==="
bash build.sh

echo "=== 2. 启动一体化导航巡逻 ==="
source install/setup.bash
ros2 launch autopatrol_robot integrated_navigation.launch.py
