#!/bin/bash
# 构建 ros2-patrol-robot-nav2 工作空间
set -e
cd "$(dirname "$0")"

# 关键：彻底清除 conda 的编译环境变量（这是所有构建问题的根源：
#   conda 的 CXX/CFLAGS/CMAKE_PREFIX_PATH 会让编译器、include、Python 全指向 miniconda）
unset CC CXX CFLAGS CXXFLAGS CPPFLAGS LDFLAGS LDFLAGS_LD \
      CMAKE_ARGS CMAKE_PREFIX_PATH CMAKE_LIBRARY_PATH CMAKE_INCLUDE_PATH \
      CXX_FOR_BUILD CC_FOR_BUILD CPATH LIBRARY_PATH \
      CONDA_PREFIX CONDA_DEFAULT_ENV 2>/dev/null || true
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH

source /opt/ros/humble/setup.bash

# 追加系统 cmake 路径（不覆盖 ROS 路径）
export CMAKE_PREFIX_PATH="/usr/lib/x86_64-linux-gnu/cmake:$CMAKE_PREFIX_PATH"

colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPython_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
  -DCMAKE_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu" \
  -Dconsole_bridge_DIR=/usr/lib/x86_64-linux-gnu/console_bridge/cmake \
  -DTINYXML2_LIBRARY=/usr/lib/x86_64-linux-gnu/libtinyxml2.so \
  -DTINYXML2_INCLUDE_DIR=/usr/include \
  -Dorocos_kdl_INCLUDE_DIRS=/usr/include
