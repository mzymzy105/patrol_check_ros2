# ROS2 巡检机器人：Nav2 自定义 A规划器与采样 MPC 控制器

## 项目概述

本项目是一个基于 ROS 2 Humble 的自主巡逻机器人系统，集成了 Gazebo 仿真、Navigation2 导航栈、自定义全局路径规划器（A*）、自定义局部路径控制器（采样优化）、自动巡逻节点和语音播报功能。机器人可以在仿真环境中按照预设的巡逻路线自主导航，并在每个目标点拍摄图像记录。

核心启动文件为 [integrated_navigation.launch.py](launch/integrated_navigation.launch.py)，它一键启动整个系统，包括仿真环境、导航栈、巡逻逻辑和语音服务。

---

## 文档与演示

### 项目文档

完整的项目文档已托管在飞书，包含详细的设计说明、算法原理与实现细节：

- 飞书文档：[ROS2 巡检机器人项目文档](https://my.feishu.cn/wiki/EZoDwqrRliSQk0kpfPKcVvFonmh)

### 效果演示

下方为仿真巡逻效果预览（15 秒片段），完整演示视频已上传至 B站：

<video src="chapter8_ws/assert/sim_demo.webm" controls width="100%"></video>

- B站完整视频：[BV1SbgE6aEmm](https://www.bilibili.com/video/BV1SbgE6aEmm/)

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                  integrated_navigation.launch.py                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐   ┌──────────────┐   ┌────────────────────┐  │
│  │  Gazebo 仿真  │   │ robot_state  │   │  spawn_entity      │  │
│  │  (物理引擎)   │   │ _publisher   │   │  (生成机器人实体)   │  │
│  └──────┬───────┘   └──────────────┘   └────────────────────┘  │
│         │                                                       │
│         ▼                                                       │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              ros2_control 控制器管理器                     │   │
│  │  ┌─────────────────────┐  ┌──────────────────────────┐   │   │
│  │  │ joint_state_        │  │ diff_drive_controller    │   │   │
│  │  │ broadcaster         │  │ (差速驱动控制器)           │   │   │
│  │  └─────────────────────┘  └──────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Navigation2 导航栈                            │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌───────────┐  │   │
│  │  │  AMCL 定位      │  │ 全局代价地图    │  │ 局部代价   │  │   │
│  │  └────────────────┘  └────────────────┘  └───────────┘  │   │
│  │  ┌────────────────────────────────────────────────────┐  │   │
│  │  │ 全局规划器: nav2_custom_planner (A* + 路径平滑)     │  │   │
│  │  └────────────────────────────────────────────────────┘  │   │
│  │  ┌────────────────────────────────────────────────────┐  │   │
│  │  │ 局部控制器: nav2_custom_controller (采样优化控制)      │  │   │
│  │  └────────────────────────────────────────────────────┘  │   │
│  │  ┌────────────────┐  ┌────────────────┐                 │   │
│  │  │  BT Navigator   │  │  行为树恢复     │                 │   │
│  │  └────────────────┘  └────────────────┘                 │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌──────────────────┐    ┌──────────────────┐                  │
│  │  patrol_node     │───▶│  speaker (语音)   │                  │
│  │  (自动巡逻节点)    │    │  (espeakng 语音)  │                  │
│  └──────────────────┘    └──────────────────┘                  │
│                                                                 │
│  ┌──────────────────┐                                          │
│  │  RViz2 可视化     │                                          │
│  └──────────────────┘                                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 工作空间包结构

```
chapter8_ws/src/
├── autopatrol_interfaces/      # 自定义服务接口包
│   └── srv/
│       └── SpeachText.srv      # 语音合成服务接口
├── autopatrol_robot/           # 自动巡逻机器人包（主包）
│   ├── autopatrol_robot/
│   │   ├── patrol_node.py      # 巡逻节点
│   │   └── speaker.py          # 语音播报节点
│   ├── config/
│   │   └── patrol_config.yaml  # 巡逻配置（初始位姿 + 目标点）
│   └── launch/
│       ├── autopatrol.launch.py              # 仅启动巡逻+语音
│       └── integrated_navigation.launch.py   # 一体化启动（核心）
├── fishbot_description/        # 机器人描述包
│   ├── urdf/fishbot/           # URDF/Xacro 机器人模型
│   ├── config/                 # 控制器配置
│   ├── world/                  # Gazebo 世界文件
│   └── launch/                 # 仿真启动文件
├── fishbot_navigation2/        # 导航配置包
│   ├── config/
│   │   └── nav2_params.yaml    # Nav2 参数（含自定义插件配置）
│   ├── maps/
│   │   ├── room.yaml           # 地图元数据
│   │   └── room.pgm            # 地图图像
│   └── launch/                 # 导航启动文件
├── fishbot_application/        # 导航应用示例包
│   └── fishbot_application/
│       ├── nav_to_pose.py      # 单点导航示例
│       ├── waypoint_follower.py # 路径点跟随示例
│       ├── init_robot_pose.py  # 初始位姿设置
│       └── get_robot_pose.py   # 获取当前位姿
├── nav2_custom_planner/        # 自定义全局路径规划器
│   ├── include/nav2_custom_planner/
│   │   ├── nav2_custom_planner.hpp       # A* 规划器头文件
│   │   └── trajectory_processor.hpp      # 轨迹处理器头文件
│   └── src/
│       ├── nav2_custom_planner.cpp       # A* 规划器实现
│       └── trajectory_processor.cpp      # 轨迹处理器实现
└── nav2_custom_controller/    # 自定义局部路径控制器
    ├── include/nav2_custom_controller/
    │   └── custom_controller.hpp         # 控制器头文件
    └── src/
        └── custom_controller.cpp         # 控制器实现
```

---

## 启动文件详解：integrated_navigation.launch.py

[integrated_navigation.launch.py](launch/integrated_navigation.launch.py) 是整个系统的核心入口，按顺序启动以下组件：

### 1. 参数声明与路径获取

| 变量名 | 来源包 | 路径 | 说明 |
|--------|--------|------|------|
| `default_model_path` | fishbot_description | `urdf/fishbot/fishbot.urdf.xacro` | 机器人 URDF 模型 |
| `default_gazebo_world_path` | fishbot_description | `world/custom_room.world` | Gazebo 仿真世界 |
| `map_yaml_path` | fishbot_navigation2 | `maps/room.yaml` | 导航地图 |
| `nav2_param_path` | fishbot_navigation2 | `config/nav2_params.yaml` | Nav2 参数配置 |
| `rviz2_config_dir` | nav2_bringup | `rviz/nav2_default_view.rviz` | RViz2 配置 |
| `patrol_config_path` | autopatrol_robot | `config/patrol_config.yaml` | 巡逻配置 |

### 2. 启动节点列表

#### (1) robot_state_publisher — 机器人状态发布

- **功能**：读取 URDF/Xacro 模型，发布 `robot_description` 参数和 TF 变换
- **参数**：
  - `robot_description`：由 xacro 命令动态生成的 URDF 内容
  - `use_sim_time`：使用仿真时间（默认 `true`）

#### (2) Gazebo 仿真环境

- **功能**：启动 Gazebo 物理仿真引擎，加载 `custom_room.world` 世界
- **参数**：
  - `world`：世界文件路径
  - `verbose`：`true`（输出详细日志）

#### (3) spawn_entity — 生成机器人实体

- **功能**：在 Gazebo 中根据 `/robot_description` 话题生成 fishbot 机器人模型
- **参数**：
  - `-topic /robot_description`：从话题获取 URDF
  - `-entity fishbot`：实体名称

#### (4) 控制器加载（事件驱动）

采用 `OnProcessExit` 事件处理器，确保加载顺序：

```
spawn_entity 退出
    └──▶ 加载 fishbot_joint_state_broadcaster（关节状态广播器）
              └──▶ 加载 fishbot_diff_drive_controller（差速驱动控制器）
```

| 控制器 | 类型 | 说明 |
|--------|------|------|
| `fishbot_joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 发布关节状态 |
| `fishbot_diff_drive_controller` | `diff_drive_controller/DiffDriveController` | 差速驱动控制 |

差速驱动控制器关键参数（来自 [fishbot_ros2_controller.yaml](../fishbot_description/config/fishbot_ros2_controller.yaml)）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `wheel_separation` | 0.20 m | 左右轮距 |
| `wheel_radius` | 0.032 m | 轮子半径 |
| `publish_rate` | 50.0 Hz | 里程计发布频率 |
| `odom_frame_id` | odom | 里程计坐标系 |
| `base_frame_id` | base_footprint | 基座坐标系 |
| `cmd_vel_timeout` | 0.5 s | 速度命令超时 |
| `enable_odom_tf` | true | 发布 odom→base_footprint TF |

话题映射：
- 命令速度：`/cmd_vel` → `fishbot_diff_drive_controller/cmd_vel_unstamped`
- 里程计：`fishbot_diff_drive_controller/odom` → `/odom`

#### (5) Navigation2 导航栈

- **功能**：启动完整的 Nav2 导航栈，包括 AMCL 定位、全局/局部代价地图、全局规划器、局部控制器、行为树导航器、恢复行为服务器等
- **参数**：
  - `map`：地图 YAML 路径
  - `use_sim_time`：`true`
  - `params_file`：`nav2_params.yaml`（含自定义插件配置）
  - `use_composition`：`False`（不使用组件化加载）
  - `autostart`：`true`（自动启动生命周期节点）

#### (6) RViz2 可视化

- **功能**：启动 RViz2 可视化界面，使用 Nav2 默认视图配置
- **参数**：
  - `use_sim_time`：`true`

#### (7) patrol_node — 自动巡逻节点

- **功能**：按照配置的目标点循环巡逻，到达每个点后拍照记录
- **参数**：
  - 加载 `patrol_config.yaml` 配置文件
  - `use_sim_time`：`true`

#### (8) speaker — 语音播报节点

- **功能**：提供 `speech_text` 服务，使用 espeakng 进行中文语音合成
- **无额外参数**

---

## 核心节点详解

### patrol_node（巡逻节点）

[patrol_node.py](autopatrol_robot/patrol_node.py) 继承自 `BasicNavigator`，是整个巡逻逻辑的核心。

#### 主要功能

| 方法 | 说明 |
|------|------|
| `init_robot_pose()` | 通过 `initial_point` 参数发布初始位姿给 AMCL |
| `get_pose_by_xyyaw(x, y, yaw)` | 将 x, y, yaw 合成为 `PoseStamped` 消息 |
| `get_target_points()` | 从参数服务器获取目标点集合 |
| `nav_to_pose(target_pose)` | 导航到指定位姿，等待结果并反馈 |
| `get_current_pose()` | 通过 TF（map→base_footprint）获取当前位姿 |
| `speach_text(text)` | 调用语音服务播放文字 |
| `record_image()` | 保存当前相机图像，文件名包含位姿坐标 |

#### 巡逻流程

```
1. 初始化 → 语音播报 "正在初始化位置"
2. 等待 Nav2 激活
3. 发布初始位姿 → 语音播报 "位置初始化完成"
4. 循环：
   a. 获取目标点列表
   b. 对每个目标点：
      - 语音播报 "准备前往目标点 x, y"
      - 导航到目标点
      - 语音播报 "已到达目标点 x, y"
      - 拍照记录图像
      - 语音播报 "图像记录完成"
```

#### 订阅话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/camera_sensor/image_raw` | `sensor_msgs/Image` | 相机图像 |

#### 发布话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿 |

#### 调用服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `speech_text` | `autopatrol_interfaces/srv/SpeachText` | 语音合成 |

### speaker（语音节点）

[speaker.py](autopatrol_robot/speaker.py) 提供 `speech_text` 服务，使用 `espeakng` 库进行中文语音合成。

#### 服务接口

```
# SpeachText.srv
string text     # 要合成的文字
---
bool result     # 合成结果（成功/失败）
```

---

## 自定义导航插件

### nav2_custom_planner（自定义全局规划器）

[nav2_custom_planner.hpp](../nav2_custom_planner/include/nav2_custom_planner/nav2_custom_planner.hpp)

基于 **A\* 算法** 的全局路径规划器，支持 8 连通搜索和路径平滑。

#### 核心算法

1. **A\* 搜索**：在代价地图上使用 A\* 算法搜索从起点到终点的最优路径
2. **8 连通搜索**：支持 4 连通和 8 连通两种搜索方式（默认 8 连通）
3. **路径平滑**：对搜索结果进行平滑处理，减少锯齿
4. **轨迹处理**：`TrajectoryProcessor` 对规划路径进行速度限制和曲率约束处理

#### 参数配置（nav2_params.yaml）

```yaml
GridBased:
    plugin: "nav2_custom_planner::CustomPlanner"
    tolerance: 0.5          # 目标点容差（米）
    use_8connected: true    # 使用 8 连通搜索
    smooth_path: true       # 启用路径平滑
    cost_weight: 1.0        # 代价权重
    enable_trajectory_processing: true  # 启用轨迹处理
    max_linear_speed: 0.26  # 最大线速度（m/s）
    max_angular_speed: 1.0  # 最大角速度（rad/s）
    max_curvature: 5.0      # 最大曲率（1/m）
```

### nav2_custom_controller（自定义局部控制器）

[custom_controller.hpp](../nav2_custom_controller/include/nav2_custom_controller/custom_controller.hpp)

基于 **采样优化** 的局部路径控制器，通过随机采样多组速度命令并评估轨迹质量来选择最优控制输入。

#### 核心算法

1. **轨迹传播**：根据运动学模型，对每组采样速度 (v, ω) 前向模拟预测轨迹
2. **轨迹评估**：使用多目标代价函数评估轨迹质量：
   - `weight_path`：路径跟随权重
   - `weight_heading`：航向对齐权重
   - `weight_smooth`：平滑性权重
3. **最优选择**：选择代价最小的轨迹对应的速度命令

#### 参数配置（nav2_params.yaml）

```yaml
FollowPath:
    plugin: "nav2_custom_controller::CustomController"
    max_linear_speed: 0.26    # 最大线速度（m/s）
    max_angular_speed: 1.0    # 最大角速度（rad/s）
    prediction_horizon: 10    # 预测步数
    dt: 0.1                   # 预测时间步长（秒）
    num_samples: 150          # 采样数量
    weight_path: 10.0         # 路径跟随权重
    weight_heading: 1.0       # 航向对齐权重
    weight_smooth: 0.5        # 平滑性权重
```

---

## 机器人模型

### FishBot 硬件参数

[fishbot.urdf.xacro](../fishbot_description/urdf/fishbot/fishbot.urdf.xacro)

| 部件 | 参数 |
|------|------|
| 主体 | 长度 0.12m，半径 0.1m |
| 左轮 | 位置 (0.0, 0.1, -0.06) |
| 右轮 | 位置 (0.0, -0.1, -0.06) |
| 前万向轮 | 位置 (0.08, 0.0, -0.076) |
| 后万向轮 | 位置 (-0.08, 0.0, -0.076) |
| IMU | 位置 (0, 0, 0.02) |
| 激光雷达 | 位置 (0, 0, 0.10) |
| 摄像头 | 位置 (0.10, 0, 0.075) |

### 传感器配置

| 传感器 | 话题 | 类型 | 频率 | 说明 |
|--------|------|------|------|------|
| 激光雷达 | `/scan` | `sensor_msgs/LaserScan` | 100 Hz | 360° 扫描，范围 0.12~8.0m |
| IMU | `/imu` | `sensor_msgs/Imu` | 100 Hz | 六轴惯性测量 |
| 摄像头 | `/camera_sensor/image_raw` | `sensor_msgs/Image` | - | 前方摄像头 |

---

## 巡逻配置

[patrol_config.yaml](config/patrol_config.yaml)

```yaml
/patrol_node:
  ros__parameters:
    initial_point: [0.0, 0.0, 0.0]       # 初始位姿 [x, y, yaw]
    target_points: [                        # 巡逻目标点 [x, y, yaw]
        0.0, 0.0, 0.0,                     # 目标点 0：原点
        1.0, 2.0, 3.14,                    # 目标点 1
        -4.5, 1.5, 1.57,                   # 目标点 2
        -8.0, -5.0, 1.57,                  # 目标点 3
        1.0, -5.0, 3.14,                   # 目标点 4
    ]
```

> **注意**：`target_points` 是一个扁平数组，每 3 个元素构成一个目标点 `[x, y, yaw]`，其中 yaw 单位为弧度。

---

## 地图信息

[room.yaml](../fishbot_navigation2/maps/room.yaml)

| 参数 | 值 |
|------|-----|
| 图像文件 | `room.pgm` |
| 模式 | `trinary` |
| 分辨率 | 0.05 m/pixel |
| 原点 | (-10.4, -6.53, 0) |
| 占用阈值 | 0.65 |
| 空闲阈值 | 0.25 |

---

## 关键话题与服务汇总

### 话题

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 速度命令 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 里程计 |
| `/scan` | `sensor_msgs/LaserScan` | 发布 | 激光扫描 |
| `/imu` | `sensor_msgs/Imu` | 发布 | IMU 数据 |
| `/camera_sensor/image_raw` | `sensor_msgs/Image` | 发布 | 相机图像 |
| `/robot_description` | `std_msgs/String` | 发布 | URDF 模型 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 发布 | 初始位姿 |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | TF 变换 |
| `/map` | `nav_msgs/OccupancyGrid` | 发布 | 地图数据 |
| `/plan` | `nav_msgs/Path` | 发布 | 全局路径 |
| `/local_plan` | `nav_msgs/Path` | 发布 | 局部路径 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `speech_text` | `autopatrol_interfaces/srv/SpeachText` | 语音合成服务 |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 导航到指定位姿 |

### TF 树

```
map → odom → base_footprint → base_link → [各传感器 link]
                                         ├→ left_wheel_link
                                         ├→ right_wheel_link
                                         ├→ laser_cylinder_link → laser_link
                                         ├→ imu_link
                                         ├→ camera_link
                                         ├→ front_caster_link
                                         └→ back_caster_link
```

---

## 构建与运行

### 前置依赖

- ROS 2 Humble
- Gazebo 11
- Nav2 栈（`ros-humble-navigation2`）
- `gazebo_ros2_control`
- `espeakng`（Python 库，用于语音合成）
- `tf_transformations`（Python 库）
- `cv_bridge`、`opencv-python`

### 安装依赖

```bash
# ROS 2 包
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers

# Python 依赖
pip3 install espeakng transforms3d opencv-python
```

### 编译

```bash
cd /home/zxy/code/Chapter8/chapter8_ws

# 编译所有包
colcon build --symlink-install

# 或仅编译特定包
colcon build --packages-select \
    autopatrol_interfaces \
    autopatrol_robot \
    fishbot_description \
    fishbot_navigation2 \
    nav2_custom_planner \
    nav2_custom_controller
```

### 运行

```bash
# 加载工作空间环境
source /home/zxy/code/Chapter8/chapter8_ws/install/setup.bash

# 启动一体化导航巡逻系统
ros2 launch autopatrol_robot integrated_navigation.launch.py

# 使用仿真时间（默认已开启）
ros2 launch autopatrol_robot integrated_navigation.launch.py use_sim_time:=true

# 指定自定义模型路径
ros2 launch autopatrol_robot integrated_navigation.launch.py model:=/path/to/your/robot.urdf.xacro
```

### 仅启动巡逻节点（不含仿真和导航）

```bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

---

## 参数调优指南

### 导航精度调优

| 参数 | 位置 | 默认值 | 调优建议 |
|------|------|--------|---------|
| `xy_goal_tolerance` | nav2_params.yaml → general_goal_checker | 0.25 m | 减小提高精度，但可能增加到达难度 |
| `yaw_goal_tolerance` | nav2_params.yaml → general_goal_checker | 0.25 rad | 减小提高朝向精度 |
| `tolerance` | nav2_params.yaml → GridBased | 0.5 m | 全局规划器目标容差 |

### 速度调优

| 参数 | 位置 | 默认值 | 调优建议 |
|------|------|--------|---------|
| `max_linear_speed` | nav2_params.yaml → FollowPath | 0.26 m/s | 增大提高速度，但降低安全性 |
| `max_angular_speed` | nav2_params.yaml → FollowPath | 1.0 rad/s | 增大提高转弯速度 |
| `controller_frequency` | nav2_params.yaml → controller_server | 20.0 Hz | 增大提高控制平滑度 |

### 采样控制器调优

| 参数 | 位置 | 默认值 | 调优建议 |
|------|------|--------|---------|
| `num_samples` | nav2_params.yaml → FollowPath | 150 | 增大提高轨迹质量，但增加计算量 |
| `prediction_horizon` | nav2_params.yaml → FollowPath | 10 | 增大提高前瞻性，但降低响应速度 |
| `dt` | nav2_params.yaml → FollowPath | 0.1 s | 预测步长 |
| `weight_path` | nav2_params.yaml → FollowPath | 10.0 | 增大更注重路径跟随 |
| `weight_heading` | nav2_params.yaml → FollowPath | 1.0 | 增大更注重航向对齐 |
| `weight_smooth` | nav2_params.yaml → FollowPath | 0.5 | 增大更注重运动平滑 |

### A* 规划器调优

| 参数 | 位置 | 默认值 | 调优建议 |
|------|------|--------|---------|
| `use_8connected` | nav2_params.yaml → GridBased | true | true 路径更短，false 更平滑 |
| `smooth_path` | nav2_params.yaml → GridBased | true | 启用路径平滑 |
| `cost_weight` | nav2_params.yaml → GridBased | 1.0 | 增大更倾向于远离障碍物 |

### 巡逻路线调优

修改 [patrol_config.yaml](config/patrol_config.yaml) 中的 `target_points` 数组，每 3 个值为一组 `[x, y, yaw]`：

```yaml
target_points: [
    x0, y0, yaw0,    # 第 1 个目标点
    x1, y1, yaw1,    # 第 2 个目标点
    x2, y2, yaw2,    # 第 3 个目标点
    ...
]
```

> 可在 RViz2 中使用 "Publish Point" 工具在地图上点击获取坐标，或使用 `ros2 topic echo /clicked_point` 查看点击坐标。

---

## 常见问题

### 1. Gazebo 启动失败或黑屏

- 确保 Gazebo 11 正确安装：`gazebo --version`
- 尝试先启动 Gazebo 再运行 launch 文件
- 检查显卡驱动是否正常

### 2. 控制器加载失败

- 确认 `ros2 control` 相关包已安装
- 检查 `fishbot_ros2_controller.yaml` 配置是否正确
- 查看控制器状态：`ros2 control list_controllers`

### 3. 导航无法到达目标点

- 检查目标点是否在地图的空闲区域内
- 调整 `tolerance` 和 `xy_goal_tolerance` 参数
- 检查代价地图是否正确加载

### 4. 语音服务不可用

- 确认 `espeakng` 已安装：`pip3 install espeakng`
- 确认系统音频设备正常
- patrol_node 会在语音服务不可用时自动跳过，不影响导航

### 5. 图像保存失败

- 检查 `image_save_path` 参数是否设置了有效路径
- 确保路径目录存在且有写入权限

### 6. TF 变换错误

- 确认 `use_sim_time` 在所有节点中一致设置为 `true`
- 检查 AMCL 是否正常定位（粒子是否收敛）

---

## 技术栈

| 技术 | 版本/说明 |
|------|----------|
| ROS 2 | Humble Hawksbill |
| Gazebo | 11 |
| Navigation2 | Humble |
| ros2_control | Humble |
| 规划算法 | A*（自定义插件） |
| 控制算法 | 采样优化（自定义插件） |
| 语音合成 | espeakng |
| 仿真世界 | custom_room.world |
| 编程语言 | C++（插件）、Python（节点） |