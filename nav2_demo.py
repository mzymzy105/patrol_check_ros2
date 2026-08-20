#!/usr/bin/env python3
"""
Nav2 导航链路演示：设 AMCL 初始位姿 -> 等定位收敛 -> 发导航目标 -> 监控机器人移动。

用法:
    python3 nav2_demo.py [goal_x goal_y] [--origin x y] [--monitor 秒数]

默认: 目标 (3.0, 3.0)，地图 origin 从 /map 话题自动读。
"""
import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavDemo(Node):
    def __init__(self, goal_xy, monitor_secs):
        super().__init__('nav2_demo',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.goal_xy = goal_xy
        self.monitor_secs = monitor_secs
        # AMCL 订阅 /initialpose、bt_navigator 订阅 /goal_pose 都用 BEST_EFFORT，必须匹配
        best_effort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', best_effort_qos)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', best_effort_qos)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self._map_cb, map_qos)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.odom = None
        self.map_origin = None  # (x, y)

    def _odom_cb(self, msg):
        self.odom = msg

    def _map_cb(self, msg):
        if self.map_origin is None:
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def wait_odom(self, timeout=8.0):
        t = time.time()
        while self.odom is None and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom

    def wait_map_origin(self, timeout=8.0):
        t = time.time()
        while self.map_origin is None and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map_origin

    def wait_map_tf(self, timeout=15.0):
        """等 AMCL 收敛、map->odom TF 出现"""
        t = time.time()
        while time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.tf.lookup_transform('map', 'odom', rclpy.time.Time())
                return True
            except TransformException:
                pass
        return False


def main():
    rclpy.init()
    args = sys.argv[1:]
    goal = [3.0, 3.0]
    monitor = 30.0
    # 解析目标点
    nums = []
    for a in args:
        try:
            nums.append(float(a))
        except ValueError:
            pass
    if len(nums) >= 2:
        goal = [nums[0], nums[1]]
    if '--monitor' in args:
        monitor = float(args[args.index('--monitor') + 1])

    node = NavDemo(goal, monitor)

    # 1. 等 odom 和 map origin
    odom = node.wait_odom()
    origin = node.wait_map_origin()
    if odom is None:
        node.get_logger().error('未收到 /odom，Gazebo 是否还在跑？')
        return
    if origin is None:
        node.get_logger().error('未收到 /map，map_server 是否正常？')
        return

    ox, oy = odom.pose.pose.position.x, odom.pose.pose.position.y
    yaw = yaw_from_quat(odom.pose.pose.orientation)
    # Gazebo world 坐标系 = map 坐标系(墙直接放在 map 坐标)，所以 map 位姿 = odom 位姿
    mx, my = ox, oy
    node.get_logger().info(f'机器人 odom 位姿: ({ox:.2f}, {oy:.2f}) yaw={yaw:.2f}')
    node.get_logger().info(f'地图 origin(仅参考): ({origin[0]:.2f}, {origin[1]:.2f})')
    node.get_logger().info(f'-> 对应 map 初始位姿: ({mx:.2f}, {my:.2f}) yaw={yaw:.2f}')

    # 2. 发布初始位姿（先等 AMCL 订阅连接，BEST_EFFORT 在连接前发布会丢）
    t0 = time.time()
    while time.time() - t0 < 5.0 and node.init_pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f'AMCL 订阅已连接 (subscribers={node.init_pub.get_subscription_count()})')

    init = PoseWithCovarianceStamped()
    init.header.frame_id = 'map'
    init.pose.pose.position.x = mx
    init.pose.pose.position.y = my
    init.pose.pose.orientation = quat_from_yaw(yaw)
    init.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787,
    ]
    # 反复发布几次（BEST_EFFORT 首条消息易丢失），并用当前时间戳避免被 AMCL 拒绝
    for _ in range(5):
        init.header.stamp = node.get_clock().now().to_msg()
        node.init_pub.publish(init)
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info('已发布 /initialpose，等待 AMCL 收敛...')

    # 3. 等 map->odom TF
    if not node.wait_map_tf():
        node.get_logger().warn('15s 内未等到 map->odom TF，继续尝试发目标')
    else:
        node.get_logger().info('AMCL 已收敛，map->odom TF 已建立 ✓')

    # 3.5 等 navigation 生命周期激活完成（planner/bt_navigator 就绪），否则 BEST_EFFORT 的 goal 会丢
    node.get_logger().info('等待 navigation 节点激活(6s)...')
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 4. 发导航目标
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = 'map'
    goal_msg.header.stamp = rclpy.time.Time().to_msg()  # stamp=0 → 最新
    goal_msg.pose.position.x = goal[0]
    goal_msg.pose.position.y = goal[1]
    goal_msg.pose.orientation = quat_from_yaw(0.0)
    node.goal_pub.publish(goal_msg)
    node.get_logger().info(f'已发布导航目标: map({goal[0]:.2f}, {goal[1]:.2f})')

    # 5. 监控机器人移动
    node.get_logger().info(f'监控 {monitor:.0f}s 内机器人位置变化...')
    t0 = time.time()
    last = None
    while time.time() - t0 < monitor:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tr = node.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x, y = tr.transform.translation.x, tr.transform.translation.y
            if last is None or math.hypot(x - last[0], y - last[1]) > 0.05:
                d = math.hypot(x - goal[0], y - goal[1])
                node.get_logger().info(f'  机器人 map 位姿: ({x:.2f}, {y:.2f})  距目标 {d:.2f} m')
                last = (x, y)
            if math.hypot(x - goal[0], y - goal[1]) < 0.25:
                node.get_logger().info('已到达目标附近，导航成功 ✓')
                break
        except TransformException:
            pass

    node.get_logger().info('演示结束')


if __name__ == '__main__':
    main()
