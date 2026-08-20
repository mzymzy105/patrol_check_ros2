import rclpy
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion,quaternion_from_euler
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

#添加服务接口
from autopatrol_interfaces.srv import SpeachText

#导入消息接口和相关定义
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        #导航相关定义
        self.declare_parameter('initial_point',[0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        #实时位置获取TF相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_,self)
        #语音合成客户端
        self.speach_client_ = self.create_client(SpeachText,'speech_text')
        #订阅与保存图像相关定义
        self.declare_parameter('image_save_path','')
        self.image_save_path = self.get_parameter('image_save_path').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image,'/camera_sensor/image_raw',self.image_callback,10)
        #初始位姿发布器
        self.initial_pose_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def init_robot_pose(self):
        """
        通过initial_point参数发布初始位姿给AMCL
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()  # stamp=0 → 最新 TF，避免 future 时间戳被 AMCL 拒绝
        msg.pose.pose.position.x = self.initial_point_[0]
        msg.pose.pose.position.y = self.initial_point_[1]
        msg.pose.pose.position.z = 0.0
        rotation_quat = quaternion_from_euler(0, 0, self.initial_point_[2])
        msg.pose.pose.orientation.x = rotation_quat[0]
        msg.pose.pose.orientation.y = rotation_quat[1]
        msg.pose.pose.orientation.z = rotation_quat[2]
        msg.pose.pose.orientation.w = rotation_quat[3]
        # 等 AMCL 订阅连接（BEST_EFFORT 在连接前发布会丢）
        import time
        t0 = time.time()
        while time.time() - t0 < 5.0 and self.initial_pose_pub_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.initial_pose_pub_.publish(msg)
        self.get_logger().info(
            f'已发布初始位姿: x={self.initial_point_[0]}, '
            f'y={self.initial_point_[1]}, yaw={self.initial_point_[2]}')

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过x，y，yaw合成PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose


    def get_target_points(self):
        """
        通过参数值获取目标点集合
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'获取到目标点:{index}->({x},{y},{yaw})')
        return points

    def nav_to_pose(self,target_pose):
        """
        导航到指定位姿
        """
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f'预计：{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds/1e9}s后到达')
        #最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结束：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().info(f'导航结果：失败')
        else:
            self.get_logger().info(f'导航结果：返回状态无效')

    def get_current_pose(self):
        """
        通过TF获取当前位姿
        """
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform(
                    'map','base_footprint',rclpy.time.Time(seconds=0),rclpy.time.Duration(seconds=1)
                )
                transform = tf.transform
                rotation_euler = euler_from_quaternion(
                    [
                        transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w,
                    ]
                )
                self.get_logger().info(f'平移：{transform.translation},旋转四元数：{transform.rotation},旋转欧拉角：{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().info(f'不能获取，原因：{str(e)}')

    def speach_text(self, text):
        """
        调用服务播放语音，若服务不可用则跳过
        """
        if not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'语音服务未上线，跳过语音：{text}')
            return
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f'语音合成成功：{text}')
            else:
                self.get_logger().info(f'语音合成失败：{text}')
        else:
            self.get_logger().warn('语音合成服务请求失败')

    def image_callback(self,msg):
        """
        将最新的消息放到lastes_image中
        """
        self.latest_image = msg

    def record_image(self):
        """
        记录图像
        """
        if self.latest_image is not None:
            pose = self.get_current_pose()
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image)
            cv2.imwrite(f'{self.image_save_path}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',cv_image)


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speach_text(text="正在初始化位置")
    patrol.waitUntilNav2Active()
    patrol.init_robot_pose()
    patrol.speach_text(text="位置初始化完成")

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x,y,yaw)
            patrol.speach_text(text=f'准备前往目标点{x},{y}')
            patrol.nav_to_pose(target_pose)
            #记录图像
            patrol.speach_text(text=f'准备前往目标点{x},{y}')
            patrol.record_image()
            patrol.speach_text(text=f'图像记录完成')
    rclpy.shutdown()