#未完成版
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped


class SemanticDetector(Node):
    def __init__(self):
        super().__init__('semantic_detector')
        # 订阅彩色图像和对齐的深度图
        self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        
        # 发布识别到的目标位置
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.bridge = CvBridge()
        self.latest_depth = None
        self.intrinsics = None

    def info_callback(self, msg):
        # 获取相机内参，用于像素转 3D 坐标
        self.intrinsics = msg.k

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def image_callback(self, msg):
        if self.latest_depth is None or self.intrinsics is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        u, v = 320, 240 # 图像中心
        
        # 获取该点的深度值 (单位: mm)
        depth = self.latest_depth[v, u] / 1000.0 # 转换为米
        
        if depth > 0:
            # 像素坐标转相机系坐标 (X = (u - cx) * depth / fx)
            x_c = (u - self.intrinsics[2]) * depth / self.intrinsics[0]
            y_c = (v - self.intrinsics[5]) * depth / self.intrinsics[4]
            z_c = depth
            
            self.publish_goal(x_c, y_c, z_c)

    def publish_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = "camera_link" # 初始在相机坐标系
        goal.header.stamp = self.get_clock().now().to_msg()
        # 简单的坐标映射（注意：实际需根据相机安装角度进行旋转）
        goal.pose.position.x = z - 0.5 # 停在物体前方 0.5 米
        goal.pose.position.y = -x
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'已检测到目标椅子，发送导航点到前方 0.5m 处')