#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class SemanticPointCloudNode(Node):
    def __init__(self):
        super().__init__('semantic_pointcloud_node')
        
        # 1. 初始化 YOLO11 分割模型 (使用 -seg 后缀)
        self.model = YOLO('yolo11n-seg.pt') 
        self.bridge = CvBridge()

        # 2. 订阅话题
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        
        # 3. 发布语义点云
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/semantic_pointcloud', 10)
        
        self.latest_depth = None
        self.intrinsics = None # fx, fy, cx, cy

    def info_callback(self, msg):
        # 获取相机内参
        self.intrinsics = {
            'fx': msg.k[0], 'fy': msg.k[4],
            'cx': msg.k[2], 'cy': msg.k[5]
        }

    def depth_callback(self, msg):
        # 处理深度图
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def callback(self, color_msg):
        if self.latest_depth is None or self.intrinsics is None:
            return

        # 转换彩色图并进行 YOLO 推理
        cv_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        results = self.model(cv_image, verbose=False)

        points = []
        
        # 定义我们要映射到地图的语义类别 (例如: 椅子, 瓶子, 电视)
        target_classes = ['chair', 'couch', 'bed', 'potted plant']

        if results[0].masks is not None:
            # 获取掩码、类别 ID 和置信度
            masks = results[0].masks.data.cpu().numpy()
            boxes = results[0].boxes

            for i, mask in enumerate(masks):
                cls_id = int(boxes.cls[i])
                label = self.model.names[cls_id]
                
                if label in target_classes:
                    # 获取该物体的颜色标识 (用于点云可视化)
                    color_val = self.get_color_by_label(label)
                    
                    # 降低采样率以节省计算资源 (步长为 4 像素)
                    v_indices, u_indices = np.where(mask > 0.5)
                    for idx in range(0, len(v_indices), 4):
                        v, u = v_indices[idx], u_indices[idx]
                        
                        depth = self.latest_depth[v, u] / 1000.0 # 转为米
                        if depth <= 0 or depth > 4.0: continue

                        # 像素坐标 -> 相机光学坐标系 (Optical Frame)
                        z = float(depth)
                        x = (u - self.intrinsics['cx']) * z / self.intrinsics['fx']
                        y = (v - self.intrinsics['cy']) * z / self.intrinsics['fy']
                        
                        # 格式: [x, y, z, rgb_packed]
                        points.append([x, y, z, color_val])

        if points:
            # 创建 PointCloud2 消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera_color_optical_frame" # 必须与相机发布的 frame 一致
            
            # 定义字段 (XYZ + RGB)
            fields = [
                point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='rgb', offset=12, datatype=point_cloud2.PointField.UINT32, count=1),
            ]
            
            pc2_msg = point_cloud2.create_cloud(header, fields, points)
            self.pc_pub.publish(pc2_msg)

    def get_color_by_label(self, label):
        # 为不同物体分配固定的颜色 (Hex -> Int)
        colors = {
            'chair': 0xFF0000,        # 红色
            'couch': 0x00FF00,        # 绿色
            'bed': 0x0000FF,          # 蓝色
            'potted plant': 0xFFFF00   # 黄色
        }
        return colors.get(label, 0xFFFFFF)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()