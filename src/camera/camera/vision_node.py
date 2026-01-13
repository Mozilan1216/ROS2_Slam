#!/usr/bin/env python3
# 在ROS2总订阅相机话题
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # 创建一个订阅者，订阅 D435i 的彩色图像话题
        # 默认话题名通常是 /camera/camera/color/image_raw
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10) # 队列大小
        
        self.bridge = CvBridge()
        self.get_logger().info('D435i 视觉节点已启动，正在监听图像...')

    def image_callback(self, msg):
        try:
            # 使用 cv_bridge 将 ROS Image 消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- 这里可以加入你的识别代码 (比如 OpenCV 处理) ---
            # 简单的测试：在画面上画个圆
            cv2.circle(cv_image, (320, 240), 10, (0, 255, 0), -1)
            
            # 显示图像
            cv2.imshow("D435i ROS2 Node Stream", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'转换图像失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()