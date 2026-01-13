# 用于监测相机、里程计、闭环是否正常
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from rtabmap_msgs.msg import Info # 注意 ROS2 中包名略有不同

class SLAMMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # 1. 订阅彩色图像 (检查驱动)
        self.img_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_cb, 10)
            
        # 2. 订阅里程计 (检查视觉算法是否成功)
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom', self.odom_cb, 10)
            
        # 3. 订阅 RTAB-Map 状态 (检查闭环)
        self.info_sub = self.create_subscription(
            Info, '/rtabmap/info', self.info_cb, 10)

        self.get_logger().info("SLAM Monitor 启动，等待数据中...")

    def image_cb(self, msg):
        # 只要这里有输出，说明相机驱动正常
        self.get_logger().info("收到图像 [OK]", throttle_duration_sec=2.0)

    def odom_cb(self, msg):
        # 只要这里有输出，说明视觉里程计匹配成功！画面应该出来了
        pos = msg.pose.pose.position
        self.get_logger().info(f"里程计定位中: x={pos.x:.2f}, y={pos.y:.2f} [SUCCESS]")

    def info_cb(self, msg):
        # 检查是否发现闭环（地图修正）
        if msg.loop_closure_id > 0:
            self.get_logger().info(f"检测到回环! ID: {msg.loop_closure_id}")

def main():
    rclpy.init()
    node = SLAMMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()