#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import requests
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SemanticNavNode(Node):
    def __init__(self):
        super().__init__('semantic_nav_node')

        # 1. 加载语义地图数据库（指向你的 camera 功能包）
        self.room_map = {}
        self.load_rooms_config()

        # 2. 初始化 Nav2 Action 客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. 订阅用户指令话题
        self.subscription = self.create_subscription(String, '/user_command', self.listener_callback, 10)
        
        self.get_logger().info('实机语义导航节点已启动，等待指令...')

    def load_rooms_config(self):
        try:
            # 修改：获取你自己的功能包路径
            package_share_directory = get_package_share_directory('camera')
            # 路径指向你刚才创建的 config/rooms.yaml
            config_path = os.path.join(package_share_directory, 'config', 'rooms.yaml')

            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                self.room_map = config_data.get('rooms', {})
                self.get_logger().info(f"成功加载实机房间信息: {list(self.room_map.keys())}")
        except Exception as e:
            self.get_logger().error(f"无法加载 rooms.yaml 配置文件: {e}")

    def listener_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'收到指令: "{user_text}"')
        
        try:
            # 这里调用你本地部署的 Qwen-2B 服务 (FastAPI)
            response = requests.post(
                "http://localhost:8000/parse_command", 
                json={"text": user_text}, 
                timeout=10
            )
            
            if response.status_code == 200:
                res_data = response.json()
                # 兼容不同格式的解析结果
                target_room = None
                if isinstance(res_data, list) and len(res_data) > 0:
                    target_room = res_data[0].get("target") if isinstance(res_data[0], dict) else res_data[0]
                elif isinstance(res_data, dict):
                    target_room = res_data.get("target")

                if target_room in self.room_map:
                    pos = self.room_map[target_room]
                    self.get_logger().info(f"大模型解析成功，目标地: {target_room}")
                    self.send_nav_goal(pos)
                else:
                    self.get_logger().warn(f"目标 '{target_room}' 不在数据库中")
        except Exception as e:
            self.get_logger().error(f"API 请求失败: {e}")

    def send_nav_goal(self, pos):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(pos[0])
        goal_msg.pose.pose.position.y = float(pos[1])
        
        th = float(pos[2])
        goal_msg.pose.pose.orientation.w = math.cos(th * 0.5)
        goal_msg.pose.pose.orientation.z = math.sin(th * 0.5)

        # --- 在此处插入日志打印 ---
        self.get_logger().info('='*40)
        self.get_logger().info(f"🚀 语义导航动作已触发！")
        self.get_logger().info(f"📍 目标坐标 (X, Y): {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}")
        self.get_logger().info(f"🧭 目标朝向 (Yaw): {th} 弧度")
        self.get_logger().info("ℹ️  正在等待 Nav2 服务响应 (无底盘时此处可能会卡住或报错)...")
        self.get_logger().info('='*40)
        # ------------------------

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()