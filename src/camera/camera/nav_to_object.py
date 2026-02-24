import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavToObjectClient(Node):
    def __init__(self):
        super().__init__('nav_to_object_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self.get_logger().info('正在启动导航任务：前往椅子旁边...')
        return self._action_client.send_goal_async(goal_msg)

# ... 省略部分 Action 状态监听逻辑