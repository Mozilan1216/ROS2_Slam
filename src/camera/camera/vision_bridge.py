#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
from transformers import Qwen2VLForConditionalGeneration, AutoProcessor
from qwen_vl_utils import process_vision_info
import PIL.Image
import io

class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_bridge')
        self.bridge = CvBridge()
        
        # 1. 订阅与发布
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(String, '/room_command', 10)
        
        # 2. 加载本地模型 (使用你提供的路径)
        self.model_path = "./Qwen/Qwen2-VL-2B-Instruct"
        self.get_logger().info(f"正在加载模型至 RTX 4060: {self.model_path}")
        
        self.model = Qwen2VLForConditionalGeneration.from_pretrained(
            self.model_path, 
            torch_dtype="auto", 
            device_map="auto"
        ).eval()
        self.processor = AutoProcessor.from_pretrained(self.model_path)
        
        self.is_inferencing = False
        self.get_logger().info("本地模型加载完成，视觉节点就绪。")

    def image_callback(self, msg):
        if self.is_inferencing:
            return

        self.is_inferencing = True
        try:
            # A. 将 ROS 图像转换为 PIL 格式 (Qwen2-VL 需要)
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 转换颜色通道从 BGR 到 RGB
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            pil_img = PIL.Image.fromarray(rgb_img)

            # B. 构造消息格式
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "image", "image": pil_img},
                        {"type": "text", "text": "你是一个家庭服务机器人。请看这张图片，如果看到书架输出'study'；看到餐桌输出'dining_room'；看到床输出'bedroom'；看到沙发输出'living_room'；看到门口输出'front_door'。仅输出一个关键词，不要输出其他描述。"}
                    ],
                }
            ]

            # C. 模型推理
            text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
            image_inputs, _ = process_vision_info(messages)
            inputs = self.processor(text=[text], images=image_inputs, padding=True, return_tensors="pt").to("cuda")

            with torch.no_grad():
                generated_ids = self.model.generate(**inputs, max_new_tokens=20)
            
            output_text = self.processor.batch_decode(
                generated_ids, skip_special_tokens=True, clean_up_tokenization_spaces=False
            )
            
            # D. 解析并发布结果
            room_name = output_text[0].split("assistant\n")[-1].strip().lower()
            self.get_logger().info(f"识别结果: {room_name}")

            if room_name in ['study', 'dining_room', 'bedroom', 'living_room', 'front_door']:
                result_msg = String()
                result_msg.data = room_name
                self.cmd_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"推理出错: {str(e)}")
        
        self.is_inferencing = False

def main(args=None):
    rclpy.init(args=args)
    node = VisionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()