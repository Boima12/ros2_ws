#!/usr/bin/env python3
import rclpy  # <--- SỬA THÀNH rclpy
from rclpy.node import Node # <--- SỬA THÀNH rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

class MessageAdapter(Node):
    def __init__(self):
        super().__init__('msg_adapter_node')
        
        # 1. Nghe lệnh Ackermann từ Control Node
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.listener_callback,
            10)
            
        # 2. Dịch sang Twist gửi cho Gazebo
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Adapter Started: /ackermann_cmd -> /cmd_vel')

    def listener_callback(self, msg):
        twist = Twist()
        # --- LOGIC CHUYỂN ĐỔI ---
        # Tốc độ -> Linear X
        twist.linear.x = msg.drive.speed
        
        # Góc lái -> Angular Z
        twist.angular.z = msg.drive.steering_angle
        
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args) # <--- SỬA THÀNH rclpy
    node = MessageAdapter()
    rclpy.spin(node)      # <--- SỬA THÀNH rclpy
    node.destroy_node()
    rclpy.shutdown()      # <--- SỬA THÀNH rclpy

if __name__ == '__main__':
    main()
