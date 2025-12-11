#!/usr/bin/env python3
"""
Test script to manually publish steering commands to verify Gazebo is responding
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TestSteering(Node):
    def __init__(self):
        super().__init__('test_steering')
        self.publisher = self.create_publisher(Twist, '/ackermann_cmd', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.counter = 0
        
    def publish_command(self):
        msg = Twist()
        msg.linear.x = 1.0  # Constant forward speed
        
        # Oscillate steering: left -> straight -> right -> straight
        period = 50  # 5 seconds per cycle (50 * 0.1s)
        phase = (self.counter % period) / period * 2 * math.pi
        msg.angular.z = 0.5 * math.sin(phase)  # ±0.5 radians steering
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: speed={msg.linear.x:.2f}, steer={msg.angular.z:.3f}')
        self.counter += 1

def main():
    rclpy.init()
    node = TestSteering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
