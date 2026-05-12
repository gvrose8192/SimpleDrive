#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

class ResetServiceNode(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create service that resets simple_drive node
        self.reset_service = self.create_service(Trigger, 'reset', self.handle_reset)
        self.get_logger().info("Reset service created and ready to receive commands!")

    def handle_reset(self, request, response):
        """Handle reset commands"""
        self.get_logger().info(f"🟢 RESET SERVICE CALLED: {request}")
        
        # Publish a stop command
        twist_msg = Twist()
        self.publisher_.publish(twist_msg)
        
        response.success = True
        response.message = "Robot reset successfully and ready to start sequence"
        
        self.get_logger().info(f"Response: {response.message}")
        return response

def main(args=None):
    rclpy.init()
    node = ResetServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

