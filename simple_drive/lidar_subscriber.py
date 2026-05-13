#!/usr/bin/env python3
"""
LIDAR Subscriber for JetAuto Obstacle Avoidance
Subscribes to A1 LIDAR data and publishes obstacle detection status
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class LidarSubscriber(Node):
    """LIDAR data subscriber for obstacle avoidance"""
    
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # Create publisher for obstacle status (used by main node)
        self.obstacle_status_pub = self.create_publisher(
            Float64, 
            '/simple_drive/lidar_obstacle_detected',  # Custom topic
            10
        )
        
        # Subscribe to A1 LIDAR data
        # Adjust topic name based on your actual sensor configuration
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/jet_auto/laser_scan',  # Check this topic name in ROS2
            self.lidar_callback,
            10
        )
        
        # Parameters
        self.obstacle_threshold = float(self.declare_parameter(
            'obstacle_threshold',
            0.3  # meters - stop distance
        ).value)
        
        self.scan_rate = float(self.declare_parameter(
            'scan_rate',
            10  # Hz
        ).value)
        
        self.get_logger().info(f"🔍 LIDAR subscriber initialized")
        self.get_logger().info(f"Threshold: {self.obstacle_threshold}m, Rate: {self.scan_rate}Hz")

    def lidar_callback(self, msg):
        """Process LIDAR data and detect obstacles"""
        
        # Check for obstacles in front of robot (adjust angle range as needed)
        obstacle_detected = False
        min_distance = 1e9
        
        # Scan only forward sector (adjust angles based on your LIDAR setup)
        # A1 typically scans -270° to +270°, we focus on forward directions
        start_idx = int(60 * len(msg.ranges))  # Start at 60° from center
        end_idx = int(300 * len(msg.ranges))    # End at 300° from center
        
        for i in range(start_idx, end_idx):
            if msg.ranges[i] > 0 and msg.ranges[i] < msg.range_max:
                if msg.ranges[i] < self.obstacle_threshold:
                    obstacle_detected = True
                    min_distance = msg.ranges[i]
                    break
        
        # Publish obstacle detection status
        if obstacle_detected:
            msg_status = Float64()
            msg_status.data = 1.0  # Obstacle detected
            self.obstacle_status_pub.publish(msg_status)
            self.get_logger().info(f"🔴 OBSTACLE DETECTED at {min_distance:.2f}m!")
        else:
            msg_status = Float64()
            msg_status.data = 0.0  # No obstacle
            self.obstacle_status_pub.publish(msg_status)
            self.get_logger().info(f"✓ Clear path ahead")


def main(args=None):
    rclpy.init()
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

