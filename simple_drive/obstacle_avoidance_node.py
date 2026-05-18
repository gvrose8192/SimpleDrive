#!/usr/bin/env python3
"""
Obstacle Avoidance Node for Simple Drive - Simplified Version

Subscribes to /scan LIDAR and stops the robot when obstacles are detected
within 0.5m ahead, then spins in place until clear path is found.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters with defaults
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('max_scan_range', 5.0)
        self.declare_parameter('spin_speed', 1.0)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.max_scan_range = float(self.get_parameter('max_scan_range').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to LIDAR scan data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS depth
        )

        # State machine for avoidance behavior
        self.state = 'SAFE'
        self.obstacle_distance = None
        self.clear_threshold = 1.0
        self.scan_message = None  # Initialize before timer runs

        # Create timer to periodically check for obstacles (20Hz)
        self.timer_period = 0.05  # 50ms between checks
        self.timer = self.create_timer(self.timer_period, self.avoidance_callback)

        self.get_logger().info(
            f"Obstacle avoidance initialized. "
            f"Stop distance: {self.stop_distance}m, "
            f"Lidar range: {self.max_scan_range}m, "
            f"Spin speed: {self.spin_speed} rad/s"
        )

    def scan_callback(self, msg):
        """Callback when new LIDAR scan data is received."""
        self.scan_message = msg
        """Debug: print sample readings and obstacle detection."""
        if self.scan_message is None or len(self.scan_message.ranges) == 0:
            return None, []

        ranges = self.scan_message.ranges
        num_rays = len(ranges)

        fov_min_deg = -45.0
        fov_max_deg = 45.0

        # Calculate angles from angle_min and angle_increment (angles not stored in message)
        fov_min_rad = math.radians(fov_min_deg)
        fov_max_rad = math.radians(fov_max_deg)
        angle_increment = self.scan_message.angle_increment

        # Collect all readings in forward FOV for debugging
        forward_readings = []
        for i, distance in enumerate(ranges):
            angle = self.scan_message.angle_min + i * angle_increment
            if fov_min_rad < angle < fov_max_rad:
                forward_readings.append((i, angle, distance))

        print(f"[AVOIDANCE] Forward FOV rays: {len(forward_readings)} of {num_rays} total")

        # Find closest obstacle
        min_distance = float('inf')
        for i, angle, distance in forward_readings:
            if distance <= self.max_scan_range and distance > 0.1:
                if distance < min_distance:
                    min_distance = distance

        return min_distance if min_distance < float('inf') else None, forward_readings

    def find_obstacle(self):
        """Find closest obstacle in forward direction."""
        obstacle_dist, _ = self.debug_find_obstacles()
        return obstacle_dist

    def avoidance_callback(self):
        """Main callback that processes LIDAR data and controls robot."""
        obstacle_dist = self.find_obstacle()
        self.obstacle_distance = obstacle_dist

        if obstacle_dist is None:
            return

        # State machine logic - improved with STOPPED state
        prev_state = self.state

        # If obstacle within stop distance, go to DETERMINING_OBSTACLE
        if obstacle_dist <= self.stop_distance:
            if self.state not in ['DETERMINING_OBSTACLE', 'STOPPED']:
                self.state = 'DETERMINING_OBSTACLE'

        # If obstacle disappeared or moved past clear threshold, return to SAFE
        elif obstacle_dist > self.clear_threshold and self.state in ['DETERMINING_OBSTACLE', 'STOPPED']:
            self.state = 'SAFE'

        # If obstacle is approaching but beyond stop distance, stay ready to stop
        elif self.state == 'DETERMINING_OBSTACLE' and obstacle_dist > self.stop_distance:
            pass  # Obstacle approaching but not yet at stop distance

    def publish_velocity_command(self):
        """Publish velocity command based on current avoidance state."""
        if self.state == 'SAFE':
            return  # Let main node publish through

        twist = Twist()

        if self.state in ['DETERMINING_OBSTACLE']:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.state in ['STOPPED', 'SPINNING_CLEARING']:
            # Override with spin behavior (using spin_speed from main node)
            pass

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
