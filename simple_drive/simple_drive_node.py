#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
import math


class SimpleDriveNode(Node):
    def __init__(self):
        super().__init__('simple_drive')

        # Parameters
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.2).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5).value)
        self.move_distance = float(self.declare_parameter('move_distance', 1.25).value)

        # Obstacle avoidance parameters with defaults
        self.stop_distance = float(self.declare_parameter('stop_distance', 0.5).value)
        self.spin_speed = float(self.declare_parameter('spin_speed', 1.0).value)
        self.max_spin_attempts = int(self.declare_parameter('max_spin_attempts', 4).value)
        self.attempt_timeout = float(self.declare_parameter('attempt_timeout', 2.5).value)

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        # State machine variables
        self.state = 'WAIT'
        self.drive_duration = None
        self.action_start_time = None
        self.wait_timeout = 2.0

        # LIDAR subscription for obstacle avoidance (30Hz polling for responsiveness)
        self.scan_message = None
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Clock and timer
        self.clock = self.get_clock()
        self.timer_period = 0.033  # ~30Hz for responsive obstacle avoidance
        self.timer = self.create_timer(self.timer_period, self.drive_callback)

        # Create services (MUST be instance methods to access self)
        self.reset_service = self.create_service(Trigger, '/simple_drive/reset', self.handle_reset)
        self.start_service = self.create_service(Trigger, '/simple_drive/start', self.handle_start)

        # Spin attempt tracking variables (initialized in handle_start for consistency)
        self.spin_attempt_num = 0
        self.spin_attempt_time = None

        self.get_logger().info(
            f"Simple Drive Node initialized. "
            f"Linear speed: {self.linear_speed} m/s, Angular speed: {self.angular_speed} rad/s, "
            f"Move distance: {self.move_distance}m, Stop distance: {self.stop_distance}m, "
            f"Spin speed: {self.spin_speed} rad/s"
        )


    def handle_reset(self, request, response):
        """Handle reset service calls - MUST be at class level"""
        self.state = 'WAIT'
        self.action_start_time = None
        self.drive_duration = None

        # Clear any pending velocity commands
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        # Reset obstacle tracking (including spin attempt state)
        self.scan_message = None
        self.spin_attempt_num = 0
        self.spin_attempt_time = None

        response.success = True
        response.message = "Robot reset successfully"
        return response

    def scan_callback(self, msg):
        """Callback when new LIDAR scan data is received."""
        self.scan_message = msg

    def find_closest_obstacle(self):
        """Find closest obstacle in forward direction."""
        if self.scan_message is None or len(self.scan_message.ranges) == 0:
            return None

        ranges = self.scan_message.ranges
        max_range = self.scan_message.range_max
        angle_min = self.scan_message.angle_min
        angle_increment = self.scan_message.angle_increment

        # Collect all valid readings (use actual LIDAR FOV from message)
        forward_readings = []
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            # Include readings that are valid (not noise, not beyond max range)
            is_valid = (distance < max_range * 0.95) and (distance >= self.scan_message.range_min)
            if is_valid:
                forward_readings.append((i, angle, distance))

        # Find closest obstacle (valid readings only)
        min_distance = float('inf')
        for i, angle, distance in forward_readings:
            if distance < min_distance:
                min_distance = distance

        return min_distance if min_distance < float('inf') else None

    def handle_start(self, request, response):
        """Handle start service calls - immediately starts forward movement"""
        # Reset state machine and IMMEDIATELY transition to FORWARD_1
        self.state = 'FORWARD_1'
        self.action_start_time = self.clock.now().nanoseconds / 1e9
        self.drive_duration = None

        # Clear any pending velocity commands
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        # Reset obstacle avoidance tracking
        self.scan_message = None
        self.spin_attempt_num = 0
        self.spin_attempt_time = None

        response.success = True
        response.message = "Starting movement sequence"
        return response

    def drive_callback(self):
        now = self.clock.now().nanoseconds / 1e9

        # Check for obstacle in moving states (FORWARD_1, TURN_SPIN, FORWARD_2, FINISH_SPIN)
        obstacle_distance = None
        if self.state in ['FORWARD_1', 'TURN_SPIN', 'FORWARD_2', 'FINISH_SPIN']:
            obstacle_distance = self.find_closest_obstacle()

        # Wait state
        if self.state == 'WAIT':
            return

        elif self.state == 'FORWARD_1':
            # Obstacle avoidance - emergency stop if obstacle detected
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)

                duration = self.move_distance / self.linear_speed
                if self.action_start_time is not None and (now - self.action_start_time >= duration):
                    self.state = 'TURN_SPIN'
                    self.action_start_time = now

        elif self.state == 'OBSTACLE_AVOID':
            obstacle_dist = self.find_closest_obstacle()

            # Initialize angle tracking if needed
            if self.spin_attempt_time is None:
                self.spin_attempt_time = now

            # Calculate total spin angle accumulated so far (45° per additional attempt)
            angle_per_spin = math.pi / 4  # 45° per spin check
            base_angle = (self.spin_attempt_num - 1) * angle_per_spin  # Previous full spins worth of angle

            time_since_spin = now - self.spin_attempt_time
            spin_duration = angle_per_spin / self.spin_speed  # ~0.6s for 45° at 1 rad/s

            # Calculate accumulated angular movement since last timer reset
            accumulated_angle = self.spin_speed * time_since_spin
            total_angle = base_angle + accumulated_angle

            # Check if we've spun at least 45° (~0.6s) and should check for clear path
            if accumulated_angle >= angle_per_spin * 0.9:
                self.spin_attempt_time = now

                # Check if obstacle has moved far enough away to resume forward movement
                if obstacle_dist is not None and obstacle_dist > self.stop_distance:
                    self.state = 'FORWARD_1'
                    self.action_start_time = now

            current_angular_vel = 0.0

            if obstacle_dist is None:
                # No LIDAR data - keep spinning to try to find clear path
                if self.spin_attempt_num <= self.max_spin_attempts:
                    current_angular_vel = -self.spin_speed
            elif self.spin_attempt_num < self.max_spin_attempts and accumulated_angle < angle_per_spin * 0.9:
                # Keep spinning to accumulate more angle for next path check
                current_angular_vel = -self.spin_speed

            # Always apply zero linear velocity in OBSTACLE_AVOID state
            self.twist.linear.x = 0.0
            self.twist.angular.z = current_angular_vel
            self.publisher_.publish(self.twist)

        elif self.state == 'TURN_SPIN':
            # Obstacle avoidance check
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed
                self.publisher_.publish(self.twist)

                angle_to_spin = math.pi  # 180° in radians
                spin_duration = angle_to_spin / self.angular_speed

                if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                    self.state = 'FORWARD_2'
                    self.action_start_time = now

        elif self.state == 'FORWARD_2':
            # Obstacle avoidance check
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                speed = self.linear_speed  # moving backward relative to original heading
                self.twist.linear.x = speed
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)

                duration = self.move_distance / abs(speed)
                if self.action_start_time is not None and (now - self.action_start_time >= duration):
                    self.state = 'FINISH_SPIN'
                    self.action_start_time = now

        elif self.state == 'FINISH_SPIN':
            # Obstacle avoidance check
            if obstacle_distance is not None and obstacle_distance <= self.stop_distance:
                self.state = 'OBSTACLE_AVOID'
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed  # Right turn for 180°
                self.publisher_.publish(self.twist)

                angle_to_spin = math.pi  # 180° in radians
                spin_duration = angle_to_spin / self.angular_speed

                if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                    self.state = 'DONE'
                    self.action_start_time = now

        elif self.state == 'DONE':
            # Keep publishing stopped velocity in DONE state
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)

    def set_duration(self, state_name, duration):
        """Helper to set how long the robot should move in seconds"""
        self.drive_duration = duration
        self.action_start_time = self.clock.now().nanoseconds / 1e9

    def stop_and_wait(self):
        """Stop and wait - always publishes zero velocity"""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def move_forward(self, speed=None):
        """Helper to move forward - respects OBSTACLE_AVOID state"""
        if speed is None:
            speed = self.linear_speed

        # Always check obstacle avoidance first
        if self.state == 'OBSTACLE_AVOID':
            return True

        self.twist.linear.x = speed
        self.twist.angular.z = 0.0
        duration = self.move_distance / speed
        now = self.clock.now().nanoseconds / 1e9

        if self.action_start_time is not None and (now - self.action_start_time >= duration):
            self.state = 'TURN_SPIN'
            self.action_start_time = now
            return False
        self.publisher_.publish(self.twist)
        return True

    def spin_robot(self):
        """Helper to spin robot - respects OBSTACLE_AVOID state"""
        speed = self.angular_speed

        # Always check obstacle avoidance first
        if self.state == 'OBSTACLE_AVOID':
            return True

        self.twist.linear.x = 0.0
        self.twist.angular.z = speed
        angle_to_spin = math.pi
        spin_duration = angle_to_spin / speed

        if self.state == 'TURN_SPIN':
            now = self.clock.now().nanoseconds / 1e9
            if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                self.state = 'FORWARD_2'
                self.action_start_time = now
                return False

        self.publisher_.publish(self.twist)
        return True

    def stop_drive_and_exit(self):
        """Stop the robot - respects OBSTACLE_AVOID state"""
        if self.state == 'OBSTACLE_AVOID':
            pass  # Already stopped by obstacle avoidance
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)

    def spin_finish(self):
        """Spin for finish - respects OBSTACLE_AVOID state"""
        if self.state == 'OBSTACLE_AVOID':
            return True

        speed = self.angular_speed
        self.twist.linear.x = 0.0
        self.twist.angular.z = speed
        self.publisher_.publish(self.twist)
        return True

def main(args=None):
    rclpy.init()
    node = SimpleDriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

