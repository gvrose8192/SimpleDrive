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
        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5).value)  # Changed from 1.0 to 0.5
        self.move_distance = float(self.declare_parameter('move_distance', 0.5).value)
        self.obstacle_threshold = float(self.declare_parameter(
            'obstacle_threshold',
            0.3  # meters - stop distance
        ).value)

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        # State machine variables
        self.state = 'WAIT'
        self.drive_duration = None
        self.action_start_time = None
        self.wait_timeout = 2.0

        # Clock and timer
        self.clock = self.get_clock()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.drive_callback)

        # Create service handler at class level (NOT inside drive_callback!)
        self.reset_service = self.create_service(Trigger, '/simple_drive/reset', self.handle_reset)
        print("Service '/simple_drive/reset' created and ready to receive commands!")

        # Create start service handler at class level
        self.start_service = self.create_service(Trigger, '/simple_drive/start', self.handle_start)
        print("Service '/simple_drive/start' created and ready to receive commands!")


    def handle_reset(self, request, response):
        """Handle reset service calls - MUST be at class level"""
        self.get_logger().info(f"Received reset command: {request}")

        # Reset the state machine
        self.state = 'WAIT'
        self.action_start_time = None
        self.drive_duration = None

        # Clear any pending velocity commands
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        response.success = True
        response.message = "Robot reset successfully and ready to start sequence"

        self.get_logger().info(f"Response: {response.message}")
        return response

    def handle_start(self, request, response):
        """Handle start service calls - immediately starts forward movement"""
        self.get_logger().info(f"🟢 START SERVICE CALLED: {request}")

        # Reset state machine and IMMEDIATELY transition to FORWARD_1
        self.state = 'FORWARD_1'  # ← Start immediately!
        self.action_start_time = self.clock.now().nanoseconds / 1e9  # ← Set current time!
        self.drive_duration = None

        # Clear any pending velocity commands
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        response.success = True
        response.message = "Robot starting forward movement sequence"

        self.get_logger().info(f"Response: {response.message}")
        return response

    def drive_callback(self):
        now = self.clock.now().nanoseconds / 1e9

        # Wait state
        if self.state == 'WAIT':
            print(f"State: WAIT (waiting for /simple_drive/start command)")

        elif self.state == 'FORWARD_1':
            # Check LIDAR obstacle detection FIRST!
            twist_cmd = self.check_obstacle_avoidance()

            # PUBLISH FORWARD VELOCITY EVERY CYCLE
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0

            # Apply obstacle avoidance override if needed
            self.twist = twist_cmd

            self.publisher_.publish(self.twist)

            duration = self.move_distance / self.linear_speed
            if self.action_start_time is not None and (now - self.action_start_time >= duration):
                print("Forward distance completed, transitioning to TURN_SPIN")
                self.state = 'TURN_SPIN'
                self.action_start_time = now

        elif self.state == 'TURN_SPIN':
            # Check LIDAR obstacle detection FIRST!
            twist_cmd = self.check_obstacle_avoidance()

            # PUBLISH SPIN VELOCITY EVERY CYCLE
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.angular_speed

            # Apply obstacle avoidance override if needed
            self.twist = twist_cmd

            self.publisher_.publish(self.twist)

            angle_to_spin = math.pi  # 180° in radians
            spin_duration = angle_to_spin / self.angular_speed

            if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                print("Spin completed, transitioning to FORWARD_2")
                self.state = 'FORWARD_2'
                self.action_start_time = now

        elif self.state == 'FORWARD_2':
            # Check LIDAR obstacle detection FIRST!
            twist_cmd = self.check_obstacle_avoidance()

            # PUBLISH BACKWARD VELOCITY EVERY CYCLE
            speed = self.linear_speed
            self.twist.linear.x = -speed  # Negative for backward motion
            self.twist.angular.z = 0.0

            # Apply obstacle avoidance override if needed
            self.twist = twist_cmd

            self.publisher_.publish(self.twist)

            duration = self.move_distance / abs(speed)
            if self.action_start_time is not None and (now - self.action_start_time >= duration):
                print("Return distance completed, transitioning to FINISH_SPIN")
                self.state = 'FINISH_SPIN'
                self.action_start_time = now

        elif self.state == 'FINISH_SPIN':
            # Check LIDAR obstacle detection FIRST!
            twist_cmd = self.check_obstacle_avoidance()

            # PUBLISH SPIN VELOCITY EVERY CYCLE
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.angular_speed  # Right turn for 180°

            # Apply obstacle avoidance override if needed
            self.twist = twist_cmd

            self.publisher_.publish(self.twist)

            angle_to_spin = math.pi  # 180° in radians
            spin_duration = angle_to_spin / self.angular_speed

            if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                print("Final orientation completed, transitioning to DONE")
                self.state = 'DONE'
                self.action_start_time = now

        elif self.state == 'DONE':
            print(f"State: DONE! Robot has executed its path.")
            # Keep publishing stopped velocity in DONE state
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)

    def check_obstacle_avoidance(self):
        """Check for obstacles using LIDAR data"""
        twist_cmd = Twist()

        # Subscribe to obstacle detection topic (from lidar_subscriber.py)
        try:
            msg = self.get_message('/simple_drive/lidar_obstacle_detected', Float64)

            if msg is not None and msg.data > 0.5:  # Obstacle detected
                self.get_logger().info(f"🔴 OBSTACLE DETECTED! Stopping robot.")

                # Emergency stop
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.0

                # Optionally reset state machine to WAIT when obstacle detected
                # Uncomment below if you want automatic state reset:
                # if self.state in ['FORWARD_1', 'TURN_SPIN', 'FORWARD_2', 'FINISH_SPIN']:
                #     self.state = 'WAIT'
                #     self.action_start_time = None
            else:
                self.get_logger().info(f"✓ Path clear, continuing normal operation")

        except Exception as e:
            self.get_logger().error(f"LIDAR subscription error: {e}")

        return twist_cmd

    def set_duration(self, state_name, duration):
        """Helper to set how long the robot should move in seconds"""
        self.drive_duration = duration
        self.action_start_time = self.clock.now().nanoseconds / 1e9

    def stop_and_wait(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def move_forward(self, speed=None):
        if speed is None:
            speed = self.linear_speed
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
        speed = self.angular_speed
        self.twist.linear.x = 0.0
        self.twist.angular.z = speed
        angle_to_spin = 1 * math.pi
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
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def spin_finish(self):
        speed = self.angular_speed
        self.twist.linear.x = 0.0
        self.twist.angular.z = speed
        self.publisher_.publish(self.twist)

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

