#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import math

class SimpleDriveNode(Node):
    def __init__(self):
        super().__init__('simple_drive')
        
        # Parameters
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.2).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 1.0).value)
        self.move_distance = float(self.declare_parameter('move_distance', 0.5).value)
        
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
        self.reset_service = self.create_service(Trigger, 'reset', self.handle_reset)
        print("Service 'reset' created and ready to receive commands!")

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

    def drive_callback(self):
        now = self.clock.now().nanoseconds / 1e9
        
        # Wait state
        if self.state == 'WAIT':
            print(f"State: WAIT (time={now:.1f}s)")
            if now >= self.wait_timeout:
                print("Timeout reached, transitioning to FORWARD_1")
                self.state = 'FORWARD_1'
                self.action_start_time = now
        
        elif self.state == 'FORWARD_1':
            print(f"State: FORWARD_1 (moving forward at {self.linear_speed} m/s)")
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)
            
            duration = self.move_distance / self.linear_speed
            if self.action_start_time is not None and (now - self.action_start_time >= duration):
                print("Forward distance completed, transitioning to TURN_SPIN")
                self.state = 'TURN_SPIN'
                self.action_start_time = now
        
        elif self.state == 'TURN_SPIN':
            print(f"State: TURN_SPIN (spinning at {self.angular_speed} rad/s)")
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed
            self.publisher_.publish(self.twist)
            
            angle_to_spin = math.pi
            spin_duration = angle_to_spin / self.angular_speed
            if self.action_start_time is not None and (now - self.action_start_time >= spin_duration):
                print("Spin completed, transitioning to FORWARD_2")
                self.state = 'FORWARD_2'
                self.action_start_time = now
        
        elif self.state == 'FORWARD_2':
            print(f"State: FORWARD_2 (moving backward at {abs(-self.linear_speed)} m/s)")
            speed = -self.linear_speed
            self.twist.linear.x = speed
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)
            
            duration = self.move_distance / abs(speed)
            if self.action_start_time is not None and (now - self.action_start_time >= duration):
                print("Return distance completed, transitioning to DONE")
                self.state = 'DONE'
                self.action_start_time = now
        
        elif self.state == 'DONE':
            print(f"State: DONE! Robot has returned home.")
            # Keep publishing stopped velocity in DONE state
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)

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
        angle_to_spin = 2 * math.pi
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

