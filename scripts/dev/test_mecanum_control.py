#!/usr/bin/env python3
"""
Mecanum Drive Controller Test Node

Tests various movement patterns for mecanum wheel robot:
- Forward/Backward
- Left/Right strafing
- Rotation
- Diagonal movement
- Combined movements
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from enum import Enum


class TestPhase(Enum):
    """Test phases for mecanum drive"""
    INIT = 0
    FORWARD = 1
    BACKWARD = 2
    STRAFE_RIGHT = 3
    STRAFE_LEFT = 4
    ROTATE_CCW = 5
    ROTATE_CW = 6
    DIAGONAL_FR = 7
    DIAGONAL_FL = 8
    DIAGONAL_BR = 9
    DIAGONAL_BL = 10
    COMBINED_MOVE = 11
    STOP = 12
    COMPLETED = 13


class MecanumTestNode(Node):
    def __init__(self):
        super().__init__('mecanum_test_node')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/mecanum_drive_controller/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mecanum_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Test parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.test_duration = 3.0  # seconds per test

        # State variables
        self.current_phase = TestPhase.INIT
        self.phase_start_time = None
        self.elapsed_time = 0.0

        # Odometry data
        self.current_odom = None
        self.phase_start_odom = None
        self.test_results = {}

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Mecanum Test Node Started')
        self.get_logger().info(f'Linear Speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular Speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')
        self.get_logger().info('=' * 60)

    def odom_callback(self, msg):
        """Store odometry data"""
        self.current_odom = msg

    def start_new_phase(self, phase):
        """Start a new test phase"""
        self.current_phase = phase
        self.phase_start_time = self.get_clock().now()
        self.phase_start_odom = self.current_odom
        self.elapsed_time = 0.0

        phase_name = phase.name
        self.get_logger().info(f'\n>>> Starting Test Phase: {phase_name}')

    def calculate_distance_traveled(self):
        """Calculate distance traveled since phase start"""
        if self.phase_start_odom is None or self.current_odom is None:
            return 0.0

        start_pos = self.phase_start_odom.pose.pose.position
        current_pos = self.current_odom.pose.pose.position

        dx = current_pos.x - start_pos.x
        dy = current_pos.y - start_pos.y

        distance = math.sqrt(dx**2 + dy**2)
        return distance

    def calculate_rotation(self):
        """Calculate rotation since phase start"""
        if self.phase_start_odom is None or self.current_odom is None:
            return 0.0

        # Extract yaw from quaternion
        start_quat = self.phase_start_odom.pose.pose.orientation
        current_quat = self.current_odom.pose.pose.orientation

        start_yaw = self.quaternion_to_yaw(start_quat)
        current_yaw = self.quaternion_to_yaw(current_quat)

        rotation = current_yaw - start_yaw
        return rotation

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def save_phase_results(self, phase_name):
        """Save test results for current phase"""
        distance = self.calculate_distance_traveled()
        rotation = self.calculate_rotation()

        self.test_results[phase_name] = {
            'distance': distance,
            'rotation': rotation,
            'duration': self.elapsed_time
        }

        self.get_logger().info(f'  Distance: {distance:.3f} m')
        self.get_logger().info(f'  Rotation: {math.degrees(rotation):.2f} degrees')
        self.get_logger().info(f'  Duration: {self.elapsed_time:.2f} s')

    def control_loop(self):
        """Main control loop"""
        if self.current_phase == TestPhase.COMPLETED:
            return

        # Initialize phase timing
        if self.phase_start_time is None:
            self.start_new_phase(TestPhase.FORWARD)
            return

        # Update elapsed time
        current_time = self.get_clock().now()
        self.elapsed_time = (current_time - self.phase_start_time).nanoseconds / 1e9

        # Check if phase is complete
        if self.elapsed_time >= self.test_duration:
            # Save results and move to next phase
            phase_name = self.current_phase.name
            self.save_phase_results(phase_name)

            # Transition to next phase
            next_phase = self.get_next_phase()
            if next_phase == TestPhase.COMPLETED:
                self.stop_robot()
                self.print_summary()
                self.current_phase = TestPhase.COMPLETED
                self.get_logger().info('\n' + '=' * 60)
                self.get_logger().info('All tests completed!')
                self.get_logger().info('=' * 60)
                return
            else:
                # Brief stop between phases
                self.stop_robot()
                time.sleep(0.5)
                self.start_new_phase(next_phase)

        # Execute current phase command
        self.execute_phase_command()

    def get_next_phase(self):
        """Get next test phase"""
        phase_order = [
            TestPhase.FORWARD,
            TestPhase.BACKWARD,
            TestPhase.STRAFE_RIGHT,
            TestPhase.STRAFE_LEFT,
            TestPhase.ROTATE_CCW,
            TestPhase.ROTATE_CW,
            TestPhase.DIAGONAL_FR,
            TestPhase.DIAGONAL_FL,
            TestPhase.DIAGONAL_BR,
            TestPhase.DIAGONAL_BL,
            TestPhase.COMBINED_MOVE,
        ]

        try:
            current_index = phase_order.index(self.current_phase)
            if current_index < len(phase_order) - 1:
                return phase_order[current_index + 1]
        except ValueError:
            pass

        return TestPhase.COMPLETED

    def execute_phase_command(self):
        """Execute velocity command for current phase"""
        msg = Twist()

        if self.current_phase == TestPhase.FORWARD:
            msg.linear.x = self.linear_speed

        elif self.current_phase == TestPhase.BACKWARD:
            msg.linear.x = -self.linear_speed

        elif self.current_phase == TestPhase.STRAFE_RIGHT:
            msg.linear.y = self.linear_speed

        elif self.current_phase == TestPhase.STRAFE_LEFT:
            msg.linear.y = -self.linear_speed

        elif self.current_phase == TestPhase.ROTATE_CCW:
            msg.angular.z = self.angular_speed

        elif self.current_phase == TestPhase.ROTATE_CW:
            msg.angular.z = -self.angular_speed

        elif self.current_phase == TestPhase.DIAGONAL_FR:
            # Forward-Right diagonal
            msg.linear.x = self.linear_speed * 0.707
            msg.linear.y = self.linear_speed * 0.707

        elif self.current_phase == TestPhase.DIAGONAL_FL:
            # Forward-Left diagonal
            msg.linear.x = self.linear_speed * 0.707
            msg.linear.y = -self.linear_speed * 0.707

        elif self.current_phase == TestPhase.DIAGONAL_BR:
            # Backward-Right diagonal
            msg.linear.x = -self.linear_speed * 0.707
            msg.linear.y = self.linear_speed * 0.707

        elif self.current_phase == TestPhase.DIAGONAL_BL:
            # Backward-Left diagonal
            msg.linear.x = -self.linear_speed * 0.707
            msg.linear.y = -self.linear_speed * 0.707

        elif self.current_phase == TestPhase.COMBINED_MOVE:
            # Move forward while rotating
            msg.linear.x = self.linear_speed * 0.5
            msg.linear.y = self.linear_speed * 0.3
            msg.angular.z = self.angular_speed * 0.5

        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        """Send stop command"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def print_summary(self):
        """Print test summary"""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('=' * 60)

        for phase_name, results in self.test_results.items():
            self.get_logger().info(f'\n{phase_name}:')
            self.get_logger().info(f"  Distance: {results['distance']:.3f} m")
            self.get_logger().info(f"  Rotation: {math.degrees(results['rotation']):.2f} deg")
            self.get_logger().info(f"  Duration: {results['duration']:.2f} s")

        self.get_logger().info('\n' + '=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        # Stop robot before shutting down
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
