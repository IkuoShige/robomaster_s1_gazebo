#!/usr/bin/env python3
"""
Mecanum Wheel Teleop Keyboard Node for RoboMaster S1

Controls mecanum wheels with full omnidirectional movement:
- Forward/Backward (linear.x)
- Left/Right lateral movement (linear.y)
- Rotation (angular.z)

Key Bindings:
   u    i    o
   j    k    l
   m    ,    .

Moving:
   u/o : Forward + Rotate
   i   : Forward
   j/l : Lateral Left/Right (mecanum)
   ,   : Backward
   m/. : Backward + Rotate
   k   : Stop

Speed Control:
   q/z : Increase/Decrease linear speed
   w/x : Increase/Decrease angular speed
   SPACE : Emergency stop

Press CTRL+C to quit
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Key bindings for movement
moveBindings = {
    'i': (1, 0, 0, 0),   # Forward
    'o': (1, 0, 0, -1),  # Forward + Rotate Right
    'j': (0, 1, 0, 0),   # Lateral Left
    'l': (0, -1, 0, 0),  # Lateral Right
    'u': (1, 0, 0, 1),   # Forward + Rotate Left
    ',': (-1, 0, 0, 0),  # Backward
    '.': (-1, 0, 0, -1), # Backward + Rotate Right
    'm': (-1, 0, 0, 1),  # Backward + Rotate Left
    'k': (0, 0, 0, 0),   # Stop
}

# Speed control bindings
speedBindings = {
    'q': (1.1, 1.0),  # Increase linear speed
    'z': (0.9, 1.0),  # Decrease linear speed
    'w': (1.0, 1.1),  # Increase angular speed
    'x': (1.0, 0.9),  # Decrease angular speed
}


def getKey(settings):
    """Get a single keypress from terminal"""
    tty.setraw(sys.stdin.fileno())
    # Use select to check if data is available
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    """Return formatted velocity string"""
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"


class MecanumTeleopNode(Node):
    def __init__(self):
        super().__init__('mecanum_teleop_keyboard')

        # Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initial speeds
        self.speed = 0.5   # m/s
        self.turn = 1.0    # rad/s

        # Current velocities
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(__doc__)
        self.get_logger().info(vels(self.speed, self.turn))

    def run(self):
        """Main teleop loop"""
        try:
            while rclpy.ok():
                key = getKey(self.settings)

                if key == ' ':
                    # Emergency stop
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    self.get_logger().info("EMERGENCY STOP")

                elif key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]

                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]

                    # Limit speeds
                    self.speed = max(0.1, min(2.0, self.speed))
                    self.turn = max(0.1, min(3.0, self.turn))

                    self.get_logger().info(vels(self.speed, self.turn))

                elif key == '\x03':  # CTRL+C
                    break

                else:
                    # Unknown key - continue without stopping
                    if key != '':
                        self.get_logger().warn(f"Unknown key: {key}")
                    continue

                # Publish velocity command
                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn

                self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # Stop robot before exiting
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)

    node = MecanumTeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
