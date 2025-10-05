#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Simple teleop:
Use keys:
  w/s : forward/back
  a/d : rotate left/right
  x   : stop
  q   : quit
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_simple')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Teleop started. Use keyboard in terminal.")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            print(msg)
            while True:
                ch = sys.stdin.read(1)
                twist = Twist()
                if ch == 'w':
                    twist.linear.x = 0.2
                elif ch == 's':
                    twist.linear.x = -0.1
                elif ch == 'a':
                    twist.angular.z = 0.5
                elif ch == 'd':
                    twist.angular.z = -0.5
                elif ch == 'x':
                    twist.linear.x = twist.angular.z = 0.0
                elif ch == 'q':
                    break
                self.pub.publish(twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
