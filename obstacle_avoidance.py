#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('rot_speed', 0.6)
        self.declare_parameter('min_dist', 0.45)
        self.get_logger().info('ObstacleAvoidNode started.')

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=1e3, posinf=1e3, neginf=1e3)
        n = len(ranges)
        mid = n // 2
        # define front sector (approx ±20°)
        sector_width = int(max(5, n * 20 // 360))
        sector = ranges[mid-sector_width: mid+sector_width+1] if n>2*sector_width else ranges
        min_front = sector.min()
        twist = Twist()
        min_dist = self.get_parameter('min_dist').value
        if min_front < min_dist:
            # obstacle detected: rotate in place
            twist.linear.x = 0.0
            twist.angular.z = self.get_parameter('rot_speed').value
            self.get_logger().debug(f'Obstacle detected: min_front={min_front:.2f}')
        else:
            twist.linear.x = self.get_parameter('forward_speed').value
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
