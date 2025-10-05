#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from enum import Enum

class State(Enum):
    IDLE = 0
    EXPLORING = 1
    AVOIDING = 2

class FSMController(Node):
    def __init__(self):
        super().__init__('fsm_controller')
        self.state = State.IDLE
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.srv = self.create_service(Trigger, 'start_exploration', self.start_cb)
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.declare_parameter('min_dist', 0.45)
        self.get_logger().info('FSM Controller started in IDLE.')
        self.latest_min_dist = 100.0
        self.explore_turn = 0

    def start_cb(self, req, resp):
        if self.state == State.IDLE:
            self.state = State.EXPLORING
            resp.success = True
            resp.message = 'Exploration started'
            self.get_logger().info('Start command received: going to EXPLORING.')
        else:
            resp.success = False
            resp.message = 'Already running'
        return resp

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=1e3, posinf=1e3, neginf=1e3)
        n = len(ranges)
        mid = n // 2
        sector_width = int(max(5, n * 20 // 360))
        sector = ranges[mid-sector_width: mid+sector_width+1] if n>2*sector_width else ranges
        self.latest_min_dist = sector.min()

    def timer_cb(self):
        min_dist = self.get_parameter('min_dist').value
        twist = Twist()

        if self.state == State.IDLE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.state == State.EXPLORING:
            if self.latest_min_dist < min_dist:
                self.get_logger().info('Transition to AVOIDING (obstacle).')
                self.state = State.AVOIDING
            else:
                twist.linear.x = 0.12
                twist.angular.z = 0.06 * math.sin(self.explore_turn)
                self.explore_turn += 0.2

        elif self.state == State.AVOIDING:
            twist.linear.x = 0.0
            twist.angular.z = 0.7
            if self.latest_min_dist > (min_dist + 0.15):
                self.get_logger().info('Path clear: back to EXPLORING.')
                self.state = State.EXPLORING

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FSMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
