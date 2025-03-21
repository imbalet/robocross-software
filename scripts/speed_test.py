#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from utils import euler_from_quaternion, decart_to_polar
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose


class SpeedTest(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frequency', 30)
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.cmdPub = self.create_publisher(Twist,
                                            cmd_topic,
                                            10)

        self.mainTimer = self.create_timer(1 / 30, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.
        msg.angular.z = 0.
        self.cmdPub.publish(msg)


def main():
    rclpy.init()
    node = SpeedTest("speed_test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
