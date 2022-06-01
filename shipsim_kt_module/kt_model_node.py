#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from shipsim_msgs_module.msg import KTControl


class KtModelNode(Node):
    """ShipKTModelNode."""

    cmd_vel_Twist = Twist()

    u = 0.0
    rudder_angle_degree = 0.0

    def __init__(
        self,
    ):
        """init."""
        super().__init__("model", namespace="ship1")
        self.declare_parameter("K", 0.280)
        self.declare_parameter("T", 10.0)
        self.declare_parameter("publish_address", "/ship1/cmd_vel")
        self.declare_parameter("subscribe_address", "/ship1/cmd_control")
        self.declare_parameter("delta_time", 0.01)

        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_cmd_vel = self.create_publisher(Twist, publish_address, 10)

        subscribe_address = (
            self.get_parameter("subscribe_address").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            KTControl, subscribe_address, self.listener_callback, 10
        )

        delta_time = self.get_parameter("delta_time").value
        self.timer = self.create_timer(delta_time, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        delta_time = self.get_parameter("delta_time").value
        r_now = self.cmd_vel_Twist.angular.z
        self.cmd_vel_Twist = self.get_twist_from_KT(
            r_now,
            self.u,
            self.rudder_angle_degree,
            delta_time,
        )
        self.pub_cmd_vel.publish(self.cmd_vel_Twist)
        # self.get_logger().info('Publishing: "%s"' % self.cmd_vel_Twist)

    def get_twist_from_KT(self, r_now, u, rudder_angle_degree, delta_time):
        """get_twist_from_KT."""
        twist = Twist()

        K = self.get_parameter("K").value
        T = self.get_parameter("T").value

        twist.linear.x = u
        rudder_angle = rudder_angle_degree * np.pi / 180.0
        r_dot = (K * rudder_angle - r_now) / T
        twist.angular.z = r_now + r_dot * delta_time
        return twist

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info(
            'I heard: u="%s", \
                rudder_angle="%s"'
            % (msg.u, msg.rudder_angle_degree)
        )
        self.u = msg.u
        self.rudder_angle_degree = msg.rudder_angle_degree


def main(args=None):
    """Run main."""
    rclpy.init(args=args)

    node = KtModelNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()