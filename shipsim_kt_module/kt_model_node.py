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
        K=0.280,
        T=10.0,
        publish_address="/ship1/cmd_vel",
        timer_period=0.01,
    ):
        """init."""
        super().__init__("ship_model")
        self.delta_time = timer_period
        self.K = K
        self.T = T
        self.pub_cmd_vel = self.create_publisher(Twist, publish_address, 10)

        self.subscription = self.create_subscription(
            KTControl, "ship1/control", self.listener_callback, 10
        )

        self.timer = self.create_timer(timer_period, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        r_now = self.cmd_vel_Twist.angular.z
        self.cmd_vel_Twist = self.get_twist_from_KT(
            r_now,
            self.u,
            self.rudder_angle_degree,
            self.delta_time,
        )
        self.pub_cmd_vel.publish(self.cmd_vel_Twist)
        # self.get_logger().info('Publishing: "%s"' % self.cmd_vel_Twist)

    def get_twist_from_KT(self, r_now, u, rudder_angle_degree, delta_time):
        """get_twist_from_KT."""
        twist = Twist()
        twist.linear.x = u
        rudder_angle = rudder_angle_degree * np.pi / 180.0
        r_dot = (self.K * rudder_angle - r_now) / self.T
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