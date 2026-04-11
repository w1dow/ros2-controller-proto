#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math


class SimpleSimulator(Node):

    def __init__(self):
        super().__init__('simulator')

        self.x = 0.0 #x-coord
        self.y = 0.0 #y-coord
        self.angle = 0.0 # angle from x-axia

        self.velo = 0.0 #velocity
        self.omega = 0.0 #angular velocity

        self.create_subscription(Twist,'/cmd_vel',self.on_recv_vel,10)
        self.info_pub = self.create_publisher(Odometry,'/odom',10)  

    #simulation timer
        self.dt = 0.1  # 10 Hz
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("Simulation Started")

    def on_recv_vel(self, msg):
        self.velo = msg.linear.x
        self.omega = msg.angular.z
    


    def update(self):
        self.angle += self.omega * self.dt
        self.x += self.velo * math.cos(self.angle) * self.dt
        self.y += self.velo * math.sin(self.angle) * self.dt
        self.get_logger().info(f"x: {self.x:.2f}, y: {self.y:.2f}", throttle_duration_sec=1)

        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        self.info_pub.publish(odom)
        # self.get_logger().info(f"x: {self.x:.2f}, y: {self.y:.2f}", throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)

    node = SimpleSimulator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()