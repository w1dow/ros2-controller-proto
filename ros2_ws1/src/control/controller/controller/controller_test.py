#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class WaypointController(Node):

    def __init__(self):
        super().__init__('waypoint_controller')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.declare_parameter("waypoints", '/home/w1dow/UGV/task1/ros2_ws1/config/waypoints.yaml')
        waypoint_file = self.get_parameter('waypoints').value

        try:
            with open(waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data['waypoints']
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
                for wp in self.waypoints:
                    self.get_logger().info(f"{wp}")
        except Exception as e:
            self.get_logger().error(f"Error {e}")
            self.waypoints = []

        self.get_logger().info(f"Waypoints file loaded : {waypoint_file}")

        self.current_index = 0

        if self.waypoints:
            self.goal_x = self.waypoints[0]['x']
            self.goal_y = self.waypoints[0]['y']
        else:
            self.goal_x = 0.0
            self.goal_y = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def calc_rotation(self):
        target_angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        return (target_angle - self.yaw + math.pi) % (2 * math.pi) - math.pi

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.get_logger().info(f"Pos: ({self.x:.2f}, {self.y:.2f}) | Target: ({self.goal_x}, {self.goal_y})")
        # self.dis=math.sqrt(math.pow((self.x-self.goal_x),2)+math.pow((self.y-self.goal_y),2))


         


    

def main(args=None):
    rclpy.init(args=args)

    node = WaypointController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()