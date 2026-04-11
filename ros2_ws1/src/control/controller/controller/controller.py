#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WaypointController(Node):

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        super().__init__('waypoint_controller')
        self.declare_parameter("waypoints",'')
        self.waypoints = self.get_parameter('waypoints').get_parameter_value().string_value
        try:
            with open(self.waypoints,'r') as f:
                data=yaml.safe_load(f)
                self.waypoints=data['waypoints']
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
                for wp in self.waypoints:
                    self.get_logger().info(f"{wp}")

        except Exception as  e:
            self.get_logger().error(f"Error {e}")
        self.get_logger().info(f"Waypoints file loaded : {self.waypoints}")


def main(args=None):
    rclpy.init(args=args)

    node = WaypointController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()