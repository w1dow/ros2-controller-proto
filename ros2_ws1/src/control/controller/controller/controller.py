import rclpy
from rclpy.node import Node
import yaml
from turtlesim.srv import Spawn
import math


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller started")

        self.declare_parameter("waypoints", '/home/w1dow/UGV/task1/ros2_ws1/config/waypoints.yaml')

        self.robo = self.create_client(Spawn, '/spawn')

        while not self.robo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        self.robo_req = Spawn.Request()
        self.robo_req.x = 0.0
        self.robo_req.y = 0.0
        self.robo_req.theta = 0.0
        self.robo_req.name = "robot1"

        self.target_id = 0
        self.thres = 0.2

        self.x = 0.0
        self.y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        future = self.robo.call_async(self.robo_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
        else:
            self.get_logger().error("Failed to spawn turtle")

        self.load_waypoints()

        if self.waypoints:
            self.goal_x = self.waypoints[0]['x']
            self.goal_y = self.waypoints[0]['y']


    def spawn_turtle(self, x, y):
        while not self.robo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0

        future = self.robo.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {future.result().name}")
        else:
            self.get_logger().error("Failed to spawn turtle")


    def load_waypoints(self):
        waypoint_file = self.get_parameter('waypoints').value
        scale = 0.1

        try:
            with open(waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data['waypoints']

                for wp in self.waypoints:
                    wp['x'] *= scale
                    wp['y'] *= scale

                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            self.waypoints = []


    def check(self):
        distance=math.sqrt(math.pow((self.x-self.goal_x),2)+math.pow((self.y-self.goal_y),2))
        if distance <= self.thres:
            self.get_logger().info(f"Reached target = {self.goal_x},{self.goal_y}")


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()