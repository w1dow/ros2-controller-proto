import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import yaml
from geometry_msgs.msg import Twist
import tf2_ros
import math


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller started")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter(
            "waypoints",
            '/home/w1dow/UGV/task1/test_ws/src/simulation/config/waypoints.yaml'
        )

        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0

        self.offset_initialized = False
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.target_id = 0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.load_waypoints()

        self.timer = self.create_timer(0.1, self.control_loop)

    def update_pose(self):
        try:
            now = self.get_clock().now()

            trans = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                now,
                timeout=Duration(seconds=0.1)
            )

            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y

            q = trans.transform.rotation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.angle = math.atan2(siny, cosy)

            if not self.offset_initialized:
                self.odom_offset_x = self.x
                self.odom_offset_y = self.y
                self.offset_initialized = True

                self.get_logger().info(
                    f"Captured offset: ({self.odom_offset_x:.2f}, {self.odom_offset_y:.2f})"
                )

                self.convert_waypoints_to_odom()

        except Exception:
            self.get_logger().warn("TF not ready")
            return False

        return True

    def convert_waypoints_to_odom(self):
        for wp in self.waypoints:
            wp['x'] -= self.odom_offset_x
            wp['y'] -= self.odom_offset_y

        self.get_logger().info("Waypoints converted to odom frame")

        if self.waypoints:
            self.set_goal(0)

    def load_waypoints(self):
        waypoint_file = self.get_parameter('waypoints').value

        try:
            with open(waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data['waypoints']

            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            self.waypoints = []

    def look(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        if abs(dx) < 0.05 and abs(dy) < 0.05:
            return 0.0

        target_angle = math.atan2(dy, dx)

        return math.atan2(
            math.sin(target_angle - self.angle),
            math.cos(target_angle - self.angle)
        )

    def set_goal(self, id):
        wp = self.waypoints[id]
        self.goal_x = wp['x']
        self.goal_y = wp['y']

        self.get_logger().info(
            f"Target {id} :({self.goal_x:.2f},{self.goal_y:.2f})"
        )

    def get_distance(self):
        return math.sqrt(
            (self.goal_x - self.x) ** 2 +
            (self.goal_y - self.y) ** 2
        )

    def on_reach(self):
        if self.target_id >= len(self.waypoints) - 1:
            self.get_logger().warning("Completed all waypoints")
            return

        self.target_id += 1
        self.set_goal(self.target_id)

    def control_loop(self):
        v = 1.0
        a_v = 1.0
        pos_thres = 0.1
        ang_thres = 0.1

        if not self.waypoints:
            return

        if not self.update_pose():
            return

        if not self.offset_initialized:
            return

        e = self.look()
        distance = self.get_distance()

        print(f"\r Curr({self.x:.2f},{self.y:.2f}) || Targ({self.goal_x:.2f},{self.goal_y:.2f}) : {distance:.3f} , angle error= {e:.3f}", end="", flush=True)

        cmd = Twist()

        if abs(e) > ang_thres:
            cmd.angular.z = max(min(a_v * e, 1.0), -1.0)
            cmd.linear.x = 0.0

        elif distance > pos_thres:
            cmd.linear.x = min(v, distance)
            cmd.angular.z = max(min(a_v * e, 1.0), -1.0)

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.on_reach()

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()