import rclpy
from rclpy.node import Node
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller started")

        self.declare_parameter(
            "waypoints",
            '/home/w1dow/UGV/task1/test_ws/src/simulation/config/waypoints.yaml'
        )

        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.target_id = 0

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_recv_info, 10)

        self.load_waypoints()

        if self.waypoints:
            self.set_goal(0)

        # PID state
        self.prev_e = 0.0
        self.int_e = 0.0

        self.prev_d = 0.0
        self.int_d = 0.0

        self.dt = 0.1

        # gains
        self.kp_ang = 2.0
        self.ki_ang = 0.0
        self.kd_ang = 0.5

        self.kp_lin = 1.0
        self.ki_lin = 0.0
        self.kd_lin = 0.2

        self.timer = self.create_timer(0.1, self.control_loop)

    def on_recv_info(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.angle = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

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

        target_angle = math.atan2(dy, dx)
        e = target_angle - self.angle

        return math.atan2(math.sin(e), math.cos(e))

    def set_goal(self, i):
        wp = self.waypoints[i]
        self.goal_x = wp['x']
        self.goal_y = wp['y']
        self.get_logger().info(f"Target {i} :({self.goal_x},{self.goal_y})")

    def get_distance(self):
        return math.sqrt(
            (self.goal_x - self.x) ** 2 +
            (self.goal_y - self.y) ** 2
        )

    def control_loop(self):
        if not self.waypoints:
            return

        e = self.look()
        distance = self.get_distance()

        print(f"\rCurr({self.x:.2f},{self.y:.2f}) -> Targ({self.goal_x},{self.goal_y}) | d={distance:.2f} e={e:.2f}", end="")

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        # --- Angular PID ---
        self.int_e += e * self.dt
        der_e = (e - self.prev_e) / self.dt

        ang_vel = (
            self.kp_ang * e +
            self.ki_ang * self.int_e +
            self.kd_ang * der_e
        )

        self.prev_e = e
        ang_vel = max(min(ang_vel, 1.5), -1.5)

        # --- Linear PID ---
        self.int_d += distance * self.dt
        der_d = (distance - self.prev_d) / self.dt

        lin_vel = (
            self.kp_lin * distance +
            self.ki_lin * self.int_d +
            self.kd_lin * der_d
        )

        self.prev_d = distance
        lin_vel = max(min(lin_vel, 1.0), 0.0)

        thres = 0.15

        if abs(e) > 0.2:
            cmd.twist.angular.z = ang_vel
            cmd.twist.linear.x = 0.0

        elif distance > thres:
            cmd.twist.linear.x = lin_vel
            cmd.twist.angular.z = ang_vel * 0.3

        else:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

            if self.target_id >= len(self.waypoints) - 1:
                self.get_logger().warning("Completed all waypoints")
                return

            self.target_id += 1
            self.set_goal(self.target_id)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()