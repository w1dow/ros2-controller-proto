import rclpy
from rclpy.node import Node
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller started")

        self.declare_parameter("waypoints", '/home/w1dow/UGV/task1/test_ws/src/simulation/config/waypoints.yaml')

        self.x = 0.0
        self.y = 0.0
        self.angle=0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.target_id = 0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.on_recv_info, 10)  

        self.load_waypoints()
        
        if self.waypoints:
            self.set_goal(0)

        self.timer = self.create_timer(0.1, self.control_loop)

    def on_recv_info(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.angle = math.atan2(siny, cosy)
    
    def load_waypoints(self):
        waypoint_file = self.get_parameter('waypoints').value
        scale = 1

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


    def look(self):
        target_angle = math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        )

        e = math.atan2(
            math.sin(target_angle - self.angle),
            math.cos(target_angle - self.angle)
        )
        return e

    def set_goal(self,id):
        wp = self.waypoints[id]
        self.goal_x = wp['x']
        self.goal_y = wp['y']
        self.get_logger().info(f"Target {id} :({self.goal_x},{self.goal_y})")


    def get_distance(self):
        return math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
    
    def on_reach(self):
        if self.target_id >= len(self.waypoints) - 1:
            self.get_logger().warning("Completed all waypoints")
            return 

        # self.get_logger().info(f"Reached waypoint : id{self.target_id} @ ({self.goal_x},{self.goal_y})")
        self.target_id+=1
        self.set_goal(self.target_id)


    def control_loop(self):
        v = 1.0
        a_v = 1.0
        ang_thres = 0.15   # bigger threshold
        pos_thres = 0.15

        if not self.waypoints:
            return

        e = self.look()
        distance = self.get_distance()

        print(f"\r Curr({self.x:.2f},{self.y:.2f}) -> Targ({self.goal_x},{self.goal_y}) | d={distance:.2f} e={e:.2f}", end="")

        cmd = Twist()

        if abs(e) > ang_thres:
            cmd.angular.z = max(min(a_v * e, 1.0), -1.0)
            cmd.linear.x = 0.0

        elif distance <= pos_thres and abs(e) > ang_thres:
            # final alignment
            cmd.angular.z = max(min(a_v * e, 1.0), -1.0)
            cmd.linear.x = 0.0

        elif distance <= pos_thres:
            # fully reached
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