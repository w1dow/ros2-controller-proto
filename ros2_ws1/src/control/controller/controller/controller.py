import rclpy
from rclpy.node import Node
import yaml
from turtlesim.srv import Spawn,Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller started")

        self.declare_parameter("waypoints", '/home/w1dow/UGV/task1/ros2_ws1/config/waypoints.yaml')

        self.x = 0.0
        self.y = 0.0
        self.angle=0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.target_id = 0

        self.cmd_pub = self.create_publisher(Twist, '/robo1/cmd_vel', 10)

        self.create_subscription(Pose, '/robo1/pose', self.on_recv_info, 10)

        self.robo_spawn = self.create_client(Spawn, '/spawn')
        self.robo_kill=self.create_client(Kill,'/kill')
        while not self.robo_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting  spawn service...")
        while not self.robo_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting  kill service...")

        self.spawn_turtle(0.5,0.5,"robo1")

        self.load_waypoints()
        
        if self.waypoints:
            self.set_goal(0)
            if len(self.waypoints) > 1:
                wp = self.waypoints[0]
                self.spawn_turtle(wp['x'], wp['y'], "wp_0")

        self.timer = self.create_timer(0.1, self.control_loop)

    def on_recv_info(self,msg):
        self.x=msg.x
        self.y=msg.y
        self.angle=msg.theta
    
    def spawn_turtle(self, x, y,name):
        while not self.robo_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.name=name
        req.theta=0.0

        future = self.robo_spawn.call_async(req)

    def kill_turtle(self,id):
        while not self.robo_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        req=Kill.Request()
        req.name=f"wp_{id}"
        future=self.robo_kill.call_async(req)



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


    def look(self):
        target_angle = math.atan2(
            self.goal_y - self.y,
            self.goal_x - self.x
        )

        e = math.atan2(
            math.sin(target_angle - self.angle),
            math.cos(target_angle - self.angle)
        )
        # self.get_logger().info(f"Target: {target_angle},Self_angle: {self.angle}, error: {e}")
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

        self.get_logger().info(f"Reached waypoint : id{self.target_id} @ ({self.goal_x},{self.goal_y})")
        self.kill_turtle(self.target_id)
        self.target_id+=1
        self.set_goal(self.target_id)
        self.get_logger().info(f"id = {self.target_id}, {self.goal_x}, {self.goal_y}")
        self.spawn_turtle(self.goal_x,self.goal_y,f"wp_{self.target_id}")



    def control_loop(self):
        v=1.1
        a_v=1.1
        thres=1
        if not self.waypoints:
            return
        
        e = self.look()
        distance = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)

        cmd = Twist()

        if abs(e) > 0.02:
            cmd.angular.z = a_v * e
            cmd.linear.x = 0.0
            # self.get_logger().info("Rotating")
            print("\rRotating", end="", flush=True)

        elif distance > thres:
            cmd.linear.x = v * distance
            cmd.angular.z = 0.0
            # self.get_logger().info("Moving")
            print("\rMoving   ", end="", flush=True)

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