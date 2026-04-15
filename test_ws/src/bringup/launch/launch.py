# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():

#     # 🔹 TurtleBot3 official launch
#     turtlebot3_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('turtlebot3_gazebo'),
#                 'launch',
#                 'empty_world.launch.py'
#             )
#         )
#     )

#     # 🔹 Bridge
#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         name='bridge',
#         output='screen',
#         arguments=[
#             '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
#             '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
#         ]
#     )

#     # 🔹 Controller
#     controller = Node(
#         package='controller',
#         executable='control',   # ⚠️ verify this
#         name='controller',
#         output='screen'
#     )

#     return LaunchDescription([
#         turtlebot3_launch,
#         bridge,
#         controller
#     ])