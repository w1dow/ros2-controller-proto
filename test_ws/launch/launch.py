from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    base = os.path.expanduser('~/UGV/task1/test_ws')

    # Only simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base, 'simulation/launch/sim.launch.py')
        )
    )

    return LaunchDescription([
        sim_launch
    ])