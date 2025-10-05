from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_behaviour',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node'
        ),
    ])
