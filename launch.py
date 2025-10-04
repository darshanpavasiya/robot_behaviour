from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_behaviour', executable='obstacle_avoidance', name='obstacle_avoid'),
        Node(package='robot_behaviour', executable='fsm_controller', name='fsm_controller'),
        # For teleop, run it manually if you want because it expects interactive terminal
    ])
