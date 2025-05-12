from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='motor_node',
            name='motor_node',
            output='screen'
        )
    ])

