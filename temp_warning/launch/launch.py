from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='temp_warning',
            executable='temp_sensor',
            name='temp_sensor'
        ),
        Node(
            package='temp_warning',
            executable='temp_monitor',
            name='temp_monitor',
            parameters=[{'threshold': 60.0}]
        )
    ])
