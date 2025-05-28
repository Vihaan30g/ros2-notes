from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    subscriber_node = Node(
        package='week_3',
        executable='sub_node',
        name='subscriber_node'
    )

    publisher_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='week_3',
                executable='pub_node',
                name='publisher_node'
            )
        ]
    )

    return LaunchDescription([
        subscriber_node,
        publisher_node
    ])
