from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ds5_ros',
            executable='ds5ros_node.py',
            name='ds5_ros_node',
            output='screen',
            parameters=[{
                'noderate': 50.0,
                'joy_pub': 'joy',
                'joy_sub': 'joy/set_feedback',
                'deadzone': 0.05
            }]
        )
    ])
