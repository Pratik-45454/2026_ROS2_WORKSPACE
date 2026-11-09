from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_maintreee',
            executable='bt',
            name='main_tree',
            output='screen'
        )
        
    ])
            