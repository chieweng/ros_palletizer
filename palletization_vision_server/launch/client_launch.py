from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='palletization_vision_server',
            executable='client_member_function',
            name='client_node',
            output='screen'
        )
    ])