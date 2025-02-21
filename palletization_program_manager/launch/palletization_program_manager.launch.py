import launch
from launch_ros.actions import Node

def generate_launch_description():

    program_manager_node = Node(
        package='palletization_program_manager',
        executable="palletization_program_manager",
    )

    palletization_tree_node = Node(
        package='palletization_program_manager',
        executable='palletization_tree',
    )

    return launch.LaunchDescription(
        [program_manager_node, palletization_tree_node]
    )