from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_package = FindPackageShare("palletization_robot_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "palletization_robot.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "urdf.rviz"])
    robot_config_path = os.path.join(
        get_package_share_directory('palletization_robot_description'),
        'config',
        'palletization_robot.yaml'
        )
    robot_config = load_yaml(robot_config_path)
    robot_description = ParameterValue(
        Command(["xacro ", description_file, " ", "ur_type:=", robot_config['ur_type'], " ", 
                 "left_pallet:=", str(robot_config['left_pallet']), " ", 
                 "right_pallet:=", str(robot_config['right_pallet']), " ", 
                 "pallet_width:=", str(robot_config['pallet_width']), " ",
                 "pallet_length:=", str(robot_config['pallet_length']), " ",
                 "pallet_height:=", str(robot_config['pallet_height']), " ",
                 "pallet_distx:=", str(robot_config['pallet_distx']), " ",
                 "pallet_disty:=", str(robot_config['pallet_disty']), " ",
                 "conveyor:=", str(robot_config['conveyor']), " ",
                 "conveyor_width:=", str(robot_config['conveyor_width']), " ",
                 "conveyor_length:=", str(robot_config['conveyor_length']), " ",
                 "conveyor_height:=", str(robot_config['conveyor_height']), " ",
                 "conveyor_distx:=", str(robot_config['conveyor_distx']), " ",
                 "conveyor_disty:=", str(robot_config['conveyor_disty']), " ",
                 "conveyor_angle:=", str(robot_config['conveyor_angle']), " ",
                 ]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    palletization_robot_description_node = Node(
        package="palletization_robot_node",
        executable="palletization_robot_node"
    )

    return LaunchDescription(
        [joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node, palletization_robot_description_node]
    )

def load_yaml(file_path):
    """
    Loads a YAML file and returns its contents.
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
