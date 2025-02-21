from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('palletization_vision_server')

    # Paths to configuration files
    vision_param_path = os.path.join(package_share_dir, 'config', 'vision_param.yaml')
    rviz_config_path = os.path.join(package_share_dir, 'rviz', 'rviz_config.rviz')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'feynman_camera', 'm1nb.py'
                # 'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                # 'depth_module.profile:=1280x720x6',
                # 'color_module.profile:=1280x720x6',
                # 'pointcloud.enable:=true',
                # 'align_depth.enable:=true',
                # 'hole_filling_filter.enable:=true',
                # 'temporal_filter.enable:=true',
                # 'temporal_filter.filter_smooth_alpha:=0.5',
                # 'temporal_filter.filter_smooth_delta:=20',  
            ],
            output='screen'
        ),
        TimerAction(
            period=5.0,  # Sufficient delay to launch service
            actions=[
                Node(
                    package='palletization_vision_server',
                    executable='palletization_vision_server',
                    name='palletization_vision_server1',
                    output='screen',
                    parameters=[vision_param_path, {'camera_id': 1}], # Default camera_id for single cam = 1
                )
            ]
        ),
        TimerAction(
            period=7.0,  # Delay to launch rviz
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_node',
                    arguments=['-d', rviz_config_path],
                    output='screen'
                )
            ]
        ),
    ])
