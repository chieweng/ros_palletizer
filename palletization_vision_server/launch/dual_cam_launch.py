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
        # Multi-launch Camera 1 and 2, obtain camera serial number using cmd rs-enumerate-devices, match device type to camera model   
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_multi_camera_launch.py',
                'camera_namespace1:=camera_1', 'camera_name1:=camera', 'device_type1:=d415', 'serial_no1:=_020122063246',
                'camera_namespace2:=camera_2', 'camera_name2:=camera','device_type2:=d415', 'serial_no2:=_151322064609',
                'depth_module.profile1:=1280x720x6', 'depth_module.profile2:=1280x720x6',
                'color_module.profile1:=1280x720x6', 'color_module.profile2:=1280x720x6',
                'pointcloud.enable1:=true','pointcloud.enable2:=true',
                'align_depth.enable1:=true','align_depth.enable2:=true',
                'hole_filling_filter.enable1:=true', 'hole_filling_filter.enable2:=true',
                'temporal_filter.enable1:=true', 'temporal_filter.enable2:=true',
                'temporal_filter.filter_smooth_alpha1:=0.5', 'temporal_filter.filter_smooth_alpha2:=0.5',
                'temporal_filter.filter_smooth_delta1:=20', 'temporal_filter.filter_smooth_delta2:=20'
            ],
            output='screen'
        ),
        
        # Camera 1 (Intel_RealSense_D415)
        TimerAction(
            period=5.0, # Delay for camera initialization
            actions=[
                Node(
                    package='palletization_vision_server',
                    executable='palletization_vision_server',
                    name='palletization_vision_server1',
                    output='screen',
                    parameters=[vision_param_path, {'camera_id': 1}],
                    remappings=[
                        ('/camera/camera/color/image_raw', '/camera_1/camera/color/image_raw'),
                        ('/camera/camera/aligned_depth_to_color/image_raw', '/camera_1/camera/aligned_depth_to_color/image_raw'),
                        ('/camera/camera/color/camera_info', '/camera_1/camera/color/camera_info')
                    ]
                )
            ]
        ),

        # Camera 2 (Intel_RealSense_D435I)
        TimerAction(
            period=5.0, # Delay for camera initialization
            actions=[
                Node(
                    package='palletization_vision_server',
                    executable='palletization_vision_server',
                    name='palletization_vision_server2',
                    output='screen',
                    parameters=[vision_param_path, {'camera_id': 2}],
                    remappings=[
                        ('/camera/camera/color/image_raw', '/camera_2/camera/color/image_raw'),
                        ('/camera/camera/aligned_depth_to_color/image_raw', '/camera_2/camera/aligned_depth_to_color/image_raw'),
                        ('/camera/camera/color/camera_info', '/camera_2/camera/color/camera_info')
                    ]
                )
            ]
        ),

        # Launch RViz
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