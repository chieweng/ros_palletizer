from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'palletization_vision_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/single_cam_launch.py', 'launch/dual_cam_launch.py', 'launch/client_launch.py']),
        ('share/' + package_name + '/config', ['config/vision_param.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yttey',
    maintainer_email='yttey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'palletization_vision_server = palletization_vision_server.palletization_vision_server:main',
        'client_member_function = palletization_vision_server.client_member_function:main'
        ],
    },
)
