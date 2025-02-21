# Palletization robot node

Palletization robot node is a ROS 2 Humble package that allow changing palletization robot description configuration dynamically during runtime via ROS 2 parameters.

## Installation

The following steps assuume that ROS 2 Humble has been installed. If not please follow the official installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

1. Building the package and sourcing workspace.
```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Usage
Example of disabling conveyor:

1. Launch palletization robot description.
```bash
ros2 launch palletization_robot_description camera_robot.launch.py 
```
2. Set conveyor parameter to false
```bash
ros2 param set /palletization_robot_description conveyor False
```
