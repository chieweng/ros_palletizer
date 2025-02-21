# Palletization vision interface

Palletization vision interface is a ROS 2 Humble package that store the custom interface file required by the palletization vision server package.

## Installation

The following steps asuume that ROS 2 Humble has been installed. If not please follow the official installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

1. Building the package and sourcing workspace.
```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Usage
Example of importing BoxPose msg:
```python
from palletization_vision_interface.msg import BoxPose
```

## Contact
Tey Yu Teng:
[yutengtey868@gmail.com](mailto:yutengtey868@gmail.com)