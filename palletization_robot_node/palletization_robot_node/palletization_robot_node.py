import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_path
from xacro import process_file
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from rclpy.parameter_service import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class PalletizationRobotDescription(Node):
    def __init__(self):
        super().__init__('palletization_robot_description')

        package_share_dir = get_package_share_directory('palletization_robot_description')
        config_path = os.path.join(package_share_dir, 'config', 'palletization_robot.yaml')

        robot_config = self.load_yaml(config_path)
        
        self.mappings={
            'pallet_width': str(robot_config['pallet_width']),
            'pallet_length': str(robot_config['pallet_length']),
            'pallet_height': str(robot_config['pallet_height']),
            'ur_type': str(robot_config['ur_type']),
            'left_pallet': str(robot_config['left_pallet']), 
            'right_pallet': str(robot_config['right_pallet']), 
            'pallet_distx': str(robot_config['pallet_distx']),
            'pallet_disty': str(robot_config['pallet_disty']),
            'conveyor': str(robot_config['conveyor']),
            'conveyor_width': str(robot_config['conveyor_width']),
            'conveyor_length': str(robot_config['conveyor_length']),
            'conveyor_height': str(robot_config['conveyor_height']),
            'conveyor_distx': str(robot_config['conveyor_distx']),
            'conveyor_disty': str(robot_config['conveyor_disty']),
            'conveyor_angle': str(robot_config['conveyor_angle']),
        }

        # Declare ROS parameters
        self.declare_parameter('pallet_width', 0.8)
        self.declare_parameter('pallet_length', 1.2)
        self.declare_parameter('pallet_height', 0.144)
        self.declare_parameter('left_pallet', True)
        self.declare_parameter('right_pallet', True)
        self.declare_parameter('pallet_distx', 0.5)
        self.declare_parameter('pallet_disty', 0.5)
        self.declare_parameter('conveyor', True)
        self.declare_parameter('conveyor_width', 1.0)
        self.declare_parameter('conveyor_length', 2.0)
        self.declare_parameter('conveyor_height', 0.5)
        self.declare_parameter('conveyor_distx', 0.3)
        self.declare_parameter('conveyor_disty', 0.3)
        self.declare_parameter('conveyor_angle', 45)

        # Monitor parameter updates
        self.add_on_set_parameters_callback(self.update_parameters)

        # Publisher for robot description with transient local QoS policy
        self.publisher = self.create_publisher(
            String,
            '/robot_description',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # For the robot lifter temporary
        self.lifter = self.create_publisher(JointState, '/joint_states', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lifter_pub = self.create_timer(0.1, self.lifter_joint_publisher)

        # Path to Xacro file
        self.xacro_file = get_package_share_path('palletization_robot_description') / 'urdf' / 'palletization_robot.urdf.xacro'

        # Publish initial description
        self.update_robot_description()

        self.get_logger().info("Palletization robot description node has started !")

    def lifter_joint_publisher(self):
        joint_state = JointState()

        # Set the header
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Specify the joints and their positions
        joint_state.name = ['outer_middle_joint', 'middle_inner_joint']
        joint_state.position = [0.0, 0.0]  # Replace with your joint angles in radians
        joint_state.velocity = []  # Optional: Add velocities if required
        joint_state.effort = []  # Optional: Add efforts if required

        self.lifter.publish(joint_state)

    def update_parameters(self, params):
        # Update robot description on parameter changes
        for param in params:
            if param.name in self.mappings:
                if param.value == True:
                    self.mappings[param.name] = 'true'
                    self.get_logger().info(f"Updated parameter: {param.name} = {param.value}")
                elif param.value == False:
                    self.mappings[param.name] = 'false'
                    self.get_logger().info(f"Updated parameter: {param.name} = {param.value}")
                else:
                    # Update the value in the mappings dictionary
                    self.mappings[param.name] = str(param.value)
                    self.get_logger().info(f"Updated parameter: {param.name} = {param.value}")
        # After updating the mappings, call the update_robot_description to reflect the changes
        self.update_robot_description()
        
        return SetParametersResult(successful=True)

    def update_robot_description(self):
        
        # Generate URDF
        urdf = process_file(
            self.xacro_file,
            mappings=self.mappings
        ).toxml()

        # Publish to /robot_description
        self.publisher.publish(String(data=urdf))

    def load_yaml(self, file_path):
        """
        Loads the YAML file details given a file path
        """
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return None
        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                return data
            except yaml.YAMLError as exc:
                print(f"Error in loading YAML file: {exc}")
                return None
            
def main(args=None):
    rclpy.init(args=args)
    node = PalletizationRobotDescription()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
