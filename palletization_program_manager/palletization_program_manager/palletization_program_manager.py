import rclpy
from rclpy.node import Node
from ur_dashboard_msgs.srv import GetRobotMode

class PalletizationProgramManager(Node):
    def __init__(self):
        super().__init__('palletization_program_manager')

        self.power = False
        self.dashboard_client = self.create_client(GetRobotMode, '/dashboard_client/get_robot_mode')
        # Wait for the service to be available
        while not self.dashboard_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/dashboard_client/get_robot_mode' not available.")

        self.get_logger().info("Fetching initial robot mode...")
        self.get_robot_mode()

        param_list = {
            'power': self.power,
            'mode': 'depalletizing'
        }
        # Declare parameters 
        for param, default_value in param_list.items():
            self.declare_parameter(param, default_value)
            self.get_logger().warn(f"Parameter '{param}' declared with default value: {default_value}.")

        self.get_logger().info("Palletization Program Manager has started!")

    def get_robot_mode(self):
        """Fetch the robot mode and update the 'power' parameter."""
        request = GetRobotMode.Request()

        # Call the service synchronously
        future = self.dashboard_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            mode = response.robot_mode.mode
            self.get_logger().info(f"Robot Mode fetched: {mode} ({response.answer})")

            # Update the 'power' parameter based on robot mode
            self.get_logger().info(str(mode == 7))
            if mode == 7: 
                self.power = True
            else:
                self.power = False
        else:
            self.get_logger().error("Failed to get robot mode.")

def main(args=None):
    rclpy.init(args=args)
    node = PalletizationProgramManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
