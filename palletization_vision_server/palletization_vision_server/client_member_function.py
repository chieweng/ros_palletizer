#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from palletization_vision_interface.srv import GetPoses

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.client = self.create_client(GetPoses, 'get_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = GetPoses.Request()

    def send_request(self):
        self.get_logger().info('Sending service request...')
        future = self.client.call_async(self.req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response received: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()

    minimal_client.send_request()

    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
