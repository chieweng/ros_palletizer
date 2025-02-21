#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from .palletization_vision import *
from palletization_vision_interface.msg import BoxPose
from palletization_vision_interface.srv import GetPoses
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point

class PalletizationService(Node):

    def __init__(self):
        """
        Initialises the setup based on YAML file and subscribes to the 
        relevant topics to find the location of the boxes.
        """

        super().__init__('palletization_service')

        # Get camera id from launch file param
        self.declare_parameter('camera_id', 0)
        self.camera_id = self.get_parameter('camera_id').value
        
        # Declare publisher dictionaries
        self.pose_publishers = {}
        self.box_images = {}

        # Create custom objects
        self.box = BoxDetection()
        self.pose = PoseSelector()
        self.util = Utilities()

        # For lookup transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameter list with default values
        param_list = {
            'get_pose.roi': [0, 0, 1280, 720],  # Default region of interest
            'get_pose.priority': 'topleft',      # Default priority
            'get_pose.topic': 'camera_link',     # Default camera topic
            'get_pose.cameraDirection': 'x',     # Default camera direction
            'get_pose.width': 0.4,               # Default width
            'get_pose.height': 0.3,              # Default height
            'get_pose.frames': 1,               # Default number of frames
            'get_pose.average': False,             # Default average flag
            'get_pose.targetFrame': 'base_link', # Default target frame for transform
            'get_pose.useDepthFiltering': False,  # Use depth with canny edge
            'get_pose.binDetection': False,       # Detect Bin or boxes

            'normal_canny.gaussianKernel': 11,
            'normal_canny.cannyMin': 30,
            'normal_canny.cannyMax': 60,
            'normal_canny.dilationKernel': 9,
            'normal_canny.erosionKernel': 3,
            'normal_canny.dilationIter': 1,
            'normal_canny.erosionIter': 3,

            'depth_canny.gaussianKernelDepth': 3,
            'depth_canny.cannyMinDepth': 200,
            'depth_canny.cannyMaxDepth': 1600,
            'depth_canny.dilationKernelDepth': 3,
            'depth_canny.erosionKernelDepth': 3,
            'depth_canny.dilationIterDepth': 1,
            'depth_canny.erosionIterDepth': 3,
            'depth_canny.apertureSize': 5,
            'depth_canny.L2gradient': True,
            'depth_canny.enhanceEdge': 7,
            'depth_canny.palletCamDist': 1.45,
            'depth_canny.depthTolerance': 0.05,

            'bin.depthTolerance': 100, 
            'bin.cannyMin': 3000, 
            'bin.cannyMax': 3900, 
            'bin.dilK': 3, 
            'bin.eroK': 3, 
            'bin.dilIter': 1, 
            'bin.eroIter': 1, 
            'bin.apertureSize': 7, 
            'bin.L2gradient': True, 
            'bin.enhanceEdge': 9
        }

        # Declare parameters 
        for param, default_value in param_list.items():
                self.declare_parameter(param, default_value)
                actual_value = self.get_parameter(param).value
                
                if actual_value == default_value:
                    self.get_logger().info(f"Parameter '{param}' initialized with DEFAULT value: {actual_value}.")
                else:
                    self.get_logger().info(f"Parameter '{param}' OVERRIDDEN with final value: {actual_value}.")
                    
        # # Create subscribers for RGB and depth images
        # self.subscription_rgb = Subscriber(self, Image, '/camera/camera/color/image_raw')
        # self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        # self.info_sub = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')

        # Create subscribers for RGB and depth images
        self.subscription_rgb = Subscriber(self, Image, '/feynman_camera/snM1NB118G25012101/rgb/image_rect_color')
        self.depth_sub = Subscriber(self, Image, '/feynman_camera/snM1NB118G25012101/depthalignrgb/image_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/feynman_camera/snM1NB118G25012101/rgb/camera_info')

        # Create a time synchronizer
        self.ts = ApproximateTimeSynchronizer([self.subscription_rgb, self.depth_sub, self.info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.palletization_callback)

        # Create a publisher node for the pose and detected box image
        self.pose_publishers[self.camera_id] = self.create_publisher(PoseStamped, f'/box_pose{self.camera_id}', 10)
        self.box_images[self.camera_id] = self.create_publisher(Image, f'/box_image{self.camera_id}', 10)

        # Create CvBridge objects
        self.br_rgb = CvBridge()
        self.br_depth = CvBridge()

        # Intrinsics of the camera
        self.intrinsics = None

        # Create trigger service
        service_name = f'/get_pose{self.camera_id}'
        self.srv = self.create_service(GetPoses, service_name, self.trigger_callback)
        self.get_logger().info("Palletization Vision has started !")
        
        # Array containing X, Y, Z, Roll, Pitch, Yaw, Box ID for each box
        self.poses = []

        # Buffer to store past 10 frames of poses
        self.pose_buffer = []

    def palletization_callback(self, rgb_msg, depth_msg, info_msg):
        """
        Main callback function to determine the optimal pose
        """
        # Obtain camera intrinsic information
        intrinsics = Camera.get_intrinsics_from_msg(info_msg)
        
        # Find the location of all detected boxes
        boxes = self.detection_callback(rgb_msg, depth_msg, intrinsics)
        
        # Update the poses in the depth callback
        self.poses = self.box.calculate_3d_poses(depth_msg, boxes, intrinsics)
        
        # Update the buffer
        self.pose_buffer.append(self.poses)
        
        if len(self.pose_buffer) > self.get_parameter('get_pose.frames').value:
            self.pose_buffer.pop(0)

    def trigger_callback(self, request, response):
        """
        Trigger response called when client requests optimal pose
        """

        self.get_logger().info("Trigger service callback")

        if not self.poses:
            response.message = "No boxes detected"
            return response    

        if len(self.pose_buffer) < self.get_parameter('get_pose.frames').value:
            response.message = "Not enough frames collected"
            return response
        
        print(self.pose_buffer)
        print(self.get_parameter('get_pose.average').value)

        # Calculate the average poses from the buffered poses
        average_poses = self.pose.calculate_average_poses(self.pose_buffer, self.get_parameter('get_pose.average').value)
        
        # Find the topmost pose based on the lowest z value
        topmost_pose = self.pose.find_topmost_pose(average_poses)
        
        # Filter poses to those within 0.1m of the topmost z value
        filtered_poses = self.pose.filter_poses_by_topmost(average_poses, topmost_pose[2])
        
        # Find the optimal pose based on the priority
        optimal_pose = self.pose.find_optimal_pose(filtered_poses, self.get_parameter('get_pose.priority').value)

        response.success = False

        if optimal_pose:
            target_frame = self.get_parameter('get_pose.targetFrame').value

            for pose in self.poses:
                source_box_pose = BoxPose()
                source_box_pose.x = pose[0]
                source_box_pose.y = pose[1]
                source_box_pose.z = pose[2]
                source_box_pose.rx = float(pose[3])
                source_box_pose.ry = float(pose[4])
                source_box_pose.rz = float(pose[5])
                source_box_pose.box_id = pose[6]
                source_box_pose.barcode = pose[7]
                response.source_coord.append(source_box_pose)

                transformed_coord = self.transform_coordinates(pose[:3], target_frame, self.get_parameter('get_pose.topic').value)
                
                if transformed_coord is not None:
                    target_box_pose = BoxPose()
                    target_box_pose.x = transformed_coord[0]
                    target_box_pose.y = transformed_coord[1]
                    target_box_pose.z = transformed_coord[2]
                    target_box_pose.rx = float(pose[3])
                    target_box_pose.ry = float(pose[4])
                    target_box_pose.rz = float(pose[5])
                    target_box_pose.box_id = pose[6]
                    target_box_pose.barcode = pose[7]
                    response.target_coord.append(target_box_pose)

            # Transform target frame wrt camera frame to target frame wrt robot base frame
            source_coord = optimal_pose[:3]
            transformed_coord = self.transform_coordinates(source_coord, target_frame, self.get_parameter('get_pose.topic').value)

            # Create the response message
            response.message = self.util.create_response_message(optimal_pose, self.poses, self.get_parameter('get_pose.priority').value)
            
            # Will add attribute to response to hold transformed_coord and not transformed_coord
            if transformed_coord is not None:
                optimal_box_pose = BoxPose()
                self.get_logger().info(f"Transformed coordinate: x = {transformed_coord[0]}, y = {transformed_coord[1]}, z = {transformed_coord[2]}")
                optimal_box_pose.x = transformed_coord[0]
                optimal_box_pose.y = transformed_coord[1]
                optimal_box_pose.z = transformed_coord[2]
                optimal_box_pose.rx = float(optimal_pose[3])
                optimal_box_pose.ry = float(optimal_pose[4])
                optimal_box_pose.rz = float(optimal_pose[5])
                optimal_box_pose.box_id = int(optimal_pose[6])
                optimal_box_pose.barcode = optimal_pose[7]
                response.optimal_target = optimal_box_pose
                response.success = True
            else:
                self.get_logger().info("Coordinate not yet transformed")

            # Create and publish the PoseStamped message for visualization
            pose_viz = self.util.create_pose_viz(optimal_pose, self.get_parameter('get_pose.cameraDirection').value, self.get_clock().now().to_msg(), self.get_parameter('get_pose.topic').value)

            self.pose_publishers[self.camera_id].publish(pose_viz)

        else:
            response.message = "No boxes detected"

        return response
    
    def detection_callback(self, msg, depth_msg, intrinsics):
        if intrinsics is None:
            return

        # Obtain 2D frame
        current_frame = self.util.process_image(msg)

        # Extract ROI and get its starting coordinates
        roi = self.get_parameter('get_pose.roi').value

        roi_rgb, x_start, y_start = self.util.extract_roi(current_frame, roi[0], roi[1], roi[2], roi[3])

        # Obtain depth map
        depth_image = self.br_depth.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        if self.get_parameter('get_pose.binDetection').value:
            roi_depth, _, _ = self.util.extract_roi(depth_image, roi[0], roi[1], roi[2], roi[3])
            edge = self.box.detect_bin(roi_rgb, 
                                       roi_depth, 
                                       depth_tolerance=self.get_parameter('bin.depthTolerance').value*1000, 
                                       cannyMin=self.get_parameter('bin.cannyMin').value, 
                                       cannyMax=self.get_parameter('bin.cannyMax').value, 
                                       dilK=self.get_parameter('bin.dilK').value, 
                                       eroK=self.get_parameter('bin.eroK').value, 
                                       dilIter=self.get_parameter('bin.dilIter').value, 
                                       eroIter=self.get_parameter('bin.eroIter').value, 
                                       apertureSize=self.get_parameter('bin.apertureSize').value, 
                                       L2gradient=self.get_parameter('bin.L2gradient').value, 
                                       enhanceEdge=self.get_parameter('bin.enhanceEdge').value)
        else:
            if self.get_parameter('get_pose.useDepthFiltering').value:
                roi_depth, _, _ = self.util.extract_roi(depth_image, roi[0], roi[1], roi[2], roi[3])
                edge = self.box.detect_edges_with_depth(roi_rgb, 
                                                        roi_depth, 
                                                        depth_tolerance=self.get_parameter('depth_canny.depthTolerance').value*1000, 
                                                        gaussK=self.get_parameter('depth_canny.gaussianKernelDepth').value, 
                                                        cannyMin=self.get_parameter('depth_canny.cannyMinDepth').value, 
                                                        cannyMax=self.get_parameter('depth_canny.cannyMaxDepth').value, 
                                                        dilK=self.get_parameter('depth_canny.dilationKernelDepth').value, 
                                                        eroK=self.get_parameter('depth_canny.erosionKernelDepth').value, 
                                                        dilIter=self.get_parameter('depth_canny.dilationIterDepth').value, 
                                                        eroIter=self.get_parameter('depth_canny.erosionIterDepth').value, 
                                                        apertureSize=self.get_parameter('depth_canny.apertureSize').value, 
                                                        L2gradient=self.get_parameter('depth_canny.L2gradient').value, 
                                                        enhanceEdge=self.get_parameter('depth_canny.enhanceEdge').value, 
                                                        palletCamDist=self.get_parameter('depth_canny.palletCamDist').value*1000)
            else:
                # Detect edges in the ROI
                edge = self.box.detect_edges(roi_rgb, 
                                            gaussK=self.get_parameter('normal_canny.gaussianKernel').value, 
                                            cannyMin=self.get_parameter('normal_canny.cannyMin').value, 
                                            cannyMax=self.get_parameter('normal_canny.cannyMax').value, 
                                            dilK=self.get_parameter('normal_canny.dilationKernel').value, 
                                            eroK=self.get_parameter('normal_canny.erosionKernel').value, 
                                            dilIter=self.get_parameter('normal_canny.dilationIter').value, 
                                            eroIter=self.get_parameter('normal_canny.erosionIter').value)

        # Find contours in the edge-detected image
        contours = self.box.find_contours(edge)

        # Filter contours based on area and overlap
        filtered_boxes = self.box.filter_contours(contours, depth_image, intrinsics, x_start, y_start, self.get_parameter('get_pose.width').value, self.get_parameter('get_pose.height').value)

        # Draw filtered boxes and labels on the original frame
        boxes = self.util.draw_boxes(current_frame, filtered_boxes, x_start, y_start, roi_rgb, depth_image, intrinsics, self.get_parameter('get_pose.binDetection').value)

        # Publish the image to the topic
        # self.get_logger.info(type(self.box_images[self.camera_id]))
        self.box_images[self.camera_id].publish(self.br_rgb.cv2_to_imgmsg(current_frame, encoding="rgb8"))

        return boxes
    
    def transform_coordinates(self, source_coord, target_frame, source_frame):
              
        # Create a msg for source coordinates
        source = PointStamped()
        source.header.frame_id = source_frame
        source.point.x = source_coord[2]
        source.point.y = source_coord[1]
        source.point.z = source_coord[0]

        try:
            # Look for transformation
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

            # Perform transformation
            target = do_transform_point(source, transform)

            return [target.point.x, target.point.y, target.point.z]
        
        except Exception as e:
            print(f"Error transforming coordinates: {str(e)}")

            return None
        
def main(args=None):

    rclpy.init(args=args)

    palletization_service = PalletizationService()

    try:
        rclpy.spin(palletization_service)
    except KeyboardInterrupt:
        pass
    finally:
        palletization_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
