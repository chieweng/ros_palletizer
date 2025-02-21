import cv2
import yaml
import os
from collections import Counter
import numpy as np  
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler
from pyzbar.pyzbar import decode
from scipy import stats

class BoxDetection():

    def detect_edges(self, roi, gaussK=11, cannyMin=30, cannyMax=60, dilK=9, eroK=3, dilIter=1, eroIter=3):
        """
        Applies edge detection and morphological operations to the ROI.
        """
        # Convert the ROI to grayscale for edge detection
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection
        blurred = cv2.GaussianBlur(gray, (gaussK, gaussK), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, cannyMin, cannyMax, L2gradient = True)

        # Define a kernel for morphological operations
        dilation_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilK, dilK))
        erosion_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (eroK, eroK))
        # Apply dilation to close gaps in edges
        dilated_edges = cv2.dilate(edges, dilation_kernel, iterations=dilIter)

        # Apply erosion to remove small noise
        eroded_edges = cv2.erode(dilated_edges, erosion_kernel, iterations=eroIter)

        return eroded_edges

    def detect_edges_with_depth(self, roi, roi_depth, depth_tolerance=100, gaussK=3, cannyMin=90, cannyMax=180, dilK=3, eroK=3, dilIter=1, eroIter=1, apertureSize=3, L2gradient=True, enhanceEdge=7, palletCamDist=1450):
        """
        Applies edge detection and morphological operations to the ROI.
        """
        # Convert the ROI to grayscale for edge detection
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        masked_depth_image = np.ma.masked_equal(roi_depth, 0)

        # Get the minimum depth, ignoring the 0 values
        max_depth = palletCamDist
        min_depth = masked_depth_image.min()
        
        depth_mask = ((roi_depth - min_depth <= depth_tolerance) | (min_depth - roi_depth <= depth_tolerance)) & (min_depth < max_depth)
        depth_mask = depth_mask.astype(np.uint8)

        erosion = cv2.erode(depth_mask, cv2.getStructuringElement(cv2.MORPH_RECT,(eroK,eroK)), iterations = eroIter)
        dilation = cv2.dilate(erosion, cv2.getStructuringElement(cv2.MORPH_RECT,(dilK,dilK)), iterations = dilIter)

        filtered_img = np.multiply(gray, dilation)

        blur = cv2.GaussianBlur(filtered_img, (gaussK,gaussK), 0)

        edge = cv2.Canny(blur, cannyMin, cannyMax, apertureSize = apertureSize, L2gradient = L2gradient)

        dilated_edge = cv2.dilate(edge, cv2.getStructuringElement(cv2.MORPH_RECT,(enhanceEdge,enhanceEdge)), iterations = 1)

        return dilated_edge

    def detect_bin(self, roi, roi_depth, depth_tolerance=100, cannyMin=3000, cannyMax=3900, dilK=3, eroK=3, dilIter=1, eroIter=1, apertureSize=7, L2gradient=True, enhanceEdge=9):
        """
        Applies edge detection and morphological operations to the ROI.
        """
        # Convert the ROI to grayscale for edge detection
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        masked_depth_image = np.ma.masked_equal(roi_depth, 0)

        # Get the minimum depth, ignoring the 0 values
        min_depth = masked_depth_image.min()
        
        depth_mask = (roi_depth - min_depth <= depth_tolerance) | (min_depth - roi_depth <= depth_tolerance)
        depth_mask = depth_mask.astype(np.uint8)

        erosion = cv2.erode(depth_mask, cv2.getStructuringElement(cv2.MORPH_RECT,(eroK,eroK)), iterations = eroIter)
        dilation = cv2.dilate(erosion, cv2.getStructuringElement(cv2.MORPH_RECT,(dilK,dilK)), iterations = dilIter)

        filtered_img = np.multiply(gray, dilation)

        bilateral = cv2.bilateralFilter(filtered_img, 15, 75, 75) 

        edge = cv2.Canny(bilateral, cannyMin, cannyMax, apertureSize = apertureSize, L2gradient = L2gradient)

        dilated_edge = cv2.dilate(edge, cv2.getStructuringElement(cv2.MORPH_RECT,(enhanceEdge,enhanceEdge)), iterations = 1)

        eroded_edge = cv2.erode(dilated_edge, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations = 1)

        return eroded_edge
    
    def find_contours(self, eroded_edges):
        """
        Finds contours in the edge-detected image using hierarchical contour retrieval.
        """
        contours, _ = cv2.findContours(eroded_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def filter_contours(self, contours, depth_image, intrinsics, x_start, y_start, width, height):
        """
        Filters contours based on their area in 3D space and overlap.
        """
        filtered_boxes = []
        overlap_threshold = 0.5  # Maximum overlap of 20% over two rectangles
        tolerance = 0.05
        for contour in contours:
            # Approximate the contour to a polygon (reduce number of points)
            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if contour is rectangular (4 vertices) and filter by area
            if len(approx) == 4 and cv2.contourArea(approx) > 5000:
                # Calculate the minimum area bounding rectangle
                rect = cv2.minAreaRect(approx)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # Offset contour points to match the original frame coordinates
                box += [x_start, y_start]

                center_x, center_y = rect[0][0] + x_start, rect[0][1] + y_start
                new_box = (center_x, center_y, rect[1][0], rect[1][1], rect[2])

                # Calculate 3D area
                width_3D, height_3D = self.calculate_3d_contour_area([center_x, center_y], box, depth_image, intrinsics)
                
                if self.check_box(width_3D, height_3D, width, height, tolerance):
                    # Check for overlap between rectangles
                    if not any(self.calculate_rotated_iou(new_box, b) > overlap_threshold for b in filtered_boxes):
                        filtered_boxes.append(new_box)

        return filtered_boxes
    
    def check_box(self, width_3D, height_3D, width_actual, height_actual, tolerance):
        """
        Checks the dimensions of the box
        """
        is_box = False
        if width_3D > height_3D:
            is_box = (width_3D > width_actual - tolerance) and (width_3D < width_actual + tolerance) and (height_3D > height_actual - tolerance) and (height_3D < height_actual + tolerance)
        else:
            is_box = (height_3D > width_actual - tolerance) and (height_3D < width_actual + tolerance) and (width_3D > height_actual - tolerance) and (width_3D < height_actual + tolerance)
        return is_box
    
    def calculate_3d_contour_area(self, center, box, depth_image, intrinsics):
        """
        Converts 2D contour points to 3D points using the depth map and calculates the area of the 3D polygon.
        """
        points_3d = []
        x = int(center[0])
        y = int(center[1])

        depth = self.get_valid_depth(x, y, depth_image)
        for point in box:
            point_3d = Camera.deproject_pixel_to_point(intrinsics, (point[0], point[1]), depth)
            points_3d.append(point_3d)

        width_3D = np.sqrt((points_3d[3][0] - points_3d[0][0])**2 + (points_3d[3][1] - points_3d[0][1])**2)
        height_3D = np.sqrt((points_3d[1][0] - points_3d[0][0])**2 + (points_3d[1][1] - points_3d[0][1])**2)

        return width_3D, height_3D

    def calculate_rotated_iou(self, box1, box2):
        """
        Calculate the Intersection over Union (IoU) of two rotated bounding boxes.
        Each box is defined by (center_x, center_y, width, height, angle).
        """
        # Define the first rectangle with its center, size (width, height), and rotation angle
        rect1 = ((box1[0], box1[1]), (box1[2], box1[3]), box1[4])

        # Define the second rectangle with its center, size (width, height), and rotation angle
        rect2 = ((box2[0], box2[1]), (box2[2], box2[3]), box2[4])

        # Find the intersection points of the two rotated rectangles
        int_pts = cv2.rotatedRectangleIntersection(rect1, rect2)[1]

        # Check if there is an intersection
        if int_pts is not None:
            # Get the convex hull of the intersection points to define the intersection area
            order_pts = cv2.convexHull(int_pts, returnPoints=True)

            # Calculate the area of the intersection
            inter_area = cv2.contourArea(order_pts)

            # Calculate the area of the first rectangle
            box1_area = box1[2] * box1[3]

            # Calculate the area of the second rectangle
            box2_area = box2[2] * box2[3]

            # Calculate the Intersection over Union (IoU)
            iou = inter_area / float(box1_area + box2_area - inter_area)
            return iou
        else:
            # If there is no intersection, IoU is 0
            return 0.0

    def calculate_3d_poses(self, msg, boxes, intrinsics):
        """
        Determines the 3D position of a pixel coordinate
        """
        if intrinsics is None:
            return
        if (len(boxes) == 0):
            return

        # Create CvBridge object
        br_depth = CvBridge()

        # Obtain depth map
        depth_image = br_depth.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Initialize array
        poses = []

        for box in boxes:
            x, y, angle, boxID, barcode= box
            
            # Get depth value at the pixel
            depth = self.get_valid_depth(x, y, depth_image)

            # Deproject to 3D point
            pixel = (x, y)
            point = Camera.deproject_pixel_to_point(intrinsics, pixel, depth)

            # Output 3D point
            poses.append([point[0], point[1], point[2], 0, 0, angle, boxID, barcode])

        return poses
    
    def get_valid_depth(self, x, y, depth_map):
        """  
        Fill in the missing depth data and extract depth information.
        """

        # Ensure all NaN value is turn into 0
        mask = np.isnan(depth_map) | (depth_map <= 0)
        mask = mask.astype(np.uint8)

        # Use inpainting method to fill in missing values
        depth_inpainted = cv2.inpaint(depth_map, mask, 3, cv2.INPAINT_TELEA)

        # Extract depth value in meters
        depth = depth_inpainted[y, x]/1000.0

        return depth
    
class PoseSelector():

    def find_optimal_pose(self, poses, priority):
        """
        Find optimal pose based on the priority
        """
        # Arbitrary far distance for comparison
        top, left = float('inf'), float('inf')
        # Arbitrary close distance for comparison
        bottom, right = float('-inf'), float('-inf') 
        optimal_pose = None
        epsilon = 0.15

        for pose in poses:
            if priority == "topleft":
                if (pose[1] < top - epsilon) or (abs(pose[1] - top) < epsilon and pose[0] < left):
                    optimal_pose = pose
                    top, left = pose[1], pose[0]
            elif priority == "topright":
                if (pose[1] < top - epsilon) or (abs(pose[1] - top) < epsilon and pose[0] > right):
                    optimal_pose = pose
                    top, right = pose[1], pose[0]
            elif priority == "bottomleft":
                if (pose[1] > bottom + epsilon) or (abs(pose[1] - bottom) < epsilon and pose[0] < left):
                    optimal_pose = pose
                    bottom, left = pose[1], pose[0]
            elif priority == "bottomright":
                if (pose[1] > bottom + epsilon) or (abs(pose[1] - bottom) < epsilon and pose[0] > right):
                    optimal_pose = pose
                    bottom, right = pose[1], pose[0]
            else:
                if top > pose[2]:
                    optimal_pose = pose
                    top = pose[2]

        return optimal_pose
    
    def calculate_average_poses(self, data, do_average):
        """
        Calculate the average poses from the buffered poses.
        Only consider frames with the most detected boxes (to prevent object_id conflict)
        """
        
        max_length = max(len(frame) for frame in data)

        filtered_frames = [frame for frame in data if len(frame) == max_length]

        if not do_average: # return last filtered frame in buffered frames
            return filtered_frames[-1]
        
        box_poses = np.array([[obj[:6] for obj in frame] for frame in filtered_frames]) # shape: (num_frames, num_box, 6)
        box_ids= [obj[6] for obj in filtered_frames]
        
        # Handle barcode detection
        barcodes = []
        for frame in filtered_frames:
            for obj in frame:
                barcode = obj[7]
                # If barcode is not 'No barcode detected', use the actual barcode
                if barcode != 'No barcode detected.':
                    barcodes.append(barcode)
                else:
                    barcodes.append('No barcode detected.') 
        
        # Compute the average pose for each object
        avg_poses = np.mean(box_poses, axis=0).tolist()  # shape: (objects, 6)

        result = []
        for pose, box_id, barcode in zip(avg_poses, box_ids, barcodes):
            result.append(pose + [box_id, barcode])

        return result
    
    def find_topmost_pose(self, poses):
        """
        Find the topmost pose based on the lowest z value.
        """
        return min(poses, key=lambda pose: pose[2])

    def filter_poses_by_topmost(self, poses, topmost):
        """
        Filter poses to those within 0.1m of the topmost value
        """
        return [pose for pose in poses if abs(pose[2] - topmost) <= 0.1]


class Camera:
    def get_intrinsics_from_msg(msg):
        """
        Extracts intrinsics from a ROS2 CameraInfo message

        Parameters:
            msg (sensor_msgs.msg.CameraInfo): ROS2 CameraInfo message

        Returns:
            intrinsics (dict): Dictionary with intrinsic parameters
        """
        return {
            "width": msg.width,
            "height": msg.height,
            "fx": msg.k[0],  # Focal length in x
            "fy": msg.k[4],  # Focal length in y
            "cx": msg.k[2],  # Principal point x
            "cy": msg.k[5],  # Principal point y
            "distortion_model": msg.distortion_model, # "plump_bob" or "brown_conrady"
            "coeffs": list(msg.d)  # Distortion coefficients
        }

    def deproject_pixel_to_point(intrinsics, pixel, depth):
        """
        Convert a pixel coordinate (u, v) to a 3D point

        Parameters:ss
            intrinsics (dict): Camera intrinsics with keys 'fx', 'fy', 'cx', 'cy'
            pixel (tuple): Pixel coordinate (u, v)
            depth (float): Depth value at the given pixel

        Returns:
            np.array: 3D point in camera space (x, y, z)
        """
        u, v = pixel
        fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']

        # Compute normalized coordinates
        x = (u - cx) / fx
        y = (v - cy) / fy

        # Scale by depth to get 3D point
        return np.array([x * depth, y * depth, depth])

class Utilities():
    
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
            
    def create_response_message(self, pose, poses, priority):
        """
        Create the response message with the detected box information.
        """
        return (f"{priority} box detected at: x={pose[0]}, y={pose[1]},"
                f"z={pose[2]} with box ID {pose[6]}. \n All poses: {poses}")
    
    def create_pose_viz(self, pose, camera_direction, header_stamp, header_frame_id):
        """
        Create a PoseStamped message for visualization.
        """
        pose_viz = PoseStamped()
        if camera_direction == 'x':
            pose_viz.pose.position.x = pose[2]
            pose_viz.pose.position.y = -pose[0]
            pose_viz.pose.position.z = -pose[1]
            q = quaternion_from_euler(pose[5], pose[3], pose[4])
        elif camera_direction == 'y':
            pose_viz.pose.position.x = pose[0]
            pose_viz.pose.position.y = pose[2]
            pose_viz.pose.position.z = -pose[1]
            q = quaternion_from_euler(pose[3], pose[5], pose[4])
        else:
            pose_viz.pose.position.x = pose[0]
            pose_viz.pose.position.y = pose[1]
            pose_viz.pose.position.z = pose[2]
            q = quaternion_from_euler(pose[3], pose[4], pose[5])
 
        pose_viz.pose.orientation.x = q[0]
        pose_viz.pose.orientation.y = q[1]
        pose_viz.pose.orientation.z = q[2]
        pose_viz.pose.orientation.w = q[3]
        
        pose_viz.header.stamp = header_stamp
        pose_viz.header.frame_id = header_frame_id
        return pose_viz
    
    # def rs_info(self, msg):
    #     # Populate the intrinsics of an image
    #     intrinsics = rs.intrinsics()
    #     intrinsics.width = msg.width
    #     intrinsics.height = msg.height
    #     intrinsics.ppx = msg.k[2]
    #     intrinsics.ppy = msg.k[5]
    #     intrinsics.fx = msg.k[0]
    #     intrinsics.fy = msg.k[4]
    #     intrinsics.model = rs.distortion.brown_conrady
    #     intrinsics.coeffs = [i for i in msg.d]
    #     return intrinsics
    
    def draw_boxes(self, current_frame, filtered_boxes, x_start, y_start, roi, depth_image, intrinsics, binDetection):
        """
        Draws the filtered boxes and labels on the original frame.
        """
        boxes = []
        box_count = 1  # Initialize box counter

        # Draw a red bounding box around the ROI
        cv2.rectangle(current_frame, (x_start, y_start), (x_start + roi.shape[1], y_start + roi.shape[0]), (255, 0, 0), 2)

        for rect in filtered_boxes:
            center_x, center_y, w, h, angle = rect
            center_x, center_y = int(center_x), int(center_y)

            # Calculate the box points and draw the contour boundary
            box_points = cv2.boxPoints(((center_x, center_y), (w, h), angle))
            box_points = np.int0(box_points)
            cv2.drawContours(current_frame, [box_points], -1, (0, 255, 0), 2)

            # Draw the center point of the bounding box
            cv2.circle(current_frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Detect barcode
            barcode_data = self.detect_barcode(box_points, current_frame)

            point_3d = Camera.deproject_pixel_to_point(intrinsics, (center_x, center_y), depth_image[center_y, center_x]/1000.0)

            # Round each component of the 3D point to 2 decimal places
            point_3d = [round(coord, 2) for coord in point_3d]

            # Add box label
            if binDetection:
                bin_label = f"Bin {box_count}"
                cv2.putText(current_frame, bin_label, (center_x, center_y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                box_label = f"Box {box_count}"
                cv2.putText(current_frame, box_label, (center_x, center_y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            barcode_label = f"Barcode: {barcode_data}"
            cv2.putText(current_frame, barcode_label, (center_x, center_y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            coord_label = f"{point_3d}"
            cv2.putText(current_frame, coord_label, (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Convert angle to radians before adding to box list
            angle = angle * (np.pi/180.0)
            boxes.append((center_x, center_y, angle, box_count, barcode_data))
            
            box_count += 1  # Increment the box counter

        return boxes

    def detect_barcode(self, box_points, current_frame):
        
        height, width, _ = current_frame.shape

        # Calculate ROI from the box points
        x_min, y_min = np.min(box_points, axis=0)
        x_max, y_max = np.max(box_points, axis=0)

        # Validate the coordinates to make sure the ROI is within bounds
        x_min = max(0, x_min)  # Ensure x_min is within image width
        y_min = max(0, y_min)  # Ensure y_min is within image height
        x_max = min(width, x_max)  # Ensure x_max is within image width
        y_max = min(height, y_max)  # Ensure y_max is within image height

        # Crop the ROI
        cropped_roi = current_frame[y_min:y_max, x_min:x_max]

        # Detect barcodes in the cropped ROI
        barcodes = decode(cropped_roi)

        for barcode in barcodes:
            barcode_data = barcode.data.decode("utf-8")
            return barcode_data
        
        return "No barcode detected."
    
    def process_image(self, msg):
        """
        Converts ROS image message to OpenCV image.
        """
        br_rgb = CvBridge()
        return br_rgb.imgmsg_to_cv2(msg)
    
    def extract_roi(self, current_frame, x_start, y_start, w, h):
        """
        Extracts the region of interest (ROI) from the current frame.
        """
        # Check if the image is RGB (3 channels) or depth (1 channel)
        if len(current_frame.shape) == 3:  # RGB image (height, width, channels)
            height, width, _ = current_frame.shape
        elif len(current_frame.shape) == 2:  # Depth image (height, width)
            height, width = current_frame.shape

        # Ensure the bounding box is within the frame dimensions
        x_end = min(x_start + w, width)
        y_end = min(y_start + h, height)

        # Extract the ROI using the bounding box coordinates
        roi = current_frame[y_start:y_end, x_start:x_end]
        return roi, x_start, y_start
    
