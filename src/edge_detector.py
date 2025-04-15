#!/usr/bin/env python
# Written by Ashwin Murali ashwin.murali99@gmail.com 2025

import rospy
import cv2
import yaml
import numpy as np
import message_filters
from cv_bridge import CvBridge
import tf2_ros
import tf.transformations as tft
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse


class edgeDetectionNode:

    def __init__(self) -> None: 
        rospy.init_node('edge_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1)]
        # Source frame of detection
        self.source_frame = "camera_color_optical_frame"
        self.points_3d = np.array([])
        
        try:
            # Rosparams for camera topics
            camera_depth_aligned_pc_topic = rospy.get_param('~camera_depth_aligned_pc_topic', '/camera/depth/points')
            camera_depth_image_topic = rospy.get_param('~camera_depth_image_topic', '/camera/depth/image_rect_raw')
            camera_rgb_image_topic = rospy.get_param('~camera_rgb_image_topic', '/camera/color/image_raw')
        except (KeyError, TypeError) as e:
            rospy.logerr("Required rosparams not set. %s", e)

        # Subscribers
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        # points_sub = message_filters.Subscriber(camera_depth_aligned_pc_topic, PointCloud2, queue_size=10)
        depth_img_sub = message_filters.Subscriber(camera_depth_image_topic, Image, queue_size=10)
        rgb_img_sub = message_filters.Subscriber(camera_rgb_image_topic, Image, queue_size=10)
        
        # Publishers
        self.edge_image_pub = rospy.Publisher('/edge_image', Image, queue_size=1)
        self.edge_pc_pub = rospy.Publisher('/edge_points', PointCloud2, queue_size=1)
        self.edge_marker_pub = rospy.Publisher('/edge_points_marker', Marker, queue_size=1)
        
        # Service
        self.edge_detector_srv = rospy.Service('edge_detection_server', EdgeDetection, self.detect_edges_srv)

        # Time Synchronizer for RGB and Depth Images
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_img_sub, depth_img_sub], queue_size=2, slop=0.01, allow_headerless=False)
        self.ts.registerCallback(self.main)

        rospy.spin()   
       
    def camera_info_callback(self, camera_info)-> None:
        '''
            Camera Parameters (Intrinsics)    
        '''
    
        self.camera_info = camera_info
        self.camera_matrix = np.array(camera_info.K).reshape(3,3) 
        self.dist_coeffs = np.array(camera_info.D)
        self.fx, self.fy, self.cx, self.cy = self.camera_matrix[0,0], self.camera_matrix[1,1], self.camera_matrix[0,2], self.camera_matrix[1,2]
        self.camera_info_sub.unregister()

    def detect_edges_srv(self, req):
        '''
            Service call: Returns detected edges in the image        
        '''

        try:
            img_path = req.img_path
            img = cv2.imread(img_path, cv2.IMREAD_COLOR) 
            edges = self.detect_keypoints(img)
            height, width = edges.shape
            return EdgeDetectionResponse(info=True, height=height, width=width, data=edges.flatten().tolist())
        except:
            return EdgeDetectionResponse(info=False, height=0, width=0, data=[])

    @staticmethod
    def detect_keypoints(rgb_image, isServiceCall=True) -> np.ndarray:
        '''
            Detect edges and contours in the input image
        '''
        
        img_gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        # Noise removal
        img_blur = cv2.bilateralFilter(img_gray, d=9, sigmaColor=75, sigmaSpace=75)
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
        
        if isServiceCall:
            # Return only edges for a service call
            return edges
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logwarn("Contours not found!")
            return edges, None
        
        return edges, contours
    
    @staticmethod
    def get_edge_pixels(edges, contours) -> np.ndarray:
        '''
            Returns detected edges as pixels        
        '''

        edge_points = np.column_stack(np.where(edges > 0))
        # Convert (row, col) to (x, y)
        edge_pixels = [(int(x), int(y)) for y, x in edge_points]

        if contours is None:
            return np.asarray(edge_pixels)

        '''
        # Remove edges outside the checkerboard boundary
        largest_contour = max(contours, key=cv2.contourArea)
        hull = cv2.convexHull(largest_contour)
        # Filter edge pixels to keep only those inside the hull
        filtered_edge_pixels = []
        for (x, y) in edge_pixels:
            if cv2.pointPolygonTest(hull, (x, y), False) >= 0:
                filtered_edge_pixels.append((int(x), int(y)))
        '''

        return np.asarray(edge_pixels)
    
    def edge_pixels_to_3d(self, edge_pixels, depth_image) -> np.ndarray:
        '''
            Converts 2D pixels to 3D points in a batch for optimization
        '''
        
        u = edge_pixels[:, 0]
        v = edge_pixels[:, 1]
        # Extract depth values at those pixels and convert to meters
        Z = depth_image[v, u].astype(np.float32) / 1000.0
        # Create mask for valid depth
        valid = Z > 0
        Z, u, v = Z[valid], u[valid], v[valid]
        # Apply projection equations in vectorized form
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        # Stack into (N, 3) point array
        self.points_3d = np.stack((X, Y, Z), axis=-1)

    def publish_pointcloud_from_points(self) -> None:
        '''
            Publishes 3D points as pointcloud
        '''
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.source_frame
        pc_msg = pc2.create_cloud(header, self.fields, self.points_3d)
        self.edge_pc_pub.publish(pc_msg)
    
    def publish_edge_markers(self) -> None:
        '''
            Publishes 3D edge points as MarkerArray for Optimization
        '''
        marker = Marker()
        marker.header.frame_id = self.source_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edge_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for pt in self.points_3d:
            marker.points.append(Point(x=pt[0], y=pt[1], z=pt[2]))
        
        self.edge_marker_pub.publish(marker)

    def main(self, rgb_msg, depth_msg) -> None:
        '''
            Main Function
        '''
        
        rospy.loginfo("Received synchronized messages.")
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        try:
            edges, contours = self.detect_keypoints(rgb_image, isServiceCall=False)
            edge_pixels = self.get_edge_pixels(edges, contours)
            self.edge_pixels_to_3d(edge_pixels, depth_image)

            edge_img = np.zeros_like(rgb_image)
            edge_img[edges != 0] = [0, 255, 0]  # Green color for edges
            edge_img = cv2.addWeighted(rgb_image, 0.5, edge_img, 1.5, 0)
            edge_img_msg = self.bridge.cv2_to_imgmsg(edge_img, encoding="bgr8")
            edge_img_msg.header = rgb_msg.header

            self.edge_image_pub.publish(edge_img_msg)
            self.publish_pointcloud_from_points()
            self.publish_edge_markers()
            rospy.loginfo("Published edge image, edge points and edge markers.")
            
        except Exception as e:
            rospy.logerr("Edge detection failed: %s", e)

if __name__ == '__main__':
    edge_detection_node = edgeDetectionNode()
