#!/usr/bin/env python

import rospy
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse


def edge_detection_client(img_dir):
    rospy.wait_for_service('edge_detection_server')
    image_files = [os.path.join(img_dir, img_file) for img_file in os.listdir(img_dir) if img_file.endswith(('.jpg', '.png'))]
    results_dir = os.path.join(img_dir, 'results')
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    for img_file in image_files:
        try:
            aruco_pose_detection = rospy.ServiceProxy('edge_detection_server', EdgeDetection)
            response = aruco_pose_detection(img_path=img_file)
            if response.info == False:
                rospy.logerr(f"Edge Detection Failed!")
            else:
                rospy.loginfo(f"Edge Detection for {img_file.split('/')[-1]} saved to {results_dir}.")
                img = cv2.imread(img_file, cv2.IMREAD_COLOR) 
                edges = np.array(response.data, dtype=np.uint8).reshape((response.height, response.width))
                edge_img = np.zeros_like(img)
                edge_img[edges != 0] = [0, 255, 0]  # Green color for edges
                edge_img = cv2.addWeighted(img, 0.5, edge_img, 1.5, 0)
                cv2.imwrite(os.path.join(results_dir, img_file.split('/')[-1]), edge_img)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('edge_detection_client')
    img_dir = '/neura/data/edge_detection_data/data/'
    edge_detection_client(img_dir=img_dir)
