#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "edge_detection/EdgeDetection.h"

namespace edge_detection {

class EdgeDetector {
public:
    EdgeDetector(ros::NodeHandle& nh);
    void init();
	void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void mainCallback(const sensor_msgs::ImageConstPtr& rgb_img,
                      const sensor_msgs::ImageConstPtr& depth_img);
	cv::Mat detectEdges(const cv::Mat& rgb_image, bool is_service_call = true) const;
	cv::Mat edgePixelsTo3D(const std::vector<cv::Point>& edge_pixels, const cv::Mat& depth_image) const;
	std::vector<cv::Point> getEdgePixels(const cv::Mat& edges) const;
	void publishPointCloudFromPoints(const cv::Mat& points_3d);
	void publishEdgeMarkers(const cv::Mat& points_3d);
	bool detectEdgesSrv(edge_detection::EdgeDetection::Request& req,
                        edge_detection::EdgeDetection::Response& res);

private:
    ros::NodeHandle nh_;

	std::string source_frame_id_;

	ros::Subscriber camera_info_sub_;
	sensor_msgs::CameraInfo camera_info_;
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;
	double fx_, fy_, cx_, cy_;

	message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
  	message_filters::Subscriber<sensor_msgs::Image> depth_sub;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;

	ros::Publisher edge_image_pub_;
	ros::Publisher edge_pc_pub_;
	ros::Publisher edge_marker_pub_;

	ros::ServiceServer edge_detector_srv_;
};

}  // namespace edge_detection
