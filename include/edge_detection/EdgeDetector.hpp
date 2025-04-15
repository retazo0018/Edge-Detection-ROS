#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace edge_detection {

class EdgeDetector {
public:
    EdgeDetector(ros::NodeHandle& nh);
    void init();
	void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
	void mainCallback(const sensor_msgs::ImageConstPtr& rgb_img,
                      const sensor_msgs::ImageConstPtr& depth_img);
	cv::Mat detectEdges(const cv::Mat& rgb_image, bool is_service_call = true) const;

private:
    ros::NodeHandle nh_;

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
};

}  // namespace edge_detection
