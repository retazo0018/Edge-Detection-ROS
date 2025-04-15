#include <edge_detection/EdgeDetector.hpp>
#include <ros/ros.h>

using namespace edge_detection;

namespace edge_detection {

	EdgeDetector::EdgeDetector(ros::NodeHandle& nh) 
		: nh_(nh){
		ROS_INFO("EdgeDetector constructor called.");

		rgb_sub.subscribe(nh_, "/camera/color/image_raw", 1);
    	depth_sub.subscribe(nh_, "/camera/depth/image_rect_raw", 1);
		sync_.reset(new Sync(MySyncPolicy(10), rgb_sub, depth_sub));
    	sync_->registerCallback(boost::bind(&EdgeDetector::mainCallback, this, _1, _2));

		camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &EdgeDetector::cameraInfoCallback, this);
	}

	void EdgeDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    	camera_info_ = *msg;
		// Convert K to 3x3 matrix
		camera_matrix_ = cv::Mat(3, 3, CV_64F);
		for (int i = 0; i < 9; ++i) {
			camera_matrix_.at<double>(i / 3, i % 3) = msg->K[i];
		}
		// Convert D (distortion coefficients)
		dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
		for (size_t i = 0; i < msg->D.size(); ++i) {
			dist_coeffs_.at<double>(i, 0) = msg->D[i];
		}
		fx_ = camera_matrix_.at<double>(0, 0);
		fy_ = camera_matrix_.at<double>(1, 1);
		cx_ = camera_matrix_.at<double>(0, 2);
		cy_ = camera_matrix_.at<double>(1, 2);

		camera_info_sub_.shutdown();  // Unsubscribe after first callback
		ROS_INFO("Camera intrinsics extracted and camera_info subscriber shut down.");
	}

	void EdgeDetector::mainCallback(const sensor_msgs::ImageConstPtr& rgb_img, const sensor_msgs::ImageConstPtr& depth_img) {
		ROS_INFO("Received synchronized messages.");

	}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "edge_detection_node");
    ros::NodeHandle nh;

    edge_detection::EdgeDetector detector(nh);

    ros::spin();
    return 0;
}
