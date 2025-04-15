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

		edge_image_pub_ = nh_.advertise<sensor_msgs::Image>("/edge_image", 1);
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

	cv::Mat EdgeDetector::detectEdges(const cv::Mat& rgb_image, bool is_service_call) const {
		cv::Mat img_gray, img_blur, edges;

		cv::cvtColor(rgb_image, img_gray, cv::COLOR_BGR2GRAY);
		cv::bilateralFilter(img_gray, img_blur, 9, 75, 75);
		cv::Canny(img_blur, edges, 100, 200);

		if (is_service_call) {
			return edges;
		}

		// ToDo: include contour logic
		return edges;
	}

	void EdgeDetector::mainCallback(const sensor_msgs::ImageConstPtr& rgb_img, const sensor_msgs::ImageConstPtr& depth_img) {
		ROS_INFO("Received synchronized messages.");

		cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(rgb_img, sensor_msgs::image_encodings::BGR8);
    	cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1); // assuming 16-bit depth

    	cv::Mat rgb_image = cv_rgb_ptr->image;
    	cv::Mat depth_image = cv_depth_ptr->image;

		try {
			// Detect edges and contours
        	cv::Mat edges = detectEdges(rgb_image, false);

			// Prepare the superimposed edge image for visualization
			cv::Mat edge_img = cv::Mat::zeros(rgb_image.size(), rgb_image.type());
			edge_img.setTo(cv::Scalar(0, 255, 0), edges != 0); // green for edges
			cv::addWeighted(rgb_image, 0.5, edge_img, 1.5, 0, edge_img);

			// Convert Edge Image to ROS image message and Publish
			sensor_msgs::ImagePtr edge_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", edge_img).toImageMsg();
			edge_image_pub_.publish(edge_img_msg);
			ROS_INFO("Published Edge Image.");
		
		} catch (const std::exception& e) {
        	ROS_ERROR("Edge detection failed: %s", e.what());
    	}
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "edge_detection_node");
    ros::NodeHandle nh;

    edge_detection::EdgeDetector detector(nh);

    ros::spin();
    return 0;
}
