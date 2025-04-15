#include <edge_detection/EdgeDetector.hpp>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

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
		edge_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/edge_points", 1);
		edge_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/edge_points_marker", 1);

		source_frame_id_ = "camera_color_optical_frame";

		edge_detector_srv_ = nh_.advertiseService("edge_detection_server", &EdgeDetector::detectEdgesSrv, this);
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

	bool EdgeDetector::detectEdgesSrv(edge_detection::EdgeDetection::Request &req,
                                      edge_detection::EdgeDetection::Response &res) {
		try {
			cv::Mat img = cv::imread(req.img_path, cv::IMREAD_COLOR);
			if (img.empty()) {
				ROS_ERROR("Failed to read image from path: %s", req.img_path.c_str());
				res.info = false;
				return true;
			}

			cv::Mat edges = detectEdges(img);  // Your own function, returns cv::Mat

			res.height = edges.rows;
			res.width = edges.cols;
			res.info = true;

			// Flatten image matrix into 1D vector
			res.data.reserve(edges.total());
			for (int i = 0; i < edges.rows; ++i) {
				for (int j = 0; j < edges.cols; ++j) {
					res.data.push_back(edges.at<uchar>(i, j));
				}
			}

			return true;
		} catch (const std::exception &e) {
			ROS_ERROR("Exception in detectEdgesService: %s", e.what());
			res.info = false;
			res.height = 0;
			res.width = 0;
			res.data.clear();
			return true;
		}
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

	std::vector<cv::Point> EdgeDetector::getEdgePixels(const cv::Mat& edges) const {
		std::vector<cv::Point> edge_pixels;

		for (int y = 0; y < edges.rows; ++y) {
			for (int x = 0; x < edges.cols; ++x) {
				if (edges.at<uchar>(y, x) > 0) {
					edge_pixels.emplace_back(x, y);
				}
			}
		}
		return edge_pixels;
	}

	cv::Mat EdgeDetector::edgePixelsTo3D(const std::vector<cv::Point>& edge_pixels,
                                     const cv::Mat& depth_image) const {
		std::vector<cv::Vec3f> points_3d;

		for (const auto& pt : edge_pixels) {
			int u = pt.x;
			int v = pt.y;

			float z = depth_image.at<uint16_t>(v, u) / 1000.0f;  // convert mm to meters
			if (z <= 0.0f) continue;

			float x = (u - cx_) * z / fx_;
			float y = (v - cy_) * z / fy_;
			points_3d.emplace_back(x, y, z);
		}

		return cv::Mat(points_3d).clone(); // (N, 3) CV_32F matrix
	}

	void EdgeDetector::publishPointCloudFromPoints(const cv::Mat& points_3d) {
		sensor_msgs::PointCloud2 pc_msg;
		pc_msg.header.stamp = ros::Time::now();
		pc_msg.header.frame_id = source_frame_id_;

		pc_msg.height = 1;
		pc_msg.width = points_3d.rows;

		sensor_msgs::PointCloud2Modifier modifier(pc_msg);
		modifier.setPointCloud2FieldsByString(1, "xyz");
		modifier.resize(points_3d.rows);

		sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

		for (int i = 0; i < points_3d.rows; ++i, ++iter_x, ++iter_y, ++iter_z) {
			cv::Vec3f pt = points_3d.at<cv::Vec3f>(i);
			*iter_x = pt[0];
			*iter_y = pt[1];
			*iter_z = pt[2];
		}

		edge_pc_pub_.publish(pc_msg);
	}

	void EdgeDetector::publishEdgeMarkers(const cv::Mat& points_3d) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = source_frame_id_;
		marker.header.stamp = ros::Time::now();
		marker.ns = "edge_points";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;

		marker.scale.x = 0.01;
		marker.scale.y = 0.01;

		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0f;

		for (int i = 0; i < points_3d.rows; ++i) {
			cv::Vec3f pt = points_3d.at<cv::Vec3f>(i);
			geometry_msgs::Point p;
			p.x = pt[0];
			p.y = pt[1];
			p.z = pt[2];
			marker.points.push_back(p);
		}

		edge_marker_pub_.publish(marker);
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
			std::vector<cv::Point> edge_pixels = getEdgePixels(edges);

			// Convert edge pixels to 3D points
        	cv::Mat points_3d = edgePixelsTo3D(edge_pixels, depth_image);

			// Prepare the superimposed edge image for visualization
			cv::Mat edge_img = cv::Mat::zeros(rgb_image.size(), rgb_image.type());
			edge_img.setTo(cv::Scalar(0, 255, 0), edges != 0); // green for edges
			cv::addWeighted(rgb_image, 0.5, edge_img, 1.5, 0, edge_img);

			// Publish the 3D point cloud and markers
        	publishPointCloudFromPoints(points_3d);
			publishEdgeMarkers(points_3d);

			// Convert Edge Image to ROS image message and Publish
			sensor_msgs::ImagePtr edge_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", edge_img).toImageMsg();
			edge_image_pub_.publish(edge_img_msg);
			ROS_INFO("Published Edge image, Edge points and Edge Markers.");
		
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
