#include <edge_detection/EdgeDetector.hpp>
#include <ros/ros.h>

using namespace edge_detection;

namespace edge_detection {

EdgeDetector::EdgeDetector(ros::NodeHandle& nh) : nh_(nh) {
    // Constructor
    ROS_INFO("EdgeDetector constructor called.");
}

void EdgeDetector::init() {
    // Initialization logic goes here
    ROS_INFO("EdgeDetector initialized.");
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "edge_detection_node");
    ros::NodeHandle nh;

    edge_detection::EdgeDetector detector(nh);
    detector.init();

    ros::spin();
    return 0;
}
