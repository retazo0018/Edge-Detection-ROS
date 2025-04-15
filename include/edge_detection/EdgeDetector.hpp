#pragma once

#include <ros/ros.h>

namespace edge_detection {

class EdgeDetector {
public:
    EdgeDetector(ros::NodeHandle& nh);
    void init();

private:
    ros::NodeHandle nh_;
};

}  // namespace edge_detection
