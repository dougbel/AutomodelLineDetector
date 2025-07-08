#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "automodel_line_detector.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace automodel::line_detector;

int main(int argc, char **argv) {
  ros::init(argc, argv, "automodel_line_detector_node");

  // node handler
  ros::NodeHandle n("~");

  AutomodelLineDetector lineDetector(n);

  ros::spin();
  return 0;
}
