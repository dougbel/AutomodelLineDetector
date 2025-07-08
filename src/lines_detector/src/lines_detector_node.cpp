#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "lines_detector.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace automodel::lines_detector;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lines_detector_node");

  // node handler
  ros::NodeHandle n("~");

  AutomodelLinesDetector lineDetector(n);

  ros::spin();
  return 0;
}
