#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "automodel_line_detector.hpp"

using namespace automodel;

int main(int argc, char **argv)
{
  // initialize node
  ros::init(argc, argv, "automodel_line_detector");

  // node handler
  ros::NodeHandle n;

  AutomodelLineDetector lineDetector(n);

  return 0;
}
