// lines_detector.cpp
// Header of the AutomodelLineDetector class for line detection using OpenCV and
// ROS.
//
// Created on: Mar 16, 2018
// Author: Abel Pacheco Ortega

#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <lines_detector/LinesDetectorConfig.h>  // auto-generated
#include <ros/ros.h>

#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>

namespace automodel {
namespace lines_detector {

class AutomodelLinesDetector {
 public:
  AutomodelLinesDetector(ros::NodeHandle &nodeHandle);
  virtual ~AutomodelLinesDetector();

  void detect(const sensor_msgs::ImageConstPtr &);

 private:
  cv::Mat preprocessImage(const cv::Mat &img_mono);
  void detectLines(const cv::Mat &edges);
  void publishDetectedLines();
  void publishImageDetectedLines(const cv::Mat &image);
  void publishImageDetectedEdges(const cv::Mat &image);
  void readParameters();
  void saveParameters();

  ros::NodeHandle nh_;
  std::string image_topic_;
  image_transport::Subscriber subscriber_;

  // Para Cannny
  int canny_low_thresh_;
  int canny_high_thresh_;
  int canny_percentage_horizon_;

  // Para Hough
  double rho_;
  double theta_;
  int hough_int_rho_;
  int hough_int_theta_;
  int hough_threshold_;
  int line_circular_buffer_size_;

  boost::circular_buffer<cv::Vec2f> cb_lines_left_;
  boost::circular_buffer<cv::Vec2f> cb_lines_right_;
  std::optional<cv::Vec2f> computeAverageLine(
      const boost::circular_buffer<cv::Vec2f> &cbLines) const;

  ros::Publisher publisher_left_line_;
  ros::Publisher publisher_right_line_;

  image_transport::Publisher publisher_lines_image_;
  image_transport::Publisher publisher_edges_image_;

  void updateParameters(dynamic::LinesDetectorConfig &config, uint32_t level);
  dynamic_reconfigure::Server<dynamic::LinesDetectorConfig> config_server_;
};

}  // namespace lines_detector
}  // namespace automodel
