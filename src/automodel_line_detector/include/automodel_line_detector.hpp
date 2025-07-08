// automodel_line_detector.cpp
// Header of the AutomodelLineDetector class for line detection using OpenCV and
// ROS.
//
// Created on: Mar 16, 2018
// Author: Abel Pacheco Ortega

#pragma once

#include <automodel_line_detector/LineDetectorConfig.h>  // auto-generated
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>

namespace automodel {
namespace line_detector {

class AutomodelLineDetector {
 public:
  AutomodelLineDetector(ros::NodeHandle &nodeHandle);
  virtual ~AutomodelLineDetector();

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

  void updateParameters(automodel_line_detector::LineDetectorConfig &config,
                        uint32_t level);
  dynamic_reconfigure::Server<automodel_line_detector::LineDetectorConfig>
      config_server_;
};

}  // namespace line_detector
}  // namespace automodel
