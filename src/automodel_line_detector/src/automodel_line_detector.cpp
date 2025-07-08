// automodel_line_detector.cpp
//
// Implementation of the AutomodelLineDetector class.
// Handles line detection using OpenCV and ROS.
//
// Author: Abel Pacheco Ortega
// Created: Mar 16, 2018

#include "automodel_line_detector.hpp"

#include <image_transport/image_transport.h>
#include <ros/package.h>

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Float32MultiArray.h"

namespace automodel {
namespace line_detector {

AutomodelLineDetector::AutomodelLineDetector(ros::NodeHandle &nodeHandle)
    : nh_(nodeHandle) {
  readParameters();

  image_transport::ImageTransport it(nh_);
  subscriber_ =
      it.subscribe(image_topic_, 1, &AutomodelLineDetector::detect, this);

  publisher_left_line_ = nh_.advertise<std_msgs::Float32MultiArray>("left", 1);
  publisher_right_line_ =
      nh_.advertise<std_msgs::Float32MultiArray>("right", 1);

  publisher_lines_image_ = it.advertise("img_lines", 1);
  publisher_edges_image_ = it.advertise("img_edges", 1);

  cb_lines_left_ =
      boost::circular_buffer<cv::Vec2f>(line_circular_buffer_size_);
  cb_lines_right_ =
      boost::circular_buffer<cv::Vec2f>(line_circular_buffer_size_);

  dynamic_reconfigure::Server<
      automodel_line_detector::line_detectorConfig>::CallbackType f;

  f = boost::bind(&AutomodelLineDetector::updateParameters, this, _1, _2);
  config_server_.setCallback(f);
}

void AutomodelLineDetector::updateParameters(
    automodel_line_detector::line_detectorConfig &config, uint32_t level) {
  // Update your internal parameters
  canny_percentage_horizon_ = config.canny_perBlindHorizon;
  canny_low_thresh_ = config.canny_lowThreshold;
  canny_high_thresh_ = config.canny_highThreshold;

  hough_int_rho_ = config.hough_int_rho;
  hough_int_theta_ = config.hough_int_theta;
  hough_threshold_ = config.hough_threshold;

  if (config.line_circular_buffer_size != cb_lines_left_.capacity()) {
    cb_lines_left_.rset_capacity(config.line_circular_buffer_size);
    cb_lines_right_.rset_capacity(config.line_circular_buffer_size);
  }
}

AutomodelLineDetector::~AutomodelLineDetector() {
  ROS_INFO_STREAM("Destroying Automodel Line Detector");
}

void AutomodelLineDetector::detect(const sensor_msgs::ImageConstPtr &msg) {
  auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat imgColor = cv_ptr->image;
  cv::Mat imgMono;
  cv::cvtColor(imgColor, imgMono, cv::COLOR_BGR2GRAY);

  cv::Mat edges = preprocessImage(imgMono);
  detectLines(edges);

  publishDetectedLines();
  publishImageDetectedLines(imgColor);
  publishImageDetectedEdges(edges);
}

cv::Mat AutomodelLineDetector::preprocessImage(const cv::Mat &imgMono) {
  cv::Mat maskWhite, maskedImage;
  inRange(imgMono, canny_low_thresh_, canny_high_thresh_, maskWhite);
  bitwise_and(imgMono, maskWhite, maskedImage);

  const int horizon_y =
      static_cast<int>(0.01 * canny_percentage_horizon_ * imgMono.rows);
  maskedImage(cv::Rect(0, 0, imgMono.cols, horizon_y)) = cv::Scalar(0);

  cv::Mat edges;
  cv::Canny(maskedImage, edges, canny_low_thresh_, canny_high_thresh_);

  return edges;
}

void AutomodelLineDetector::detectLines(const cv::Mat &edges) {
  rho_ = hough_int_rho_;
  theta_ = hough_int_theta_ * CV_PI / 180;

  std::vector<cv::Vec2f> allDetectedLines;
  HoughLines(edges, allDetectedLines, rho_, theta_, hough_threshold_);

  bool foundLeft = false, foundRight = false;

  for (const auto &line : allDetectedLines) {
    const float angle = line[1];

    if (!foundLeft && angle >= 0 && angle <= (CV_PI / 3))  // 0 to 60 degrees
    {
      cb_lines_left_.push_back(line);
      foundLeft = true;
    } else if (!foundRight && angle >= (2 * CV_PI / 3) &&
               angle <= CV_PI)  // 120 to 180 degrees
    {
      cb_lines_right_.push_back(line);
      foundRight = true;
    }

    if (foundLeft && foundRight) break;
  }

  if (!foundLeft && !cb_lines_left_.empty()) cb_lines_left_.pop_front();
  if (!foundRight && !cb_lines_right_.empty()) cb_lines_right_.pop_front();
}

std::optional<cv::Vec2f> AutomodelLineDetector::computeAverageLine(
    const boost::circular_buffer<cv::Vec2f> &cbLines) const {
  if (cbLines.empty()) return std::nullopt;

  float rho_sum = 0, theta_sum = 0;
  for (const auto &line : cbLines) {
    rho_sum += line[0];
    theta_sum += line[1];
  }
  return cv::Vec2f(rho_sum / cbLines.size(), theta_sum / cbLines.size());
}

void AutomodelLineDetector::publishDetectedLines() {
  auto publish_line = [](ros::Publisher &pub, const cv::Vec2f &detectedLine,
                         const std::string &label) {
    const float kRho = detectedLine[0];
    const float kTheta = detectedLine[1];

    std_msgs::Float32MultiArray msg;
    msg.layout.dim.emplace_back();
    msg.layout.dim[0].size = 3;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = label;
    msg.data = {cos(kTheta), sin(kTheta), kRho};
    pub.publish(msg);
  };

  auto leftLine = computeAverageLine(cb_lines_left_);
  if (leftLine) {
    publish_line(publisher_left_line_, *leftLine, "left");
  }

  auto rightLine = computeAverageLine(cb_lines_right_);
  if (rightLine) {
    publish_line(publisher_right_line_, *rightLine, "right");
  }
}

void AutomodelLineDetector::publishImageDetectedLines(const cv::Mat &image) {
  if (publisher_lines_image_.getNumSubscribers() == 0) return;

  cv::Mat imageColor = image.clone();
  auto draw_line = [&](const cv::Vec2f &detectedLine, cv::Scalar color,
                       const std::string &label, cv::Point textPosition) {
    const float rho = detectedLine[0];
    const float theta = detectedLine[1];
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
    cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
    line(imageColor, pt1, pt2, color, 3, cv::LINE_AA);
    putText(imageColor,
            label + ": rho " + std::to_string(rho) +
                " angle: " + std::to_string(theta),
            textPosition, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1,
            cv::LINE_AA);
  };

  auto leftLine = computeAverageLine(cb_lines_left_);
  if (leftLine) {
    draw_line(*leftLine, cv::Scalar(0, 255, 0), "LEFT", cv::Point(30, 80));
  }

  auto rightLine = computeAverageLine(cb_lines_right_);
  if (rightLine) {
    draw_line(*rightLine, cv::Scalar(0, 0, 255), "RIGHT", cv::Point(30, 30));
  }

  sensor_msgs::ImagePtr imgMsg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageColor).toImageMsg();
  imgMsg->header.stamp = ros::Time::now();
  imgMsg->header.frame_id = "img_lines";
  publisher_lines_image_.publish(imgMsg);
}

void AutomodelLineDetector::publishImageDetectedEdges(const cv::Mat &image) {
  if (publisher_edges_image_.getNumSubscribers() == 0) return;

  sensor_msgs::ImagePtr imgMsg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  imgMsg->header.stamp = ros::Time::now();
  imgMsg->header.frame_id = "img_edges";
  publisher_edges_image_.publish(imgMsg);
}

void AutomodelLineDetector::readParameters() {
  nh_.param("/automodel/line_detector/image_topic", image_topic_,
            std::string("/app/camera/rgb/image_raw"));
  nh_.param("/automodel/line_detector/canny_lowThreshold", canny_low_thresh_,
            172);
  nh_.param("/automodel/line_detector/canny_highThreshold", canny_high_thresh_,
            179);
  nh_.param("/automodel/line_detector/canny_perBlindHorizon",
            canny_percentage_horizon_, 46);
  nh_.param("/automodel/line_detector/hough_int_rho", hough_int_rho_, 1);
  nh_.param("/automodel/line_detector/hough_int_theta", hough_int_theta_, 1);
  nh_.param("/automodel/line_detector/hough_threshold", hough_threshold_, 45);
  nh_.param("/automodel/line_detector/line_circular_buffer_size",
            line_circular_buffer_size_, 10);

  ROS_INFO_STREAM("Image topic: " << image_topic_);
  ROS_INFO_STREAM("Canny lowThreshold: " << canny_low_thresh_);
  ROS_INFO_STREAM("Canny highThreshold: " << canny_high_thresh_);
  ROS_INFO_STREAM("Blind horizon (%): " << canny_percentage_horizon_);
  ROS_INFO_STREAM("Hough rho: " << hough_int_rho_);
  ROS_INFO_STREAM("Hough theta: " << hough_int_theta_);
  ROS_INFO_STREAM("Hough threshold: " << hough_threshold_);
  ROS_INFO_STREAM("Circular Buffer " << line_circular_buffer_size_);
}

void AutomodelLineDetector::saveParameters() {
  std::string file = ros::package::getPath("automodel_line_detector") +
                     "/config/config_new.yaml";
  cv::FileStorage fs(file, cv::FileStorage::WRITE);
  fs << "image_topic" << image_topic_;
  fs << "canny_lowThreshold" << canny_low_thresh_;
  fs << "canny_highThreshold" << canny_high_thresh_;
  fs << "canny_perBlindHorizon" << canny_percentage_horizon_;
  fs << "hough_int_rho" << hough_int_rho_;
  fs << "hough_int_theta" << hough_int_theta_;
  fs << "hough_threshold" << hough_threshold_;
  fs << "line_circular_buffer_size" << line_circular_buffer_size_;

  ROS_WARN_STREAM("File saved at " << file);
  ROS_WARN_STREAM("EDIT AND REPLACE to APPLY!");
}
}  // namespace line_detector
}  // namespace automodel
