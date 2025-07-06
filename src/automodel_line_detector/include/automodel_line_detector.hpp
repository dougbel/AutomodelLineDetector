// automodel_line_detector.cpp
// Header of the AutomodelLineDetector class for line detection using OpenCV and ROS.
//
// Created on: Mar 16, 2018
// Author: Abel Pacheco Ortega

#pragma once

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <boost/circular_buffer.hpp>
#include <image_transport/image_transport.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <automodel_line_detector/line_detectorConfig.h> // auto-generated

namespace automodel::line_detector
{

	const std::string IN_NAMED_WINDOW = "Input";
	const std::string OUT_NAMED_WINDOW = "Output";
	const std::string OUT2_NAMED_WINDOW = "Output prev";

	class AutomodelLineDetector
	{
	public:
		AutomodelLineDetector(ros::NodeHandle &nodeHandle);
		virtual ~AutomodelLineDetector();

		void detect(const sensor_msgs::ImageConstPtr &);

		cv::Mat preprocess_image(const cv::Mat &img_mono);
		void detect_lines(const cv::Mat &edges);
		void publishLines();

		// void createGUI();
		void publish_img_lines(cv::Mat &image);
		void publish_img_edges(const cv::Mat &image);
		void read_parameters();

	private:
		void saveParameters();

		ros::NodeHandle nh_;
		std::string image_topic;
		image_transport::Subscriber sub_;

		// Para Cannny
		int canny_low_thresh_;
		int canny_high_thresh_;
		int canny_percentage_horizon;

		// Para Hough
		double rho_;
		double theta_;
		int hough_int_rho_;
		int hough_int_theta_;
		int hough_threshold_;
		int line_circular_buffer_size_;

		cv::Mat mask_yw_image_;
		cv::Mat imageColor_;

		boost::circular_buffer<cv::Vec2f> cb_lines_left_;
		boost::circular_buffer<cv::Vec2f> cb_lines_right_;
		std::optional<cv::Vec2f> compute_average_line(const boost::circular_buffer<cv::Vec2f> &cb_lines) const;

		ros::Publisher pub_line_left_;
		ros::Publisher pub_line_right_;

		image_transport::Publisher pub_img_lines_;
		image_transport::Publisher pub_img_edges_;

		void set_parameters(automodel_line_detector::line_detectorConfig &config, uint32_t level);
		dynamic_reconfigure::Server<automodel_line_detector::line_detectorConfig> config_server_;
	};

}
