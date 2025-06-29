/*
 * AutomodelLineDetector.h
 *
 *  Created on: Mar 16, 2018
 *      Author: dougbel
 */

#ifndef SRC_AUTOMODELLINEDETECTOR_H_
#define SRC_AUTOMODELLINEDETECTOR_H_

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <automodel_line_detector/line_detectorConfig.h> // auto-generated

using namespace std;
using namespace cv;

namespace automodel::line_detector
{

	const string IN_NAMED_WINDOW = "Input";
	const string OUT_NAMED_WINDOW = "Output";
	const string OUT2_NAMED_WINDOW = "Output prev";

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
		void publish_img_lines(Mat &image);
		void publish_img_edges(Mat &image);
		void read_parameters();

		void set_parameters(automodel_line_detector::line_detectorConfig &config, uint32_t level);

	private:
		void saveParameters();

		ros::NodeHandle _nh;
		string image_topic;
		image_transport::Subscriber sub;

		// Para Cannny
		int canny_lowThreshold;
		int canny_highThreshold;
		int canny_perBlindHorizon;

		// Para Hough
		double rho;
		double theta;
		int hough_int_rho;
		int hough_int_theta;
		int hough_threshold;

		Mat mask_yw_image;
		Mat imageColor;

		// to use in Hough tranform
		vector<Vec2f> _linesLeft;
		vector<Vec2f> _linesRight;

		ros::Publisher _pub_line_left;
		ros::Publisher _pub_line_right;

		image_transport::Publisher _pub_img_lines;
		image_transport::Publisher _pub_img_edges;

		dynamic_reconfigure::Server<automodel_line_detector::line_detectorConfig> server;
	};

}

#endif
