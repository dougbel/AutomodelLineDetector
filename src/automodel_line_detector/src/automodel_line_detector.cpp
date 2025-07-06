/*
 * AutomodelLineDetector.cpp
 *
 *  Created on: Mar 16, 2018
 *      Abel Pacheco Ortega
 */

#include "automodel_line_detector.hpp"

using namespace cv;

namespace automodel::line_detector
{

	AutomodelLineDetector::AutomodelLineDetector(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
	{
		read_parameters();

		image_transport::ImageTransport it(_nh);
		sub = it.subscribe(image_topic, 1, &AutomodelLineDetector::detect, this);

		_pub_line_left = _nh.advertise<std_msgs::Float32MultiArray>("left", 1);
		_pub_line_right = _nh.advertise<std_msgs::Float32MultiArray>("right", 1);

		_pub_img_lines = it.advertise("img_lines", 1);
		_pub_img_edges = it.advertise("img_edges", 1);

		_cb_lines_left = boost::circular_buffer<Vec2f>(line_circular_buffer_size);
		_cb_lines_right = boost::circular_buffer<Vec2f>(line_circular_buffer_size);

		dynamic_reconfigure::Server<automodel_line_detector::line_detectorConfig>::CallbackType f;

		f = boost::bind(&AutomodelLineDetector::set_parameters, this, _1, _2);
		config_server.setCallback(f);
	}

	void AutomodelLineDetector::set_parameters(automodel_line_detector::line_detectorConfig &config, uint32_t level)
	{
		// Update your internal parameters
		canny_perBlindHorizon = config.canny_perBlindHorizon;
		canny_lowThreshold = config.canny_lowThreshold;
		canny_highThreshold = config.canny_highThreshold;

		hough_int_rho = config.hough_int_rho;
		hough_int_theta = config.hough_int_theta;
		hough_threshold = config.hough_threshold;

		if (config.line_circular_buffer_size != _cb_lines_left.capacity())
		{
			ROS_INFO_STREAM("Resizing circular buffers from " << _cb_lines_left.capacity()
															  << " to " << config.line_circular_buffer_size);
			_cb_lines_left.rset_capacity(config.line_circular_buffer_size);
			_cb_lines_right.rset_capacity(config.line_circular_buffer_size);
		}
	}

	AutomodelLineDetector::~AutomodelLineDetector()
	{
		ROS_INFO_STREAM("Destroying Automodel Line Detector");
	}

	void AutomodelLineDetector::detect(const sensor_msgs::ImageConstPtr &msg)
	{
		auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat img_rgb = cv_ptr->image;
		cv::Mat img_mono;
		cv::cvtColor(img_rgb, img_mono, cv::COLOR_BGR2GRAY);

		cv::Mat edges = preprocess_image(img_mono);
		detect_lines(edges);

		publishLines();
		publish_img_lines(img_rgb);
		publish_img_edges(edges);
	}

	cv::Mat AutomodelLineDetector::preprocess_image(const cv::Mat &img_mono)
	{
		cv::Mat mask_white, masked_image;
		inRange(img_mono, canny_lowThreshold, canny_highThreshold, mask_white);
		bitwise_and(img_mono, mask_white, masked_image);

		int horizon_y = static_cast<int>(0.01 * canny_perBlindHorizon * img_mono.rows);
		masked_image(Rect(0, 0, img_mono.cols, horizon_y)) = Scalar(0);

		cv::Mat edges;
		cv::Canny(masked_image, edges, canny_lowThreshold, canny_highThreshold);

		return edges;
	}

	void AutomodelLineDetector::detect_lines(const cv::Mat &edges)
	{
		rho = hough_int_rho;
		theta = hough_int_theta * CV_PI / 180;

		std::vector<Vec2f> all_lines;
		HoughLines(edges, all_lines, rho, theta, hough_threshold);

		bool detected;

		detected = false;
		for (const auto &line : all_lines)
		{
			if (line[1] >= 0 && line[1] <= (CV_PI / 3))
			{
				// 0 to 60 degrees
				detected = true;
				_cb_lines_left.push_back(line);
				break;
			}
		}
		if (!detected && _cb_lines_left.size() > 0)
		{
			_cb_lines_left.pop_front();
		}

		detected = false;
		for (const auto &line : all_lines)
		{
			if (line[1] >= (2 * CV_PI / 3) && line[1] <= CV_PI)
			{
				// 120 to 180 degrees
				detected = true;
				_cb_lines_right.push_back(line);
				break;
			}
		}

		if (!detected && _cb_lines_right.size() > 0)
		{
			_cb_lines_right.pop_front();
		}
	}

	void AutomodelLineDetector::publishLines()
	{
		auto publish_line = [](ros::Publisher &pub, const boost::circular_buffer<Vec2f> &cb_lines, const std::string &label)
		{
			if (!cb_lines.empty())
			{
				// float rho = lines[0][0], theta = lines[0][1];
				float rho = 0, theta = 0;
				for (Vec2f x : cb_lines)
				{
					rho += x[0];
					theta += x[1];
				}
				rho /= cb_lines.size();
				theta /= cb_lines.size();

				std_msgs::Float32MultiArray msg;
				msg.layout.dim.emplace_back();
				msg.layout.dim[0].size = 3;
				msg.layout.dim[0].stride = 1;
				msg.layout.dim[0].label = label;
				msg.data = {cos(theta), sin(theta), rho};
				pub.publish(msg);
			}
		};

		publish_line(_pub_line_left, _cb_lines_left, "left");
		publish_line(_pub_line_right, _cb_lines_right, "right");
	}

	void AutomodelLineDetector::publish_img_lines(Mat &image)
	{
		if (_pub_img_lines.getNumSubscribers() == 0)
			return;

		imageColor = image.clone();
		auto draw_line = [&](const boost::circular_buffer<Vec2f> &cb_lines, Scalar color, const std::string &label, Point text_pos)
		{
			if (!cb_lines.empty())
			{
				float rho = 0, theta = 0;
				for (Vec2f x : cb_lines)
				{
					rho += x[0];
					theta += x[1];
				}
				rho /= cb_lines.size();
				theta /= cb_lines.size();
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
				Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
				line(imageColor, pt1, pt2, color, 3, LINE_AA);
				putText(imageColor, label + ": rho " + to_string(rho) + " angle: " + to_string(theta), text_pos,
						FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, LINE_AA);
			}
		};
		draw_line(_cb_lines_right, Scalar(0, 0, 255), "RIGHT", Point(30, 30));
		draw_line(_cb_lines_left, Scalar(0, 255, 0), "LEFT", Point(30, 80));

		sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageColor).toImageMsg();
		imgMsg->header.stamp = ros::Time::now();
		imgMsg->header.frame_id = "img_lines";
		_pub_img_lines.publish(imgMsg);
	}

	void AutomodelLineDetector::publish_img_edges(Mat &image)
	{
		if (_pub_img_edges.getNumSubscribers() == 0)
			return;

		sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
		imgMsg->header.stamp = ros::Time::now();
		imgMsg->header.frame_id = "img_edges";
		_pub_img_edges.publish(imgMsg);
	}

	void AutomodelLineDetector::read_parameters()
	{
		_nh.param("/automodel/line_detector/image_topic", image_topic, std::string("/app/camera/rgb/image_raw"));
		_nh.param("/automodel/line_detector/canny_lowThreshold", canny_lowThreshold, 172);
		_nh.param("/automodel/line_detector/canny_highThreshold", canny_highThreshold, 179);
		_nh.param("/automodel/line_detector/canny_perBlindHorizon", canny_perBlindHorizon, 46);
		_nh.param("/automodel/line_detector/hough_int_rho", hough_int_rho, 1);
		_nh.param("/automodel/line_detector/hough_int_theta", hough_int_theta, 1);
		_nh.param("/automodel/line_detector/hough_threshold", hough_threshold, 45);
		_nh.param("/automodel/line_detector/line_circular_buffer_size", line_circular_buffer_size, 10);

		ROS_INFO_STREAM("Image topic: " << image_topic);
		ROS_INFO_STREAM("Canny lowThreshold: " << canny_lowThreshold);
		ROS_INFO_STREAM("Canny highThreshold: " << canny_highThreshold);
		ROS_INFO_STREAM("Blind horizon (%): " << canny_perBlindHorizon);
		ROS_INFO_STREAM("Hough rho: " << hough_int_rho);
		ROS_INFO_STREAM("Hough theta: " << hough_int_theta);
		ROS_INFO_STREAM("Hough threshold: " << hough_threshold);
		ROS_INFO_STREAM("Circular Buffer" << line_circular_buffer_size);
	}

	void AutomodelLineDetector::saveParameters()
	{
		std::string file = ros::package::getPath("automodel_line_detector") + "/config/config_new.yaml";
		FileStorage fs(file, FileStorage::WRITE);
		fs << "image_topic" << image_topic;
		fs << "canny_lowThreshold" << canny_lowThreshold;
		fs << "canny_highThreshold" << canny_highThreshold;
		fs << "canny_perBlindHorizon" << canny_perBlindHorizon;
		fs << "hough_int_rho" << hough_int_rho;
		fs << "hough_int_theta" << hough_int_theta;
		fs << "hough_threshold" << hough_threshold;
		fs << "line_circular_buffer_size" << line_circular_buffer_size;

		ROS_WARN_STREAM("File saved at " << file);
		ROS_WARN_STREAM("EDIT AND REPLACE to APPLY!");
	}

} // namespace automodel::line_detector
