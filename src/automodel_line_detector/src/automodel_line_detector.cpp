// automodel_line_detector.cpp
// Implementation of the AutomodelLineDetector class for line detection using OpenCV and ROS.
//
// Created on: Mar 16, 2018
// Author: Abel Pacheco Ortega

#include "automodel_line_detector.hpp"

namespace automodel::line_detector
{

	AutomodelLineDetector::AutomodelLineDetector(ros::NodeHandle &nodeHandle) : nh_(nodeHandle)
	{
		read_parameters();

		image_transport::ImageTransport it(nh_);
		sub_ = it.subscribe(image_topic, 1, &AutomodelLineDetector::detect, this);

		pub_line_left_ = nh_.advertise<std_msgs::Float32MultiArray>("left", 1);
		pub_line_right_ = nh_.advertise<std_msgs::Float32MultiArray>("right", 1);

		pub_img_lines_ = it.advertise("img_lines", 1);
		pub_img_edges_ = it.advertise("img_edges", 1);

		cb_lines_left_ = boost::circular_buffer<cv::Vec2f>(line_circular_buffer_size_);
		cb_lines_right_ = boost::circular_buffer<cv::Vec2f>(line_circular_buffer_size_);

		dynamic_reconfigure::Server<automodel_line_detector::line_detectorConfig>::CallbackType f;

		f = boost::bind(&AutomodelLineDetector::set_parameters, this, _1, _2);
		config_server_.setCallback(f);
	}

	void AutomodelLineDetector::set_parameters(automodel_line_detector::line_detectorConfig &config, uint32_t level)
	{
		// Update your internal parameters
		canny_percentage_horizon = config.canny_perBlindHorizon;
		canny_low_thresh_ = config.canny_lowThreshold;
		canny_high_thresh_ = config.canny_highThreshold;

		hough_int_rho_ = config.hough_int_rho;
		hough_int_theta_ = config.hough_int_theta;
		hough_threshold_ = config.hough_threshold;

		if (config.line_circular_buffer_size != cb_lines_left_.capacity())
		{
			cb_lines_left_.rset_capacity(config.line_circular_buffer_size);
			cb_lines_right_.rset_capacity(config.line_circular_buffer_size);
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
		inRange(img_mono, canny_low_thresh_, canny_high_thresh_, mask_white);
		bitwise_and(img_mono, mask_white, masked_image);

		const int horizon_y = static_cast<int>(0.01 * canny_percentage_horizon * img_mono.rows);
		masked_image(cv::Rect(0, 0, img_mono.cols, horizon_y)) = cv::Scalar(0);

		cv::Mat edges;
		cv::Canny(masked_image, edges, canny_low_thresh_, canny_high_thresh_);

		return edges;
	}

	void AutomodelLineDetector::detect_lines(const cv::Mat &edges)
	{
		rho_ = hough_int_rho_;
		theta_ = hough_int_theta_ * CV_PI / 180;

		std::vector<cv::Vec2f> all_lines;
		HoughLines(edges, all_lines, rho_, theta_, hough_threshold_);

		bool found_left = false, found_right = false;

		for (const auto &line : all_lines)
		{
			const float angle = line[1];

			if (!found_left && angle >= 0 && angle <= (CV_PI / 3)) // 0 to 60 degrees
			{
				cb_lines_left_.push_back(line);
				found_left = true;
			}
			else if (!found_right && angle >= (2 * CV_PI / 3) && angle <= CV_PI) // 120 to 180 degrees
			{
				cb_lines_right_.push_back(line);
				found_right = true;
			}

			if (found_left && found_right)
				break;
		}

		if (!found_left && !cb_lines_left_.empty())
			cb_lines_left_.pop_front();
		if (!found_right && !cb_lines_right_.empty())
			cb_lines_right_.pop_front();
	}

	std::optional<cv::Vec2f> AutomodelLineDetector::compute_average_line(const boost::circular_buffer<cv::Vec2f> &cb_lines) const
	{
		if (cb_lines.empty())
			return std::nullopt;

		float rho_sum = 0, theta_sum = 0;
		for (const auto &line : cb_lines)
		{
			rho_sum += line[0];
			theta_sum += line[1];
		}
		return cv::Vec2f(rho_sum / cb_lines.size(), theta_sum / cb_lines.size());
	}

	void AutomodelLineDetector::publishLines()
	{
		auto publish_line = [](ros::Publisher &pub, const cv::Vec2f &detected_line, const std::string &label)
		{
			// float rho = lines[0][0], theta = lines[0][1];
			const float rho = detected_line[0];
			const float theta = detected_line[1];

			std_msgs::Float32MultiArray msg;
			msg.layout.dim.emplace_back();
			msg.layout.dim[0].size = 3;
			msg.layout.dim[0].stride = 1;
			msg.layout.dim[0].label = label;
			msg.data = {cos(theta), sin(theta), rho};
			pub.publish(msg);
		};

		auto left_line = compute_average_line(cb_lines_left_);
		if (left_line)
		{
			publish_line(pub_line_left_, *left_line, "left");
		}

		auto right_line = compute_average_line(cb_lines_right_);
		if (right_line)
		{
			publish_line(pub_line_right_, *right_line, "right");
		}
	}

	void AutomodelLineDetector::publish_img_lines(cv::Mat &image)
	{
		if (pub_img_lines_.getNumSubscribers() == 0)
			return;

		imageColor_ = image.clone();
		auto draw_line = [&](const cv::Vec2f &detected_line, cv::Scalar color, const std::string &label, cv::Point text_pos)
		{
			const float rho = detected_line[0];
			const float theta = detected_line[1];
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
			cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
			line(imageColor_, pt1, pt2, color, 3, cv::LINE_AA);
			putText(imageColor_, label + ": rho " + std::to_string(rho) + " angle: " + std::to_string(theta), text_pos,
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, cv::LINE_AA);
		};

		auto left_line = compute_average_line(cb_lines_left_);
		if (left_line)
		{
			draw_line(*left_line, cv::Scalar(0, 255, 0), "LEFT", cv::Point(30, 80));
		}

		auto right_line = compute_average_line(cb_lines_right_);
		if (right_line)
		{
			draw_line(*right_line, cv::Scalar(0, 0, 255), "RIGHT", cv::Point(30, 30));
		}

		sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageColor_).toImageMsg();
		imgMsg->header.stamp = ros::Time::now();
		imgMsg->header.frame_id = "img_lines";
		pub_img_lines_.publish(imgMsg);
	}

	void AutomodelLineDetector::publish_img_edges(const cv::Mat &image)
	{
		if (pub_img_edges_.getNumSubscribers() == 0)
			return;

		sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
		imgMsg->header.stamp = ros::Time::now();
		imgMsg->header.frame_id = "img_edges";
		pub_img_edges_.publish(imgMsg);
	}

	void AutomodelLineDetector::read_parameters()
	{
		nh_.param("/automodel/line_detector/image_topic", image_topic, std::string("/app/camera/rgb/image_raw"));
		nh_.param("/automodel/line_detector/canny_lowThreshold", canny_low_thresh_, 172);
		nh_.param("/automodel/line_detector/canny_highThreshold", canny_high_thresh_, 179);
		nh_.param("/automodel/line_detector/canny_perBlindHorizon", canny_percentage_horizon, 46);
		nh_.param("/automodel/line_detector/hough_int_rho", hough_int_rho_, 1);
		nh_.param("/automodel/line_detector/hough_int_theta", hough_int_theta_, 1);
		nh_.param("/automodel/line_detector/hough_threshold", hough_threshold_, 45);
		nh_.param("/automodel/line_detector/line_circular_buffer_size", line_circular_buffer_size_, 10);

		ROS_INFO_STREAM("Image topic: " << image_topic);
		ROS_INFO_STREAM("Canny lowThreshold: " << canny_low_thresh_);
		ROS_INFO_STREAM("Canny highThreshold: " << canny_high_thresh_);
		ROS_INFO_STREAM("Blind horizon (%): " << canny_percentage_horizon);
		ROS_INFO_STREAM("Hough rho: " << hough_int_rho_);
		ROS_INFO_STREAM("Hough theta: " << hough_int_theta_);
		ROS_INFO_STREAM("Hough threshold: " << hough_threshold_);
		ROS_INFO_STREAM("Circular Buffer " << line_circular_buffer_size_);
	}

	void AutomodelLineDetector::saveParameters()
	{
		std::string file = ros::package::getPath("automodel_line_detector") + "/config/config_new.yaml";
		cv::FileStorage fs(file, cv::FileStorage::WRITE);
		fs << "image_topic" << image_topic;
		fs << "canny_lowThreshold" << canny_low_thresh_;
		fs << "canny_highThreshold" << canny_high_thresh_;
		fs << "canny_perBlindHorizon" << canny_percentage_horizon;
		fs << "hough_int_rho" << hough_int_rho_;
		fs << "hough_int_theta" << hough_int_theta_;
		fs << "hough_threshold" << hough_threshold_;
		fs << "line_circular_buffer_size" << line_circular_buffer_size_;

		ROS_WARN_STREAM("File saved at " << file);
		ROS_WARN_STREAM("EDIT AND REPLACE to APPLY!");
	}

} // namespace automodel::line_detector
