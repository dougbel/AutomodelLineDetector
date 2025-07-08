#include "video_publisher.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/videoio.hpp>
#include <string>
#include <thread>

namespace video_publisher {

VideoPublisher::VideoPublisher(ros::NodeHandle &nh) : nh_(nh) {
  nh_.param<std::string>("video_path", video_file_,
                         "/home/docker/catkin_ws/data/CarroAutonomo.avi");
  nh_.param<bool>("loop", loop_, true);

  // Note: This approach is not recommended according to the ROS image_transport
  // documentation: http://wiki.ros.org/image_transport
  //  _pub = _nh.advertise<sensor_msgs::Image>("frames", 10);

  image_transport::ImageTransport it(nh_);
  frame_publisher_ = it.advertise("frames", 1);

  running_ = false;
  video_cap_ = cv::VideoCapture(video_file_);
  if (!video_cap_.isOpened()) {
    ROS_ERROR_STREAM("Cannot open video file " << video_file_);
  }
}

VideoPublisher::~VideoPublisher() {
  running_ = false;
  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }
  video_cap_.release();
}

void VideoPublisher::publish_frames() {
  if (!running_) {
    running_ = true;
    stream_thread_ = std::thread(&VideoPublisher::start, this);
  }
}

void VideoPublisher::start() {
  // Set publish rate (30 Hz)
  ros::Rate rate(30);
  cv::Mat frame;
  sensor_msgs::ImagePtr imageMsg;
  while (ros::ok() && running_ && video_cap_.isOpened()) {
    video_cap_ >> frame;

    if (frame.empty()) {
      if (loop_) {
        video_cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        ROS_INFO("Looping video stream");
        continue;
      } else {
        ROS_INFO("End of video stream");
        break;
      }
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera_frame";
    try {
      imageMsg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      frame_publisher_.publish(imageMsg);
    } catch (const cv_bridge::Exception &e) {
      ROS_ERROR_STREAM("cv_bridge expcetion: " << e.what());
    }
    rate.sleep();
  }
}
}  // namespace video_publisher