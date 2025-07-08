#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <atomic>
#include <opencv2/videoio.hpp>
#include <thread>

namespace video_publisher {

class VideoPublisher {
 public:
  VideoPublisher(ros::NodeHandle &nh);
  ~VideoPublisher();
  void publish_frames();

 private:
  ros::NodeHandle nh_;

  // This is not a best practice http://wiki.ros.org/image_transport
  // ros::Publisher _pub;
  image_transport::Publisher frame_publisher_;

  std::string video_file_;
  cv::VideoCapture video_cap_;
  bool loop_;

  std::atomic<bool> running_;
  std::thread stream_thread_;
  void start();
};
}  // namespace video_publisher