#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include "video_publisher.hpp"

using namespace video_publisher;

VideoPublisher::VideoPublisher(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
{
    _pub = _nh.advertise<sensor_msgs::Image>("frames", 10);
    _nh.param<std::string>("video_path", _video_file, "/home/docker/catkin_ws/CarroAutonomo.avi");
};

void VideoPublisher::publish_frames()
{
    cv::VideoCapture cap(_video_file);

    if (!cap.isOpened())
    {
        ROS_ERROR_STREAM("Cannot open video file " << _video_file);
    }

    // Set publish rate (30 Hz)
    ros::Rate rate(30);
    cv::Mat frame;
    sensor_msgs::ImagePtr imgMsg;

    while (ros::ok() && cap.isOpened())
    {
        cap >> frame;

        if (frame.empty())
        {
            ROS_INFO("End of video stream");
        }

        try
        {
            imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            _pub.publish(imgMsg);
        }
        catch (cv_bridge::Exception e)
        {
            ROS_ERROR_STREAM("cv_bridge expcetion: " << e.what());
        }
        rate.sleep();
    }
    cap.release();
}
