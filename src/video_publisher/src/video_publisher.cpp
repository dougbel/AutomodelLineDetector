#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include <thread>
#include <string>
#include "video_publisher.hpp"

using namespace video_publisher;

VideoPublisher::VideoPublisher(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
{
    image_transport::ImageTransport it(_nh);

    _nh.param<std::string>("video_path", _video_file, "/home/docker/catkin_ws/CarroAutonomo.avi");
    _nh.param<bool>("loop", _loop, true);

    _pub = it.advertise("frames", 1);

    _running = false;
    _cap = cv::VideoCapture(_video_file);
    if (!_cap.isOpened())
    {
        ROS_ERROR_STREAM("Cannot open video file " << _video_file);
    }
}

VideoPublisher::~VideoPublisher()
{
    _running = false;
    if (_thread.joinable())
    {
        _thread.join();
    }
    _cap.release();
}

void VideoPublisher::publish_frames()
{
    if (!_running)
    {
        _running = true;
        _thread = std::thread(&VideoPublisher::start, this);
    }
}

void VideoPublisher::start()
{
    // Set publish rate (30 Hz)
    ros::Rate rate(30);
    cv::Mat frame;
    sensor_msgs::ImagePtr imgMsg;
    while (ros::ok() && _running && _cap.isOpened())
    {
        _cap >> frame;

        if (frame.empty())
        {
            if (_loop)
            {
                _cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                ROS_INFO("Looping video stream");
                continue;
            }
            else
            {
                ROS_INFO("End of video stream");
                break;
            }
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_frame";
        try
        {
            imgMsg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            _pub.publish(imgMsg);
        }
        catch (cv_bridge::Exception e)
        {
            ROS_ERROR_STREAM("cv_bridge expcetion: " << e.what());
        }
        rate.sleep();
    }
}