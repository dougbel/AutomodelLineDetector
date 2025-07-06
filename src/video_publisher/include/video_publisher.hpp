#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include <thread>

namespace video_publisher
{

    class VideoPublisher
    {

    public:
        VideoPublisher(ros::NodeHandle &nodeHandle);
        ~VideoPublisher();
        void publish_frames();

    private:
        ros::NodeHandle nh_;

        // this is not a best practice http://wiki.ros.org/image_transport
        //  ros::Publisher _pub;
        image_transport::Publisher pub_;

        std::string video_file_;
        cv::VideoCapture video_cap_;
        bool loop_;

        std::atomic<bool> running_;
        std::thread stream_thread_;
        void start();
    };
} // namespace video_publisher