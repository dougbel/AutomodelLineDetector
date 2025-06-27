#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include <thread>
using namespace std;

namespace video_publisher
{

    class VideoPublisher
    {
    private:
        ros::NodeHandle _nh;

        // this is not a best practice http://wiki.ros.org/image_transport
        //  ros::Publisher _pub;
        image_transport::Publisher _pub;

        string _video_file;
        cv::VideoCapture _cap;
        bool _loop;

        std::atomic<bool> _running;
        std::thread _thread;
        void start();

    public:
        VideoPublisher(ros::NodeHandle &nodeHandle);
        ~VideoPublisher();
        void publish_frames();
    };
}