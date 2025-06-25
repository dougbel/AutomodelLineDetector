#include <ros/ros.h>

using namespace std;

namespace video_publisher
{

    class VideoPublisher
    {
    private:
        ros::NodeHandle _nh;
        ros::Publisher _pub;
        string _video_file;
        bool _loop;

    public:
        VideoPublisher(ros::NodeHandle &nodeHandle);
        void publish_frames();
    };
}