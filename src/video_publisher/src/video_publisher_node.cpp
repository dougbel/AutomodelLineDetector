#include <ros/ros.h>
#include "video_publisher.hpp"

using namespace video_publisher;

int main(int argc, char **argv)
{
  // initialize node
  ros::init(argc, argv, "video_publisher");

  // node handler
  ros::NodeHandle n("~");

  VideoPublisher publisher(n);

  publisher.publish_frames();

  ros::spin();

  return 0;
}
