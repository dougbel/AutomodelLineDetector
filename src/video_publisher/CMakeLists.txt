cmake_minimum_required(VERSION 3.0.2)
project(video_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES video_publisher
 CATKIN_DEPENDS cv_bridge image_transport rospy sensor_msgs std_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(
  ${PROJECT_NAME}_node 
  src/video_publisher_node.cpp
  src/video_publisher.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
