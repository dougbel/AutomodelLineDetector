cmake_minimum_required(VERSION 2.8.3)
project(automodel_line_detector)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  dynamic_reconfigure
)

find_package( OpenCV REQUIRED )



## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/line_detector.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES automodel_line_detector
   CATKIN_DEPENDS roscpp std_msgs message_runtime cv_bridge sensor_msgs
   DEPENDS dynamic_reconfigure
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
	src/automodel_line_detector_node.cpp
	src/automodel_line_detector.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
