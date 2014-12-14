#CMakeLists.txt
#Created by: 	Curtis Muntz
#Created for: 	Visual Servoing Project
#Summary:	CMake File for Opencv
###############################################

#####You must have this version of Cmake installed#####
cmake_minimum_required(VERSION 2.8.3)

#####This is the overall project name######
project(production)

##### Find catkin and any catkin packages or libraries#####
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge sensor_msgs image_transport geometry_msgs tf)

##### Find OpenCV libraries #####
find_package(OpenCV REQUIRED)

#	Note:  You must have linux environment variables configured
#		terminal command:	$OpenCV_INCLUDE_DIRS=`pkg-config --cflags opencv`
# 	 	terminal command:	$OpenCV_LIBRARIES=`pkg-config --libs opencv`


##### Generate added messages and services #####
generate_messages(DEPENDENCIES std_msgs)

##### Declare a catkin package #####
catkin_package()


##### Tell Cmake which include files are required #####
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})

##### build (opencv) executable  object locator #####
add_executable(objectLocator src/objectLocator.cpp)
target_link_libraries(objectLocator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(objectLocator production_gencpp)
