cmake_minimum_required(VERSION 2.8.3)
project(python_pcd)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES python_pcd
#  CATKIN_DEPENDS rospy sensor_msgs
#  DEPENDS system_lib
)
