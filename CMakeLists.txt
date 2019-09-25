cmake_minimum_required(VERSION 2.8.3)
project(kinova_arm_apps)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_simple_manipulator
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

