cmake_minimum_required(VERSION 2.8.3)
project(kinova_arm_apps)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

# Compile as C++11 for ROS Kinetic and newer
add_compile_options(-std=c++11
  -O3
  -march=native
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

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(segment ros/src/segment.cpp)
# target_link_libraries(segment ${catkin_LIBRARIES})
add_executable(closest_obj ros/src/kinova_arm_apps/closest_obj.cpp)
target_link_libraries(closest_obj ${catkin_LIBRARIES})
