cmake_minimum_required(VERSION 2.8.3)
project(axxb_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  ensenso
  rospy
  tf_conversions
)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system thread)
catkin_package(CATKIN_DEPENDS
  ensenso
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})
