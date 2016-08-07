cmake_minimum_required(VERSION 2.8.3)
project(axxb_data_collection)

find_package(catkin REQUIRED COMPONENTS
  denso_control
  ensenso
  image_view
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
